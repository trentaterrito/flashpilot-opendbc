#pragma once

#include "opendbc/safety/sunnypilot/mads.h"

// FlashPilot integration boundary for Always-On Lateral, reusing the existing
// independent-lateral transport. The adapter grants controls_allowed_lateral
// directly from fresh host and vehicle evidence and never writes
// controls_allowed. The safety flag name remains legacy wire compatibility.
#define FORD_SP_STATUS_MAX_AGE_US 100000U

typedef struct {
  bool enabled;
  bool host_clear_seen;
  bool platform_ready;
  bool mode_ready;
  bool lateral_ok;
  bool lateral_progress_seen;
  unsigned int lateral_counter;
  uint32_t lateral_progress_ts;
  bool main_on;
  bool brake_ok;
  uint32_t reason;
} FordSunnyMadsGate;

static FordSunnyMadsGate ford_sp_gate;
static bool (*ford_sp_board_ready)(void) = NULL;

static bool ford_sp_status_checksum_valid(const CANPacket_t *msg) {
  // Measured 0x3CC status-group checksum, not full-frame authentication.
  const unsigned int state = msg->data[2] & 7U;
  const unsigned int limit = msg->data[4] & 3U;
  const unsigned int capability = msg->data[4] >> 6U;
  const unsigned int counter = (msg->data[4] >> 2U) & 15U;
  return (GET_LEN(msg) == 8U) && (msg->data[5] == (255U - state - limit - capability - counter));
}

static bool ford_sp_status_ready(void) {
  return ford_sp_gate.lateral_ok && ford_sp_gate.lateral_progress_seen &&
         (safety_get_ts_elapsed(microsecond_timer_get(), ford_sp_gate.lateral_progress_ts) <= FORD_SP_STATUS_MAX_AGE_US);
}

static inline void ford_sp_set_board_check(bool (*check)(void)) {
  ford_sp_board_ready = check;
}

static void ford_sp_reset_upstream(bool enabled) {
  // Upstream's mode initializer alone does not reset every edge-history field.
  // Clear history as on MCU BSS reset before calling the upstream initializer.
  m_mads_state = (MADSState){0};
  mads_button_press = MADS_BUTTON_UNAVAILABLE;
  // Granting after fresh host and vehicle validation must not erase the
  // host veto input. Reset/revocation does clear it and requires a new heartbeat.
  if (!enabled) {
    heartbeat_engaged_mads = false;
  }
  heartbeat_engaged_mads_mismatches = 0U;
  mads_set_system_state(enabled, false, false);  // SunnyPilot REMAIN_ACTIVE; no pause/resume.
}

static void ford_sp_revoke(lateral_revocation_reason reason) {
  // Generic safety already cancels ordinary longitudinal permission on brake /
  // regen. Only selected Lightning MADS retains independent lateral permission;
  // all non-brake reasons still take the immediate revocation path below.
  if (ford_sp_gate.enabled && (reason != LATERAL_REVOKE_BRAKE) && (reason != LATERAL_REVOKE_REGEN)) {
    const bool was_authorized = controls_allowed_lateral;
    ford_sp_reset_upstream(false);
    ford_sp_gate.platform_ready = false;
    if (was_authorized) {
      ford_sp_gate.host_clear_seen = false;
    }
    ford_sp_gate.reason = (uint32_t)reason;
    if (reason == LATERAL_REVOKE_RESET) {
      ford_sp_gate = (FordSunnyMadsGate){0};
    }
  }
}

static bool ford_sp_vehicle_ready(void) {
  if (ford_sp_board_ready != NULL) {
    ford_sp_gate.platform_ready = ford_sp_gate.platform_ready && ford_sp_board_ready();
  }
  bool valid = ford_sp_gate.platform_ready && ford_sp_gate.mode_ready;
  valid = valid && !relay_malfunction && !safety_rx_checks_invalid && !steering_disengage;
  // Repeated counter values do not renew freshness, even if RX continues.
  // This detects a frozen source; a replayed changing sequence is NOT authenticated.
  valid = valid && ford_sp_status_ready();
  valid = valid && ford_sp_gate.main_on && ford_sp_gate.brake_ok;
  valid = valid && (SAFETY_ABS(vehicle_speed.values[0] - vehicle_speed_2.values[0]) <= (2 * VEHICLE_SPEED_FACTOR));
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxCheck *check = &current_safety_config.rx_checks[i];
    valid = valid && check->status.msg_seen && check->status.valid_checksum && check->status.valid_quality_flag;
    // Match the existing Ford RX validator rather than imposing first-error
    // or three-period overlays. Its invalid/lag callbacks still revoke locally.
    valid = valid && !check->status.lagging && (check->status.wrong_counters < MAX_WRONG_COUNTERS);
  }
  return valid;
}

static void ford_sp_check(void) {
  if (ford_sp_gate.enabled && !ford_sp_vehicle_ready()) {
    ford_sp_revoke(LATERAL_REVOKE_VEHICLE);
  }
}

static void ford_sp_authorize_if_ready(void);

// Called under the board's interrupt lock after legacy heartbeat bookkeeping.
// Valid host input is one of several mandatory gates for lateral authorization.
static inline void ford_sp_host_heartbeat(uint16_t longitudinal, uint16_t lateral, uint16_t length) {
  const bool well_formed = (longitudinal <= 1U) && (length == 0U);
  const bool positive = well_formed && (lateral == 1U);
  const bool negative = well_formed && (lateral == 0U);
  heartbeat_engaged_mads = positive;
  if (ford_sp_gate.enabled) {
    // After reset/revocation, ignore stale positive intent until the host has
    // explicitly observed and acknowledged cleared lateral authorization.
    ford_sp_gate.platform_ready = positive && (controls_allowed_lateral || ford_sp_gate.host_clear_seen);
  }
  if (!heartbeat_engaged_mads) {
    safety_lateral_revoke(LATERAL_REVOKE_HOST);
    // Record the acknowledgement after revocation clears the old latch.
    ford_sp_gate.host_clear_seen = negative;
  }
}

static bool ford_sp_lateral_allowed(void) {
  ford_sp_check();
  return ford_sp_gate.enabled ? controls_allowed_lateral : controls_allowed;
}

static void ford_sp_authorize_if_ready(void) {
  if (ford_sp_gate.enabled && !controls_allowed_lateral && ford_sp_vehicle_ready()) {
    // StarPilot's Ford AOL is an ACC-main permission independent of ordinary
    // controls_allowed. Apply that directly, retaining FlashPilot's platform,
    // host acknowledgement and vehicle vetoes in ford_sp_vehicle_ready().
    // No artificial button edge or MADS engagement state machine is needed.
    controls_allowed_lateral = true;
    ford_sp_gate.host_clear_seen = false;
    ford_sp_gate.reason = 0U;
  }
}

static void ford_sp_rx(const CANPacket_t *msg, bool valid) {
  if (ford_sp_gate.enabled) {
    bool required = false;
    for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
      required |= msg->addr == (unsigned int)current_safety_config.rx_checks[i].msg[0].addr;
    }
    // Host Ford CANParser/CarState owns gear, EPS, pinion and ESP validity as
    // in SunnyPilot. No duplicate raw state or unbounded last-known fallback.
    required |= msg->addr == 0x3CCU;
    const bool lateral_status = (msg->addr == 0x3CCU) && (msg->bus == 0U);
    const bool status_integrity = !lateral_status || ford_sp_status_checksum_valid(msg);
    if (!valid || !status_integrity || (required && (msg->bus == 0U) && (GET_LEN(msg) != 8U))) {
      if (lateral_status) {
        ford_sp_gate.lateral_ok = false;
      }
      ford_sp_revoke(LATERAL_REVOKE_INVALID_RX);
    } else if (required && (msg->bus == 0U)) {
      if (msg->addr == 0x3CCU) {
        const unsigned int state = msg->data[2] & 7U;
        const unsigned int counter = (msg->data[4] >> 2U) & 15U;
        if (!ford_sp_gate.lateral_progress_seen || (counter != ford_sp_gate.lateral_counter)) {
          ford_sp_gate.lateral_progress_seen = true;
          ford_sp_gate.lateral_counter = counter;
          ford_sp_gate.lateral_progress_ts = microsecond_timer_get();
        }
        ford_sp_gate.lateral_ok = (state >= 1U) && (state <= 3U);
      }
      if (msg->addr == 0x165U) {
        const unsigned int cruise = msg->data[1] & 7U;
        ford_sp_gate.main_on = (cruise >= 3U) && (cruise <= 5U);
        const unsigned int brake_state = (msg->data[0] >> 4U) & 3U;
        // DBC: 1 = released, 2 = driver braking, 0/3 = not allowed.
        ford_sp_gate.brake_ok = (brake_state == 1U) || (brake_state == 2U);
      }
      ford_sp_check();
      // Always-On Lateral uses the existing heartbeat and vehicle-state gates.
      // SET/CANCEL and the TJA button are deliberately non-authoritative.
      ford_sp_authorize_if_ready();
    } else {
      // Other buses and unrelated messages cannot refresh required state.
    }
  }
}
