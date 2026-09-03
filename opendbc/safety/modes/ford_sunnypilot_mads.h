#pragma once

#include "opendbc/safety/sunnypilot/mads.h"

// FlashPilot integration boundary, not a replacement MADS state machine.
// Upstream owns controls_allowed_lateral. This adapter can veto it, feeds only
// fresh physical TJA intent, and never writes controls_allowed.
// Selected only by the explicit Lightning CAN-FD MADS safety parameter.
#define FORD_SP_STATUS_MAX_AGE_US 100000U

typedef struct {
  bool enabled;
  bool platform_ready;
  bool mode_ready;
  bool lateral_ok;
  bool lateral_progress_seen;
  unsigned int lateral_counter;
  uint32_t lateral_progress_ts;
  bool main_on;
  bool cruise_engaged_prev;
  bool cruise_release_seen;
  bool brake_ok;
  bool release_seen;
  bool button_prev;
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
  // Granting on a fresh physical press must not erase the already-validated
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
    ford_sp_reset_upstream(false);
    ford_sp_gate.release_seen = false;
    ford_sp_gate.button_prev = false;
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

// Called under the board's interrupt lock after legacy heartbeat bookkeeping.
// Valid host input can restore eligibility, never grant lateral authorization.
static inline void ford_sp_host_heartbeat(uint16_t longitudinal, uint16_t lateral, uint16_t length) {
  heartbeat_engaged_mads = (lateral == 1U) && (longitudinal <= 1U) && (length == 0U);
  if (ford_sp_gate.enabled) {
    ford_sp_gate.platform_ready = heartbeat_engaged_mads;
    ford_sp_check();
  }
  if (!heartbeat_engaged_mads) {
    safety_lateral_revoke(LATERAL_REVOKE_HOST);
  }
}

static bool ford_sp_lateral_allowed(void) {
  ford_sp_check();
  return ford_sp_gate.enabled ? controls_allowed_lateral : controls_allowed;
}

static void ford_sp_physical_request(bool toggle) {
  if (controls_allowed_lateral) {
    if (toggle) {
      ford_sp_revoke(LATERAL_REVOKE_BUTTON);
    }
  } else {
    ford_sp_reset_upstream(true);
    mads_button_press = MADS_BUTTON_NOT_PRESSED;
    mads_state_update(vehicle_moving, false, false, false, false);
    mads_button_press = MADS_BUTTON_PRESSED;
    mads_state_update(vehicle_moving, false, false, false, false);
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
        const bool cruise_engaged = (cruise == 4U) || (cruise == 5U);
        const bool cruise_rising = cruise_engaged && !ford_sp_gate.cruise_engaged_prev;
        const unsigned int brake_state = (msg->data[0] >> 4U) & 3U;
        // DBC: 1 = released, 2 = driver braking, 0/3 = not allowed.
        ford_sp_gate.brake_ok = (brake_state == 1U) || (brake_state == 2U);
        if (!cruise_engaged && ford_sp_gate.main_on) {
          ford_sp_gate.cruise_release_seen = true;
        }
        // SET/RESUME is represented by Ford's transition from ACC-main ready
        // (state 3) to engaged (4/5). It may grant lateral, but never toggles
        // an already-active lateral session off. Requiring state 3 first
        // prevents a panda restart during active cruise from auto-engaging.
        if (cruise_rising && ford_sp_gate.cruise_release_seen && ford_sp_vehicle_ready()) {
          ford_sp_physical_request(false);
        }
        ford_sp_gate.cruise_engaged_prev = cruise_engaged;
      }
      ford_sp_check();
      if ((msg->addr == 0x83U) && ford_sp_vehicle_ready()) {
        const bool pressed = (msg->data[5] & 1U) != 0U;
        if (pressed && !ford_sp_gate.button_prev && ford_sp_gate.release_seen) {
          ford_sp_physical_request(true);
        }
        if (!pressed) {
          ford_sp_gate.release_seen = true;
        }
        ford_sp_gate.button_prev = pressed;
      }
    } else {
      // Other buses and unrelated messages cannot refresh required state.
    }
  }
}
