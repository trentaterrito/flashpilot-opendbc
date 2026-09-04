// Offline characterization only. Never linked into panda or production safety.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "opendbc/safety/sunnypilot/mads.h"

void sp_test_reset(bool enabled, int brake_mode) {
  // Simulate MCU BSS reset. Do not confuse this with upstream's mode initializer.
  memset(&m_mads_state, 0, sizeof(m_mads_state));
  mads_button_press = MADS_BUTTON_UNAVAILABLE;
  heartbeat_engaged_mads = false;
  heartbeat_engaged_mads_mismatches = 0U;
  mads_set_system_state(enabled, brake_mode == 2, brake_mode == 1);
}

void sp_test_update(bool main, bool longitudinal, bool button, bool brake, bool override) {
  mads_button_press = button ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
  mads_state_update(true, main, longitudinal, brake, override);
}

bool sp_test_allowed(void) { return controls_allowed_lateral; }
bool sp_test_requested(void) { return m_mads_state.controls_requested_lateral; }
void sp_test_revoke(int reason) { mads_exit_controls((DisengageReason)reason); }
void sp_test_heartbeat(bool engaged) {
  heartbeat_engaged_mads = engaged;
  mads_heartbeat_engaged_check();
}
