#include "opendbc/safety/safety.h"

// this file is checked by cppcheck

extern uint32_t microsecond_timer_get(void);

// Ignore misra-c2012-8.7 as these functions are only called from libsafety
SAFETY_UNUSED(heartbeat_engaged);

SAFETY_UNUSED(safety_rx_hook);
SAFETY_UNUSED(safety_tx_hook);
SAFETY_UNUSED(safety_fwd_hook);
SAFETY_UNUSED(safety_tick);
SAFETY_UNUSED(set_safety_hooks);
SAFETY_UNUSED(safety_lateral_revoke);
  SAFETY_UNUSED(ford_sp_set_board_check);
  SAFETY_UNUSED(ford_sp_host_heartbeat);
SAFETY_UNUSED(mads_heartbeat_engaged_check);
