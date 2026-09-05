"""Unwind history lifetime under the actual Ford independent-lateral hooks."""
import pytest

from opendbc.safety.tests.test_ford_sunnypilot_mads import Harness


@pytest.mark.parametrize('reason', [4, 5, 9, 12])
def test_history_lifetime_on_revocation(reason):
  h = Harness()
  h.engage()
  assert h.steer(.001)
  assert h.safety.test_fp_accepted_path_angle() != 0
  h.safety.safety_lateral_revoke(reason)
  retained = reason in (4, 5)  # brake/regen intentionally retain independent lateral
  assert h.allowed() == retained
  assert (h.safety.test_fp_accepted_path_angle() != 0) == retained
  if not retained:
    h.engage()
    assert h.safety.test_fp_accepted_path_angle() == 0


@pytest.mark.parametrize('event', ['cruise_off', 'invalid_rx', 'stale', 'host_clear'])
def test_rx_check_and_heartbeat_revocation_clear_history(event):
  h = Harness()
  h.engage()
  assert h.steer(.001)
  assert h.safety.test_fp_accepted_path_angle() != 0
  if event == 'cruise_off':
    h.rx('EngBrakeData', CcStat_D_Actl=0, BpedDrvAppl_D_Actl=1)
  elif event == 'invalid_rx':
    msg = h.ford._speed_msg(15.)
    msg[0].data[3] ^= 1
    assert not h.safety.safety_rx_hook(msg)
  elif event == 'stale':
    h.now += 1000001
    h.safety.set_timer(h.now)
    # Invoke production lateral_check via a real TX; stale authorization rejects.
    assert not h.steer(.002)
  else:
    h.safety.test_sp_heartbeat(0, 0, 0)
  assert not h.allowed()
  assert h.safety.test_fp_accepted_path_angle() == 0
