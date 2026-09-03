"""Selected Lightning brake parity through the actual Ford safety dispatcher."""
import pytest

from opendbc.safety.tests.test_ford_sunnypilot_mads import Harness


class BrakeHarness(Harness):
  def __init__(self, lightning=True):
    self.braking = False
    self.stopped = False
    self.speed = 15.
    super().__init__(lightning)
    reset = self.ford._reset_curvature_measurement
    self.ford._reset_curvature_measurement = lambda c, v: reset(c, self.speed)

  def rx(self, name, **values):
    if name == "EngBrakeData" and self.braking:
      values["BpedDrvAppl_D_Actl"] = 2
    if name == "DesiredTorqBrk" and self.stopped:
      values["VehStop_D_Stat"] = 1
    return super().rx(name, **values)

  def brake(self, pressed, cruise=3):
    self.braking = pressed
    self.rx("EngBrakeData", CcStat_D_Actl=cruise, BpedDrvAppl_D_Actl=2 if pressed else 1)


@pytest.mark.parametrize("long_on", [False, True])
@pytest.mark.parametrize("stopped", [False, True])
def test_brake_retains_lateral_cancels_long_and_release_does_not_resume(long_on, stopped):
  h = BrakeHarness()
  h.engage()
  if long_on:
    h.brake(False, cruise=4)
    assert h.safety.get_controls_allowed()
  h.brake(True, cruise=4 if long_on else 3)
  assert not h.safety.get_controls_allowed()
  assert not h.safety.get_longitudinal_allowed()
  assert h.allowed()
  for _ in range(60):
    h.now += 10000
    h.safety.set_timer(h.now)
    # Avoid fabricating a 15 -> 0 m/s cross-sensor mismatch. That unrelated
    # non-brake veto must still work while reaching and holding standstill.
    if stopped:
      h.speed = max(0., h.speed - .5)
      h.stopped = h.speed == 0.
    h.refresh()  # keeps brake held; updates every required source, no new press
    assert h.allowed()
    assert not h.safety.get_longitudinal_allowed()
  assert h.steer()
  h.brake(False)
  assert h.allowed()
  assert not h.safety.get_controls_allowed()
  h.brake(False, cruise=4)  # ordinary explicit cruise re-engagement path
  assert h.safety.get_controls_allowed()


def test_repeated_brakes_and_tja_disable_while_braking():
  h = BrakeHarness()
  h.engage()
  for _ in range(8):
    h.brake(True)
    assert h.allowed()
    h.brake(False)
    assert h.allowed()
    assert not h.safety.get_longitudinal_allowed()
  h.brake(True)
  h.button(False)
  h.button(True)
  assert not h.allowed()
  h.brake(False)
  assert not h.allowed()  # brake release cannot re-engage lateral either


@pytest.mark.parametrize("reason", [r for r in range(1, 15) if r not in (4, 5)])
def test_every_non_brake_revocation_still_clears_while_braking(reason):
  h = BrakeHarness()
  h.engage()
  h.brake(True)
  h.safety.safety_lateral_revoke(reason)
  assert not h.allowed()
  h.brake(False)
  assert not h.allowed()


@pytest.mark.parametrize("reason", [4, 5])
def test_brake_and_regen_notifications_do_not_grant_or_revoke_selected_lateral(reason):
  h = BrakeHarness()
  h.safety.safety_lateral_revoke(reason)
  assert not h.allowed()
  h.engage()
  h.safety.safety_lateral_revoke(reason)
  assert h.allowed()


@pytest.mark.parametrize("message,fields", [
  ("Lane_Assist_Data3_FD1", dict(LatCtlSte_D_Stat=4)),
])
def test_local_status_fault_while_braking(message, fields):
  h = BrakeHarness()
  h.engage()
  h.brake(True)
  h.rx(message, **fields)
  assert not h.allowed()


@pytest.mark.parametrize("raw", [0, 3])
def test_invalid_brake_encoding_still_revokes(raw):
  h = BrakeHarness()
  h.engage()
  h.brake(True)
  Harness.rx(h, "EngBrakeData", CcStat_D_Actl=3, BpedDrvAppl_D_Actl=raw)
  assert not h.allowed()


def test_bad_checksum_while_braking_revokes_and_release_cannot_restore():
  h = BrakeHarness()
  h.engage()
  h.brake(True)
  bad = h.ford._speed_msg(15.)
  bad.data[3] ^= 1
  assert not h.safety.safety_rx_hook(bad)
  assert not h.allowed()
  h.brake(False)
  assert not h.allowed()


@pytest.mark.parametrize("path_angle", [.30, - .30])
def test_path_angle_limits_enforced_while_braking(path_angle):
  h = BrakeHarness()
  h.engage()
  h.brake(True)
  assert not h.steer(path_angle)
  assert not h.allowed()


def test_mads_off_preserves_ordinary_brake_behavior():
  h = BrakeHarness(lightning=False)
  h.brake(False, cruise=4)
  assert h.safety.get_controls_allowed()
  h.brake(True, cruise=4)
  assert not h.safety.get_controls_allowed()
  assert not h.allowed()
  h.brake(False)
  assert not h.safety.get_controls_allowed()
