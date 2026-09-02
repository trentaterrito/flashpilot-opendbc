#!/usr/bin/env python3
"""
FlashPilot: panda safety tests for the narrow Ford path-angle-primary support added
in opendbc/safety/modes/ford.h (fp_angle_mode_engaged, fp_path_angle_cmd_checks,
fp_shadow_curvature_error_check). Additive to test_ford.py -- does not modify it.

These tests only apply to the CAN FD Ford safety mode (LateralMotionControl2 /
Lane_Assist_Data1's extra bits); the classic LateralMotionControl (non-FD) path is
untouched by FlashPilot and is already fully covered by test_ford.py.
"""
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

MSG_Lane_Assist_Data1 = 0x3CA
MSG_LateralMotionControl2 = 0x3D6
MSG_Yaw_Data_FD1 = 0x91
MSG_BrakeSysFeatures = 0x415
MSG_EngVehicleSpThrottle2 = 0x202


def checksum(msg):
  # identical to test_ford.py's checksum() -- duplicated locally to keep this file
  # fully additive/standalone rather than importing private helpers from test_ford.py
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == MSG_Yaw_Data_FD1:
    chksum = dat[0] + dat[1]
    chksum += dat[2] + dat[3]
    chksum += dat[5]
    chksum += dat[6] >> 6
    chksum += (dat[6] >> 4) & 0x3
    chksum = 0xff - (chksum & 0xff)
    ret[4] = chksum

  elif addr == MSG_BrakeSysFeatures:
    chksum = dat[0] + dat[1]
    chksum += (dat[2] >> 2) & 0xf
    chksum += dat[2] >> 6
    chksum = 0xff - (chksum & 0xff)
    ret[3] = chksum

  elif addr == MSG_EngVehicleSpThrottle2:
    chksum = (dat[2] >> 3) & 0xf
    chksum += (dat[4] >> 5) & 0x3
    chksum += dat[6] + dat[7]
    chksum = 0xff - (chksum & 0xff)
    ret[1] = chksum

  return addr, ret, bus

DEG_TO_CAN = 50000            # curvature_to_can
PATH_ANGLE_DEG_TO_CAN = 5000  # FORD_FP_PATH_ANGLE_LIMITS.angle_deg_to_can
FORD_FP_PATH_ANGLE_MIN = -0.25
FORD_FP_PATH_ANGLE_MAX = 0.25
FORD_FP_DBC_PATH_ANGLE_MIN = -0.5
FORD_FP_DBC_PATH_ANGLE_MAX = 0.5235
CURVATURE_ERROR_MIN_SPEED = 10.0
MAX_CURVATURE_ERROR = 100  # CAN units, FORD_STEERING_LIMITS.max_curvature_error


class TestFlashPilotFordPathAngleSafety(unittest.TestCase):
  """Exercises the FlashPilot-only code paths in ford.h: everything here requires
  fp_angle_mode_engaged=True, which is only ever set by a Lane_Assist_Data1 frame
  carrying the flag bit -- i.e. the FlashPilot Lightning carcontroller path. A normal
  Ford (curvature-primary) never sets this bit, so none of these checks can fire for
  any other Ford platform; see test_non_flashpilot_ford_never_engages_angle_mode."""

  packer: CANPackerSafety
  safety: libsafety_py.LibSafety

  LATERAL_FREQUENCY = 20  # Hz, matches FORD_FP_PATH_ANGLE_LIMITS.frequency / FORD_STEERING_LIMITS.frequency

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.CANFD)
    self.safety.init_tests()
    self.cnt_lat_ctl = 0
    self.cnt_speed = 0
    self.cnt_speed_2 = 0
    self.cnt_yaw_rate = 0

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  # --- message builders -----------------------------------------------------

  def _lka_msg(self, action: int = 0, angle_mode_engaged: bool = False, shadow_curvature_can: int = 0):
    """Builds Lane_Assist_Data1, then patches in FlashPilot's extra bits exactly as
    opendbc/car/ford/fordcan.py's create_lka_msg does: bit 0 of byte 4, and bytes 5-6
    as a big-endian signed int16 at scale 1e-6 1/m (shadow_curvature_can is in the
    same CAN units as curvature_to_can, i.e. divide by 0.05 to get the raw int16)."""
    values = {"LkaActvStats_D2_Req": action}
    addr, dat, bus = self.packer.make_can_msg("Lane_Assist_Data1", 0, values)
    dat = bytearray(dat)
    if angle_mode_engaged:
      dat[4] |= 0x1
    else:
      dat[4] &= ~0x1
    raw = int(round(shadow_curvature_can / 0.05))
    raw = max(-32768, min(32767, raw))
    dat[5] = (raw >> 8) & 0xFF
    dat[6] = raw & 0xFF
    return libsafety_py.make_CANPacket(addr, bus, dat)

  def _lat_ctl2_msg(self, steer_control_enabled: bool, path_angle: float, curvature: float = 0.0,
                    path_offset: float = 0.0, curvature_rate: float = 0.0):
    self.safety.set_timer(self.cnt_lat_ctl * int(1e6 / self.LATERAL_FREQUENCY))
    self.cnt_lat_ctl += 1
    values = {
      "LatCtl_D2_Rq": 1 if steer_control_enabled else 0,
      "LatCtlPathOffst_L_Actl": path_offset,
      "LatCtlPath_An_Actl": path_angle,
      "LatCtlCrv_NoRate2_Actl": curvature_rate,
      "LatCtlCurv_No_Actl": curvature,
    }
    return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  def _speed_msg(self, speed: float):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3, "VehVActlBrk_No_Cnt": self.cnt_speed % 16}
    self.cnt_speed += 1
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values, fix_checksum=checksum)

  def _speed_msg_2(self, speed: float):
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3, "VehVActlEng_No_Cnt": self.cnt_speed_2 % 16}
    self.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values, fix_checksum=checksum)

  def _yaw_rate_msg(self, curvature: float, speed: float):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3, "VehRollYaw_No_Cnt": self.cnt_yaw_rate % 256}
    self.cnt_yaw_rate += 1
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values, fix_checksum=checksum)

  def _reset_curvature_measurement(self, curvature, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  def _engage_angle_mode(self, shadow_curvature_can: int = 0):
    """Sends the LKA frame that flips fp_angle_mode_engaged on, as the real
    carcontroller does every LKA_STEP frames before/alongside LMC2."""
    self._tx(self._lka_msg(action=0, angle_mode_engaged=True, shadow_curvature_can=shadow_curvature_can))

  def _disengage_angle_mode(self):
    self._tx(self._lka_msg(action=0, angle_mode_engaged=False, shadow_curvature_can=0))

  # --- tests -----------------------------------------------------------------

  def test_non_flashpilot_ford_never_engages_angle_mode(self):
    """A stock (non-FlashPilot) Ford always sends Lane_Assist_Data1 with the
    FlashPilot bits at 0 -- confirm the flag reads back false and LMC2 stays on the
    unmodified curvature-primary path (path_angle must be the inactive sentinel)."""
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 20.0)
    self._disengage_angle_mode()
    # curvature-primary: path_angle must stay 0 (inactive), curvature may move
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0, curvature=0)))
    self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=0.1, curvature=0)),
                      "non-angle-mode frame must reject nonzero path_angle")

  def test_angle_mode_requires_engaged_flag(self):
    """Without the Lane_Assist_Data1 flag set, LMC2 must reject a path_angle-primary
    frame (curvature at inactive sentinel, nonzero path_angle) exactly like any other
    Ford -- confirms the wide DBC range can't be reached by an LMC2 frame alone."""
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 20.0)
    self._disengage_angle_mode()
    self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=0.2, curvature=0)))

  def test_angle_mode_curvature_must_stay_inactive(self):
    """Once angle mode is engaged, LMC2 must reject any nonzero curvature -- angle
    mode and curvature-primary mode are mutually exclusive per frame."""
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 20.0)
    self._engage_angle_mode(shadow_curvature_can=0)
    self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=0.05, curvature=0.001)))
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0.05, curvature=0)))

  def test_angle_mode_path_angle_value_range(self):
    """In angle mode, path_angle must stay within the DBC-wide range
    (FORD_FP_DBC_PATH_ANGLE_MIN/MAX); outside it must always be rejected regardless
    of rate-of-change state."""
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 20.0)
    self._engage_angle_mode(shadow_curvature_can=0)

    for angle in (FORD_FP_DBC_PATH_ANGLE_MAX + 0.01, -(abs(FORD_FP_DBC_PATH_ANGLE_MIN) + 0.01)):
      self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=angle, curvature=0)))

  def test_angle_mode_path_angle_inactive_when_not_steering(self):
    """steer_control_enabled=False must force path_angle to exactly 0, in angle mode
    just as in curvature-primary mode."""
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 20.0)
    self._engage_angle_mode(shadow_curvature_can=0)
    self.assertTrue(self._tx(self._lat_ctl2_msg(False, path_angle=0, curvature=0)))
    self.assertFalse(self._tx(self._lat_ctl2_msg(False, path_angle=0.05, curvature=0)))

  def test_angle_mode_path_angle_rate_of_change(self):
    """fp_path_angle_cmd_checks: a path_angle jump larger than the rate-of-change
    table allows must be rejected; a jump within it must pass."""
    self.safety.set_controls_allowed(True)
    speed = 20.0
    self._reset_curvature_measurement(0, speed)
    self._engage_angle_mode(shadow_curvature_can=0)

    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0.0, curvature=0)))
    # a small step should be allowed
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0.01, curvature=0)))
    # a huge single-frame jump to the opposite end of the range must be rejected
    self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=FORD_FP_DBC_PATH_ANGLE_MAX, curvature=0)))

  def test_angle_mode_shadow_curvature_deviation_check(self):
    """fp_shadow_curvature_error_check: shadow_curvature carried on Lane_Assist_Data1
    must stay within max_curvature_error of the real measured curvature once above
    curvature_error_min_speed, exactly mirroring curvature-primary mode's own check."""
    self.safety.set_controls_allowed(True)
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    self._reset_curvature_measurement(0, speed)

    # shadow_curvature close to measured (0) must pass
    self._engage_angle_mode(shadow_curvature_can=0)
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0.0, curvature=0)))

    # shadow_curvature far outside the error band must fail
    self._engage_angle_mode(shadow_curvature_can=MAX_CURVATURE_ERROR * 4)
    self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=0.0, curvature=0)))

  def test_angle_mode_shadow_curvature_check_skipped_below_min_speed(self):
    """Below curvature_error_min_speed, the deviation check must not fire (matches
    curvature-primary mode's own existing behavior at low speed)."""
    self.safety.set_controls_allowed(True)
    speed = CURVATURE_ERROR_MIN_SPEED - 5
    self._reset_curvature_measurement(0, speed)
    self._engage_angle_mode(shadow_curvature_can=MAX_CURVATURE_ERROR * 4)
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0.0, curvature=0)))

  def test_angle_mode_blocked_without_controls_allowed(self):
    """No path_angle steering is allowed when controls_allowed is False, in angle
    mode just as in curvature-primary mode."""
    self.safety.set_controls_allowed(False)
    self._reset_curvature_measurement(0, 20.0)
    self._engage_angle_mode(shadow_curvature_can=0)
    self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=0.05, curvature=0)))

  def test_no_reset_bypass_latch(self):
    """Regression test: FlashPilot must NOT port BluePilot's reset-bypass latch (a
    time-windowed override that skips rate-of-change checks for N seconds after a
    controls reset). fp_path_angle_cmd_checks has no timestamp/timer state of any
    kind -- every frame is checked purely against the immediately-preceding command,
    with no elapsed-time-based grace window. Prove this by alternating between two
    values whose delta always exceeds one frame's rate-of-change limit: if any kind
    of time-windowed bypass existed, repeatedly "resetting" via this pattern would
    eventually let a jump through; here every single alternation must still violate,
    with no upward trend in pass rate over 50 repeats."""
    self.safety.set_controls_allowed(True)
    speed = 20.0
    self._reset_curvature_measurement(0, speed)
    self._engage_angle_mode(shadow_curvature_can=0)
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=0.0, curvature=0)))

    for i in range(50):
      self._engage_angle_mode(shadow_curvature_can=0)
      self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=FORD_FP_DBC_PATH_ANGLE_MAX, curvature=0)),
                        f"iter {i}: jump to max must violate ROC regardless of prior history")
      self._engage_angle_mode(shadow_curvature_can=0)
      self.assertFalse(self._tx(self._lat_ctl2_msg(True, path_angle=FORD_FP_DBC_PATH_ANGLE_MIN, curvature=0)),
                        f"iter {i}: jump to min must violate ROC regardless of prior history -- "
                        f"a time-windowed bypass would eventually let one of these two through")

    # a legal, small, in-range step from the actual last-commanded value still works --
    # confirms the rejections above are real per-frame checks, not a stuck failure
    self._engage_angle_mode(shadow_curvature_can=0)
    self.assertTrue(self._tx(self._lat_ctl2_msg(True, path_angle=FORD_FP_DBC_PATH_ANGLE_MIN, curvature=0)))


if __name__ == "__main__":
  unittest.main()
