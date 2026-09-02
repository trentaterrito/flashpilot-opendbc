"""
FlashPilot: tests for the Lightning-only path-angle-primary lateral control gate.

Constructs a real CarController and calls the real update() (not a mock of it) --
CarControl must be a genuine capnp reader (round-tripped through to_bytes/from_bytes,
matching how it actually arrives via msgq in production) for actuators.as_builder()
to work; a locally-constructed builder object doesn't support that call.
"""
import os
import unittest
from collections import defaultdict
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.flashpilot_angle import path_angle_curvature_factor
from opendbc.car.ford.values import CAR, DBC, FordFlags

FORD_LateralMotionControl2 = 0x3D6
FORD_Lane_Assist_Data1 = 0x3CA


class TestFlashPilotAngleTuning(unittest.TestCase):
  def test_bluepilot_effective_endpoint_gains(self):
    self.assertAlmostEqual(path_angle_curvature_factor(13.5, 0.0007), 1.00)
    self.assertAlmostEqual(path_angle_curvature_factor(13.5, 0.0010), 1.274)
    self.assertAlmostEqual(path_angle_curvature_factor(26.82, 0.0007), 0.7885)
    self.assertAlmostEqual(path_angle_curvature_factor(26.82, 0.0010), 0.855)

  def test_bluepilot_representative_interpolation_points(self):
    # Mid-speed/mid-curvature and quarter/three-quarter points calculated from
    # BluePilot's same nested linear interpolation.
    cases = (
      (20.16, 0.000850, 0.979375),
      (16.83, 0.000775, 1.00265625),
      (23.49, 0.000925, 0.93015625),
    )
    for speed, curvature, expected in cases:
      with self.subTest(speed=speed, curvature=curvature):
        self.assertAlmostEqual(path_angle_curvature_factor(speed, curvature), expected)


def _make_controller(fingerprint):
  CP = structs.CarParams(carFingerprint=fingerprint, flags=int(FordFlags.CANFD), openpilotLongitudinalControl=False)
  dbc_names = {Bus.pt: DBC[fingerprint][Bus.pt]}
  return CarController(dbc_names, CP)


def _make_cs(v_ego=0.0, yaw_rate=0.0, steering_pressed=False, steering_angle_deg=0.0):
  cs_out = structs.CarState(vEgoRaw=v_ego, yawRate=yaw_rate, steeringPressed=steering_pressed,
                             steeringAngleDeg=steering_angle_deg)
  return SimpleNamespace(out=cs_out, buttons_stock_values=defaultdict(int),
                          acc_tja_status_stock_values=defaultdict(int), lkas_status_stock_values=defaultdict(int))


def _make_cc_reader(curvature=0.0, lat_active=True):
  # actuators.as_builder() (used at the end of CarController.update()) only exists on
  # capnp reader objects, not on locally-constructed builders -- round-trip through
  # bytes to get a real reader, matching how CC actually arrives via msgq in production.
  builder = structs.CarControl(latActive=lat_active, actuators=structs.CarControl.Actuators(curvature=curvature))
  return structs.CarControl.from_bytes(builder.to_bytes())


def _find_msg(can_sends, addr):
  """Returns the *last* matching message (can_sends may be a list of per-frame lists
  concatenated across several update() calls; LMC2/LKA aren't sent every frame)."""
  found = None
  for a, dat, bus in can_sends:
    if a == addr:
      found = dat
  return found


def _decode_lmc2(dat: bytes):
  # Exact bit layout, cross-checked against opendbc/safety/modes/ford.h's own
  # ford_tx_hook decode of FORD_LateralMotionControl2 (same message).
  mode = (dat[0] >> 4) & 0x7
  raw_curvature = (dat[2] << 3) | (dat[3] >> 5)
  raw_path_angle = ((dat[3] & 0x1F) << 6) | (dat[4] >> 2)
  return mode, raw_curvature, raw_path_angle


def _decode_lka_extra(dat: bytes):
  angle_mode_engaged = bool(dat[4] & 0x1)
  shadow_curvature_raw = (dat[5] << 8) | dat[6]
  if shadow_curvature_raw >= 0x8000:
    shadow_curvature_raw -= 0x10000
  return angle_mode_engaged, shadow_curvature_raw


class TestFlashPilotAngleGate(unittest.TestCase):
  def setUp(self):
    os.environ.pop('FLASHPILOT_ANGLE_ENABLED', None)

  def tearDown(self):
    os.environ.pop('FLASHPILOT_ANGLE_ENABLED', None)

  def test_non_lightning_never_constructs_flashpilot_angle(self):
    for car in (CAR.FORD_F_150_MK14, CAR.FORD_EXPEDITION_MK4, CAR.FORD_MUSTANG_MACH_E_MK1, CAR.FORD_RANGER_MK2):
      os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'  # even with the flag on, only the Lightning is gated in
      cc = _make_controller(car)
      self.assertIsNone(cc.flashpilot_angle, f"{car} must never construct FlashPilotAngleController")
      self.assertFalse(cc._fp_angle_mode_engaged)

  def test_lightning_env_var_off_by_default(self):
    # No env var set (see setUp) -- default-off, matching upstream curvature-primary behavior.
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    self.assertIsNone(cc.flashpilot_angle)
    self.assertFalse(cc._fp_angle_mode_engaged)

  def test_lightning_env_var_on_constructs_controller(self):
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    self.assertIsNotNone(cc.flashpilot_angle)
    self.assertTrue(cc._fp_angle_mode_engaged)

  def test_non_lightning_output_matches_curvature_primary(self):
    """A non-Lightning Ford must always take the curvature-primary path: mode reflects
    latActive, curvature is nonzero, path_angle is exactly the inactive sentinel (0),
    and the LKA message's extra bytes stay all-zero -- byte-for-byte what upstream's
    unmodified create_lka_msg(packer, CAN) produces."""
    cc = _make_controller(CAR.FORD_F_150_MK14)
    with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
      _, can_sends = cc.update(CC, _make_cs(v_ego=20.0), 0)

    lmc2 = _find_msg(can_sends, FORD_LateralMotionControl2)
    self.assertIsNotNone(lmc2)
    mode, raw_curvature, raw_path_angle = _decode_lmc2(lmc2)
    self.assertEqual(mode, 1)
    self.assertNotEqual(raw_curvature, 1000, "curvature-primary must send nonzero curvature when steering")
    self.assertEqual(raw_path_angle, 1000, "path_angle must stay at the inactive sentinel (1000) for non-Lightning")

    lka = _find_msg(can_sends, FORD_Lane_Assist_Data1)
    self.assertIsNotNone(lka)
    angle_mode_engaged, shadow_curvature_raw = _decode_lka_extra(lka)
    self.assertFalse(angle_mode_engaged)
    self.assertEqual(shadow_curvature_raw, 0)

  def test_lightning_disabled_matches_curvature_primary(self):
    """Lightning with the flag off must be indistinguishable from any other Ford."""
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
      _, can_sends = cc.update(CC, _make_cs(v_ego=20.0), 0)

    lmc2 = _find_msg(can_sends, FORD_LateralMotionControl2)
    mode, raw_curvature, raw_path_angle = _decode_lmc2(lmc2)
    self.assertEqual(mode, 1)
    self.assertNotEqual(raw_curvature, 1000)
    self.assertEqual(raw_path_angle, 1000)

  def test_lightning_enabled_produces_nonzero_path_angle_zero_curvature(self):
    """The core positive case: Lightning + flag on + a real steering request produces
    a nonzero path_angle and curvature pinned at the inactive sentinel -- the opposite
    of curvature-primary mode."""
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    # Ramp over several frames so soft-ROC/deviation-clip settle to a steady value.
    # LMC2/LKA are sent every STEER_STEP/LKA_STEP frames, not every call -- accumulate.
    all_sends = []
    for _ in range(20):
      with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
        _, sends = cc.update(CC, _make_cs(v_ego=20.0), 0)
        all_sends += sends

    lmc2 = _find_msg(all_sends, FORD_LateralMotionControl2)
    self.assertIsNotNone(lmc2)
    mode, raw_curvature, raw_path_angle = _decode_lmc2(lmc2)
    self.assertEqual(mode, 1)
    self.assertEqual(raw_curvature, 1000, "curvature must stay at the inactive sentinel (1000) in angle mode")
    self.assertNotEqual(raw_path_angle, 1000, "path_angle must be nonzero once ramped up")

    lka = _find_msg(all_sends, FORD_Lane_Assist_Data1)
    angle_mode_engaged, shadow_curvature_raw = _decode_lka_extra(lka)
    self.assertTrue(angle_mode_engaged)
    self.assertNotEqual(shadow_curvature_raw, 0, "shadow_curvature should track the nonzero commanded curvature")

  def test_lightning_enabled_inactive_sends_mode_zero(self):
    """latActive=False must still produce mode 0 / all-zero signals in angle mode,
    exactly as it does in curvature-primary mode."""
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    with _make_cc_reader(curvature=0.01, lat_active=False) as CC:
      _, can_sends = cc.update(CC, _make_cs(v_ego=20.0), 0)

    lmc2 = _find_msg(can_sends, FORD_LateralMotionControl2)
    mode, raw_curvature, raw_path_angle = _decode_lmc2(lmc2)
    self.assertEqual(mode, 0)
    self.assertEqual(raw_curvature, 1000)
    self.assertEqual(raw_path_angle, 1000)

  def test_lightning_enabled_human_turn_forces_mode_zero(self):
    """Sustained driver override at a large wheel angle must force mode 0, not a
    wound-up path_angle command."""
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    # ramp up steering first
    for _ in range(20):
      with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
        cc.update(CC, _make_cs(v_ego=20.0), 0)
    # driver grabs the wheel hard for > 3.0s (wheel already past 45 deg at contact -> preturned
    # hold). The detector only advances once per STEER_STEP=5 cc.update() calls (0.05s/advance),
    # so >3.0s needs >300 calls, not >60 -- comfortably clear that with 400.
    # LMC2 is sent every STEER_STEP frames, not every call -- accumulate across the hold.
    all_sends = []
    for _ in range(400):
      with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
        _, sends = cc.update(CC, _make_cs(v_ego=20.0, steering_pressed=True, steering_angle_deg=50.0), 0)
        all_sends += sends

    lmc2 = _find_msg(all_sends, FORD_LateralMotionControl2)
    mode, _, raw_path_angle = _decode_lmc2(lmc2)
    self.assertEqual(mode, 0, "human turn override must force mode 0")
    self.assertEqual(raw_path_angle, 1000, "path_angle must be the inactive sentinel while overridden")


if __name__ == "__main__":
  unittest.main()
