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
from unittest.mock import patch

from opendbc.car import Bus, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.flashpilot_angle import (FlashPilotAngleController, FlashPilotAngleResult, ManualTurnState,
                                               limit_curvature_for_unwind, path_angle_curvature_factor)
from opendbc.car.ford.values import CAR, DBC, CarControllerParams, FordFlags

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

  def test_default_arguments_preserve_canonical_factors(self):
    self.assertEqual(path_angle_curvature_factor(13.5, 0.0),
                     path_angle_curvature_factor(13.5, 0.0, 0.98, 0.90, 0.83))
    self.assertEqual(path_angle_curvature_factor(26.82, 0.001),
                     path_angle_curvature_factor(26.82, 0.001, 0.98, 0.90, 0.83))

  def test_user_factors_change_only_factor_endpoints(self):
    self.assertAlmostEqual(path_angle_curvature_factor(13.5, 0.001, 1.01, 0.91, 0.84), 1.30 * 1.01)
    self.assertAlmostEqual(path_angle_curvature_factor(26.82, 0.0007, 1.01, 0.91, 0.84), 0.95 * 0.84)
    self.assertAlmostEqual(path_angle_curvature_factor(26.82, 0.001, 1.01, 0.91, 0.84), 0.95 * 0.91)

  def test_controller_factor_bounds(self):
    controller = FlashPilotAngleController()
    controller.set_adjustment_factors(2.0, 0.1, 2.0)
    self.assertEqual(controller.low_speed_factor, 1.5)
    self.assertEqual(controller.high_speed_factor, 0.5)
    self.assertEqual(controller.high_speed_low_curve_factor, 1.25)


class TestFlashPilotAngleTelemetry(unittest.TestCase):
  def test_deviation_limit_allows_only_unwind_toward_zero(self):
    error = CarControllerParams.CURVATURE_ERROR
    # Reducing magnitude may leave the ordinary measured-error envelope.
    self.assertEqual(limit_curvature_for_unwind(-0.0005, -0.006, error), -0.0005)
    self.assertEqual(limit_curvature_for_unwind(0.0005, 0.006, error), 0.0005)
    # An opposite request stops at zero until measured curvature catches up.
    self.assertEqual(limit_curvature_for_unwind(0.002, -0.006, error), 0.0)
    self.assertEqual(limit_curvature_for_unwind(-0.002, 0.006, error), 0.0)
    # Increasing same-direction authority remains strictly clipped.
    self.assertEqual(limit_curvature_for_unwind(-0.01, -0.006, error), -0.008)
    self.assertEqual(limit_curvature_for_unwind(0.01, 0.006, error), 0.008)

  def test_deviation_and_path_angle_intermediates(self):
    controller = FlashPilotAngleController()
    CC = SimpleNamespace(latActive=True)
    CS = _make_cs(v_ego=20.0, yaw_rate=0.0)
    actuators = SimpleNamespace(curvature=0.01)

    result = controller.update(CC, CS, actuators)

    self.assertEqual(result.requested_curvature, 0.01)
    self.assertTrue(result.deviation_limited)
    self.assertAlmostEqual(result.deviation_limited_curvature, CarControllerParams.CURVATURE_ERROR)
    expected_angle = (result.deviation_limited_curvature * 20.0 *
                      path_angle_curvature_factor(20.0, result.deviation_limited_curvature))
    self.assertAlmostEqual(result.calculated_path_angle, expected_angle)
    self.assertTrue(result.rate_limited)
    self.assertFalse(result.pscm_saturation_limited)
    self.assertFalse(result.range_limited)

  def test_range_and_pscm_limit_flags_report_actual_changes(self):
    controller = FlashPilotAngleController()
    CC = SimpleNamespace(latActive=True)
    actuators = SimpleNamespace(curvature=0.1)
    # Matching measured curvature prevents the deviation limiter obscuring the
    # deliberately out-of-range raw path-angle request.
    CS = _make_cs(v_ego=20.0, yaw_rate=-2.0)
    ranged = controller.update(CC, CS, actuators)
    self.assertTrue(ranged.range_limited)
    self.assertFalse(ranged.pscm_saturation_limited)

    controller.path_angle_last = 0.48
    saturated = controller.update(CC, CS, actuators)
    self.assertTrue(saturated.saturated)
    self.assertTrue(saturated.pscm_saturation_limited)
    self.assertFalse(saturated.range_limited)
    self.assertEqual(saturated.path_angle, 0.48)

  def test_schema_round_trip(self):
    output = structs.car.CarOutput.new_message()
    output.fordLateralTelemetry.active = True
    output.fordLateralTelemetry.wireMode = 1
    output.fordLateralTelemetry.requestedCurvature = 0.01
    output.fordLateralTelemetry.deviationLimited = True
    with structs.car.CarOutput.from_bytes(output.to_bytes()) as decoded:
      self.assertTrue(decoded.fordLateralTelemetry.active)
      self.assertEqual(decoded.fordLateralTelemetry.wireMode, 1)
      self.assertAlmostEqual(decoded.fordLateralTelemetry.requestedCurvature, 0.01)
      self.assertTrue(decoded.fordLateralTelemetry.deviationLimited)


def _make_controller(fingerprint):
  CP = structs.CarParams(carFingerprint=fingerprint, flags=int(FordFlags.CANFD), openpilotLongitudinalControl=False)
  dbc_names = {Bus.pt: DBC[fingerprint][Bus.pt]}
  return CarController(dbc_names, CP)


def _make_cs(v_ego=0.0, yaw_rate=0.0, steering_pressed=False, steering_angle_deg=0.0,
             steering_torque=0.0, can_valid=True, steer_fault_temporary=False, steer_fault_permanent=False):
  cs_out = structs.CarState(vEgoRaw=v_ego, yawRate=yaw_rate, steeringPressed=steering_pressed,
                             steeringAngleDeg=steering_angle_deg, steeringTorque=steering_torque,
                             canValid=can_valid, steerFaultTemporary=steer_fault_temporary,
                             steerFaultPermanent=steer_fault_permanent)
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

  def test_telemetry_final_values_match_can_bytes(self):
    """Instrumentation is observational: CAN is still built from the exact same
    result fields, with the established internal-to-wire sign conversion."""
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
      _, can_sends = cc.update(CC, _make_cs(v_ego=20.0), 0)

    actual = next(msg for msg in can_sends if msg[0] == FORD_LateralMotionControl2)
    telemetry = cc.ford_lateral_telemetry
    wire_path_angle = max(-0.5, min(0.5235, -telemetry.path_angle))
    expected = fordcan.create_lat_ctl2_msg(
      cc.packer, cc.CAN, telemetry.mode, -telemetry.path_offset, wire_path_angle, 0.0,
      -telemetry.curvature_rate, 0, ramp_type=telemetry.ramp_type, precision_type=telemetry.precision_type)
    self.assertEqual(actual, expected)

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

  def test_wire_endpoint_clamp_prevents_wrap(self):
    """Both wire endpoints and the demonstrated Route16 overflow pack exactly."""
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    cases = ((0.500626862, 0), (0.5, 0), (-0.5235, 2047), (-0.5240, 2047))
    for internal_angle, expected_raw in cases:
      fp = FlashPilotAngleResult(mode=1, path_angle=internal_angle)
      with patch.object(cc.flashpilot_angle, 'update', return_value=fp):
        # Advance to the next 20 Hz steering frame.
        while cc.frame % CarControllerParams.STEER_STEP:
          with _make_cc_reader(curvature=0.0, lat_active=True) as CC:
            cc.update(CC, _make_cs(v_ego=8.0), 0)
        with _make_cc_reader(curvature=0.0, lat_active=True) as CC:
          _, sends = cc.update(CC, _make_cs(v_ego=8.0), 0)
      mode, _, raw_path_angle = _decode_lmc2(_find_msg(sends, FORD_LateralMotionControl2))
      self.assertEqual(mode, 1)
      self.assertEqual(raw_path_angle, expected_raw)

  def test_manual_turn_yield_and_state_based_reacquisition(self):
    controller = FlashPilotAngleController()
    CC = SimpleNamespace(latActive=True)
    actuators = SimpleNamespace(curvature=0.1)

    # Five 20 Hz samples span 0.20 s. The driver supplies >1.5 Nm and moves the
    # wheel in the torque direction at >60 deg/s through the 35-degree threshold.
    result = None
    for angle in (20.0, 25.0, 30.0, 35.0, 40.0):
      result = controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8, steering_pressed=True,
                                               steering_angle_deg=angle, steering_torque=2.0), actuators)
    self.assertEqual(controller.human_turn_detector.state, ManualTurnState.YIELDED)
    self.assertEqual(result.mode, 0)
    self.assertEqual(result.path_angle, 0.0)
    self.assertTrue(result.human_turn_active)
    self.assertEqual(controller.path_angle_last, 0.0)
    self.assertTrue(CC.latActive, "yield must not clear the logical AOL request")

    # Release alone is insufficient while steering is still moving.
    for angle in (35.0, 30.0, 25.0, 20.0):
      result = controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8, steering_angle_deg=angle), actuators)
      self.assertEqual(result.mode, 0)

    # Once the rolling rate is settled for 0.20 s and curvature is compatible,
    # the first active command resumes from zero through the existing ROC cap.
    result = None
    for _ in range(8):
      result = controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8, steering_angle_deg=20.0), actuators)
      if result.mode == 1:
        break
    self.assertEqual(controller.human_turn_detector.state, ManualTurnState.REACQUIRE)
    self.assertEqual(result.mode, 1)
    self.assertTrue(result.human_turn_active)
    self.assertLessEqual(abs(result.path_angle), 0.055)

    result = controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8, steering_angle_deg=20.0), actuators)
    self.assertEqual(controller.human_turn_detector.state, ManualTurnState.TRACKING)
    self.assertEqual(result.mode, 1)

  def test_manual_turn_rule_rejects_clean_curve_and_lane_change_profiles(self):
    CC = SimpleNamespace(latActive=True)
    actuators = SimpleNamespace(curvature=0.002)

    # Clean high-speed curve: modest steering motion despite driver contact.
    curve = FlashPilotAngleController()
    for angle in (40.0, 40.3, 40.6, 40.9, 41.2, 41.5, 41.8, 42.1):
      result = curve.update(CC, _make_cs(v_ego=34.0, yaw_rate=-0.068, steering_pressed=True,
                                         steering_angle_deg=angle, steering_torque=2.0), actuators)
      self.assertEqual(result.mode, 1)
    self.assertEqual(curve.human_turn_detector.state, ManualTurnState.TRACKING)

    # Lane-change steering can be fast, but lacks sustained driver pressure.
    lane_change = FlashPilotAngleController()
    for angle in (0.0, 5.0, 12.0, 20.0, 12.0, 5.0, 0.0):
      result = lane_change.update(CC, _make_cs(v_ego=30.0, yaw_rate=0.0, steering_angle_deg=angle), actuators)
      self.assertEqual(result.mode, 1)
    self.assertEqual(lane_change.human_turn_detector.state, ManualTurnState.TRACKING)

  def test_faults_block_reacquisition_and_lat_inactive_stays_inactive(self):
    controller = FlashPilotAngleController()
    CC = SimpleNamespace(latActive=True)
    actuators = SimpleNamespace(curvature=0.1)
    for angle in (20.0, 25.0, 30.0, 35.0, 40.0):
      controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8, steering_pressed=True,
                                     steering_angle_deg=angle, steering_torque=2.0), actuators)
    for _ in range(12):
      result = controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8, steering_angle_deg=40.0,
                                               steer_fault_temporary=True), actuators)
    self.assertEqual(controller.human_turn_detector.state, ManualTurnState.YIELDED)
    self.assertEqual(result.mode, 0)

    CC.latActive = False
    result = controller.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.8), actuators)
    self.assertEqual(result.mode, 0)
    self.assertEqual(controller.human_turn_detector.state, ManualTurnState.TRACKING)

  def test_lightning_enabled_human_turn_sends_inactive_lmc2(self):
    os.environ['FLASHPILOT_ANGLE_ENABLED'] = '1'
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1)
    all_sends = []
    # Controller samples at 20 Hz (one of every five update calls).
    for i in range(30):
      sample = i // CarControllerParams.STEER_STEP
      angle = 20.0 + 5.0 * sample
      with _make_cc_reader(curvature=0.01, lat_active=True) as CC:
        _, sends = cc.update(CC, _make_cs(v_ego=8.0, yaw_rate=-0.08, steering_pressed=True,
                                          steering_angle_deg=angle, steering_torque=2.0), 0)
        all_sends += sends

    lmc2 = _find_msg(all_sends, FORD_LateralMotionControl2)
    mode, raw_curvature, raw_path_angle = _decode_lmc2(lmc2)
    self.assertEqual(mode, 0, "human turn override must force mode 0")
    self.assertEqual(raw_curvature, 1000, "curvature must use its inactive sentinel while yielded")
    self.assertEqual(raw_path_angle, 1000, "path_angle must be the inactive sentinel while overridden")


if __name__ == "__main__":
  unittest.main()
