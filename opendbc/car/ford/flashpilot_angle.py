"""
FlashPilot: minimal BluePilot-derived Ford path-angle-primary lateral control.

Scoped to the F-150 Lightning (CAR.FORD_F_150_LIGHTNING_MK1) only, by construction:
this module is only ever instantiated/called from carcontroller.py's Lightning-gated
branch. Every other Ford platform's behavior is byte-for-byte unchanged.

Ported from and cross-checked against BluePilotDev/bluepilot@501a7c0 (branch bp-dev):
opendbc/sunnypilot/car/ford/lateral_angle_ext.py and human_turn.py. See
docs/flashpilot/FLASHPILOT_PATH_ANGLE_PHASE_A.md for the full dependency-chain audit
this file implements, including what was deliberately left out and why:
  - No model-predicted-curvature blending or Variable Lookup Time -- actuators.curvature
    is used directly. This is a smoothness refinement, not required to steer correctly.
  - No lane-change-aware precision/gain scaling (follows from the above -- no modelV2
    subscription is added). precision_type is always 1 (Precise), matching upstream's
    own existing hardcoded default.
  - No lane centering trim, no pinion-angle curvature measurement, no post-override
    "stall blip" recovery pulse (BluePilot's own
    thresholds for that are tuned against Mach-E road-test data we have no reason to
    believe transfers to the Lightning's PSCM; adding it speculatively would be
    exactly the kind of unverified change this project's guardrails prohibit).

All values returned by update() are in openpilot's internal sign convention, i.e. the
SAME convention as CarControl.Actuators.curvature -- NOT the DBC wire convention.
Callers must negate path_angle/shadow_curvature before packing onto CAN, exactly as
upstream carcontroller.py already does for plain curvature (`-self.apply_curvature_last`).
BluePilot's own carcontroller.py does the same for angle mode (`-lat.path_angle`,
`-self.bp_kappa_cmd`) with a code comment citing a real hardware bug when this was
missed (the deviation check found a "divergence" every frame once un-negated) --
this file's docstring exists specifically so that mistake doesn't get made twice.
"""
import math
from collections import deque
from enum import IntEnum

from numpy import clip, interp, median

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams

# DBC LatCtlPath_An_Actl wire range (rad). The internal controller convention is
# negated at the CAN boundary, so its representable range is reflected.
FORD_WIRE_PATH_ANGLE_MIN = -0.5
FORD_WIRE_PATH_ANGLE_MAX = 0.5235
FORD_INTERNAL_PATH_ANGLE_MIN = -FORD_WIRE_PATH_ANGLE_MAX
FORD_INTERNAL_PATH_ANGLE_MAX = -FORD_WIRE_PATH_ANGLE_MIN

# PSCM internal lookahead distance (m) vs speed (m/s). Empirical BluePilot constant
# (lateral_angle_ext.py's pscm_d_ref_m) -- treat as a starting point needing
# re-validation against the Lightning's actual PSCM, not a given.
_PSCM_DREF_SPEEDS_MS = (0.0, 4.17, 27.78, 41.67, 50.0, 55.56)
_PSCM_DREF_M = (0.5, 0.95, 1.4, 2.075, 2.75, 3.875)

# Speed-factor calibration: BluePilot's "CAN-FD body-on-frame truck" preset, which
# already groups the Lightning with F-150 MK14/Expedition/Ranger. Since this module
# is Lightning-only, the multi-platform dispatch table in lateral_angle_ext.py
# collapses to this one constant pair. Unvalidated for the Lightning specifically.
_GAIN_LOW_HIGH_SPEED = 0.95
_GAIN_HIGH_HIGH_SPEED = 0.95

# Known-good BluePilot Angle tuning from the physical Lightning. These are the
# three user adjustment factors applied on top of the platform base gains. These
# remain the controller defaults; openpilot may override them once at startup.
FLASHPILOT_LOW_SPEED_ADJUSTMENT_FACTOR = 0.98
FLASHPILOT_HIGH_SPEED_ADJUSTMENT_FACTOR = 0.90
FLASHPILOT_HIGH_SPEED_LOW_CURVE_ADJUSTMENT_FACTOR = 0.83

_GAIN_SPEED_BP_MS = (13.5, 26.82)
_GAIN_CURVATURE_BP = (0.0007, 0.001)

_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick

# Validated AOL manual-turn yield/reacquisition thresholds.
HUMAN_TURN_ENTRY_WINDOW_S = 0.20
HUMAN_TURN_TORQUE_NM = 1.5
HUMAN_TURN_RATE_DEG_S = 60.0
HUMAN_TURN_ANGLE_DEG = 35.0
HUMAN_TURN_RELEASE_TORQUE_NM = 1.0
HUMAN_TURN_SETTLE_RATE_DEG_S = 10.0
HUMAN_TURN_SETTLE_WINDOW_S = 0.20

# PSCM saturation: rate cap on path_angle magnitude decrease while pinned near the
# DBC range edge (rad/call = 0.02 * 20 Hz = 0.40 rad/s), from lateral_angle_ext.py's
# _PSCM_SAT_UNWIND_RATE.
_PSCM_SAT_UNWIND_RATE = 0.02
_DBC_SAT_FRACTION = 0.90

# Python-side soft rate-of-change limit -- a backstop, deliberately tighter than
# panda's C-side FORD_PATH_ANGLE_LIMITS (see opendbc/safety/modes/ford.h), so this
# is always the binding constraint in normal operation. From lateral_angle_ext.py's
# _soft_roc table.
_SOFT_ROC_BP = (9., 10., 15., 25.)
_SOFT_ROC_V = (0.055, 0.055, 0.0425, 0.009)


def limit_curvature_for_unwind(requested: float, measured: float, max_error: float) -> float:
  """Keep added authority inside the measured-error envelope, while permitting
  a command to shed existing curvature toward (but never through) zero.

  Opposite-sign requests stop at zero until measured curvature enters the normal
  error envelope. The separate path-angle ROC remains the per-frame actuator
  bound, so this only removes the stale same-direction hold seen on curve exit.
  """
  limited = float(clip(requested, measured - max_error, measured + max_error))
  if measured > max_error and requested < measured - max_error:
    return float(max(0.0, requested))
  if measured < -max_error and requested > measured + max_error:
    return float(min(0.0, requested))
  return limited


def pscm_d_ref_m(v_ego_ms: float) -> float:
  v = max(float(v_ego_ms), 0.0)
  d = float(interp(v, _PSCM_DREF_SPEEDS_MS, _PSCM_DREF_M))
  if v > _PSCM_DREF_SPEEDS_MS[-1]:
    d = min(5.0, d)
  return d


def path_angle_curvature_factor(v_ego_ms: float, curvature: float,
                                low_speed_factor: float = FLASHPILOT_LOW_SPEED_ADJUSTMENT_FACTOR,
                                high_speed_factor: float = FLASHPILOT_HIGH_SPEED_ADJUSTMENT_FACTOR,
                                high_speed_low_curve_factor: float = FLASHPILOT_HIGH_SPEED_LOW_CURVE_ADJUSTMENT_FACTOR) -> float:
  """BluePilot-equivalent Lightning gain interpolation with canonical defaults."""
  low_gain = float(interp(v_ego_ms, _GAIN_SPEED_BP_MS,
                          (1.0, _GAIN_LOW_HIGH_SPEED * high_speed_low_curve_factor)))
  high_gain = float(interp(v_ego_ms, _GAIN_SPEED_BP_MS,
                           (1.30 * low_speed_factor, _GAIN_HIGH_HIGH_SPEED * high_speed_factor)))
  return float(interp(abs(curvature), _GAIN_CURVATURE_BP, (low_gain, high_gain)))


class ManualTurnState(IntEnum):
  TRACKING = 0
  YIELDED = 1
  REACQUIRE = 2


def curvature_compatible_or_unwinding(requested: float, measured: float, max_error: float) -> bool:
  if abs(requested - measured) <= max_error:
    return True
  if measured > max_error:
    return 0.0 <= requested < measured
  if measured < -max_error:
    return measured < requested <= 0.0
  return False


class HumanTurnDetector:
  """Validated TRACKING -> YIELDED -> REACQUIRE AOL manual-turn state."""

  def __init__(self):
    self.state = ManualTurnState.TRACKING
    self._samples = deque(maxlen=round(HUMAN_TURN_ENTRY_WINDOW_S / _STEER_DT) + 1)
    self._settled_s = 0.0

  def _rolling_rate(self) -> float:
    if len(self._samples) < 2:
      return 0.0
    return (self._samples[-1][0] - self._samples[0][0]) / ((len(self._samples) - 1) * _STEER_DT)

  def update(self, steering_pressed: bool, steering_angle_deg: float, driver_torque_nm: float,
             inputs_healthy: bool, curvature_compatible: bool) -> ManualTurnState:
    self._samples.append((float(steering_angle_deg), float(driver_torque_nm), bool(steering_pressed)))
    window_ready = (len(self._samples) == self._samples.maxlen and
                    (len(self._samples) - 1) * _STEER_DT >= HUMAN_TURN_ENTRY_WINDOW_S)
    rolling_rate = self._rolling_rate()

    if self.state == ManualTurnState.TRACKING:
      if window_ready:
        torque_median = float(median([abs(x[1]) for x in self._samples]))
        signed_torque_median = float(median([x[1] for x in self._samples]))
        motion_agrees = (self._samples[-1][0] - self._samples[0][0]) * signed_torque_median > 0.0
        enter = (all(x[2] for x in self._samples) and torque_median >= HUMAN_TURN_TORQUE_NM and
                 motion_agrees and abs(rolling_rate) >= HUMAN_TURN_RATE_DEG_S and
                 abs(steering_angle_deg) >= HUMAN_TURN_ANGLE_DEG)
        if enter:
          self.state = ManualTurnState.YIELDED
          self._settled_s = 0.0

    elif self.state == ManualTurnState.YIELDED:
      released_and_settled = (not steering_pressed and abs(driver_torque_nm) < HUMAN_TURN_RELEASE_TORQUE_NM and
                              abs(rolling_rate) < HUMAN_TURN_SETTLE_RATE_DEG_S)
      self._settled_s = self._settled_s + _STEER_DT if released_and_settled else 0.0
      if (self._settled_s >= HUMAN_TURN_SETTLE_WINDOW_S and inputs_healthy and curvature_compatible):
        self.state = ManualTurnState.REACQUIRE

    elif steering_pressed:
      # A renewed grab during the one-frame hand-back yields immediately.
      self.state = ManualTurnState.YIELDED
      self._settled_s = 0.0
    else:
      self.state = ManualTurnState.TRACKING

    return self.state

  def reset(self) -> None:
    self.state = ManualTurnState.TRACKING
    self._samples.clear()
    self._settled_s = 0.0


class FlashPilotAngleResult:
  """All angle/curvature fields are in openpilot's internal sign convention -- see
  module docstring. `mode` is the raw LatCtl_D2_Rq value (0=inactive, 1=active)."""
  __slots__ = ("mode", "path_angle", "path_offset", "curvature_rate", "ramp_type",
               "precision_type", "shadow_curvature", "human_turn_active", "saturated",
               "rate_limited", "requested_curvature", "deviation_limited_curvature",
               "calculated_path_angle", "deviation_limited", "pscm_saturation_limited",
               "range_limited")

  def __init__(self, mode: int = 0, path_angle: float = 0.0, path_offset: float = 0.0,
               curvature_rate: float = 0.0, ramp_type: int = 0, precision_type: int = 1,
               shadow_curvature: float = 0.0, human_turn_active: bool = False,
               saturated: bool = False, rate_limited: bool = False,
               requested_curvature: float = 0.0, deviation_limited_curvature: float = 0.0,
               calculated_path_angle: float = 0.0, deviation_limited: bool = False,
               pscm_saturation_limited: bool = False, range_limited: bool = False):
    self.mode = mode
    self.path_angle = path_angle
    self.path_offset = path_offset
    self.curvature_rate = curvature_rate
    self.ramp_type = ramp_type
    self.precision_type = precision_type
    self.shadow_curvature = shadow_curvature
    self.human_turn_active = human_turn_active
    self.saturated = saturated
    self.rate_limited = rate_limited
    self.requested_curvature = requested_curvature
    self.deviation_limited_curvature = deviation_limited_curvature
    self.calculated_path_angle = calculated_path_angle
    self.deviation_limited = deviation_limited
    self.pscm_saturation_limited = pscm_saturation_limited
    self.range_limited = range_limited


class FlashPilotAngleController:
  def __init__(self):
    self.path_angle_last = 0.0
    self.human_turn_detector = HumanTurnDetector()
    self.low_speed_factor = FLASHPILOT_LOW_SPEED_ADJUSTMENT_FACTOR
    self.high_speed_factor = FLASHPILOT_HIGH_SPEED_ADJUSTMENT_FACTOR
    self.high_speed_low_curve_factor = FLASHPILOT_HIGH_SPEED_LOW_CURVE_ADJUSTMENT_FACTOR

  def set_adjustment_factors(self, low_speed: float, high_speed: float, high_speed_low_curve: float) -> None:
    # Same bounds as BluePilot's existing angle controls. This only replaces
    # constants with startup-loaded values; interpolation and limits are unchanged.
    self.low_speed_factor = float(clip(low_speed, 0.5, 1.5))
    self.high_speed_factor = float(clip(high_speed, 0.5, 1.5))
    self.high_speed_low_curve_factor = float(clip(high_speed_low_curve, 0.25, 1.25))

  def reset(self) -> None:
    self.path_angle_last = 0.0
    self.human_turn_detector.reset()

  @staticmethod
  def _current_curvature(CS) -> float:
    # Matches upstream carcontroller.py's own curvature-mode computation exactly
    # (same sign convention as actuators.curvature / apply_curvature_last).
    return -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

  def update(self, CC, CS, actuators) -> FlashPilotAngleResult:
    v_ego = float(CS.out.vEgoRaw)

    if not CC.latActive:
      self.reset()
      return FlashPilotAngleResult(mode=0, shadow_curvature=self._current_curvature(CS))

    requested_curvature = float(actuators.curvature)
    current_curvature = self._current_curvature(CS)
    inputs_healthy = (bool(CS.out.canValid) and not CS.out.canTimeout and not CS.out.vehicleSensorsInvalid and
                      not CS.out.steerFaultTemporary and not CS.out.steerFaultPermanent and
                      math.isfinite(requested_curvature) and math.isfinite(current_curvature) and
                      math.isfinite(float(CS.out.steeringAngleDeg)))
    curvature_compatible = curvature_compatible_or_unwinding(
      requested_curvature, current_curvature, CarControllerParams.CURVATURE_ERROR)
    manual_turn_state = self.human_turn_detector.update(
      CS.out.steeringPressed, CS.out.steeringAngleDeg, CS.out.steeringTorque,
      inputs_healthy, curvature_compatible)
    if manual_turn_state == ManualTurnState.YIELDED:
      # Force lateral fully inactive (mode 0) rather than letting path_angle wind up
      # to a stale command the PSCM has to reconcile on release. Panda-clean by
      # construction: every ford.h check has a legitimate !steer_control_enabled
      # branch, so no bypass of any kind is needed here.
      self.path_angle_last = 0.0
      return FlashPilotAngleResult(mode=0, human_turn_active=True,
                                    shadow_curvature=self._current_curvature(CS))

    kappa_cmd = requested_curvature

    # Added same-direction authority remains inside the normal measured-curvature
    # envelope. Reducing existing curvature may move toward zero faster, but an
    # opposite-sign request stops at zero until it fits the normal envelope. The
    # matched Panda rule enforces the same asymmetric, unwind-only allowance.
    if v_ego > 9:
      kappa_cmd = limit_curvature_for_unwind(kappa_cmd, current_curvature, CarControllerParams.CURVATURE_ERROR)
    deviation_limited = abs(kappa_cmd - requested_curvature) > 1e-12

    curvature_factor = path_angle_curvature_factor(v_ego, kappa_cmd, self.low_speed_factor,
                                                   self.high_speed_factor, self.high_speed_low_curve_factor)

    d_ref = pscm_d_ref_m(v_ego)  # noqa: F841 -- geometry documented for reference; the
    # gain-table form above (ported from lateral_angle_ext.py) is what's actually applied.
    path_angle = kappa_cmd * v_ego * curvature_factor
    calculated_path_angle = path_angle

    # PSCM saturation clamp: a function of our own tracked path_angle_last only --
    # no PSCM signal is read (LatCtlLim_D_Stat does not fire in angle mode; see
    # docs/flashpilot/FLASHPILOT_PATH_ANGLE_PHASE_A.md item 5).
    dbc_sat = (self.path_angle_last >= FORD_INTERNAL_PATH_ANGLE_MAX * _DBC_SAT_FRACTION or
               self.path_angle_last <= FORD_INTERNAL_PATH_ANGLE_MIN * _DBC_SAT_FRACTION)
    path_angle_pre_pscm = path_angle
    if dbc_sat:
      last_mag = abs(self.path_angle_last)
      curr_mag = abs(path_angle)
      if curr_mag > last_mag:
        path_angle = self.path_angle_last
      elif last_mag - curr_mag > _PSCM_SAT_UNWIND_RATE:
        limited_mag = last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = limited_mag if self.path_angle_last >= 0 else -limited_mag
    pscm_saturation_limited = abs(path_angle - path_angle_pre_pscm) > 1e-12

    path_angle_pre_range = path_angle
    path_angle = float(clip(path_angle, FORD_INTERNAL_PATH_ANGLE_MIN, FORD_INTERNAL_PATH_ANGLE_MAX))
    range_limited = abs(path_angle - path_angle_pre_range) > 1e-12

    soft_roc = float(interp(v_ego, _SOFT_ROC_BP, _SOFT_ROC_V))
    path_angle_pre_roc = path_angle
    path_angle = float(clip(path_angle, self.path_angle_last - soft_roc, self.path_angle_last + soft_roc))
    rate_limited = abs(path_angle - path_angle_pre_roc) > 1e-9

    self.path_angle_last = path_angle

    # Shadow curvature for the panda-side deviation check: while the driver is
    # actively pressing, the honest command is the car's real measured curvature
    # (mirrors lateral_angle_ext.py -- avoids the clipped planner value lagging a
    # driver fighting a sustained curve, the one in-drive lateral safety block
    # BluePilot observed in road-test replay).
    shadow_curvature = current_curvature if CS.out.steeringPressed else kappa_cmd

    return FlashPilotAngleResult(
      mode=1, path_angle=path_angle, path_offset=0.0, curvature_rate=0.0,
      ramp_type=2, precision_type=1, shadow_curvature=shadow_curvature,
      human_turn_active=manual_turn_state == ManualTurnState.REACQUIRE,
      saturated=dbc_sat, rate_limited=rate_limited,
      requested_curvature=requested_curvature, deviation_limited_curvature=kappa_cmd,
      calculated_path_angle=calculated_path_angle, deviation_limited=deviation_limited,
      pscm_saturation_limited=pscm_saturation_limited, range_limited=range_limited,
    )
