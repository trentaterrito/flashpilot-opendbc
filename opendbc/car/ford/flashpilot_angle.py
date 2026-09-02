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
  - No lane centering trim, no pinion-angle curvature measurement, no Params-backed
    user tuning, no post-override "stall blip" recovery pulse (BluePilot's own
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
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams

# DBC LatCtlPath_An_Actl range (rad) -- see opendbc/car/ford/fordcan.py's
# create_lat_ctl2_msg docstring: "Path angle [-0.5|0.5235] radians"
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235

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
# three user adjustment factors applied on top of the platform base gains; they
# are constants here until FlashPilot grows a dedicated Params/UI surface.
FLASHPILOT_LOW_SPEED_ADJUSTMENT_FACTOR = 0.98
FLASHPILOT_HIGH_SPEED_ADJUSTMENT_FACTOR = 0.90
FLASHPILOT_HIGH_SPEED_LOW_CURVE_ADJUSTMENT_FACTOR = 0.83

_GAIN_SPEED_BP_MS = (13.5, 26.82)
_GAIN_CURVATURE_BP = (0.0007, 0.001)

_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick

# Human-turn override thresholds, from opendbc/sunnypilot/car/ford/human_turn.py
HUMAN_TURN_ANGLE_DEG = 45.0
HUMAN_TURN_HOLD_S = 1.5
HUMAN_TURN_HOLD_PRETURNED_S = 3.0

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


def pscm_d_ref_m(v_ego_ms: float) -> float:
  v = max(float(v_ego_ms), 0.0)
  d = float(interp(v, _PSCM_DREF_SPEEDS_MS, _PSCM_DREF_M))
  if v > _PSCM_DREF_SPEEDS_MS[-1]:
    d = min(5.0, d)
  return d


def path_angle_curvature_factor(v_ego_ms: float, curvature: float) -> float:
  """BluePilot-equivalent Lightning gain interpolation without its Params/UI layer."""
  low_gain = float(interp(v_ego_ms, _GAIN_SPEED_BP_MS,
                          (1.0, _GAIN_LOW_HIGH_SPEED * FLASHPILOT_HIGH_SPEED_LOW_CURVE_ADJUSTMENT_FACTOR)))
  high_gain = float(interp(v_ego_ms, _GAIN_SPEED_BP_MS,
                           (1.30 * FLASHPILOT_LOW_SPEED_ADJUSTMENT_FACTOR,
                            _GAIN_HIGH_HIGH_SPEED * FLASHPILOT_HIGH_SPEED_ADJUSTMENT_FACTOR)))
  return float(interp(abs(curvature), _GAIN_CURVATURE_BP, (low_gain, high_gain)))


class HumanTurnDetector:
  """
  Shared manual-steering-override detection, ported near-verbatim from
  opendbc/sunnypilot/car/ford/human_turn.py. Latches `active` once the driver holds
  real steering pressure AND |wheel angle| > HUMAN_TURN_ANGLE_DEG continuously for
  HUMAN_TURN_HOLD_S (HUMAN_TURN_HOLD_PRETURNED_S if the wheel was already past the
  threshold when contact began -- distinguishes an intentional takeover from a
  mid-curve nudge).
  """

  def __init__(self):
    self.hold_timer_s = 0.0
    self.active = False
    self._pressed_last = False
    self._press_started_preturned = False

  def update(self, enabled: bool, steering_pressed: bool, steering_angle_deg: float) -> bool:
    if steering_pressed and not self._pressed_last:
      self._press_started_preturned = abs(steering_angle_deg) > HUMAN_TURN_ANGLE_DEG
    self._pressed_last = steering_pressed

    if not enabled:
      self.hold_timer_s = 0.0
    elif steering_pressed and abs(steering_angle_deg) > HUMAN_TURN_ANGLE_DEG:
      self.hold_timer_s += _STEER_DT
    else:
      self.hold_timer_s = 0.0

    hold_req = HUMAN_TURN_HOLD_PRETURNED_S if self._press_started_preturned else HUMAN_TURN_HOLD_S
    self.active = self.hold_timer_s >= hold_req
    return self.active

  def reset(self) -> None:
    self.hold_timer_s = 0.0
    self.active = False
    self._pressed_last = False
    self._press_started_preturned = False


class FlashPilotAngleResult:
  """All angle/curvature fields are in openpilot's internal sign convention -- see
  module docstring. `mode` is the raw LatCtl_D2_Rq value (0=inactive, 1=active)."""
  __slots__ = ("mode", "path_angle", "path_offset", "curvature_rate", "ramp_type",
               "precision_type", "shadow_curvature", "human_turn_active", "saturated",
               "rate_limited")

  def __init__(self, mode: int = 0, path_angle: float = 0.0, path_offset: float = 0.0,
               curvature_rate: float = 0.0, ramp_type: int = 0, precision_type: int = 1,
               shadow_curvature: float = 0.0, human_turn_active: bool = False,
               saturated: bool = False, rate_limited: bool = False):
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


class FlashPilotAngleController:
  def __init__(self):
    self.path_angle_last = 0.0
    self.human_turn_detector = HumanTurnDetector()

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

    human_turn_active = self.human_turn_detector.update(True, CS.out.steeringPressed, CS.out.steeringAngleDeg)
    if human_turn_active:
      # Force lateral fully inactive (mode 0) rather than letting path_angle wind up
      # to a stale command the PSCM has to reconcile on release. Panda-clean by
      # construction: every ford.h check has a legitimate !steer_control_enabled
      # branch, so no bypass of any kind is needed here.
      self.path_angle_last = 0.0
      return FlashPilotAngleResult(mode=0, human_turn_active=True,
                                    shadow_curvature=self._current_curvature(CS))

    kappa_cmd = float(actuators.curvature)

    # Deviation clip: kappa_cmd may not lead the measured curvature by more than
    # CarControllerParams.CURVATURE_ERROR, mirroring curvature-primary mode's own
    # clip so the shadow-curvature panda check never rejects a legitimate command.
    current_curvature = self._current_curvature(CS)
    if v_ego > 9:
      kappa_cmd = float(clip(kappa_cmd, current_curvature - CarControllerParams.CURVATURE_ERROR,
                             current_curvature + CarControllerParams.CURVATURE_ERROR))

    curvature_factor = path_angle_curvature_factor(v_ego, kappa_cmd)

    d_ref = pscm_d_ref_m(v_ego)  # noqa: F841 -- geometry documented for reference; the
    # gain-table form above (ported from lateral_angle_ext.py) is what's actually applied.
    path_angle = kappa_cmd * v_ego * curvature_factor

    # PSCM saturation clamp: a function of our own tracked path_angle_last only --
    # no PSCM signal is read (LatCtlLim_D_Stat does not fire in angle mode; see
    # docs/flashpilot/FLASHPILOT_PATH_ANGLE_PHASE_A.md item 5).
    dbc_sat = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * _DBC_SAT_FRACTION or
               self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * _DBC_SAT_FRACTION)
    if dbc_sat:
      last_mag = abs(self.path_angle_last)
      curr_mag = abs(path_angle)
      if curr_mag > last_mag:
        path_angle = self.path_angle_last
      elif last_mag - curr_mag > _PSCM_SAT_UNWIND_RATE:
        limited_mag = last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = limited_mag if self.path_angle_last >= 0 else -limited_mag

    path_angle = float(clip(path_angle, FORD_DBC_PATH_ANGLE_MIN, FORD_DBC_PATH_ANGLE_MAX))

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
      human_turn_active=False, saturated=dbc_sat, rate_limited=rate_limited,
    )
