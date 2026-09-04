import math
import os
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan, flashpilot_angle
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.hands_free_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

    # FlashPilot: BluePilot-derived path-angle-primary lateral control, gated to the
    # F-150 Lightning only (every other Ford's behavior below is byte-for-byte
    # unchanged) and developer-only/default-off via an env var, not a user-facing
    # toggle -- see docs/flashpilot/FLASHPILOT_PATH_ANGLE_PHASE_A.md and
    # FLASHPILOT_ARCHITECTURE.md for why. Mirrors the FINGERPRINT/SKIP_FW_QUERY/
    # DISABLE_FW_CACHE env-var pattern opendbc.car.car_helpers already uses for
    # exactly this kind of developer override.
    self._flashpilot_angle_enabled = (CP.carFingerprint == CAR.FORD_F_150_LIGHTNING_MK1 and
                                       os.environ.get('FLASHPILOT_ANGLE_ENABLED', '0') == '1')
    self.flashpilot_angle = flashpilot_angle.FlashPilotAngleController() if self._flashpilot_angle_enabled else None
    # Static for the life of this CarController: True iff this car is running
    # FlashPilot angle mode at all, independent of whether it's actively steering
    # this frame -- ford.h needs to know the *mode*, not the instantaneous state
    # (mirrors BluePilot's own angle_mode_engaged semantics).
    self._fp_angle_mode_engaged = self.flashpilot_angle is not None
    self._fp_shadow_curvature = 0.0

  @property
  def _ford_hands_free_cluster(self):
    # card.py applies this startup Param to the shared CP after get_car() has
    # constructed this controller. Read the finalized flag instead of caching
    # its pre-initialization value. This is a display-only Lightning option.
    return (self.CP.carFingerprint == CAR.FORD_F_150_LIGHTNING_MK1 and
            bool(self.CP.flags & FordFlags.HANDS_FREE_CLUSTER))

  def update(self, CC, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if self.flashpilot_angle is not None:
        # FlashPilot: BluePilot-derived path-angle-primary control (Lightning only,
        # developer-gated -- see __init__). Curvature/curvature-rate/path-offset stay
        # pinned at 0 on the wire, exactly as they already are for every other Ford;
        # path_angle is the only new nonzero signal. All fields from
        # flashpilot_angle.update() are in openpilot's internal sign convention and
        # MUST be negated before packing -- see flashpilot_angle.py's module
        # docstring for the real hardware bug this guards against.
        fp = self.flashpilot_angle.update(CC, CS, actuators)
        self.apply_curvature_last = 0.0
        self._fp_shadow_curvature = fp.shadow_curvature
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(
          self.packer, self.CAN, fp.mode, -fp.path_offset, -fp.path_angle, -self.apply_curvature_last,
          -fp.curvature_rate, counter, ramp_type=fp.ramp_type, precision_type=fp.precision_type))
      else:
        # Bronco and some other cars consistently overshoot curv requests
        # Apply some deadzone + smoothing convergence to avoid oscillations
        if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
          self.anti_overshoot_curvature_last = anti_overshoot(actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
          apply_curvature = self.anti_overshoot_curvature_last
        else:
          apply_curvature = actuators.curvature

        # apply rate limits, curvature error limit, and clip to signal range
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
        if CS.out.vEgoRaw > 9:
          apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                          current_curvature + CarControllerParams.CURVATURE_ERROR))
        apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                            0., CC.latActive, CarControllerParams.STEER_STEP)
        self.apply_curvature_last = apply_curvature

        if self.CP.flags & FordFlags.CANFD:
          # TODO: extended mode
          # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02 m^-1)
          # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
          # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
          # A detailed explanation on ford control can be found here:
          # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
          mode = 1 if CC.latActive else 0
          counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))
        else:
          can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      # angle_mode_engaged/shadow_curvature default to False/0.0 for every non-
      # FlashPilot-angle car (self._fp_angle_mode_engaged is only ever True for a
      # Lightning with the feature enabled), producing byte-identical output to
      # upstream's create_lka_msg(self.packer, self.CAN) for everyone else.
      can_sends.append(fordcan.create_lka_msg(
        self.packer, self.CAN, angle_mode_engaged=self._fp_angle_mode_engaged,
        shadow_curvature=-self._fp_shadow_curvature if self._fp_angle_mode_engaged else 0.0))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        accel = apply_creep_compensation(accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      # TODO: look into using the actuators packet to send the desired speed
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas

    ### ui ###
    hands_free = self._ford_hands_free_cluster and CC.latActive and CC.longActive
    send_ui = ((self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or
               (self.steer_alert_last != steer_alert) or (self.hands_free_last != hands_free))
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values,
                                                  hands_free_cluster=hands_free))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      show_distance_bars = self.frame - self.distance_bar_frame < 400
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                 hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.hands_free_last = hands_free
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
