"""Actual Ford dispatcher/TX integration around the unmodified sunnypilot core."""
import pytest

REQUIRED_RX = [0x415, 0x202, 0x91, 0x165, 0x204, 0x213, 0x83, 0x3CC]

@pytest.mark.parametrize("addr", REQUIRED_RX)
def test_each_missing_required_message_fails_closed(addr):
  h = Harness(omit_address=addr)
  h.button(True)
  assert not h.allowed()


@pytest.mark.parametrize("addr", REQUIRED_RX)
def test_each_required_message_stales_despite_other_fresh_inputs(addr):
  h = Harness()
  h.engage()
  h.omit_address = addr
  for _ in range(120):
    h.now += 10000
    h.safety.set_timer(h.now)
    h.refresh()
    if h.now % 1000000 == 1000:
      h.safety.safety_tick_current_safety_config()
  h.safety.safety_tick_current_safety_config()
  assert not h.allowed()
  h.omit_address = None
  for _ in range(3):
    h.refresh()
  h.safety.safety_tick_current_safety_config()
  assert not h.allowed()
  h.refresh()  # fresh host eligibility after native lagging state clears
  h.engage()


@pytest.mark.parametrize("source", ["speed", "yaw"])
@pytest.mark.parametrize("delta", [-1, -2, 2])
def test_counter_skip_matches_native_tolerance_then_invalidity_revokes(h, source, delta):
  h.engage()
  if source == "speed":
    h.ford.cnt_speed += delta
    bad = h.ford._speed_msg(15.)
  else:
    h.ford.cnt_yaw_rate += delta
    bad = h.ford._yaw_rate_msg(0., 15.)
  assert h.safety.safety_rx_hook(bad)
  assert h.allowed()  # native Ford tolerance, not a new first-error veto
  for _ in range(5):
    if source == "speed":
      h.ford.cnt_speed -= 1
      bad = h.ford._speed_msg(15.)
    else:
      h.ford.cnt_yaw_rate -= 1
      bad = h.ford._yaw_rate_msg(0., 15.)
    h.safety.safety_rx_hook(bad)
  assert not h.allowed()
  for _ in range(3):
    h.refresh()
  assert not h.allowed()
  h.engage()


@pytest.mark.parametrize("extra", ["Lane_Assist_Data3_FD1"])
def test_missing_extra_message_never_grants(extra):
  h = Harness()
  h.safety.test_sp_configure(True)
  original_rx = h.rx
  h.rx = lambda name, **values: True if name == extra else original_rx(name, **values)
  h.refresh()
  h.button(True)
  assert not h.allowed()


@pytest.mark.parametrize("timeout", [100001, 200000, 1000000])
def test_heartbeat_cannot_resurrect_expired_status(h, timeout):
  h.engage()
  h.now += timeout
  h.safety.set_timer(h.now)
  h.safety.test_sp_heartbeat(0, 1, 0)
  assert not h.allowed()
  h.refresh()
  h.button(True)
  assert h.allowed()  # refresh observed release; physical edge is still required


def test_duplicate_current_heartbeat_is_not_new_intent(h):
  for _ in range(10):
    h.safety.test_sp_heartbeat(0, 1, 0)
  assert not h.allowed()


from opendbc.car.structs import CarParams
from opendbc.safety.tests import test_flashpilot_ford_safety as angle
from opendbc.safety.tests.libsafety import libsafety_py


class Harness:
  def __init__(self, lightning=True, omit_address=None):
    self.ford = angle.TestFlashPilotFordPathAngleSafety()
    self.ford.setUp()
    self.safety = self.ford.safety
    self.omit_address = omit_address
    self.lateral_counter = 0
    original_rx = self.ford._rx
    self.ford._rx = lambda msg: True if msg.addr == self.omit_address else original_rx(msg)
    self.now = 1000
    self.safety.set_timer(self.now)
    self.safety.test_sp_configure(lightning)
    self.refresh()

  def rx(self, name, **values):
    if name == "Lane_Assist_Data3_FD1":
      # Synthetic moving counter for general lifecycle tests; production does
      # not require this +1 sequence. Raw-capture tests use recorded counters.
      values.setdefault("LatCtlCpbltyDStat_No_Cnt", self.lateral_counter)
      self.lateral_counter = (self.lateral_counter + 1) % 16
      values.setdefault("LatCtlCpbltyDStat_No_Cs", 255 - sum(values.get(field, 0) for field in
                        ("LatCtlSte_D_Stat", "LatCtlLim_D_Stat", "LatCtlCpblty_D_Stat", "LatCtlCpbltyDStat_No_Cnt")))
    msg = self.ford.packer.make_can_msg_safety(name, 0, values)
    return True if msg.addr == self.omit_address else self.safety.safety_rx_hook(msg)

  def refresh(self):
    self.safety.test_sp_platform(True)
    self.ford._engage_angle_mode()
    self.ford._reset_curvature_measurement(0, 15.)
    self.rx("EngBrakeData", CcStat_D_Actl=3, BpedDrvAppl_D_Actl=1)
    self.rx("EngVehicleSpThrottle", ApedPos_Pc_ActlArb=0)
    self.rx("DesiredTorqBrk", VehStop_D_Stat=0, PrkBrkStatus=4)
    self.rx("PowertrainData_10", TrnRng_D_Rq=3)
    self.rx("EPAS_INFO", EPAS_Failure=0, SteMdule_D_Stat=2, SteeringColumnTorque=0)
    self.rx("Lane_Assist_Data3_FD1", LatCtlSte_D_Stat=1)
    self.rx("SteeringPinion_Data", StePinCompAnEst_D_Qf=3)
    self.rx("Cluster_Info1_FD1", DrvSlipCtlMde_D_Rq=0)
    self.button(False)
    # Incomplete startup CAN revokes eligibility. A subsequent real heartbeat
    # is required after the full vehicle snapshot, just as on the board.
    self.safety.test_sp_platform(True)
    self.button(False)

  def button(self, pressed):
    self.rx("Steering_Data_FD1", TjaButtnOnOffPress=int(pressed))

  def engage(self):
    self.button(False)
    self.button(True)
    assert self.allowed()

  def allowed(self):
    return self.safety.test_sp_authorized()

  def steer(self, path_angle=.001, angle_mode=True):
    if not angle_mode:
      self.ford._tx(self.ford._lka_msg(angle_mode_engaged=False))
    msg = self.ford._lat_ctl2_msg(True, path_angle=path_angle)
    self.safety.set_timer(self.now)
    return self.ford._tx(msg)


@pytest.fixture
def h():
  return Harness()


def test_independent_lateral_without_longitudinal(h):
  h.engage()
  assert h.steer()
  assert not h.safety.get_controls_allowed()
  assert not h.safety.get_longitudinal_allowed()
  for cruise in (4, 5, 3):
    h.rx("EngBrakeData", CcStat_D_Actl=cruise, BpedDrvAppl_D_Actl=1)
    assert h.allowed()


def test_set_transition_engages_lateral_but_cancel_does_not_disable_it(h):
  assert not h.allowed()
  h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert h.allowed()
  h.rx("EngBrakeData", CcStat_D_Actl=3, BpedDrvAppl_D_Actl=1)
  assert h.allowed()
  h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert h.allowed()


def test_restart_while_cruise_engaged_cannot_autoengage(h):
  h.safety.set_safety_hooks(CarParams.SafetyModel.ford, 6)
  h.safety.test_sp_configure(True)
  h.safety.test_sp_platform(True)
  h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert not h.allowed()


def test_fault_while_cruise_remains_engaged_cannot_auto_reengage(h):
  h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert h.allowed()
  h.safety.safety_lateral_revoke(2)
  assert not h.allowed()
  for _ in range(3):
    h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
    assert not h.allowed()


def test_cruise_master_off_revokes_both_and_requires_new_set(h):
  h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert h.allowed()
  h.rx("EngBrakeData", CcStat_D_Actl=0, BpedDrvAppl_D_Actl=1)
  assert not h.allowed()
  h.refresh()
  assert not h.allowed()
  h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert h.allowed()


@pytest.mark.parametrize("host_packet", [(0, 0, 0), (1, 0, 0), (2, 1, 0), (65535, 1, 0),
                                     (0, 2, 0), (0, 65535, 0), (0, 1, 1), (0, 1, 65535)])
def test_malformed_or_negative_host_transport_revokes(h, host_packet):
  h.engage()
  h.safety.test_sp_heartbeat(*host_packet)
  assert not h.allowed()
  h.safety.test_sp_heartbeat(0, 1, 0)
  assert not h.allowed()
  h.button(True)
  assert not h.allowed()
  h.button(False)
  h.button(True)
  assert h.allowed()


@pytest.mark.parametrize("longitudinal", [0, 1])
def test_valid_heartbeat_is_only_eligibility(h, longitudinal):
  h.safety.test_sp_heartbeat(longitudinal, 1, 0)
  assert not h.allowed()
  h.engage()
  h.safety.test_sp_heartbeat(longitudinal, 1, 0)
  assert h.allowed()


@pytest.mark.parametrize("reason", [r for r in range(1, 15) if r not in (4, 5)])
def test_every_shared_revocation_clears_and_requires_new_physical_intent(h, reason):
  h.engage()
  h.safety.safety_lateral_revoke(reason)
  assert not h.allowed()
  if reason == 1:
    assert not h.safety.test_sp_enabled()
  else:
    h.refresh()
    assert not h.allowed()
    # A new physical SET transition is a valid fresh lateral request.
    h.rx("EngBrakeData", CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
    assert h.allowed()
    assert h.steer()


def test_invalid_can_skips_ford_rx_but_still_revokes(h):
  h.engage()
  bad = h.ford._speed_msg(15.)
  bad.data[3] ^= 1
  assert not h.safety.safety_rx_hook(bad)
  assert not h.allowed()
  h.refresh()
  assert not h.allowed()
  assert not h.steer()


@pytest.mark.parametrize("addr", REQUIRED_RX)
def test_malformed_required_rx_revokes(h, addr):
  h.engage()
  h.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, b"\0" * 7))
  assert not h.allowed()


@pytest.mark.parametrize("msg,fields", [
  *[("EngBrakeData", dict(CcStat_D_Actl=3, BpedDrvAppl_D_Actl=i)) for i in (0, 3)],
  ("EngBrakeData", dict(CcStat_D_Actl=2, BpedDrvAppl_D_Actl=1)),
  *[("Lane_Assist_Data3_FD1", dict(LatCtlSte_D_Stat=i)) for i in (0,4,5,6,7)],
])
def test_vehicle_faults_clear_without_resume(h, msg, fields):
  h.engage()
  h.rx(msg, **fields)
  assert not h.allowed()
  h.refresh()
  assert not h.allowed()


def test_held_button_after_fault_does_not_reengage(h):
  h.engage()
  h.safety.safety_lateral_revoke(2)
  h.button(True)
  assert not h.allowed()
  h.button(False)
  assert not h.allowed()
  h.button(True)
  assert not h.allowed()  # physical intent alone cannot replace host eligibility
  h.safety.test_sp_platform(True)
  h.button(True)
  assert not h.allowed()  # held press does not become a new selection
  h.button(False)
  h.button(True)
  assert h.allowed()
  h.button(False)
  h.button(True)
  assert not h.allowed()  # second deliberate press disengages


def test_status_read_does_not_refresh_status_liveness(h):
  h.engage()
  h.now += 100001
  h.safety.set_timer(h.now)
  assert not h.allowed()
  h.refresh()
  assert not h.allowed()


def test_negative_platform_immediate_veto_and_no_auto_return(h):
  h.engage()
  h.safety.test_sp_platform(False)
  assert not h.allowed()
  h.refresh()
  assert not h.allowed()


def test_reset_and_non_lightning_default(h):
  h.engage()
  h.safety.set_safety_hooks(CarParams.SafetyModel.ford, 2)
  assert not h.allowed()
  assert not h.safety.test_sp_enabled()
  other = Harness(False)
  other.button(True)
  assert not other.allowed()


def test_existing_rate_limit_and_curvature_mode_no_escape(h):
  h.engage()
  assert not h.steer(path_angle=.30)
  assert not h.allowed()
  h.refresh()
  h.engage()
  assert not h.steer(angle_mode=False)


def test_nonwhitelisted_tx_is_also_a_revocation(h):
  h.engage()
  assert not h.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x123, 0, b"\0"*8))
  assert not h.allowed()
