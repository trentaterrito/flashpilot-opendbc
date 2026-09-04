"""Production selector, independent rejected-long semantics and parity limits."""
import pytest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_ford_mads_remain_active import BrakeHarness


@pytest.mark.parametrize("param,selected", [(0, False), (1, False), (2, False), (3, False),
                                          (4, False), (5, False), (6, True), (7, True)])
def test_real_safety_initializer_default_off_and_canfd_only(param, selected):
  h = BrakeHarness()
  h.safety.set_safety_hooks(CarParams.SafetyModel.ford, param)
  assert h.safety.test_sp_enabled() == selected
  assert not h.allowed()
  h.safety.test_sp_platform(True)
  assert not h.allowed()  # selection or heartbeat is never engagement


def test_rejected_long_after_brake_does_not_revoke_lateral():
  h = BrakeHarness()
  h.safety.set_safety_hooks(CarParams.SafetyModel.ford, 7)
  h.refresh()
  h.engage()
  h.brake(False, cruise=4)
  h.brake(True, cruise=4)
  assert h.allowed()
  assert not h.safety.get_longitudinal_allowed()
  msg = h.ford.packer.make_can_msg_safety("ACCDATA", 0, {
    "AccPrpl_A_Rq": .5, "AccPrpl_A_Pred": .5, "AccBrkTot_A_Rq": 0.,
    "AccBrkPrchg_B_Rq": 0, "AccBrkDecel_B_Rq": 0, "CmbbDeny_B_Actl": 0,
  })
  for _ in range(10):
    assert not h.safety.safety_tx_hook(msg)
    assert h.allowed()
    assert not h.safety.get_longitudinal_allowed()
  assert h.steer()
  h.brake(False)
  assert not h.safety.get_longitudinal_allowed()
  assert not h.steer(.30)  # same path-angle limits still apply
  assert not h.allowed()


@pytest.mark.parametrize("address,bus,length", [(0x186, 1, 8), (0x186, 0, 7),
                                               (0x3D6, 0, 7), (0x123, 0, 8)])
def test_malformed_wrong_bus_or_unknown_tx_still_revokes(address, bus, length):
  h = BrakeHarness()
  h.engage()
  assert not h.safety.safety_tx_hook(libsafety_py.make_CANPacket(address, bus, bytes(length)))
  assert not h.allowed()


def test_raw_driver_torque_is_not_an_extra_latch_cancel():
  h = BrakeHarness()
  h.engage()
  h.rx("EPAS_INFO", EPAS_Failure=0, SteMdule_D_Stat=2, SteeringColumnTorque=1.0625)
  assert h.allowed()
  h.safety.safety_lateral_revoke(6)  # genuine steering disengage unchanged
  assert not h.allowed()


@pytest.mark.parametrize("address", [0x176, 0x82, 0x7E, 0x430])
def test_host_owned_source_does_not_add_panda_deadline(address):
  h = BrakeHarness()
  h.engage()
  h.omit_address = address
  for _ in range(120):
    h.now += 10000
    h.safety.set_timer(h.now)
    h.refresh()
  assert h.allowed()
  # Host CANParser/operating fault now owns this veto, not a stale raw flag.
  h.safety.test_sp_heartbeat(0, 0, 0)
  assert not h.allowed()


def test_ordinary_heartbeat_spacing_and_mode_metadata_do_not_add_100ms_cutoff():
  h = BrakeHarness()
  h.engage()
  original_mode = h.ford._engage_angle_mode
  h.ford._engage_angle_mode = lambda: None
  # Board heartbeat eligibility remains true; existing native heartbeat fault
  # predicate owns loss. Keep all required CAN traffic and status fresh.
  original_platform = h.safety.test_sp_platform
  for _ in range(20):
    h.now += 10000
    h.safety.set_timer(h.now)
    h.refresh()
    assert h.allowed()
  h.ford._engage_angle_mode = original_mode
  original_platform(False)
  assert not h.allowed()
