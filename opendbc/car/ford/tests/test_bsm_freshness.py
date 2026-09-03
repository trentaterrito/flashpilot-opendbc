from types import SimpleNamespace

import pytest

from opendbc.car.ford.carstate import BSM_FRESHNESS_NS, read_bsm_state
from opendbc.car.structs import car


class FakeParser:
  def __init__(self, value, fresh):
    self.vl = {"Side_Detect_L_Stat": {"SodDetctLeft_D_Stat": value}}
    self.fresh = fresh

  def message_fresh(self, message, max_age_nanos):
    assert message == "Side_Detect_L_Stat"
    assert max_age_nanos == BSM_FRESHNESS_NS
    return self.fresh


@pytest.mark.parametrize("raw", range(8))
def test_all_nonzero_bsm_states_fail_closed(raw):
  occupied, valid = read_bsm_state(FakeParser(raw, True), "Side_Detect_L_Stat", "SodDetctLeft_D_Stat")
  assert occupied == (raw != 0)
  assert valid


@pytest.mark.parametrize("fresh", [False, True])
def test_validity_is_independent_of_cached_clear(fresh):
  occupied, valid = read_bsm_state(FakeParser(0, fresh), "Side_Detect_L_Stat", "SodDetctLeft_D_Stat")
  assert not occupied
  assert valid is fresh


def test_carstate_schema_carries_per_side_validity():
  state = car.CarState(leftBlindspotValid=True, rightBlindspotValid=False)
  assert state.leftBlindspotValid
  assert not state.rightBlindspotValid
