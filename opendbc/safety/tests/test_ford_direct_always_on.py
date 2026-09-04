"""ACC-main-derived AOL permissions, independent of SET and TJA edges."""
from opendbc.safety.tests.test_ford_sunnypilot_mads import Harness


def test_main_on_authorizes_without_any_tja_press():
  class NoButtonHarness(Harness):
    def button(self, pressed):
      # Preserve required message freshness; the button is always released.
      self.rx('Steering_Data_FD1', TjaButtnOnOffPress=0)

  h = NoButtonHarness()
  h.safety.test_sp_heartbeat(0, 0, 0)
  h.refresh()
  h.safety.test_sp_heartbeat(0, 1, 0)
  h.refresh()
  assert h.allowed()
  assert not h.safety.get_controls_allowed()


def test_cancel_state_preserves_lateral_and_main_off_revokes():
  h = Harness()
  assert h.allowed()
  h.rx('EngBrakeData', CcStat_D_Actl=4, BpedDrvAppl_D_Actl=1)
  assert h.safety.get_controls_allowed()
  h.rx('EngBrakeData', CcStat_D_Actl=3, BpedDrvAppl_D_Actl=1)
  assert not h.safety.get_controls_allowed()
  assert h.allowed()
  h.rx('EngBrakeData', CcStat_D_Actl=0, BpedDrvAppl_D_Actl=1)
  assert not h.allowed()
