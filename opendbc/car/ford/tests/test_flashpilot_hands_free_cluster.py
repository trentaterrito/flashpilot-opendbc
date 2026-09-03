"""
FlashPilot: tests for the Lightning-only Ford hands-free cluster display option
(IPMA_Data / LaHandsOff_D_Dsply). Display-only -- see
docs/flashpilot/FLASHPILOT_UI_FORD_HANDS_FREE_CLUSTER_AUDIT.md. Constructs a real
CarController and calls the real update() (not a mock of it), matching the
established pattern in test_flashpilot_angle.py.
"""
import unittest
from collections import defaultdict
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.values import CAR, DBC, FordFlags

FORD_IPMA_Data = 0x3D8
VisualAlert = structs.CarControl.HUDControl.VisualAlert


def _make_controller(fingerprint, hands_free_cluster=False):
  flags = int(FordFlags.CANFD)
  if hands_free_cluster:
    flags |= int(FordFlags.HANDS_FREE_CLUSTER)
  CP = structs.CarParams(carFingerprint=fingerprint, flags=flags, openpilotLongitudinalControl=False)
  dbc_names = {Bus.pt: DBC[fingerprint][Bus.pt]}
  return CarController(dbc_names, CP)


def _make_cs():
  cs_out = structs.CarState()
  return SimpleNamespace(out=cs_out, buttons_stock_values=defaultdict(int),
                          acc_tja_status_stock_values=defaultdict(int), lkas_status_stock_values=defaultdict(int))


def _make_cc_reader(lat_active=True, steer_alert=False):
  # See test_flashpilot_angle.py's _make_cc_reader: actuators.as_builder() (used at
  # the end of CarController.update()) only exists on capnp reader objects, not on
  # locally-constructed builders -- round-trip through bytes to get a real reader.
  visual_alert = VisualAlert.steerRequired if steer_alert else VisualAlert.none
  builder = structs.CarControl(latActive=lat_active, hudControl=structs.CarControl.HUDControl(visualAlert=visual_alert))
  return structs.CarControl.from_bytes(builder.to_bytes())


def _find_msg(can_sends, addr):
  """Returns the *last* matching message (IPMA_Data is only sent every
  LKAS_UI_STEP frames, not every call -- accumulate across several update() calls
  and take the last)."""
  found = None
  for a, dat, _bus in can_sends:
    if a == addr:
      found = dat
  return found


def _get_signal(packer, addr, sig_name, dat: bytes) -> int:
  """Decodes a single signal's raw (unscaled) value out of a packed CAN message,
  using the same DBC signal metadata (lsb/size/is_little_endian) the packer itself
  packs with -- this is packer.py's set_value() run in reverse, so it can't
  disagree with how the message was actually built."""
  sig = packer.dbc.msgs[addr].sigs[sig_name]
  i = sig.lsb // 8
  bits = sig.size
  ival = 0
  shift_total = 0
  while 0 <= i < len(dat) and bits > 0:
    shift = sig.lsb % 8 if (sig.lsb // 8) == i else 0
    size = min(bits, 8 - shift)
    chunk = (dat[i] >> shift) & ((1 << size) - 1)
    ival |= chunk << shift_total
    shift_total += size
    bits -= size
    i = i + 1 if sig.is_little_endian else i - 1
  return ival


def _run(cc, lat_active, steer_alert, frames=100):
  """LKAS_UI_STEP is 100 frames (1Hz @ 100Hz base) -- run enough frames that
  IPMA_Data is guaranteed to be sent at least once, accumulating can_sends."""
  all_sends = []
  for _ in range(frames):
    with _make_cc_reader(lat_active=lat_active, steer_alert=steer_alert) as CC:
      _, sends = cc.update(CC, _make_cs(), 0)
      all_sends += sends
  return all_sends


class TestFlashPilotFordHandsFreeCluster(unittest.TestCase):
  def _hands_off_value(self, can_sends, cc):
    dat = _find_msg(can_sends, FORD_IPMA_Data)
    self.assertIsNotNone(dat, "IPMA_Data must be sent")
    return _get_signal(cc.packer, FORD_IPMA_Data, "LaHandsOff_D_Dsply", dat)

  def test_toggle_off_no_alert_is_0(self):
    """1. Toggle OFF + no alert: LaHandsOff_D_Dsply = 0."""
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=False)
    sends = _run(cc, lat_active=True, steer_alert=False)
    self.assertEqual(self._hands_off_value(sends, cc), 0)

  def test_toggle_off_steer_alert_is_1(self):
    """2. Toggle OFF + steering alert: existing value 1."""
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=False)
    sends = _run(cc, lat_active=True, steer_alert=True)
    self.assertEqual(self._hands_off_value(sends, cc), 1)

  def test_toggle_on_lightning_engaged_no_alert_is_2(self):
    """3. Toggle ON + Lightning + latActive + no alert: value 2."""
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=True)
    sends = _run(cc, lat_active=True, steer_alert=False)
    self.assertEqual(self._hands_off_value(sends, cc), 2)

  def test_toggle_on_lateral_inactive_is_0(self):
    """4. Toggle ON + lateral inactive: value 0."""
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=True)
    sends = _run(cc, lat_active=False, steer_alert=False)
    self.assertEqual(self._hands_off_value(sends, cc), 0)

  def test_toggle_on_steer_alert_wins_is_1(self):
    """5. Toggle ON + steering alert: value 1, alert wins over hands-free display."""
    cc = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=True)
    sends = _run(cc, lat_active=True, steer_alert=True)
    self.assertEqual(self._hands_off_value(sends, cc), 1)

  def test_non_lightning_ford_unaffected_even_if_flag_set(self):
    """6. Non-Lightning Ford: unchanged even if flag is artificially set (defense
    in depth -- CarController's own fingerprint re-check, independent of whatever
    gated the flag in card.py)."""
    for car in (CAR.FORD_F_150_MK14, CAR.FORD_EXPEDITION_MK4, CAR.FORD_MUSTANG_MACH_E_MK1):
      with self.subTest(car=car):
        cc = _make_controller(car, hands_free_cluster=True)
        self.assertFalse(cc._ford_hands_free_cluster, f"{car} must never honor HANDS_FREE_CLUSTER")
        sends = _run(cc, lat_active=True, steer_alert=False)
        self.assertEqual(self._hands_off_value(sends, cc), 0)
        sends = _run(cc, lat_active=True, steer_alert=True)
        self.assertEqual(self._hands_off_value(sends, cc), 1)

  def test_all_other_ipma_fields_unchanged_on_vs_off(self):
    """7. All other IPMA_Data fields: unchanged ON vs OFF (same scenario --
    engaged, no alert -- where the toggle *does* change LaHandsOff_D_Dsply)."""
    cc_off = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=False)
    cc_on = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=True)
    dat_off = _find_msg(_run(cc_off, lat_active=True, steer_alert=False), FORD_IPMA_Data)
    dat_on = _find_msg(_run(cc_on, lat_active=True, steer_alert=False), FORD_IPMA_Data)

    other_fields = [
      "LaActvStats_D_Dsply", "FeatConfigIpmaActl", "FeatNoIpmaActl", "PersIndexIpma_D_Actl",
      "AhbcRampingV_D_Rq", "LaDenyStats_B_Dsply", "CamraDefog_B_Req", "CamraStats_D_Dsply",
      "DasAlrtLvl_D_Dsply", "DasStats_D_Dsply", "DasWarn_D_Dsply", "AhbHiBeam_D_Rq",
    ]
    for field in other_fields:
      with self.subTest(field=field):
        self.assertEqual(_get_signal(cc_off.packer, FORD_IPMA_Data, field, dat_off),
                         _get_signal(cc_on.packer, FORD_IPMA_Data, field, dat_on))

    # and confirm the one field that's *allowed* to differ actually does, so this
    # test isn't vacuously passing on two identical (both-0) messages
    self.assertNotEqual(_get_signal(cc_off.packer, FORD_IPMA_Data, "LaHandsOff_D_Dsply", dat_off),
                        _get_signal(cc_on.packer, FORD_IPMA_Data, "LaHandsOff_D_Dsply", dat_on))

  def test_all_other_can_messages_unchanged_on_vs_off(self):
    """8. All other CAN messages: unchanged ON vs OFF -- proves no steering,
    engagement, or longitudinal behavior changed. Runs enough frames to cover
    every message's own send cadence (LKA/ACC/buttons/etc.), all under identical
    CS/CC inputs except the toggle itself."""
    cc_off = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=False)
    cc_on = _make_controller(CAR.FORD_F_150_LIGHTNING_MK1, hands_free_cluster=True)
    sends_off = _run(cc_off, lat_active=True, steer_alert=False, frames=100)
    sends_on = _run(cc_on, lat_active=True, steer_alert=False, frames=100)

    self.assertEqual(len(sends_off), len(sends_on), "identical message cadence expected")
    for i, ((addr_off, dat_off, bus_off), (addr_on, dat_on, bus_on)) in enumerate(zip(sends_off, sends_on, strict=True)):
      with self.subTest(i=i, addr=hex(addr_off)):
        self.assertEqual(addr_off, addr_on)
        self.assertEqual(bus_off, bus_on)
        if addr_off == FORD_IPMA_Data:
          continue  # the one message this toggle is allowed to change
        self.assertEqual(dat_off, dat_on, f"message {hex(addr_off)} must be byte-identical regardless of the toggle")

  def test_param_unset_matches_off(self):
    """9. Param false/unset: current behavior (FordFlags.HANDS_FREE_CLUSTER unset
    is exactly what an unset/False Param produces via card.py -- see
    test_toggle_off_* above; this test pins the default-constructed-CP case
    specifically, with no flags argument at all beyond CANFD)."""
    CP = structs.CarParams(carFingerprint=CAR.FORD_F_150_LIGHTNING_MK1, flags=int(FordFlags.CANFD),
                           openpilotLongitudinalControl=False)
    dbc_names = {Bus.pt: DBC[CAR.FORD_F_150_LIGHTNING_MK1][Bus.pt]}
    cc = CarController(dbc_names, CP)
    self.assertFalse(cc._ford_hands_free_cluster)
    sends = _run(cc, lat_active=True, steer_alert=False)
    self.assertEqual(self._hands_off_value(sends, cc), 0)


if __name__ == "__main__":
  unittest.main()
