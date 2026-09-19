import random
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.structs import CarParams
from opendbc.car.ford.carstate import CarState
from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car
from opendbc.car.ford.values import CAR, DBC, FW_QUERY_CONFIG, FW_PATTERN, get_platform_codes, FordFlags
from opendbc.car.ford.fingerprints import FW_VERSIONS
from opendbc.car.ford.interface import CarInterface
from opendbc.testing import fuzzy_test, parameterized

Ecu = CarParams.Ecu


class TestFordLongitudinalActuatorDelay(unittest.TestCase):
  def test_lightning_actuator_delay(self):
    # Physically validated on real F-150 Lightning hardware (FlashPilot fork); do not change
    # without new physical validation.
    CP = CarInterface.get_non_essential_params(CAR.FORD_F_150_LIGHTNING_MK1)
    assert CP.longitudinalActuatorDelay == 0.25

  def test_other_ford_platforms_unaffected(self):
    default_delay = CarInterface.get_non_essential_params(CAR.FORD_F_150_MK14).longitudinalActuatorDelay
    for platform in CAR:
      if platform == CAR.FORD_F_150_LIGHTNING_MK1:
        continue
      CP = CarInterface.get_non_essential_params(platform)
      assert CP.longitudinalActuatorDelay == default_delay, \
        f"{platform} actuator delay changed unexpectedly"


ECU_ADDRESSES = {
  Ecu.eps: 0x730,          # Power Steering Control Module (PSCM)
  Ecu.abs: 0x760,          # Anti-Lock Brake System (ABS)
  Ecu.fwdRadar: 0x764,     # Cruise Control Module (CCM)
  Ecu.fwdCamera: 0x706,    # Image Processing Module A (IPMA)
  Ecu.engine: 0x7E0,       # Powertrain Control Module (PCM)
  Ecu.shiftByWire: 0x732,  # Gear Shift Module (GSM)
  Ecu.debug: 0x7D0,        # Accessory Protocol Interface Module (APIM)
}


ECU_PART_NUMBER = {
  Ecu.eps: [
    b"14D003",
  ],
  Ecu.abs: [
    b"2D053",
  ],
  Ecu.fwdRadar: [
    b"14D049",
  ],
  Ecu.fwdCamera: [
    b"14F397",  # Ford Q3
    b"14H102",  # Ford Q4
  ],
}


class TestFordFW(unittest.TestCase):
  def test_lightning_ipma_bursty_rate_is_explicit(self):
    cp = CarParams(carFingerprint=CAR.FORD_F_150_LIGHTNING_MK1)
    parsers = CarState.get_can_parsers(cp)
    ipma = parsers[Bus.cam].message_states[0x3D8]
    assert ipma.frequency == 1
    assert ipma.timeout_threshold == 10_000_000_000

  def test_fw_query_config(self):
    for (ecu, addr, subaddr) in FW_QUERY_CONFIG.extra_ecus:
      assert ecu in ECU_ADDRESSES, "Unknown ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

  @parameterized("car_model, fw_versions", FW_VERSIONS.items())
  def test_fw_versions(self, car_model, fw_versions):
    for (ecu, addr, subaddr), fws in fw_versions.items():
      assert ecu in ECU_PART_NUMBER, "Unexpected ECU"
      assert addr == ECU_ADDRESSES[ecu], "ECU address mismatch"
      assert subaddr is None, "Unexpected ECU subaddress"

      for fw in fws:
        assert len(fw) == 24, "Expected ECU response to be 24 bytes"

        match = FW_PATTERN.match(fw)
        assert match is not None, f"Unable to parse FW: {fw!r}"
        if match:
          part_number = match.group("part_number")
          assert part_number in ECU_PART_NUMBER[ecu], f"Unexpected part number for {fw!r}"

        codes = get_platform_codes([fw])
        assert 1 == len(codes), f"Unable to parse FW: {fw!r}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([
      b"JX6A-14C204-BPL\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"NZ6T-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"PJ6T-14H102-ABJ\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      b"LB5A-14C204-EAC\x00\x00\x00\x00\x00\x00\x00\x00\x00",
    ])
    assert results == {(b"X6A", b"J"), (b"Z6T", b"N"), (b"J6T", b"P"), (b"B5A", b"L")}

  def test_fuzzy_match(self):
    for platform, fw_by_addr in FW_VERSIONS.items():
      # Ensure there's no overlaps in platform codes
      for _ in range(20):
        car_fw = []
        for ecu, fw_versions in fw_by_addr.items():
          ecu_name, addr, sub_addr = ecu
          fw = random.choice(fw_versions)
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

        CP = CarParams(carFw=car_fw)
        matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
        assert matches == {platform}

  def test_match_fw_fuzzy(self):
    offline_fw = {
      (Ecu.eps, 0x730, None): [
        b"L1MC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.abs, 0x760, None): [
        b"L1MC-2D053-BA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"L1MC-2D053-BD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      (Ecu.fwdRadar, 0x764, None): [
        b"LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"LB5T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
      # We consider all model year hints for ECU, even with different platform codes
      (Ecu.fwdCamera, 0x706, None): [
        b"LB5T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"NC5T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
      ],
    }
    expected_fingerprint = CAR.FORD_EXPLORER_MK6

    # ensure that we fuzzy match on all non-exact FW with changed revisions
    live_fw = {
      (0x730, None): {b"L1MC-14D003-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x760, None): {b"L1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x764, None): {b"LB5T-14D049-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
      (0x706, None): {b"LB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"},
    }
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert candidates == {expected_fingerprint}

    # model year hint in between the range should match
    live_fw[(0x706, None)] = {b"MB5T-14F397-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw,})
    assert candidates == {expected_fingerprint}

    # unseen model year hint should not match
    live_fw[(0x760, None)] = {b"M1MC-2D053-XX\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"}
    candidates = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fw, '', {expected_fingerprint: offline_fw})
    assert len(candidates) == 0, "Should not match new model year hint"

  def test_my2024_lightning_exact_match(self):
    """
    Regression anchor (FlashPilot project) for a real 2024 F-150 Lightning capture.
    ABS/EPS/camera already matched firmware on file; the radar firmware
    (RB5T-14D049-AB) was new for this model year and is why this needs its own
    case: before it was added to FW_VERSIONS, this exact combination matched
    nothing at all via match_fw_to_car -- not exact, not fuzzy (Ford's fuzzy
    matcher also requires the radar's platform code to be recognized) -- and
    would have fallen through to MOCK. Uses match_fw_to_car (the same
    brand-agnostic dispatcher get_car() calls), not FW_QUERY_CONFIG's
    Ford-specific matcher directly, so this exercises the real match path.
    """
    car_fw = [
      CarParams.CarFw(brand="ford", ecu=Ecu.abs, address=0x760, subAddress=0,
                       fwVersion=b"RL38-2D053-BD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"),
      CarParams.CarFw(brand="ford", ecu=Ecu.eps, address=0x730, subAddress=0,
                       fwVersion=b"RL38-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"),
      CarParams.CarFw(brand="ford", ecu=Ecu.fwdCamera, address=0x706, subAddress=0,
                       fwVersion=b"RJ6T-14H102-BBC\x00\x00\x00\x00\x00\x00\x00\x00\x00"),
      CarParams.CarFw(brand="ford", ecu=Ecu.fwdRadar, address=0x764, subAddress=0,
                       fwVersion=b"RB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"),
    ]
    exact_match, matches = match_fw_to_car(car_fw, "")
    assert exact_match, "Real 2024 Lightning capture should resolve via exact match"
    assert matches == {CAR.FORD_F_150_LIGHTNING_MK1}


class TestFordLightningBsm(unittest.TestCase):
  """
  Regression test for a 3-crash chain in the V1 BSM port (message_fresh() does
  not exist on this baseline's CANParser; leftBlindspotValid/rightBlindspotValid
  are not CarState fields; cp.message_states[934] raised KeyError because
  Side_Detect_L_Stat/R_Stat are never in either parser's constructor message
  list for the real Lightning config -- only reachable via cp.vl's lazy
  auto-registration, which plain dict indexing into message_states bypasses).

  No V2-4 consumer of BSM validity/freshness exists (modeld.py's
  DesireHelper.update() call and the MICI-only UI indicator that read
  leftBlindspotValid/rightBlindspotValid in V1 were never ported), so this
  reverts to pristine upstream's plain two-line behavior instead of inventing
  another freshness mechanism. This test uses the REAL, unmodified
  CarState.get_can_parsers() factory -- the actual production parser
  configuration -- not a synthetic parser pre-registered with the BSM message.
  """

  def setUp(self):
    self.CP = CarParams(carFingerprint=CAR.FORD_F_150_LIGHTNING_MK1, enableBsm=True,
                         flags=int(FordFlags.CANFD))
    self.parsers = CarState.get_can_parsers(self.CP)
    self.cp_bsm = self.parsers[Bus.cam]  # CANFD Lightning: cp_bsm = cp_cam
    dbc_name = DBC[CAR.FORD_F_150_LIGHTNING_MK1][Bus.pt]
    self.packer = CANPacker(dbc_name)

  def _send(self, t_nanos: int, left: bool | None = None, right: bool | None = None):
    bus = self.cp_bsm.bus
    msgs = []
    if left is not None:
      msgs.append(self.packer.make_can_msg("Side_Detect_L_Stat", bus, {"SodDetctLeft_D_Stat": 1 if left else 0}))
    if right is not None:
      msgs.append(self.packer.make_can_msg("Side_Detect_R_Stat", bus, {"SodDetctRight_D_Stat": 1 if right else 0}))
    self.cp_bsm.update([(t_nanos, msgs)])

  def _read(self) -> tuple[bool, bool]:
    left = self.cp_bsm.vl["Side_Detect_L_Stat"]["SodDetctLeft_D_Stat"] != 0
    right = self.cp_bsm.vl["Side_Detect_R_Stat"]["SodDetctRight_D_Stat"] != 0
    return left, right

  def test_parser_construction_does_not_preregister_bsm(self):
    # Confirms the real bug precondition: BSM is not in the constructor list,
    # only IPMA_Data is, for the real Lightning cam-bus parser.
    assert 934 not in self.cp_bsm.message_states
    assert 935 not in self.cp_bsm.message_states

  def test_no_bsm_frame_yet_reads_inactive_without_keyerror(self):
    left, right = self._read()
    assert left is False
    assert right is False

  def test_inactive_bsm(self):
    self._send(0, left=False, right=False)
    left, right = self._read()
    assert left is False
    assert right is False

  def test_active_left_bsm(self):
    # Side_Detect_L_Stat/R_Stat aren't in the constructor message list (only
    # IPMA_Data is), so CANParser.update() silently drops the very first raw
    # frame for an address it hasn't seen before -- cp.vl's lazy
    # auto-registration (VLDict.__getitem__ -> _add_message) only runs on
    # access, one step behind update()'s own address lookup. True of pristine
    # upstream too, not something this port changed: production reads
    # cp.vl[...] every real cycle, so it self-heals from the second frame
    # onward. Warm up once here to test steady-state behavior, not that
    # one-cycle startup transient (already covered by the "no frame yet" case).
    self._read()
    self._send(0, left=True, right=False)
    left, right = self._read()
    assert left is True
    assert right is False

  def test_active_right_bsm(self):
    self._read()  # see test_active_left_bsm
    self._send(0, left=False, right=True)
    left, right = self._read()
    assert left is False
    assert right is True

  def test_repeated_update_no_keyerror(self):
    for i in range(5):
      self._send(i * 20_000_000, left=bool(i % 2), right=bool((i + 1) % 2))
    left, right = self._read()
    assert isinstance(left, bool)
    assert isinstance(right, bool)
