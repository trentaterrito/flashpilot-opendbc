import unittest

from opendbc.car.can_definitions import CanData
from opendbc.car.ford.values import CAR, FW_QUERY_CONFIG
from opendbc.car.ford.fingerprints import FW_VERSIONS
from opendbc.car.fw_versions import build_fw_dict, get_fw_versions, match_fw_to_car_exact
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from opendbc.car.structs import CarParams

# Observed ABS CAN-FD response, including its actual padding.
ABS_FRAME = bytes.fromhex('001b62f188524c33382d32443035332d42440000000000000000000000cccccc')
ABS_VERSION = ABS_FRAME[5:29]
CAMERA_FRAME = bytes.fromhex('001b62f188524a36542d3134483130322d424243000000000000000000cccccc')
CAMERA_VERSION = CAMERA_FRAME[5:29]


class FirmwareBus:
  """Respond to direct F188 only; tester-present intentionally receives no reply."""
  def __init__(self, frame=ABS_FRAME, bus=0, address=0x768, request_address=0x760):
    self.frame, self.bus, self.address = frame, bus, address
    self.request_address = request_address
    self.pending = []
    self.sent = []

  def send(self, messages):
    self.sent.extend(messages)
    for m in messages:
      if m.src == 0 and m.address == self.request_address and m.dat[:4] == b'\x03\x22\xf1\x88' and self.frame is not None:
        self.pending.append(CanData(self.address, self.frame, self.bus))

  def recv(self, wait_for_one=False):
    result, self.pending = self.pending, []
    return [result] if result else []


def run_query(request, bus, address=0x760):
  return IsoTpParallelQuery(bus.send, bus.recv, request.bus, [address], request.request,
                            request.response, request.rx_offset).get_data(0.002)


class TestDirectFirmware(unittest.TestCase):
  def test_original_handshake_blocks_version_read(self):
    bus = FirmwareBus()
    self.assertEqual(run_query(FW_QUERY_CONFIG.requests[1], bus), {})
    self.assertEqual([m.dat[:3] for m in bus.sent], [b'\x02\x3e\x00'])

  def test_direct_read_decodes_recorded_can_fd_response(self):
    self.assertEqual(run_query(FW_QUERY_CONFIG.requests[-1], FirmwareBus()), {(0x760, None): ABS_VERSION})

  def test_bad_or_missing_response_is_rejected(self):
    for bus in (FirmwareBus(None), FirmwareBus(ABS_FRAME, bus=1), FirmwareBus(ABS_FRAME, address=0x739),
                FirmwareBus(ABS_FRAME[:4] + b'\x90' + ABS_FRAME[5:])):
      with self.subTest(bus=bus):
        self.assertEqual(run_query(FW_QUERY_CONFIG.requests[-1], bus), {})

  def test_acquisition_retains_ford_provenance_and_exact_bytes(self):
    bus = FirmwareBus()
    records = get_fw_versions(bus.recv, bus.send, lambda _: None, query_brand='ford', timeout=0.002)
    self.assertEqual(len(records), 1)
    fw = records[0]
    self.assertEqual((fw.brand, fw.address, fw.responseAddress, fw.bus, fw.logging), ('ford', 0x760, 0x768, 0, False))
    self.assertEqual(list(fw.request), [b'\x22\xf1\x88'])
    self.assertEqual(fw.fwVersion, ABS_VERSION)
    self.assertEqual(build_fw_dict(records, 'ford'), {(0x760, None): {ABS_VERSION}})

  def test_exact_matching_still_requires_correct_abs(self):
    fingerprint = CAR.FORD_F_150_LIGHTNING_MK1
    live = {(addr, sub): {versions[0]} for (_, addr, sub), versions in FW_VERSIONS[fingerprint].items()}
    live[(0x760, None)] = {ABS_VERSION}
    self.assertEqual(match_fw_to_car_exact(live, 'ford', log=False), {fingerprint})
    del live[(0x760, None)]
    self.assertNotIn(fingerprint, match_fw_to_car_exact(live, 'ford', log=False))
    live[(0x760, None)] = {b'wrong firmware'}
    self.assertNotIn(fingerprint, match_fw_to_car_exact(live, 'ford', log=False))

  def test_camera_handshake_failure_and_direct_response(self):
    original = FirmwareBus(CAMERA_FRAME, address=0x70e, request_address=0x706)
    self.assertEqual(run_query(FW_QUERY_CONFIG.requests[1], original, 0x706), {})
    direct = FirmwareBus(CAMERA_FRAME, address=0x70e, request_address=0x706)
    self.assertEqual(run_query(FW_QUERY_CONFIG.requests[-1], direct, 0x706), {(0x706, None): CAMERA_VERSION})

  def test_camera_acquisition_and_missing_camera_match(self):
    bus = FirmwareBus(CAMERA_FRAME, address=0x70e, request_address=0x706)
    records = get_fw_versions(bus.recv, bus.send, lambda _: None, query_brand='ford', timeout=0.002)
    self.assertEqual(len(records), 1)
    fw = records[0]
    self.assertEqual((fw.brand, fw.address, fw.responseAddress, fw.bus, fw.logging), ('ford', 0x706, 0x70e, 0, False))
    self.assertEqual(fw.fwVersion, CAMERA_VERSION)
    fingerprint = CAR.FORD_F_150_LIGHTNING_MK1
    live = {(addr, sub): {versions[0]} for (_, addr, sub), versions in FW_VERSIONS[fingerprint].items()}
    live[(0x706, None)] = {CAMERA_VERSION}
    self.assertEqual(match_fw_to_car_exact(live, 'ford', log=False), {fingerprint})
    del live[(0x706, None)]
    self.assertNotIn(fingerprint, match_fw_to_car_exact(live, 'ford', log=False))
    live[(0x706, None)] = {b'wrong firmware'}
    self.assertNotIn(fingerprint, match_fw_to_car_exact(live, 'ford', log=False))

  def test_query_scope(self):
    r = FW_QUERY_CONFIG.requests[-1]
    self.assertEqual(set(r.whitelist_ecus), {CarParams.Ecu.abs, CarParams.Ecu.eps, CarParams.Ecu.fwdCamera})
    self.assertEqual((r.bus, r.logging, r.rx_offset), (0, False, 8))
    self.assertEqual(r.request, [b'\x22\xf1\x88'])
    self.assertEqual(r.response, [b'\x62\xf1\x88'])


if __name__ == '__main__':
  unittest.main()
