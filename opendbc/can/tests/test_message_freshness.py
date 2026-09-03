from opendbc.can import CANPacker, CANParser
from opendbc.can.tests import TEST_DBC


def test_message_fresh_uses_parser_time_and_exact_boundary():
  parser = CANParser(TEST_DBC, [("CAN_FD_MESSAGE", 10)], 0)
  packer = CANPacker(TEST_DBC)
  assert not parser.message_fresh("CAN_FD_MESSAGE", 100_000_000)
  msg = packer.make_can_msg("CAN_FD_MESSAGE", 0, {})
  parser.update([1_000_000_000, [msg]])
  assert parser.message_fresh("CAN_FD_MESSAGE", 100_000_000)
  parser.update([1_100_000_000, []])
  assert parser.message_fresh("CAN_FD_MESSAGE", 100_000_000)
  parser.update([1_100_000_001, []])
  assert not parser.message_fresh("CAN_FD_MESSAGE", 100_000_000)
  assert not parser.message_fresh("UNKNOWN", 100_000_000)
  assert not parser.message_fresh("CAN_FD_MESSAGE", -1)
