"""0x3CC corruption/frozen-stream protection; explicit changing-replay limits."""
import json
from pathlib import Path

import pytest

from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_ford_sunnypilot_mads import Harness

CORPUS = json.loads((Path(__file__).parent / "data/ford_3cc_observed.json").read_text())
PAYLOADS = [bytes.fromhex(raw) for raw in CORPUS["payloads"]]
CAPTURE = bytes.fromhex("a80002809cf40000")


def packet(data, bus=0):
  return libsafety_py.make_CANPacket(0x3CC, bus, data)


def advance(h, delta):
  # Keep independent required vehicle messages/host fresh at 100 Hz. 0x3CC
  # may be omitted by the test so it cannot accidentally refresh from a fixture.
  while delta:
    dt = min(delta, 10000)
    h.now = (h.now + dt) & 0xffffffff
    h.safety.set_timer(h.now)
    h.refresh()
    delta -= dt


def setup_stream():
  h = Harness()
  h.engage()
  h.omit_address = 0x3CC
  h.safety.safety_rx_hook(packet(CAPTURE))
  assert h.allowed()
  return h


def test_actual_c_checksum_all_captured_payloads():
  assert sum(CORPUS["payloads"].values()) == 377209
  assert all(libsafety_py.libsafety.test_sp_status_checksum(packet(p)) for p in PAYLOADS)


@pytest.mark.parametrize("byte,bit", [(2, b) for b in range(3)] + [(4, b) for b in range(8)] + [(5, b) for b in range(8)])
def test_covered_corruptions_rejected_by_c_and_revoke_through_dispatcher(byte, bit):
  for raw in PAYLOADS:
    bad = bytearray(raw)
    bad[byte] ^= 1 << bit
    assert not libsafety_py.libsafety.test_sp_status_checksum(packet(bad))
  h = setup_stream()
  bad = bytearray(CAPTURE)
  bad[byte] ^= 1 << bit
  h.safety.safety_rx_hook(packet(bad))
  assert not h.allowed()
  assert not h.safety.test_sp_status_ready()
  h.safety.safety_rx_hook(packet(CAPTURE))
  h.safety.test_sp_platform(True)
  assert not h.allowed()
  h.engage()


@pytest.mark.parametrize("length", [0, 1, 7, 12, 64])
def test_malformed_revokes(length):
  h = setup_stream()
  h.safety.safety_rx_hook(packet(bytes(length)))
  assert not h.allowed()


@pytest.mark.parametrize("change_payload", [False, True])
def test_identical_replay_or_frozen_counter_with_changing_payload_expires(change_payload):
  h = setup_stream()
  for i in range(10):
    advance(h, 10000)
    raw = bytearray(CAPTURE)
    if change_payload:
      raw[0] ^= (i % 2) << 7  # Excluded hands-off bit cannot simulate counter progress.
    h.safety.safety_rx_hook(packet(raw))
    assert h.allowed()
  advance(h, 1)
  h.safety.safety_rx_hook(packet(CAPTURE))
  assert not h.allowed()
  assert not h.safety.test_sp_status_ready()
  # Renewed heartbeats and unchanged captured data cannot clear frozen status.
  h.safety.test_sp_platform(True)
  h.button(False)
  h.button(True)
  assert not h.allowed()


def test_isolated_duplicate_does_not_grant_or_immediately_revoke():
  h = setup_stream()
  advance(h, 10000)
  h.safety.safety_rx_hook(packet(CAPTURE))
  assert h.allowed()  # Duplicate does not renew the progress timestamp.
  h.safety.safety_lateral_revoke(3)
  h.safety.test_sp_platform(True)
  h.safety.safety_rx_hook(packet(CAPTURE))
  assert not h.allowed()


@pytest.mark.parametrize("order", ["original", "reversed", "alternating"])
def test_changing_old_sequence_is_not_detectable_as_replay(order):
  # An actual 64-frame / ~1.9-second old sequence with its recorded intervals,
  # shifted to new receive times. No ECU session token exists to distinguish it.
  recorded = json.loads((Path(__file__).parent / "data/ford_3cc_sequence.json").read_text())["frames"]
  sequence = [bytes.fromhex(frame["payload"]) for frame in recorded]
  assert all(libsafety_py.libsafety.test_sp_status_checksum(packet(p)) for p in sequence)
  if order == "reversed":
    sequence.reverse()
  elif order == "alternating":
    sequence = sequence[:2]
  h = setup_stream()
  previous_us = 0
  for i, frame in enumerate(recorded):
    advance(h, frame["relative_us"] - previous_us)
    h.safety.safety_rx_hook(packet(sequence[i % len(sequence)]))
    assert h.allowed()  # Documented blocker: checksum + progress != authentication/order.
    previous_us = frame["relative_us"]
  # The same stream cannot grant permission after reset or without TJA intent.
  h.safety.test_sp_configure(True)
  for i in range(3):
    advance(h, 30000)
    h.safety.safety_rx_hook(packet(sequence[i % len(sequence)]))
    assert not h.allowed()


def test_expired_progress_checked_before_changed_frame_and_recovery_requires_tja():
  h = setup_stream()
  advance(h, 100001)
  changed = bytes.fromhex("a8000280bcec0000")
  h.safety.safety_rx_hook(packet(changed))
  h.safety.test_sp_platform(True)
  assert h.safety.test_sp_status_ready()
  assert not h.allowed()
  h.engage()


def test_receive_timer_wrap_uses_unsigned_elapsed():
  h = Harness()
  h.now = 0xfffffff0
  h.safety.set_timer(h.now)
  h.refresh()
  h.engage()
  h.omit_address = 0x3CC
  h.safety.safety_rx_hook(packet(CAPTURE))
  advance(h, 90000)
  assert h.allowed()
  advance(h, 10001)
  assert not h.allowed()


def test_wrong_bus_cannot_renew_counter_progress():
  h = setup_stream()
  for _ in range(11):
    advance(h, 10000)
    h.safety.safety_rx_hook(packet(bytes.fromhex("a8000280bcec0000"), 2))
  assert not h.allowed()


def test_mads_off_does_not_apply_new_status_checks():
  h = Harness(lightning=False)
  h.safety.set_controls_allowed(True)
  h.safety.safety_rx_hook(packet(b"\0" * 8))
  assert h.safety.get_controls_allowed()
  assert not h.safety.test_sp_enabled()
