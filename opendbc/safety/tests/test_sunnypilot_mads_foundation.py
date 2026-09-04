"""Characterize the pinned, unmodified sunnypilot core; not vehicle validation."""
import ctypes
import hashlib
import itertools
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def library(tmp_path_factory):
  output = tmp_path_factory.mktemp("sunnypilot_mads") / "core.so"
  cc = shutil.which("cc")
  assert cc is not None, "C compiler required for actual upstream-core tests"
  mode = "-dynamiclib" if sys.platform == "darwin" else "-shared"
  subprocess.run([cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", mode, "-fPIC", "-I", str(ROOT),
                  str(Path(__file__).with_name("sunnypilot_mads_harness.c")), "-o", str(output)], check=True)
  lib = ctypes.CDLL(str(output))
  for name, args, result in (
    ("sp_test_reset", [ctypes.c_bool, ctypes.c_int], None),
    ("sp_test_update", [ctypes.c_bool] * 5, None),
    ("sp_test_allowed", [], ctypes.c_bool),
    ("sp_test_requested", [], ctypes.c_bool),
    ("sp_test_revoke", [ctypes.c_int], None),
    ("sp_test_heartbeat", [ctypes.c_bool], None),
  ):
    fn = getattr(lib, name)
    fn.argtypes, fn.restype = args, result
  return lib


@pytest.fixture
def core(library):
  library.sp_test_reset(True, 2)
  library.sp_test_update(False, False, False, False, False)
  return library


def test_exact_upstream_provenance():
  manifest = json.loads((ROOT / "opendbc/safety/sunnypilot/UPSTREAM.json").read_text())
  assert manifest["commit"] == "f95f996f5917dcbbf2e32fe51b606a24cf836af6"
  for relative, expected in manifest["git_blobs"].items():
    data = (ROOT / relative).read_bytes()
    # Only a documented linkage adaptation is permitted; reverse it before
    # checking the original Git blob. State-machine bodies remain identical.
    if relative.endswith("/mads.h"):
      data = data.replace(b"static inline void mads_heartbeat_engaged_check(void) {",
                          b"inline void mads_heartbeat_engaged_check(void) {")
    elif relative.endswith("/mads_declarations.h"):
      data = data.replace(b"static inline void mads_heartbeat_engaged_check(void);",
                          b"extern void mads_heartbeat_engaged_check(void);")
    blob = hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()
    assert blob == expected, f"Unrecorded upstream modification: {relative}"


def test_disabled_core_never_authorizes(core):
  for mode in (0, 1, 2):
    core.sp_test_reset(False, mode)
    for inputs in itertools.product((False, True), repeat=5):
      core.sp_test_update(*inputs)
      assert not core.sp_test_allowed()


@pytest.mark.parametrize("trigger", [0, 1, 2])
def test_upstream_engagement_sources(core, trigger):
  inputs = [False] * 5
  inputs[trigger] = True
  core.sp_test_update(*inputs)
  assert core.sp_test_allowed()


def test_cruise_disengagement_does_not_disengage_lateral(core):
  core.sp_test_update(True, True, False, False, False)
  assert core.sp_test_allowed()
  core.sp_test_update(True, False, False, False, False)
  assert core.sp_test_allowed()


@pytest.mark.parametrize("veto", ["main_off", "override"])
def test_upstream_vetoes(core, veto):
  core.sp_test_update(True, False, False, False, False)
  core.sp_test_update(veto != "main_off", False, False, False, veto == "override")
  assert not core.sp_test_allowed()


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_upstream_brake_policies_are_explicit(core, mode):
  core.sp_test_reset(True, mode)
  core.sp_test_update(False, False, False, False, False)
  core.sp_test_update(True, False, False, False, False)
  core.sp_test_update(True, False, False, True, False)
  assert core.sp_test_allowed() == (mode == 0)
  core.sp_test_update(True, False, False, False, False)
  assert core.sp_test_allowed() == (mode != 2)


@pytest.mark.parametrize("reason", [1, 2, 4, 8, 16, 32, 64])
def test_explicit_upstream_revocation_clears_authorization(core, reason):
  core.sp_test_update(True, False, False, False, False)
  core.sp_test_revoke(reason)
  assert not core.sp_test_allowed()
  core.sp_test_update(True, False, False, False, False)
  assert not core.sp_test_allowed()


def test_upstream_heartbeat_threshold_is_not_immediate_revocation(core):
  # Characterization, NOT acceptance of a grace policy for FlashPilot.
  core.sp_test_update(True, False, False, False, False)
  for _ in range(2):
    core.sp_test_heartbeat(False)
    assert core.sp_test_allowed()
  core.sp_test_heartbeat(False)
  assert not core.sp_test_allowed()


def test_bss_reset_does_not_preserve_authorization(core):
  core.sp_test_update(True, False, False, False, False)
  core.sp_test_reset(False, 2)
  assert not core.sp_test_allowed()
  assert not core.sp_test_requested()
