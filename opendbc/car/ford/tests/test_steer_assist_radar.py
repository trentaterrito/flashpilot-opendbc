from collections import deque
from types import SimpleNamespace

import pytest

from opendbc.car.ford.radar_interface import RadarInterface


def tracker(msg):
  ri = object.__new__(RadarInterface)
  ri.pts = {}
  ri.track_id = 0
  ri.v_rel_history = deque(maxlen=20)
  ri.dropout_shadow = None
  ri.rcp = SimpleNamespace(vl={"Steer_Assist_Data": msg})
  return ri


def message(confidence=1, d_rel=40.0, v_rel=-2.0, y_rel=0.2):
  return {
    "CmbbObjConfdnc_D_Stat": confidence,
    "CmbbObjDistLong_L_Actl": d_rel,
    "CmbbObjRelLong_V_Actl": v_rel,
    "CmbbObjDistLat_L_Actl": y_rel,
    "CmbbObjRelLat_V_Actl": 0.0,
  }


def test_valid_point_matches_current_adapter_behavior():
  ri = tracker(message())
  ri._update_steer_assist()
  assert ri.pts[0].trackId == 0
  assert ri.pts[0].dRel == 40.0
  assert ri.pts[0].vRel == -2.0
  assert ri.pts[0].yRel == pytest.approx(0.2)
  assert ri.pts[0].deprecated.measured


def test_not_determined_removes_point_and_never_uses_sentinel():
  ri = tracker(message())
  ri._update_steer_assist()
  ri.rcp.vl["Steer_Assist_Data"] = message(0, 102.2, 0.1, 25.5)
  ri._update_steer_assist()
  assert ri.pts == {}


def test_reacquisition_gets_new_identity():
  ri = tracker(message())
  ri._update_steer_assist()
  ri.rcp.vl["Steer_Assist_Data"] = message(0, 102.2, 0.1, 25.5)
  ri._update_steer_assist()
  ri.rcp.vl["Steer_Assist_Data"] = message(d_rel=39.8)
  ri._update_steer_assist()
  assert ri.pts[0].trackId == 1
