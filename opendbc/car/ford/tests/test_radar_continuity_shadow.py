from opendbc.car.ford.radar_continuity import SteerAssistDropoutShadow


def establish(shadow, y_rel=0.1):
  shadow.update(0.00, 1, 40.0, -2.0, y_rel)
  shadow.update(0.20, 1, 39.6, -2.0, y_rel)


def test_short_centered_dropout_is_shadow_candidate_only():
  shadow = SteerAssistDropoutShadow()
  establish(shadow)
  decision = shadow.update(0.25, 0, 102.2, 0.1, 25.5)
  assert decision.would_hold
  assert decision.predicted_d_rel == 39.5
  # Shadow state never returns or mutates a RadarPoint.
  assert not hasattr(decision, "radar_point")


def test_adjacent_track_is_not_candidate():
  shadow = SteerAssistDropoutShadow()
  establish(shadow, y_rel=1.8)
  assert not shadow.update(0.25, 0, 102.2, 0.1, 25.5).would_hold


def test_extended_dropout_expires():
  shadow = SteerAssistDropoutShadow()
  establish(shadow)
  shadow.update(0.25, 0, 102.2, 0.1, 25.5)
  decision = shadow.update(0.51, 0, 102.2, 0.1, 25.5)
  assert decision.event == "hold_expired"
  assert not decision.would_hold


def test_discontinuous_reacquisition_splits_identity_in_shadow():
  shadow = SteerAssistDropoutShadow()
  establish(shadow)
  shadow.update(0.25, 0, 102.2, 0.1, 25.5)
  decision = shadow.update(0.35, 1, 25.0, -8.0, 0.1)
  assert decision.event == "reacquired_new_identity"
