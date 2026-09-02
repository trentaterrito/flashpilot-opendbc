from dataclasses import dataclass


# Shadow-only candidate values. These do not affect published RadarData.
SHADOW_HOLD_TIME_S = 0.25
SHADOW_ESTABLISH_TIME_S = 0.15
SHADOW_CENTERED_Y_M = 1.0
SHADOW_RANGE_RESIDUAL_M = 2.5
SHADOW_VELOCITY_RESIDUAL_MS = 2.0
SHADOW_LATERAL_RESIDUAL_M = 0.75


@dataclass
class ShadowDecision:
  event: str | None = None
  would_hold: bool = False
  predicted_d_rel: float | None = None
  dropout_age: float = 0.0
  reason: str | None = None
  range_residual: float | None = None
  velocity_residual: float | None = None
  lateral_residual: float | None = None


class SteerAssistDropoutShadow:
  """Evaluates an RB5T hold candidate without changing the radar output."""

  def __init__(self):
    self.last_t: float | None = None
    self.last_d_rel = 0.0
    self.last_v_rel = 0.0
    self.last_y_rel = 0.0
    self.valid_since: float | None = None
    self.dropout_since: float | None = None
    self.eligible_before_dropout = False

  def update(self, t: float, confidence: int, d_rel: float, v_rel: float, y_rel: float) -> ShadowDecision:
    if confidence > 0:
      event = None
      age = 0.0
      predicted = None
      reason = None
      range_residual = None
      velocity_residual = None
      lateral_residual = None
      if self.dropout_since is not None:
        age = max(0.0, t - self.dropout_since)
        predicted = self.last_d_rel + self.last_v_rel * max(0.0, t - (self.last_t or t))
        range_residual = abs(d_rel - predicted)
        velocity_residual = abs(v_rel - self.last_v_rel)
        lateral_residual = abs(y_rel - self.last_y_rel)
        if not self.eligible_before_dropout:
          reason = "ineligible_before_dropout"
        elif age > SHADOW_HOLD_TIME_S:
          reason = "expired"
        elif range_residual > SHADOW_RANGE_RESIDUAL_M:
          reason = "range_residual"
        elif velocity_residual > SHADOW_VELOCITY_RESIDUAL_MS:
          reason = "velocity_residual"
        elif lateral_residual > SHADOW_LATERAL_RESIDUAL_M:
          reason = "lateral_residual"
        else:
          reason = None
        continuous = reason is None
        event = "reacquired_continuous" if continuous else "reacquired_new_identity"
        self.valid_since = t if not continuous else self.valid_since
      elif self.valid_since is None:
        self.valid_since = t

      self.dropout_since = None
      self.eligible_before_dropout = False
      self.last_t, self.last_d_rel, self.last_v_rel, self.last_y_rel = t, d_rel, v_rel, y_rel
      return ShadowDecision(event=event, dropout_age=age, predicted_d_rel=predicted, reason=reason,
                            range_residual=range_residual, velocity_residual=velocity_residual,
                            lateral_residual=lateral_residual)

    if self.dropout_since is None:
      self.dropout_since = t
      established = self.valid_since is not None and (t - self.valid_since) >= SHADOW_ESTABLISH_TIME_S
      self.eligible_before_dropout = established and abs(self.last_y_rel) <= SHADOW_CENTERED_Y_M
      event = "hold_candidate" if self.eligible_before_dropout else "dropout_ineligible"
      reason = None if self.eligible_before_dropout else ("not_established" if not established else "not_centered")
    else:
      event = None
      reason = None

    age = max(0.0, t - self.dropout_since)
    would_hold = self.eligible_before_dropout and age <= SHADOW_HOLD_TIME_S
    predicted = self.last_d_rel + self.last_v_rel * max(0.0, t - (self.last_t or t)) if would_hold else None
    if self.eligible_before_dropout and age > SHADOW_HOLD_TIME_S:
      event = "hold_expired"
      reason = "expired"
      self.eligible_before_dropout = False
    return ShadowDecision(event=event, would_hold=would_hold, predicted_d_rel=predicted, dropout_age=age, reason=reason)
