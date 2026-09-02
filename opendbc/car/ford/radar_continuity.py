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
      if self.dropout_since is not None:
        age = max(0.0, t - self.dropout_since)
        predicted = self.last_d_rel + self.last_v_rel * max(0.0, t - (self.last_t or t))
        continuous = (self.eligible_before_dropout and age <= SHADOW_HOLD_TIME_S and
                      abs(d_rel - predicted) <= SHADOW_RANGE_RESIDUAL_M and
                      abs(v_rel - self.last_v_rel) <= SHADOW_VELOCITY_RESIDUAL_MS and
                      abs(y_rel - self.last_y_rel) <= SHADOW_LATERAL_RESIDUAL_M)
        event = "reacquired_continuous" if continuous else "reacquired_new_identity"
        self.valid_since = t if not continuous else self.valid_since
      elif self.valid_since is None:
        self.valid_since = t

      self.dropout_since = None
      self.eligible_before_dropout = False
      self.last_t, self.last_d_rel, self.last_v_rel, self.last_y_rel = t, d_rel, v_rel, y_rel
      return ShadowDecision(event=event)

    if self.dropout_since is None:
      self.dropout_since = t
      established = self.valid_since is not None and (t - self.valid_since) >= SHADOW_ESTABLISH_TIME_S
      self.eligible_before_dropout = established and abs(self.last_y_rel) <= SHADOW_CENTERED_Y_M
      event = "hold_candidate" if self.eligible_before_dropout else "dropout_ineligible"
    else:
      event = None

    age = max(0.0, t - self.dropout_since)
    would_hold = self.eligible_before_dropout and age <= SHADOW_HOLD_TIME_S
    predicted = self.last_d_rel + self.last_v_rel * max(0.0, t - (self.last_t or t)) if would_hold else None
    if self.eligible_before_dropout and age > SHADOW_HOLD_TIME_S:
      event = "hold_expired"
      self.eligible_before_dropout = False
    return ShadowDecision(event=event, would_hold=would_hold, predicted_d_rel=predicted, dropout_age=age)
