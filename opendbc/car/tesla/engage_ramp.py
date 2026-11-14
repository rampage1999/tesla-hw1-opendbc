"""Legacy Tesla engage / lateral-resume steering blend (ported from BogPilot, MIT).

When the planner path takes over — at engage, or when lateral comes back after a
hands pause — the commanded angle can jump 150-580 deg/s in one 50 Hz frame. The
only bound was the rate limiter, which allows 5-7 deg per frame toward centre.

This ramp starts at the measured wheel angle on the rising edge of "path active"
and blends to the live planner angle with a smoothstep over 0.3-1.0 s, scaled by
the angle error. Output still passes through the existing rate limit and the
+/-20 deg clip around the measured wheel, so it can only be slower than before
and stays inside the unchanged panda limits.
"""
from __future__ import annotations

RAMP_MIN_S = 0.3
RAMP_MAX_S = 1.0
# Error (deg) -> ramp duration: <= 2 deg -> 0.3 s, >= 30 deg -> 1.0 s.
RAMP_ERR_BP = (2.0, 30.0)
# Below this error there is nothing to blend.
RAMP_MIN_ERR_DEG = 1.0


def _interp(x: float, xp, fp) -> float:
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      t = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + t * (fp[i] - fp[i - 1])
  return fp[-1]


def ramp_duration(err_deg: float) -> float:
  return float(min(max(_interp(abs(err_deg), RAMP_ERR_BP, (RAMP_MIN_S, RAMP_MAX_S)), RAMP_MIN_S), RAMP_MAX_S))


def smoothstep(x: float) -> float:
  x = min(max(x, 0.0), 1.0)
  return x * x * (3.0 - 2.0 * x)


class Ap1EngageSteerRamp:
  def __init__(self, dt: float = 0.02):
    self.dt = dt
    self.active = False
    self.start_deg = 0.0
    self.duration = RAMP_MIN_S
    self.t = 0.0
    self.prev_path = False

  def cancel(self):
    self.active = False

  def update(self, path_active: bool, measured_deg: float, desired_deg: float) -> float:
    """Angle to request this step (before the rate limiter)."""
    if not path_active:
      self.active = False
      self.prev_path = False
      return desired_deg
    if not self.prev_path:
      err = desired_deg - measured_deg
      self.active = abs(err) > RAMP_MIN_ERR_DEG
      self.start_deg = float(measured_deg)
      self.duration = ramp_duration(err)
      self.t = 0.0
    self.prev_path = True
    if not self.active:
      return desired_deg
    self.t += self.dt
    w = smoothstep(self.t / self.duration)
    if self.t >= self.duration:
      self.active = False
    return self.start_deg + (desired_deg - self.start_deg) * w
