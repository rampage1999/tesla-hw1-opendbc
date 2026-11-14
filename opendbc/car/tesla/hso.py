"""Legacy Tesla hands-on pause (Tinkla HSO lineage, ported from BogPilot, MIT).

AP1 EPAS reports hands_on_level 0, 1 and 3 (never 2 in logged drives). Tinkla's
HSO pauses path lateral at >= 2 and does NOT drop cruise: the driver steers
through a corner, then lateral eases back in from wherever the wheel is.

`CarState.steeringPressed` stays a lower threshold (level >= 1) so controlsd sees
an override and the UI draws the border, but the actual pause is decided here.

Resume: once hands reach the pause level while engaged, 0x488 stays type NONE
until EPAS reports level 0 for AP1_RESUME_HOLD_S in a row. Any level >= 1
restarts that timer. Disengage clears the state at once, so a re-engage is never
delayed.
"""
from __future__ import annotations

# Lateral pause threshold (TinklaHandsOnLevel).
AP1_HANDS_ON_LEVEL = 2
# CarState.steeringPressed threshold (driver input / grey border).
AP1_DRIVER_INPUT_LEVEL = 1

# Seconds of level 0 required before lateral resumes (0.3 s, shortened from
# Tinkla's 0.5 s numb period after drive review).
AP1_RESUME_HOLD_S = 0.3
AP1_CONTROL_HZ = 100
AP1_RESUME_HOLD_FRAMES = int(round(AP1_RESUME_HOLD_S * AP1_CONTROL_HZ))


def ap1_steering_pressed(hands_on_level) -> bool:
  """True at the lateral-pause threshold; level 1 does not pause."""
  try:
    return int(hands_on_level) >= AP1_HANDS_ON_LEVEL
  except (TypeError, ValueError):
    return False


def ap1_driver_input(hands_on_level) -> bool:
  """CarState.steeringPressed on AP1: any driver torque EPAS reports."""
  try:
    return int(hands_on_level) >= AP1_DRIVER_INPUT_LEVEL
  except (TypeError, ValueError):
    return False


class Ap1DriverYield:
  """Engaged-only hands pause with a resume hold; one update per control step."""

  def __init__(self):
    self.active = False
    self.quiet_frames = 0
    self.resumed = False

  def update(self, enabled: bool, hands_on_level) -> bool:
    self.resumed = False
    if not enabled:
      self.active = False
      self.quiet_frames = 0
      return False
    if ap1_steering_pressed(hands_on_level):
      self.active = True
      self.quiet_frames = 0
    elif self.active:
      if ap1_driver_input(hands_on_level):
        self.quiet_frames = 0
      else:
        self.quiet_frames += 1
        if self.quiet_frames >= AP1_RESUME_HOLD_FRAMES:
          self.active = False
          self.quiet_frames = 0
          self.resumed = True
    return self.active


def ap1_lat_active(lat_active: bool, hands_on_level) -> bool:
  """Path lateral is withheld while hands are at or above the pause threshold."""
  if ap1_steering_pressed(hands_on_level):
    return False
  return bool(lat_active)
