"""AP1/legacy Tesla blind-spot flags from the stock DAS AutopilotStatus frame (0x399).

Ported from BogPilot (github.com/BogPilot/openpilot, `selfdrive/car/tesla/blindspot.py`,
MIT) and adapted to this tree's opendbc layout.

The stock DAS sends `DAS_blindSpotRearLeft` / `DAS_blindSpotRearRight` in 0x399 on the
autopilot bus. Values: 0 NO_WARNING, 1 WARNING_LEVEL_1, 2 WARNING_LEVEL_2, 3 SNA.
1 and 2 mean a car is in the blind spot; 0 and 3 do not. The frame arrives at about
2.4 Hz, so a single warning frame would otherwise flicker for one update only — each
side is latched for BSM_HOLD_S after the last frame that reported it.

Read-only: nothing here is sent to the car.
"""
from __future__ import annotations

# DBC message name in tesla_can.dbc (0x399).
MSG_NAME = "AutopilotStatus"

# Seconds a side stays on after the last frame that reported it.
BSM_HOLD_S = 1.0

BSM_LEFT = "DAS_blindSpotRearLeft"
BSM_RIGHT = "DAS_blindSpotRearRight"
BSM_DETECTED = (1, 2)  # 3 is SNA and must not raise a warning


def detected(value) -> bool:
  try:
    return int(value) in BSM_DETECTED
  except (TypeError, ValueError):
    return False


def frames_from_vl_all(vl_all) -> list[dict]:
  """0x399 frames received this step, oldest first, as {signal: value}."""
  sigs = vl_all.get(MSG_NAME, {}) if hasattr(vl_all, "get") else {}
  if not sigs:
    return []
  n = max((len(v) for v in sigs.values()), default=0)
  out = []
  for i in range(n):
    frame = {}
    for name, vals in sigs.items():
      if i < len(vals):
        frame[name] = vals[i]
    out.append(frame)
  return out


class LegacyBlindspot:
  """Per-side hold (latch-off delay) over the 0x399 blind-spot fields."""

  def __init__(self, hold_s: float = BSM_HOLD_S):
    self.hold_s = hold_s
    self.left_s = 0.0
    self.right_s = 0.0

  @property
  def left(self) -> bool:
    return self.left_s > 0.0

  @property
  def right(self) -> bool:
    return self.right_s > 0.0

  def update(self, frames, dt: float) -> tuple[bool, bool]:
    """Age the holds by dt, then apply this step's frames. Returns (left, right)."""
    self.left_s = max(0.0, self.left_s - dt)
    self.right_s = max(0.0, self.right_s - dt)
    for f in frames:
      if detected(f.get(BSM_LEFT)):
        self.left_s = self.hold_s
      if detected(f.get(BSM_RIGHT)):
        self.right_s = self.hold_s
    return self.left, self.right
