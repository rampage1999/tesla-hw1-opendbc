"""Legacy Tesla cruise-stalk button mapping (Model S/X HW1-HW3).

Ported from BogPilot (`selfdrive/car/tesla/stalk_follow.py` / `stalk_*_hold.py`,
MIT), reduced to the button edges this tree consumes: BogPilot's follow-distance
scroll and personality selection are sunnypilot-side concerns and are not mapped
here.

`SpdCtrlLvr_Stat` (STW_ACTN_RQ, 0x69) in tesla_can.dbc:
  0 IDLE, 1 FWD (push away / cancel), 2 RWD (pull toward driver / resume),
  4 UP_2ND, 8 DN_2ND, 16 UP_1ST, 32 DN_1ST.

Tip and full-tip both map to setCruise; the size of the step (1 mph vs next
multiple of 5) is decided by controlsd/sunnypilot, not here.
"""
from __future__ import annotations

from opendbc.car import create_button_events, structs

ButtonType = structs.CarState.ButtonEvent.Type

STALK_IDLE = 0
STALK_FWD = 1
STALK_RWD = 2

STALK_BUTTONS = {
  1: ButtonType.cancel,          # FWD: cancel / push away
  2: ButtonType.resumeCruise,    # RWD: resume / pull toward driver
  4: ButtonType.setCruise,       # UP_2ND
  8: ButtonType.setCruise,       # DN_2ND
  16: ButtonType.setCruise,      # UP_1ST
  32: ButtonType.setCruise,      # DN_1ST
}


def stalk_button_events(lvr_stat, prev_lvr_stat) -> list:
  """Press/release events for the cruise stalk, or [] when nothing changed."""
  try:
    cur = int(lvr_stat)
    prev = int(prev_lvr_stat)
  except (TypeError, ValueError):
    return []
  return create_button_events(cur, prev, STALK_BUTTONS)
