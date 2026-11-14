"""Legacy Tesla (Model S/X HW1-HW3) steer-fault gate.

Ported from BogPilot (`selfdrive/car/tesla/steer_fault.py`, MIT), adapted to this
tree's opendbc car-state.

Tinkla set steerWarning unless the EPAS error name was EAC_ERROR_IDLE. On AP1 the
EPAS latches EAC_ERROR_HANDS_ON (3) and EAC_ERROR_HIGH_ANGLE_REQ (6) after the
driver lets go; warning on those keeps lat_active false, so 0x488 never returns to
ANGLE and lateral control stays dead until a reboot. Both are treated as benign
here (the hands pause is handled separately by hands_on_level, not by this flag).
Every other non-idle error still raises the temporary fault, matching Tinkla.
EAC_FAULT is a permanent fault and is not decided here.
"""
from __future__ import annotations

# Error names that must not raise steerFaultTemporary on legacy Tesla.
BENIGN_LEGACY_ERRORS = (
  "EAC_ERROR_IDLE",
  "EAC_ERROR_HIGH_ANGLE_REQ",
  "EAC_ERROR_HANDS_ON",
)

# Latched names that should be commanded as ANGLE again once hands drop below 2.
LATCHED_ANGLE_ERRORS = (
  "EAC_ERROR_HIGH_ANGLE_REQ",
  "EAC_ERROR_HANDS_ON",
)


def steer_fault_temporary(eac_status: str | None, eac_error: str | None,
                          benign=BENIGN_LEGACY_ERRORS) -> bool:
  """True when this EPAS state should set CarState.steerFaultTemporary."""
  if eac_status != "EAC_INHIBITED":
    return False
  if eac_error is None:
    # unknown name: warn, matching Tinkla
    return True
  return eac_error not in benign
