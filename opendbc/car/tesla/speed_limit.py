"""Legacy Tesla posted-speed-limit extraction.

Ported from BogPilot (`selfdrive/car/tesla/speed_limit.py`, MIT), adapted to this
tree's opendbc car-state and sunnypilot's `CarStateSP.speedLimit` (m/s).

Order, matching BogPilot:
  1. Mobileye's fused limit from the stock DAS frame `AutopilotStatus` (0x399),
     signal `DAS_fusedSpeedLimit` (5-bit, scale 5). 0 = UNKNOWN_SNA, 31 = NONE.
  2. The car's own map/predictive limit from `GTW_carState`: `UI_mppSpeedLimit`
     (5-bit, scale 5), units from `UI_mapSpeedLimitUnits` (1 = KPH, 0 = MPH).

Both are 0/None when nothing usable is available; the caller keeps its previous
value rather than falling back to 0.
"""
from __future__ import annotations

from opendbc.car.common.conversions import Conversions as CV

# DAS_fusedSpeedLimit sentinel values in tesla_can.dbc VAL_ 921.
MOBILEYE_NONE = 31
MOBILEYE_UNKNOWN = 0

# UI_mppSpeedLimit is a 5-bit signal with scale 5 -> 0..155.
MPP_SCALE = 5
MPP_NONE = 0


def _to_ms(value: float, units: str | None) -> float:
  if units == "KPH":
    return value * CV.KPH_TO_MS
  if units == "MPH":
    return value * CV.MPH_TO_MS
  return 0.0


def limit_from_mobileye(raw: int, units: str | None) -> float:
  """Posted limit in m/s from DAS_fusedSpeedLimit, or 0.0 when unusable."""
  try:
    raw = int(raw)
  except (TypeError, ValueError):
    return 0.0
  if raw in (MOBILEYE_NONE, MOBILEYE_UNKNOWN):
    return 0.0
  # signal scale is 5 kph/mph per count
  return _to_ms(raw * 5, units)


def limit_from_map(mpp_raw: int, units_flag: int | None) -> float:
  """Posted limit in m/s from GTW_carState UI_mppSpeedLimit, or 0.0."""
  try:
    raw = int(mpp_raw)
  except (TypeError, ValueError):
    return 0.0
  if raw <= MPP_NONE:
    return 0.0
  units = "KPH" if units_flag == 1 else "MPH"
  return _to_ms(raw * MPP_SCALE, units)


def posted_limit(mobileye_raw, mobileye_units, mpp_raw, mpp_units_flag) -> float:
  """Mobileye first, then the car's map limit. 0.0 when neither is usable."""
  limit = limit_from_mobileye(mobileye_raw, mobileye_units)
  if limit > 0.0:
    return limit
  return limit_from_map(mpp_raw, mpp_units_flag)
