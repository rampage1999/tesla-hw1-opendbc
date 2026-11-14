"""Legacy Tesla comfort-band accel slew for DAS_control (CarController side only).

Ported from BogPilot (`selfdrive/car/tesla/long_smooth.py`, MIT), adapted to this
tree: no cereal import (enum values are documented constants), DT_CTRL from
opendbc.car.

On 2012-2016 Model S/X the brake pedal is friction-only; regen happens on
accelerator lift and tops out around 1-1.5 m/s^2. The planner's aTarget is the
only source of request jitter and CarController passes it straight through, so
one plan step can swing the request 1-3 m/s^2 — felt as a friction-brake thunk.

This smoother:
- slews rising requests at AP1_RISE_JERK (2.5 m/s^3);
- releases drive torque with a soft S-curve before regen starts, so a lift no
  longer feels like regen grabbing;
- ramps regen onset at AP1_BRAKE_ONSET_JERK (2.0), deepening with the request up
  to the full 8 m/s^3;
- passes urgent requests through on the same step: FCW, LongControl stopping, or
  a request at/below AP1_BRAKE_HARD_ACCEL (-2.0);
- passes standstill/launch/creep through below AP1_SLEW_MIN_SPEED so the
  standstill hold and launch keep their stock shape;
- seeds from the car's measured accel after a driver gas press instead of 0.

ACCEL_MIN/MAX, the panda limits, STOP_DISTANCE and the planner are untouched.
"""
from __future__ import annotations

import math

from opendbc.car import DT_CTRL

# car.capnp enum values (opendbc/car/car.capnp).
_HUD_FCW = 1          # HUDControl.VisualAlert.fcw
_LONG_STOPPING = 2    # Actuators.LongControlState.stopping

AP1_RISE_JERK = 2.5
AP1_DRIVE_RELEASE_JERK = 2.0
AP1_DRIVE_RELEASE_BRAKE_JERK = 5.0
AP1_DRIVE_RELEASE_BRAKE_ACCEL = -0.5
AP1_DRIVE_RELEASE_RAMP_S = 0.2
AP1_DRIVE_RELEASE_JOIN_ACCEL = 0.3
AP1_FALL_JERK = AP1_DRIVE_RELEASE_JERK
AP1_BRAKE_BYPASS_ACCEL = -0.5
AP1_SLEW_MIN_SPEED = 1.0

AP1_FULL_JERK_LIMIT = 8.0
AP1_COMFORT_JERK_LIMIT = 1.5
AP1_JERK_BLEND_START = -0.3
AP1_JERK_LIMIT_NARROW_RATE = 10.0

AP1_BRAKE_ONSET_JERK = 2.0
AP1_BRAKE_URGENT_ACCEL = -1.0
AP1_BRAKE_HARD_ACCEL = -2.0

AP1_GAS_HANDOFF_MAX = 2.0
AP1_LAUNCH_JERK_MAX = 1.5
AP1_STANDSTILL_ENGAGE_ACCEL = -1.4
AP1_STANDSTILL_ENGAGE_V = 0.1
AP1_STANDSTILL_ENGAGE_FRAMES = 10


def ap1_brake_urgent(hud_alert, long_control_state) -> bool:
  """FCW on the HUD, or LongControl in its stopping state (it ramps itself)."""
  return bool(hud_alert == _HUD_FCW or long_control_state == _LONG_STOPPING)


def ap1_fcw(hud_alert) -> bool:
  return bool(hud_alert == _HUD_FCW)


def ap1_brake_ramp_jerk(target) -> float:
  """Regen ramp jerk (m/s^3) for a falling request at or below 0."""
  target = float(target)
  if target >= AP1_BRAKE_URGENT_ACCEL:
    return AP1_BRAKE_ONSET_JERK
  if target <= AP1_BRAKE_HARD_ACCEL:
    return AP1_FULL_JERK_LIMIT
  frac = (AP1_BRAKE_URGENT_ACCEL - target) / (AP1_BRAKE_URGENT_ACCEL - AP1_BRAKE_HARD_ACCEL)
  return AP1_BRAKE_ONSET_JERK + frac * (AP1_FULL_JERK_LIMIT - AP1_BRAKE_ONSET_JERK)


class Ap1AccelSmoother:
  def __init__(self, rise_jerk=AP1_RISE_JERK, fall_jerk=AP1_DRIVE_RELEASE_JERK,
               hard_brake=AP1_BRAKE_HARD_ACCEL, min_speed=AP1_SLEW_MIN_SPEED,
               brake_release_jerk=AP1_DRIVE_RELEASE_BRAKE_JERK, release_ramp_s=AP1_DRIVE_RELEASE_RAMP_S):
    self.rise_jerk = rise_jerk
    self.fall_jerk = fall_jerk
    self.brake_release_jerk = brake_release_jerk
    self.release_ramp_s = release_ramp_s
    self.hard_brake = hard_brake
    self.min_speed = min_speed
    self.reset()

  def reset(self):
    self.last = None
    self.release_j = 0.0
    self.gas_seed = None
    self.was_active = False
    self.engage_hold = None
    self.engage_used = False
    self.active_frames = 0
    self.prev_target = None

  def update(self, target, active, v_ego, gas_neutral=False, urgent=False, dt=DT_CTRL, fcw=False,
             a_ego=None):
    """urgent: FCW or LongControl stopping. A falling request passes through."""
    target = float(target)
    out = self._update(target, active, v_ego, gas_neutral, urgent, dt, fcw, a_ego)
    now_active = active and not gas_neutral
    self.active_frames = self.active_frames + 1 if (now_active and self.was_active) else (1 if now_active else 0)

    first = self.active_frames == 1
    stopping = urgent and not fcw
    arm = 0 < self.active_frames <= AP1_STANDSTILL_ENGAGE_FRAMES and (stopping or (first and not fcw))
    if arm and self.engage_hold is None and v_ego < AP1_STANDSTILL_ENGAGE_V and target <= 0.0 and not self.engage_used:
      self.engage_used = True
      self.engage_hold = AP1_STANDSTILL_ENGAGE_ACCEL
    elif self.engage_hold is not None and (not active or gas_neutral or fcw or (not stopping and not first) or
                                           v_ego >= AP1_STANDSTILL_ENGAGE_V or
                                           target <= self.engage_hold or
                                           (self.prev_target is not None and target > self.prev_target + 1e-3)):
      self.engage_hold = None
    if self.engage_hold is not None:
      out = min(out, self.engage_hold)
      self.last = out
    if not now_active:
      self.engage_used = False
    self.was_active = now_active
    self.prev_target = target if active else None
    return out

  def _update(self, target, active, v_ego, gas_neutral, urgent, dt, fcw, a_ego):
    if gas_neutral:
      self.last = 0.0
      self.release_j = 0.0
      if a_ego is not None and v_ego >= self.min_speed:
        self.gas_seed = min(max(float(a_ego), 0.0), AP1_GAS_HANDOFF_MAX)
      else:
        self.gas_seed = None
      return 0.0
    if not active:
      self.last = None
      self.gas_seed = None
      return target
    if self.gas_seed is not None:
      if v_ego >= self.min_speed and not fcw:
        self.last = self.gas_seed
      self.gas_seed = None
    if self.last is None or v_ego < self.min_speed:
      self.last = target
      return target
    if target >= self.last:
      self.release_j = 0.0
      out = min(target, self.last + self.rise_jerk * dt)
    elif urgent or target <= self.hard_brake:
      self.release_j = 0.0
      out = target
    else:
      jerk = ap1_brake_ramp_jerk(target)
      if self.last > 0.0:
        jerk = self._release_jerk(target, jerk, dt)
      else:
        self.release_j = 0.0
      out = max(target, self.last - jerk * dt)
    self.last = out
    return out

  def _release_jerk(self, target, brake_jerk, dt):
    frac = min(max(target / AP1_DRIVE_RELEASE_BRAKE_ACCEL, 0.0), 1.0)
    cap = self.fall_jerk + frac * max(self.brake_release_jerk - self.fall_jerk, 0.0)
    if target < AP1_BRAKE_URGENT_ACCEL or self.release_ramp_s <= 0.0:
      j = max(cap, brake_jerk)
    else:
      if target < 0.0:
        cap = max(cap, brake_jerk)
        join = min(self.last / AP1_DRIVE_RELEASE_JOIN_ACCEL, 1.0)
        cap = brake_jerk + join * (cap - brake_jerk)
      snap = cap / self.release_ramp_s
      j = min(cap, self.release_j + snap * dt)
      if target >= 0.0:
        j = min(j, math.sqrt(2.0 * snap * (self.last - target)))
      j = max(j, snap * dt)
    self.release_j = j
    return j


def ap1_jerk_limit_target(accel) -> float:
  """Jerk-field magnitude for a request, before the narrowing rate limit."""
  accel = float(accel)
  if accel <= AP1_BRAKE_BYPASS_ACCEL:
    return AP1_FULL_JERK_LIMIT
  if accel >= AP1_JERK_BLEND_START:
    return AP1_COMFORT_JERK_LIMIT
  frac = (AP1_JERK_BLEND_START - accel) / (AP1_JERK_BLEND_START - AP1_BRAKE_BYPASS_ACCEL)
  return AP1_COMFORT_JERK_LIMIT + frac * (AP1_FULL_JERK_LIMIT - AP1_COMFORT_JERK_LIMIT)


class Ap1JerkLimit:
  """DAS_jerkMin / DAS_jerkMax magnitudes for the 0x2b9 frame."""

  def __init__(self):
    self.last = AP1_FULL_JERK_LIMIT

  def update(self, accel, active, v_ego, gas_neutral=False, urgent=False, dt=DT_CTRL, fcw=False):
    if active and not gas_neutral and not fcw and v_ego < AP1_SLEW_MIN_SPEED:
      self.last = AP1_FULL_JERK_LIMIT
      return -AP1_FULL_JERK_LIMIT, AP1_LAUNCH_JERK_MAX
    if gas_neutral or urgent or not active or v_ego < AP1_SLEW_MIN_SPEED:
      target = AP1_FULL_JERK_LIMIT
    else:
      target = ap1_jerk_limit_target(accel)
    if target >= self.last:
      self.last = target
    else:
      self.last = max(target, self.last - AP1_JERK_LIMIT_NARROW_RATE * dt)
    return -self.last, self.last
