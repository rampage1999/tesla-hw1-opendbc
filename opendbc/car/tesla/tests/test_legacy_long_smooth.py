import unittest

from opendbc.car.tesla.long_smooth import (AP1_BRAKE_HARD_ACCEL, AP1_BRAKE_ONSET_JERK, AP1_COMFORT_JERK_LIMIT,
                                           AP1_FULL_JERK_LIMIT, AP1_JERK_BLEND_START, AP1_RISE_JERK,
                                           AP1_SLEW_MIN_SPEED, AP1_STANDSTILL_ENGAGE_ACCEL, Ap1AccelSmoother,
                                           Ap1JerkLimit, ap1_brake_ramp_jerk, ap1_brake_urgent, ap1_fcw,
                                           ap1_jerk_limit_target)


class TestUrgent(unittest.TestCase):
  def test_fcw_and_stopping(self):
    self.assertTrue(ap1_brake_urgent(1, 1))
    self.assertTrue(ap1_brake_urgent(0, 2))
    self.assertFalse(ap1_brake_urgent(0, 1))
    self.assertTrue(ap1_fcw(1))
    self.assertFalse(ap1_fcw(7))


class TestBrakeRampJerk(unittest.TestCase):
  def test_onset_and_full(self):
    self.assertEqual(ap1_brake_ramp_jerk(-0.5), AP1_BRAKE_ONSET_JERK)
    self.assertEqual(ap1_brake_ramp_jerk(-2.0), AP1_FULL_JERK_LIMIT)
    mid = ap1_brake_ramp_jerk(-1.5)
    self.assertTrue(AP1_BRAKE_ONSET_JERK < mid < AP1_FULL_JERK_LIMIT)


class TestAccelSmoother(unittest.TestCase):
  def test_rise_is_slewed(self):
    s = Ap1AccelSmoother()
    self.assertEqual(s.update(0.0, True, 10.0), 0.0)  # first frame seeds the slew
    out = s.update(2.0, True, 10.0)
    self.assertAlmostEqual(out, AP1_RISE_JERK * 0.01, places=6)

  def test_hard_brake_passes_through(self):
    s = Ap1AccelSmoother()
    s.update(1.0, True, 10.0)
    out = s.update(AP1_BRAKE_HARD_ACCEL, True, 10.0)
    self.assertEqual(out, AP1_BRAKE_HARD_ACCEL)

  def test_urgent_passes_through(self):
    s = Ap1AccelSmoother()
    s.update(1.0, True, 10.0)
    out = s.update(-0.8, True, 10.0, urgent=True)
    self.assertEqual(out, -0.8)

  def test_low_speed_passes_through(self):
    s = Ap1AccelSmoother()
    out = s.update(-2.0, True, AP1_SLEW_MIN_SPEED - 0.1)
    self.assertEqual(out, -2.0)

  def test_regen_onset_is_limited(self):
    s = Ap1AccelSmoother()
    s.update(0.0, True, 10.0)
    out = s.update(-1.0, True, 10.0)
    self.assertGreater(out, -1.0)
    self.assertLess(out, 0.0)

  def test_gas_neutral_resets_to_zero(self):
    s = Ap1AccelSmoother()
    s.update(1.0, True, 10.0)
    out = s.update(1.5, True, 10.0, gas_neutral=True, a_ego=1.2)
    self.assertEqual(out, 0.0)
    # first active frame after the press seeds from measured accel, then rises at the rise jerk
    out2 = s.update(1.5, True, 10.0)
    self.assertAlmostEqual(out2, 1.2 + AP1_RISE_JERK * 0.01, places=6)

  def test_standstill_engage_hold(self):
    s = Ap1AccelSmoother()
    out = s.update(-2.0, True, 0.0, urgent=True)
    self.assertLessEqual(out, AP1_STANDSTILL_ENGAGE_ACCEL)

  def test_inactive_passthrough(self):
    s = Ap1AccelSmoother()
    self.assertEqual(s.update(1.0, False, 10.0), 1.0)

  def test_output_between_last_and_target(self):
    s = Ap1AccelSmoother()
    prev = s.update(0.0, True, 10.0)
    for target in (2.0, 0.5, -0.6, -1.2):
      out = s.update(target, True, 10.0)
      self.assertTrue(min(prev, target) - 1e-9 <= out <= max(prev, target) + 1e-9)
      prev = out


class TestJerkLimit(unittest.TestCase):
  def test_target(self):
    self.assertEqual(ap1_jerk_limit_target(0.0), AP1_COMFORT_JERK_LIMIT)
    self.assertEqual(ap1_jerk_limit_target(-1.0), AP1_FULL_JERK_LIMIT)
    mid = ap1_jerk_limit_target(-0.4)
    self.assertTrue(AP1_COMFORT_JERK_LIMIT < mid < AP1_FULL_JERK_LIMIT)
    self.assertEqual(ap1_jerk_limit_target(AP1_JERK_BLEND_START), AP1_COMFORT_JERK_LIMIT)

  def test_standstill_launch(self):
    j = Ap1JerkLimit()
    lo, hi = j.update(-2.0, True, 0.0)
    self.assertEqual(lo, -AP1_FULL_JERK_LIMIT)
    self.assertAlmostEqual(hi, 1.5)

  def test_narrowing_is_rate_limited(self):
    j = Ap1JerkLimit()
    j.update(-2.0, True, 20.0)  # full
    lo, hi = j.update(0.0, True, 20.0)
    self.assertGreater(hi, AP1_COMFORT_JERK_LIMIT)


if __name__ == "__main__":
  unittest.main()
