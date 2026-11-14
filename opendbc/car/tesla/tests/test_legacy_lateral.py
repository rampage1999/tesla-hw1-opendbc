import unittest

from opendbc.car.tesla.engage_ramp import (Ap1EngageSteerRamp, RAMP_MAX_S, RAMP_MIN_S, ramp_duration,
                                           smoothstep)
from opendbc.car.tesla.hso import (AP1_RESUME_HOLD_FRAMES, Ap1DriverYield, ap1_driver_input,
                                   ap1_lat_active, ap1_steering_pressed)


class TestHsoGates(unittest.TestCase):
  def test_pause_threshold(self):
    self.assertFalse(ap1_steering_pressed(0))
    self.assertFalse(ap1_steering_pressed(1))
    self.assertTrue(ap1_steering_pressed(2))
    self.assertTrue(ap1_steering_pressed(3))

  def test_driver_input_threshold(self):
    self.assertFalse(ap1_driver_input(0))
    self.assertTrue(ap1_driver_input(1))

  def test_lat_active_withheld(self):
    self.assertTrue(ap1_lat_active(True, 1))
    self.assertFalse(ap1_lat_active(True, 2))
    self.assertFalse(ap1_lat_active(False, 0))


class TestDriverYield(unittest.TestCase):
  def test_pauses_at_level_two(self):
    y = Ap1DriverYield()
    self.assertTrue(y.update(True, 2))

  def test_resumes_after_quiet_hold(self):
    y = Ap1DriverYield()
    y.update(True, 3)
    for _ in range(AP1_RESUME_HOLD_FRAMES - 1):
      self.assertTrue(y.update(True, 0))
    self.assertFalse(y.update(True, 0))
    self.assertTrue(y.resumed)

  def test_level_one_restarts_quiet_count(self):
    y = Ap1DriverYield()
    y.update(True, 3)
    for _ in range(AP1_RESUME_HOLD_FRAMES - 1):
      y.update(True, 0)
    y.update(True, 1)  # restart
    for _ in range(AP1_RESUME_HOLD_FRAMES - 1):
      self.assertTrue(y.update(True, 0))
    self.assertFalse(y.update(True, 0))

  def test_disengage_clears_immediately(self):
    y = Ap1DriverYield()
    y.update(True, 3)
    self.assertFalse(y.update(False, 3))
    self.assertFalse(y.active)

  def test_resumed_only_on_hold_end(self):
    y = Ap1DriverYield()
    y.update(True, 3)
    y.update(True, 0)
    self.assertFalse(y.resumed)


class TestEngageRamp(unittest.TestCase):
  def test_duration_scaling(self):
    self.assertAlmostEqual(ramp_duration(0.5), RAMP_MIN_S)
    self.assertAlmostEqual(ramp_duration(30), RAMP_MAX_S)
    self.assertAlmostEqual(ramp_duration(-30), RAMP_MAX_S)
    mid = ramp_duration(16)
    self.assertTrue(RAMP_MIN_S < mid < RAMP_MAX_S)

  def test_smoothstep_bounds(self):
    self.assertEqual(smoothstep(-1), 0.0)
    self.assertEqual(smoothstep(2), 1.0)
    self.assertAlmostEqual(smoothstep(0.5), 0.5)

  def test_no_ramp_when_path_inactive(self):
    r = Ap1EngageSteerRamp()
    self.assertEqual(r.update(False, 10.0, 40.0), 40.0)

  def test_starts_at_measured_and_converges(self):
    r = Ap1EngageSteerRamp(dt=0.1)
    first = r.update(True, 5.0, 25.0)
    # the first step already advances t by dt, so it starts just off the measured angle
    self.assertLess(abs(first - 5.0), 1.0)
    out = first
    for _ in range(20):
      out = r.update(True, 5.0, 25.0)
    self.assertAlmostEqual(out, 25.0, places=3)

  def test_small_error_no_ramp(self):
    r = Ap1EngageSteerRamp()
    self.assertEqual(r.update(True, 10.0, 10.5), 10.5)

  def test_monotonic_blend(self):
    r = Ap1EngageSteerRamp(dt=0.05)
    vals = [r.update(True, 0.0, 20.0)]
    for _ in range(30):
      vals.append(r.update(True, 0.0, 20.0))
    self.assertTrue(all(b >= a for a, b in zip(vals, vals[1:])))
    self.assertTrue(max(vals) <= 20.0)


if __name__ == "__main__":
  unittest.main()
