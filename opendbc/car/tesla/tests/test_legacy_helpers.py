import unittest

from opendbc.car.structs import CarState
from opendbc.car.tesla.speed_limit import limit_from_map, limit_from_mobileye, posted_limit
from opendbc.car.tesla.stalk import stalk_button_events
from opendbc.car.tesla.steer_fault import LATCHED_ANGLE_ERRORS, steer_fault_temporary

ButtonType = CarState.ButtonEvent.Type


class TestSteerFaultGate(unittest.TestCase):
  def test_latched_errors_are_benign(self):
    for name in LATCHED_ANGLE_ERRORS:
      self.assertFalse(steer_fault_temporary("EAC_INHIBITED", name))

  def test_idle_is_benign(self):
    self.assertFalse(steer_fault_temporary("EAC_INHIBITED", "EAC_ERROR_IDLE"))

  def test_other_errors_warn(self):
    self.assertTrue(steer_fault_temporary("EAC_INHIBITED", "EAC_ERROR_TORQUE"))
    self.assertTrue(steer_fault_temporary("EAC_INHIBITED", None))

  def test_only_inhibited_is_temporary(self):
    self.assertFalse(steer_fault_temporary("EAC_FAULT", "EAC_ERROR_TORQUE"))
    self.assertFalse(steer_fault_temporary("EAC_ENABLED", "EAC_ERROR_TORQUE"))
    self.assertFalse(steer_fault_temporary(None, "EAC_ERROR_TORQUE"))


class TestSpeedLimit(unittest.TestCase):
  def test_mobileye_kph(self):
    # 13 * 5 kph = 65 kph
    self.assertAlmostEqual(limit_from_mobileye(13, "KPH"), 65 * 0.2777778, places=3)

  def test_mobileye_sentinels(self):
    self.assertEqual(limit_from_mobileye(31, "KPH"), 0.0)
    self.assertEqual(limit_from_mobileye(0, "KPH"), 0.0)

  def test_map_units(self):
    self.assertAlmostEqual(limit_from_map(12, 1), 60 * 0.2777778, places=3)
    self.assertAlmostEqual(limit_from_map(12, 0), 60 * 0.44704, places=3)

  def test_map_none(self):
    self.assertEqual(limit_from_map(0, 1), 0.0)

  def test_mobileye_wins(self):
    self.assertAlmostEqual(posted_limit(13, "KPH", 12, 1), 65 * 0.2777778, places=3)

  def test_map_fallback(self):
    self.assertAlmostEqual(posted_limit(31, "KPH", 12, 1), 60 * 0.2777778, places=3)

  def test_nothing_available(self):
    self.assertEqual(posted_limit(31, "KPH", 0, 1), 0.0)


class TestStalkButtons(unittest.TestCase):
  def test_no_change(self):
    self.assertEqual(stalk_button_events(0, 0), [])

  def test_tip_up_press(self):
    ev = stalk_button_events(16, 0)
    self.assertEqual(len(ev), 1)
    self.assertTrue(ev[0].pressed)
    self.assertEqual(ev[0].type, ButtonType.setCruise)

  def test_tip_up_release(self):
    ev = stalk_button_events(0, 16)
    self.assertEqual(len(ev), 1)
    self.assertFalse(ev[0].pressed)
    self.assertEqual(ev[0].type, ButtonType.setCruise)

  def test_pull_is_resume(self):
    ev = stalk_button_events(2, 0)
    self.assertEqual(ev[0].type, ButtonType.resumeCruise)

  def test_push_is_cancel(self):
    ev = stalk_button_events(1, 0)
    self.assertEqual(ev[0].type, ButtonType.cancel)

  def test_full_tip_is_set_cruise(self):
    for value in (4, 8, 32):
      ev = stalk_button_events(value, 0)
      self.assertEqual(ev[0].type, ButtonType.setCruise)

  def test_switch_without_idle_emits_both(self):
    ev = stalk_button_events(32, 16)
    self.assertEqual(len(ev), 2)
    self.assertEqual([e.pressed for e in ev], [False, True])


if __name__ == "__main__":
  unittest.main()
