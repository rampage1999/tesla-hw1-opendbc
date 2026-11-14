import unittest

from opendbc.car.tesla.blindspot import BSM_HOLD_S, LegacyBlindspot, detected, frames_from_vl_all


def frame(left=0, right=0):
  return {"DAS_blindSpotRearLeft": left, "DAS_blindSpotRearRight": right}


class TestLegacyBlindspot(unittest.TestCase):
  def test_detection_values(self):
    self.assertTrue(detected(1))
    self.assertTrue(detected(2))
    self.assertFalse(detected(0))
    self.assertFalse(detected(3))  # SNA is not a warning
    self.assertFalse(detected(None))
    self.assertFalse(detected("x"))

  def test_hold_after_single_frame(self):
    bsm = LegacyBlindspot()
    left, right = bsm.update([frame(left=1)], 0.4)
    self.assertTrue(left)
    self.assertFalse(right)
    # no further frames: still latched until the hold expires
    # (the hold restarts at BSM_HOLD_S *after* the frame's step, so 1.0 s of aging follows it)
    left, _ = bsm.update([], 0.6)
    self.assertTrue(left)
    left, _ = bsm.update([], 0.5)
    self.assertFalse(left)

  def test_run_extends_hold(self):
    bsm = LegacyBlindspot()
    for _ in range(3):
      bsm.update([frame(right=2)], 0.4)
    self.assertTrue(bsm.right)
    bsm.update([], BSM_HOLD_S - 0.1)
    self.assertTrue(bsm.right)
    bsm.update([], 0.2)
    self.assertFalse(bsm.right)

  def test_sna_does_not_set(self):
    bsm = LegacyBlindspot()
    left, right = bsm.update([frame(left=3, right=3)], 0.4)
    self.assertFalse(left)
    self.assertFalse(right)

  def test_clears_immediately_on_zero_after_hold(self):
    bsm = LegacyBlindspot()
    bsm.update([frame(left=1)], 0.1)
    bsm.update([], BSM_HOLD_S)
    left, _ = bsm.update([frame(left=0)], 0.1)
    self.assertFalse(left)

  def test_frames_from_vl_all(self):
    vl_all = {"AutopilotStatus": {
      "DAS_blindSpotRearLeft": [1, 0],
      "DAS_blindSpotRearRight": [0, 2],
    }}
    frames = frames_from_vl_all(vl_all)
    self.assertEqual(len(frames), 2)
    self.assertEqual(frames[0]["DAS_blindSpotRearLeft"], 1)
    self.assertEqual(frames[1]["DAS_blindSpotRearRight"], 2)

  def test_frames_from_vl_all_empty(self):
    self.assertEqual(frames_from_vl_all({}), [])
    self.assertEqual(frames_from_vl_all({"AutopilotStatus": {}}), [])


if __name__ == "__main__":
  unittest.main()
