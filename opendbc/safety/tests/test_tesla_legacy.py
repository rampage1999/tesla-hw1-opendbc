#!/usr/bin/env python3
# Smoke tests for the teslaLegacy panda safety model (Model S/X HW1/HW2/HW3).
#
# The HW1 port added safety/modes/tesla_legacy.h without a dedicated test
# class (test_tesla.py only covers the Model 3 "tesla" safety). These tests
# exercise the safety-critical invariants directly through libsafety:
# TX allowlisting, steering permission, stock-LKAS/AEB interlocks, HW1
# ignition detection via GTW_status, and bus forwarding.
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.libsafety.libsafety_py import make_CANPacket

SAFETY_TESLA_LEGACY = CarParams.SafetyModel.teslaLegacy

TESLA_FLAG_EXTERNAL_PANDA = 4
TESLA_FLAG_HW1 = 8
TESLA_FLAG_HW2 = 16
TESLA_FLAG_HW3 = 32

MSG_ESP_private1 = 0x155
MSG_BrakeMessage = 0x20a
MSG_DI_torque1_hw1 = 0x108
MSG_GTW_status = 0x348
MSG_EPAS_sysStatus = 0x370
MSG_DI_state = 0x368
MSG_DAS_control_hw1 = 0x2b9
MSG_DAS_control_pt = 0x2bf
MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27D

ANGLE_DEG_TO_CAN = 10  # 1/10 deg per unit
INACTIVE_ACCEL = 375


def steer_msg(angle_10th_deg: int, control_type: int, bus: int = 0):
  raw = angle_10th_deg + 16384
  assert 0 <= raw <= 0x7FFF
  dat = bytearray([0, 0, 0, 0])
  dat[0] = (raw >> 8) & 0x7F
  dat[1] = raw & 0xFF
  dat[2] = (control_type << 6) & 0xFF
  return make_CANPacket(MSG_DAS_steeringControl, bus, bytes(dat))


def das_control_hw1_msg(raw_accel_max: int = INACTIVE_ACCEL, raw_accel_min: int = INACTIVE_ACCEL, aeb_event: int = 0):
  dat = bytearray(8)
  dat[2] = aeb_event & 0x3
  dat[5] = (((raw_accel_max & 0xF) << 4) | ((raw_accel_min >> 5) & 0xF)) & 0xFF
  dat[4] = ((raw_accel_min & 0x1F) << 3) & 0xFF
  dat[6] = (raw_accel_max >> 4) & 0x1F
  return make_CANPacket(MSG_DAS_control_hw1, 0, bytes(dat))


class TeslaLegacyTestBase(unittest.TestCase):
  PARAM: int = 0

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.t = 0
    self._reinit()

  def _reinit(self):
    self.safety.set_safety_hooks(SAFETY_TESLA_LEGACY, self.PARAM)
    self.safety.init_tests()

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _fwd(self, bus, addr):
    # returns -1 when blocked, else the destination bus (0->2, 2->0)
    return self.safety.safety_fwd_hook(bus, addr)

  def _advance(self, micros: int = 100_000):
    # advance the injected microsecond timer well past any real-time window
    self.t += micros
    self.safety.set_timer(self.t)


class TestTeslaLegacyHW1(TeslaLegacyTestBase):
  PARAM = TESLA_FLAG_HW1

  def test_registered_and_param(self):
    self.assertEqual(int(SAFETY_TESLA_LEGACY), 39)
    self.assertEqual(self.safety.get_current_safety_mode(), 39)
    self.assertEqual(self.safety.get_current_safety_param(), self.PARAM)

  def test_tx_allowlist_blocks_unlisted(self):
    self.safety.set_controls_allowed(True)
    # anything outside {0x488, 0x2b9} must be blocked on bus 0
    self.assertFalse(self._tx(make_CANPacket(0x123, 0, bytes(8))))
    # powertrain DAS_control (0x2bf) is not in the HW1 TX list
    self.assertFalse(self._tx(make_CANPacket(MSG_DAS_control_pt, 0, bytes(8))))

  def test_steering_blocked_without_controls(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(steer_msg(100, 1)))  # ANGLE_CONTROL while not allowed

  def test_steering_allowed_with_controls_small_steps(self):
    self.safety.set_controls_allowed(True)
    for angle in (0, 50, 100, 150):
      self._advance()
      self.assertTrue(self._tx(steer_msg(angle, 1)), f"angle {angle}")

  def test_inactive_steer_cmd_allowed_without_controls(self):
    self.safety.set_controls_allowed(False)
    self._advance()
    self.assertTrue(self._tx(steer_msg(0, 0)))  # NONE + 0 deg is always safe

  def test_invalid_steer_control_type_blocked(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertFalse(self._tx(steer_msg(0, 2)))   # LANE_KEEP_ASSIST not allowed from us
    self._advance()
    self.assertFalse(self._tx(steer_msg(0, 3)))   # reserved

  def test_angle_over_epas_fault_limit_blocked(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertFalse(self._tx(steer_msg(4000, 1)))  # > 360 deg faults the EPAS

  def test_stock_lkas_blocks_steering(self):
    self.safety.set_controls_allowed(False)
    # rising edge of LANE_KEEP_ASSIST while not controlling arms the stock-LKAS interlock
    self.assertTrue(self._rx(steer_msg(0, 2, bus=2)))
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertFalse(self._tx(steer_msg(0, 1)))

  def test_aeb_event_in_das_control_blocked(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertFalse(self._tx(das_control_hw1_msg(aeb_event=1)))

  def test_negative_accel_blocked(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertTrue(self._tx(das_control_hw1_msg()))  # both inactive: allowed
    self._advance()
    # both accel limits below inactive (0 m/s^2) would reverse the car after standstill
    self.assertFalse(self._tx(das_control_hw1_msg(raw_accel_max=300, raw_accel_min=300)))

  def test_accel_over_max_blocked(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertTrue(self._tx(das_control_hw1_msg(raw_accel_max=425, raw_accel_min=375)))   # 2 m/s^2
    self._advance()
    self.assertFalse(self._tx(das_control_hw1_msg(raw_accel_max=430, raw_accel_min=375)))  # > 2 m/s^2

  def test_ignition_via_gtw_status(self):
    # ignition_can_hook runs at the panda CAN-driver layer (fdcan.h), not in
    # safety_rx_hook, so call it directly like the gm/mazda/rivian tests do
    self.assertFalse(self.safety.get_ignition_can())

    def gtw(counter, drive):
      dat = bytearray(8)
      dat[0] = 1 if drive else 0
      dat[6] = counter & 0xF
      return make_CANPacket(MSG_GTW_status, 0, bytes(dat))

    # counter must increment between frames (the hook's static prev_counter may
    # be seeded from a prior test), so drive a sequence until ignition sets
    got_ignition = False
    for i in range(20):
      self.safety.ignition_can_hook(gtw(i % 16, drive=True))
      if self.safety.get_ignition_can():
        got_ignition = True
        break
    self.assertTrue(got_ignition, "GTW_status never set ignition")

    for i in range(20):
      self.safety.ignition_can_hook(gtw((i + 0x5A) % 16, drive=False))
      if not self.safety.get_ignition_can():
        break
    self.assertFalse(self.safety.get_ignition_can(), "ignition stayed on with drive bit cleared")

  def test_gas_pressed_from_di_torque1(self):
    self.assertFalse(self.safety.get_gas_pressed_prev())
    dat = bytearray(8)
    dat[6] = 0x11  # non-zero pedal
    self._rx(make_CANPacket(MSG_DI_torque1_hw1, 0, bytes(dat)))
    self.assertTrue(self.safety.get_gas_pressed_prev())

  def test_fwd_hook_hw1(self):
    # openpilot must see EAC monitor and its own steering cmd is blocked from the car bus
    self.assertEqual(self._fwd(2, MSG_APS_eacMonitor), 0)     # not blocked on HW1 (allowed = dest bus 0)
    self.assertEqual(self._fwd(2, MSG_DAS_steeringControl), -1)
    self.assertEqual(self._fwd(2, MSG_DAS_control_hw1), -1)   # we replace it
    self.assertEqual(self._fwd(0, MSG_EPAS_sysStatus), 2)     # allowed = dest bus 2


class TestTeslaLegacyHW2(TeslaLegacyTestBase):
  PARAM = TESLA_FLAG_HW2

  def test_tx_allowlist_hw2(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertTrue(self._tx(steer_msg(0, 1)))
    self.assertTrue(self._tx(make_CANPacket(MSG_APS_eacMonitor, 0, bytes(3))))
    # no longitudinal TX on HW2 (no external panda)
    self.assertFalse(self._tx(make_CANPacket(MSG_DAS_control_hw1, 0, bytes(8))))
    self.assertFalse(self._tx(make_CANPacket(MSG_DAS_control_pt, 0, bytes(8))))

  def test_fwd_hook_hw2(self):
    # EAC monitor must be blocked so the EPS stays asleep for us
    self.assertEqual(self._fwd(2, MSG_APS_eacMonitor), -1)
    self.assertEqual(self._fwd(2, MSG_DAS_steeringControl), -1)
    self.assertEqual(self._fwd(2, MSG_DAS_control_pt), 0)


class TestTeslaLegacyHW3(TeslaLegacyTestBase):
  PARAM = TESLA_FLAG_HW3

  def test_tx_allowlist_hw3(self):
    self.safety.set_controls_allowed(True)
    self._advance()
    self.assertTrue(self._tx(steer_msg(0, 1)))
    self.assertFalse(self._tx(make_CANPacket(MSG_DAS_control_hw1, 0, bytes(8))))

  def test_fwd_hook_hw3(self):
    self.assertEqual(self._fwd(2, MSG_APS_eacMonitor), -1)
    self.assertEqual(self._fwd(2, MSG_DAS_steeringControl), -1)


class TestTeslaLegacyExternalPanda(TeslaLegacyTestBase):
  PARAM = TESLA_FLAG_EXTERNAL_PANDA | TESLA_FLAG_HW2

  def test_eac_monitor_not_blocked_external(self):
    self.assertEqual(self._fwd(2, MSG_APS_eacMonitor), 0)


if __name__ == "__main__":
  unittest.main()
