import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.engage_ramp import Ap1EngageSteerRamp
from opendbc.car.tesla.hso import Ap1DriverYield, ap1_lat_active
from opendbc.car.tesla.long_smooth import Ap1AccelSmoother, ap1_brake_urgent, ap1_fcw
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.tesla.coop_steering import CoopSteeringCarController


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase, CoopSteeringCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    CoopSteeringCarController.__init__(self)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1,):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}
      self.yield_ = Ap1DriverYield()
      self.steer_ramp = Ap1EngageSteerRamp(dt=0.02)
      self.accel_smoother = Ap1AccelSmoother()
      self.tesla_can = TeslaCANRaven(self.packers)
      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CC_SP, CS, now_nanos):
    CoopSteeringCarController.update(self, self.CP_SP)
    actuators = CC.actuators
    can_sends = []

    # Tesla EPS enforces disabling steering on heavy lateral override force.
    # When enabling in a tight curve, we wait until user reduces steering force to start steering.
    # Canceling is done on rising edge and is handled generically with CC.cruiseControl.cancel
    if self.CP.carFingerprint in LEGACY_CARS:
      # HSO: hands at level >= 2 withhold the planner angle without dropping cruise
      self.yield_.update(CC.enabled, CS.hands_on_level)
      lat_active = ap1_lat_active(CC.latActive, CS.hands_on_level)
    else:
      lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      if self.CP.carFingerprint in LEGACY_CARS:
        # Blend from the measured wheel to the planner angle after engage / hands pause
        desired_angle = self.steer_ramp.update(lat_active, CS.out.steeringAngleDeg, actuators.steeringAngleDeg)
      else:
        desired_angle = actuators.steeringAngleDeg
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(desired_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active, self.coop_steering.control_type))

    if self.frame % 10 == 0 and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1,):
      cntr = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(cntr))

    # Comfort-band accel slew: AP1's brake pedal is friction-only, regen tops out ~1-1.5 m/s^2
    smoothed_accel = actuators.accel
    if self.CP.openpilotLongitudinalControl and self.CP.carFingerprint in LEGACY_CARS:
      smoothed_accel = self.accel_smoother.update(actuators.accel, CC.longActive, CS.out.vEgo,
                                                  gas_neutral=CS.out.gasPressed,
                                                  urgent=ap1_brake_urgent(CC.hudControl.visualAlert,
                                                                          CC.actuators.longControlState),
                                                  fcw=ap1_fcw(CC.hudControl.visualAlert),
                                                  a_ego=CS.out.aEgo)

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(smoothed_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        if self.CP.carFingerprint in LEGACY_CARS:
          can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.out.gasPressed))
        else:
          can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        if self.CP.carFingerprint in LEGACY_CARS:
          can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False, CS.out.gasPressed))
        else:
          can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
