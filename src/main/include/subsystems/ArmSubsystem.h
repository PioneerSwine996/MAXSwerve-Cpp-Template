
#pragma once

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/DigitalInput.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include <frc2/command/Commands.h>

#include "Constants.h"

using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol::can;

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  // void Periodic() override;

  // void setPosition(double Actuator_Angle, double Chain_Motor);
  void setChain_Motor(double Chain_Motor);
  double getChain_Motor();
  int atlimitswitch();

  void setWheel(double Wheel_Speed);
  double getWheel_Speed;

  void setActuator(double Actuator_Angle);
  double getActuator_Angle();
  double getRotation_Encoder();

  void Periodic() noexcept override;

  frc2::CommandPtr zero_arm();
  frc2::CommandPtr level_three();

 private:
  SparkMax Rotation;
  SparkMax Actuator;
  VictorSPX Wheel;
  frc::DigitalInput LimitSwitch;
  SparkAbsoluteEncoder RotationEncoder = Rotation.GetAbsoluteEncoder();
  SparkRelativeEncoder ActuatorEncoder = Actuator.GetEncoder();
};