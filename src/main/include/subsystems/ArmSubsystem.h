
#pragma once

#include <array>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/DigitalInput.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/controller/PIDController.h>

#include <frc2/command/Commands.h>

#include <frc/Joystick.h>

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

  frc2::CommandPtr Raise();
  frc2::CommandPtr Lower(); 

  frc2::CommandPtr zero_arm(double rotation);
  frc2::CommandPtr to_position(frc::Joystick *joy);

 private:
 bool zeroed = false;
 int state = 0;
  SparkMax Rotation;
  SparkMax Actuator;
  VictorSPX Wheel;
  frc::DigitalInput LimitSwitch;
  SparkAbsoluteEncoder RotationEncoder = Rotation.GetAbsoluteEncoder();
  SparkRelativeEncoder ActuatorEncoder = Actuator.GetEncoder();
  frc::PIDController m_RotationFeedback{40, 0.0, 0.0};
  frc::PIDController m_ActuatorFeedback{3, 0.0, 0.0};
};