
#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol::can;

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  // void Periodic() override;

  // void setPosition(double Actuator_Angle, double Chain_Motor);
  void set(double Actuator_Angle, double Chain_Motor);
    double getActuator_Angle();
  // double getChain_Motor();
    int atlimitswitch();

  void setWheel(double Wheel_Speed);
    double getWheel_Speed;

 private:
   SparkMax Rotation;
   SparkMax Actuator;
   VictorSPX Wheel;
   frc::DigitalInput LimitSwitch;
   SparkAbsoluteEncoder RotationEncoder = Rotation.GetAbsoluteEncoder();
   SparkRelativeEncoder ActuatorEncoder =
      Actuator.GetEncoder();
};