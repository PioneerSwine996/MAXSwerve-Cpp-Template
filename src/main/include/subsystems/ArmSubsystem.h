
#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/DigitalInput.h>

using namespace rev::spark;

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void Periodic() override;

  void setPosition(double Actuator_Angle, double Chain_Motor);
  void set(double Actuator_Angle, double Chain_Motor);
    double getActuator_Angle();
    double getChain_Motor();
    int atlimitswitch();
 private:
   SparkMax Rotation;
   SparkMax Actuator;
   frc::DigitalInput LimitSwitch;
   SparkAbsoluteEncoder RotationEncoder = Rotation.GetAbsoluteEncoder();
   SparkRelativeEncoder ActuatorEncoder =
      Actuator.GetEncoder();
};