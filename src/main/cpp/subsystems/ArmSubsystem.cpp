#include "subsystems/ArmSubsystem.h"
#include "Constants.h"

using namespace ArmConstants;

using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol;

ArmSubsystem::ArmSubsystem()
    : Actuator{kActuatorId, SparkMax::MotorType::kBrushless},
      Rotation{kRotationId, SparkMax::MotorType::kBrushless},
      Wheel{kWheelId},
      LimitSwitch{kLimitSwitch} {
    //
}

int ArmSubsystem::atlimitswitch() {
    return LimitSwitch.Get();
}

double ArmSubsystem::getActuator_Angle() {
    return ActuatorEncoder.GetPosition();

}
void ArmSubsystem::set(double Actuator_Angle, double Chain_Motor) {
    Actuator.Set(Actuator_Angle);
    Rotation.Set(Chain_Motor);
}
void ArmSubsystem::setWheel(double Wheel_Speed){
    Wheel.Set(ControlMode::PercentOutput, Wheel_Speed);
}
