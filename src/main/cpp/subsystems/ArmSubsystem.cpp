#include "subsystems/ArmSubsystem.h"
#include "Constants.h"
using namespace ArmConstants;

using namespace rev::spark;

ArmSubsystem::ArmSubsystem()
    : Actuator{kActuatorId, SparkMax::MotorType::kBrushless},
      Rotation{kRotationId, SparkMax::MotorType::kBrushless},
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