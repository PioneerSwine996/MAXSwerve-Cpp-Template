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
        ActuatorEncoder.SetPosition(0);
}

int ArmSubsystem::atlimitswitch() {
    return LimitSwitch.Get();
}

double ArmSubsystem::getActuator_Angle() {
    return ActuatorEncoder.GetPosition();

}
void ArmSubsystem::setActuator(double Actuator_Angle) {
    Actuator.Set(Actuator_Angle);
}
void ArmSubsystem::setChain_Motor(double Chain_Motor){
    Rotation.Set(Chain_Motor);
}
void ArmSubsystem::setWheel(double Wheel_Speed){
    Wheel.Set(ControlMode::PercentOutput, Wheel_Speed);
}
