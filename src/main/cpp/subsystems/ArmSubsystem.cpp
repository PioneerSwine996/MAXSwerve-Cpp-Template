#include "subsystems/ArmSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>


using namespace ArmConstants;

using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol;

ArmSubsystem::ArmSubsystem()
    : Rotation{kRotationId, SparkMax::MotorType::kBrushless},
      Actuator{kActuatorId, SparkMax::MotorType::kBrushless},
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

double ArmSubsystem::getRotation_Encoder() {
    return RotationEncoder.GetPosition();
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

void ArmSubsystem::Periodic() noexcept {
    frc::SmartDashboard::PutNumber("At limit switch", atlimitswitch());
    frc::SmartDashboard::PutNumber("Actuator Encoder", getActuator_Angle());
    frc::SmartDashboard::PutNumber("Rotation Encoder", getRotation_Encoder());       
}
