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

frc2::CommandPtr ArmSubsystem::zero_arm() {
    return frc2::cmd::Sequence(frc2::cmd::Run(
        [this] {
            setActuator(0.2);
        }
    ).Until(
        [this] {
            return atlimitswitch();
        }),
    frc2::cmd::Run([this] {
        setActuator(-0.1);
    })
    .Until([this] {return !atlimitswitch();}),
    frc2::cmd::RunOnce([this] {
        ActuatorEncoder.SetPosition(0);
     }));
}

frc2::CommandPtr ArmSubsystem::level_three() {
    return frc2::cmd::Sequence(frc2::cmd::Run(
        [this] {
            setChain_Motor(-0.07);
        }
    ).Until(
        [this] {
            return getRotation_Encoder() == 0.1600;
        }),
    frc2::cmd::Run([this] {
        setChain_Motor(0.02);
    })
    .Until([this] {
        return getRotation_Encoder() == 0.1600;
    }));
}
