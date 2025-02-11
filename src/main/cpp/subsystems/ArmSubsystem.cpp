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
      LimitSwitch{kLimitSwitch}
{
        ActuatorEncoder.SetPosition(0);
          m_ActuatorFeedback.SetTolerance(0.01);
          m_RotationFeedback.SetTolerance(0.01);
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
    m_ActuatorFeedback.SetP(frc::SmartDashboard::GetNumber("P", 0));
    m_ActuatorFeedback.SetI(frc::SmartDashboard::GetNumber("I", 0));
    m_ActuatorFeedback.SetD(frc::SmartDashboard::GetNumber("D", 0));   
    m_RotationFeedback.SetP(frc::SmartDashboard::GetNumber("P", 0));   
    m_RotationFeedback.SetI(frc::SmartDashboard::GetNumber("I", 0));   
    m_RotationFeedback.SetD(frc::SmartDashboard::GetNumber("D", 0));        
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

frc2::CommandPtr ArmSubsystem::to_position(double Actuator_Target, double Rotation_Target) {
    return frc2::cmd::Parallel(
             // Run the shooter flywheel at the desired setpoint using
             // feedforward and feedback
             Run([this, Actuator_Target, Rotation_Target] {
               Actuator.SetVoltage(
                //    m_shooterFeedforward.Calculate(setpoint) +
                   units::volt_t(m_ActuatorFeedback.Calculate(
                        getActuator_Angle(), Actuator_Target)));
               Rotation.SetVoltage(
                //    m_shooterFeedforward.Calculate(setpoint) +
                   units::volt_t(m_RotationFeedback.Calculate(
                        getRotation_Encoder(), Rotation_Target)));
             }));
    }
