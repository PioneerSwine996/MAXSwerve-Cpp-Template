#include "subsystems/ArmSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>


using namespace ArmConstants;

using namespace rev::spark;
using namespace ctre::phoenix::motorcontrol;

namespace State {
  double rotation_setpoints[] = {0.315, 0.315, 0.280, 0.312, 0.094};
  double actuator_setpoints[] = {-120, -5, -5, -5, -5};
  int max_state = sizeof(rotation_setpoints)/sizeof(double) - 1;
}

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
}

frc2::CommandPtr ArmSubsystem::zero_arm(double rotation) {
    return frc2::cmd::Sequence(
        frc2::cmd::Run(
            [this] {
                setChain_Motor(-0.3);
            }
        ).Until(
            [this, rotation] {
                return getRotation_Encoder() < rotation;
            }
        ).AndThen(
            [this] {
                setChain_Motor(0.0);
            }
        ), frc2::cmd::Run(
        [this] {
            setActuator(0.4);
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
        zeroed = true;
     })).Until([this] {return zeroed;});
}

frc2::CommandPtr ArmSubsystem::Raise(){
    return RunOnce([this] {
        if (state < State::max_state) {
            state++;
        }
    });
}

frc2::CommandPtr ArmSubsystem::Lower(){
    return RunOnce([this] {
        if (state > 0) {
            state--;
        }
    });
}

frc2::CommandPtr ArmSubsystem::to_position(frc::Joystick *joy) {
    return frc2::cmd::Parallel(
             // Run the shooter flywheel at the desired setpoint using
             // feedforward and feedback
             Run([this, joy] {
                double Rotation_Target = State::rotation_setpoints[state];
                double Actuator_Target = State::actuator_setpoints[state];

                double Rotation_calc = m_RotationFeedback.Calculate(
                        getRotation_Encoder(), Rotation_Target);
                double Actuator_calc = m_ActuatorFeedback.Calculate(
                        getActuator_Angle(), Actuator_Target);
                if (Rotation_calc > 8)
                    Rotation_calc = 8; 
                if (Rotation_calc < -8)
                    Rotation_calc = -8;
                if (Actuator_calc > 6)
                    Actuator_calc = 6; 
                if (Actuator_calc < -6)
                    Actuator_calc = -6;

                Rotation_calc = frc::ApplyDeadband(Rotation_calc, 0.3, 12.0);
                Actuator_calc = frc::ApplyDeadband(Actuator_calc, 0.3, 12.0);

                if (joy->GetRawButton(7)) {
                    state = 1;
                   if (joy->GetRawButton(3)) {
                    Rotation_calc = -2;
                   } else if (joy->GetRawButton(5)) {
                    Rotation_calc = 2;
                   } else {
                    Rotation_calc = 0;
                   }

                   Actuator_calc = 0;
                }


                frc::SmartDashboard::PutNumber("Rotation calc", Rotation_calc);

               Actuator.SetVoltage(
                //    m_shooterFeedforward.Calculate(setpoint) +
                   units::volt_t(Actuator_calc));
               Rotation.SetVoltage(
                //    m_shooterFeedforward.Calculate(setpoint) +
                   units::volt_t(Rotation_calc));
             }));
    }
