// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <cmath>
#include <ctime>
#include <iostream>
#include <utility>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "DriveToTag.h"

time_t start;

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  start = std::time(0);

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        
        bool slow_mode = m_driverController.GetRawButton(1);

        double y = m_driverController.GetY();
        double x = m_driverController.GetX();

        double angle_rads = -m_drive.GetHeading().value() * M_PI / 180.0;
        if (slow_mode) {
          nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",1);
          angle_rads = 0;
        } else {
          
          nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);
        }

        double c = std::cos(angle_rads);
        double s = std::sin(angle_rads);

        double y_rot = c * y - s * x;
        double x_rot = s * y + c * x;


        double x_factor = 1;
        double y_factor = 1;
        double theta_factor = 1;

        if (slow_mode) {
          x_factor = 0.3;
          y_factor = 0.3;
          theta_factor = 0.5;
        }

        m_drive.Drive(units::meters_per_second_t{y_factor*frc::ApplyDeadband(
                          y_rot, OIConstants::kDeadband)},
                      units::meters_per_second_t{
                          x_factor*frc::ApplyDeadband(x_rot, OIConstants::kDeadband)},
                      units::radians_per_second_t{
                          theta_factor*frc::ApplyDeadband(-m_driverController.GetRawAxis(3),
                                             OIConstants::kDriveDeadband)},
                      false);
        frc::SmartDashboard::PutNumber("Pose X", m_drive.GetPose().X().value());
        frc::SmartDashboard::PutNumber("Pose Y", m_drive.GetPose().Y().value());
        frc::SmartDashboard::PutNumber("Gyro yaw",
                                       m_drive.GetHeading().value());
      },
      {&m_drive}));

  m_arm.SetDefaultCommand(
      frc2::cmd::Parallel(m_arm.zero_arm(0.250).AndThen(m_arm.to_position())));
  m_arm.setWheel(-0.05);

  m_vision.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        frc::SmartDashboard::PutNumber("tx", m_vision.GetX());
        frc::SmartDashboard::PutNumber("tz", m_vision.GetZ());
        frc::SmartDashboard::PutNumber("ry", m_vision.GetYaw());
      },
      {&m_vision}));
}

/*
L3: 0, 0.09
L2: 32, 0.22
*/

void RobotContainer::ConfigureButtonBindings() {
  // frc2::JoystickButton(&m_driverController, 1)
  //     .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  frc2::JoystickButton(&m_driverController, 3).OnTrue(std::move(m_arm.Raise()));

  frc2::JoystickButton(&m_driverController, 5).OnTrue(std::move(m_arm.Lower()));
  //   frc2::JoystickButton(&m_driverController, 3)
  //      .OnTrue(std::move(m_arm.to_position(0, 0.108)));

  //   frc2::JoystickButton(&m_driverController, 5)
  //      .OnTrue(std::move(m_arm.to_position(0, 0.347)));

  frc2::JoystickButton(&m_driverController, 4)
      .WhileTrue(
          new frc2::RunCommand([this] { m_arm.setWheel(-0.6); }, {&m_arm}))
      .OnFalse(
          new frc2::RunCommand([this] { m_arm.setWheel(-0.05); }, {&m_arm}));

  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(
          new frc2::RunCommand([this] { m_arm.setWheel(0.6); }, {&m_arm}))
      .OnFalse(
          new frc2::RunCommand([this] { m_arm.setWheel(-0.05); }, {&m_arm}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
//   auto forward = new frc2::RunCommand(
//       [this] {
//         m_drive.start_timer();
//         if (m_drive.get_timer() < 1.0) {
//           m_drive.Drive(-0.3_mps, 0_mps, 0_rad_per_s, false);
//         } else {
//           m_drive.Drive(0.0_mps, 0_mps, 0_rad_per_s, false);
//         }
//       },
//       {&m_drive});

//   return forward;
    return new DriveToTagCommand(
        &m_drive, &m_vision
    );
}
