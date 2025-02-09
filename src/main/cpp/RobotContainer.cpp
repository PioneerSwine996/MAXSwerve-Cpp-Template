// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <iostream>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"

#include <ctime>

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
        m_drive.Drive(
            units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetY(), OIConstants::kDeadband)},
            units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetX(), OIConstants::kDeadband)},
            units::radians_per_second_t{frc::ApplyDeadband(
                -m_driverController.GetRawAxis(3), OIConstants::kDriveDeadband)},
            true);
            frc::SmartDashboard::PutNumber("Gyro yaw", m_drive.GetHeading().value());
      },
      {&m_drive}));


  m_arm.SetDefaultCommand(frc2::RunCommand(
    [this] {
        // frc::SmartDashboard::PutNumber("At limit switch", m_arm.atlimitswitch());
        // frc::SmartDashboard::PutNumber("Actuator Encoder", m_arm.getActuator_Angle());
        // frc::SmartDashboard::PutNumber("Rotation Encoder", m_arm.getRotation_Encoder());
        
        frc::SmartDashboard::PutNumber("time", std::difftime(std::time(0), start));

        m_arm.setActuator(0.0);
        m_arm.setChain_Motor(0.0);
        m_arm.setWheel(0.0);
    },
    {&m_arm}
  ));

}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,1)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  frc2::JoystickButton(&m_driverController, 3)
      .WhileTrue(new frc2::RunCommand([this] { m_arm.setActuator(-0.3);}, {&m_arm}));
  frc2::JoystickButton(&m_driverController, 4)
      .WhileTrue(new frc2::RunCommand([this] { m_arm.setActuator(0.3);}, {&m_arm}));
//   frc2::JoystickButton(&m_driverController, 2)
//       .WhileTrue(new frc2::RunCommand([this] {m_arm.setWheel(-0.6);}, {&m_arm}));
  frc2::JoystickButton(&m_driverController, 5)
      .WhileTrue(new frc2::RunCommand([this] { m_arm.setChain_Motor(-0.1);}, {&m_arm}));
  frc2::JoystickButton(&m_driverController, 6)
      .WhileTrue(new frc2::RunCommand([this] { m_arm.setChain_Motor(0.1);}, {&m_arm}));

  frc2::JoystickButton(&m_driverController, 7)
     .OnTrue(std::move(m_arm.zero_arm()));
  frc2::JoystickButton(&m_driverController, 2) 
     .OnTrue(std::move(m_arm.zero_arm()));
  frc2::JoystickButton(&m_driverController, 2) 
     .OnTrue(std::move(m_arm.level_three()));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));
}
