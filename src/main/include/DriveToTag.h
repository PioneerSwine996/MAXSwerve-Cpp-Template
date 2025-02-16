//=== DriveToTagCommand.h ===
#pragma once

#include <cmath>
#include <algorithm>
#include <iostream>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Command that drives the robot to a point 1 meter from an AprilTag.
 * It computes a translation vector based on the measured distance (from ty)
 * and the horizontal offset (tx) so that the robot drives directly toward the tag.
 * Simultaneously, it rotates the robot until the tag is centered.
 */
class DriveToTagCommand
    : public frc2::CommandHelper<frc2::Command, DriveToTagCommand> {
public:
  explicit DriveToTagCommand(DriveSubsystem* driveSubsystem, VisionSubsystem *vision)
      : m_driveSubsystem(driveSubsystem),
        m_visionSubsystem(vision),
        m_distancePID(0.5, 0.0, 0.1),   // PID gains for distance control (tune these)
        m_rotationPID(0.03, 0.0, 0.005)  // PID gains for rotation control (tune these)
  {
    AddRequirements({driveSubsystem});
    m_distancePID.SetTolerance(0.1);  // 10 cm tolerance for distance
    m_rotationPID.SetTolerance(2.0);  // 2° tolerance for the horizontal offset
  }

  void Initialize() override {
    m_distancePID.Reset();
    m_rotationPID.Reset();
  }

  void Execute() override {
    // If no valid target is detected, stop the robot.
    if (!HasValidTarget()) {
        m_driveSubsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
      return;
    }

    // Get current distance from the tag (meters) using the vertical offset.
    double currentDistance = GetDistanceFromTag();
    // Compute translation speed using the distance PID.
    double distanceSpeed = m_distancePID.Calculate(currentDistance, m_targetDistance);
    distanceSpeed = std::clamp(distanceSpeed, -m_maxSpeed, m_maxSpeed);

    // Get horizontal offset (tx) from the Limelight (degrees).
    double tx = m_visionSubsystem->GetYaw();
    double tx_rad = tx * (M_PI / 180.0);

    // Compute the robot-relative translation vector:
    // The target’s direction (relative to the robot’s forward axis) is given by tx.
    // Decompose the translation command into forward and strafe components.
    double forwardCmd = distanceSpeed * std::cos(tx_rad);
    double strafeCmd  = distanceSpeed * std::sin(tx_rad);

    // Compute a rotation command so the robot turns to center the target (tx => 0).
    double rotationCmd = -m_rotationPID.Calculate(tx, 0.0);
    rotationCmd = std::clamp(rotationCmd, -m_maxRotation, m_maxRotation);

    // Command the swerve drive subsystem.
    m_driveSubsystem->Drive(units::meters_per_second_t{forwardCmd}, units::meters_per_second_t{strafeCmd}, units::radians_per_second_t{rotationCmd}, false);
  }

  void End(bool interrupted) override {
    m_driveSubsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
  }

  bool IsFinished() override {
    // Command finishes when both the distance error and the angle error are within tolerance.
    return m_distancePID.AtSetpoint() && m_rotationPID.AtSetpoint();
  }

private:
  DriveSubsystem* m_driveSubsystem;
  VisionSubsystem* m_visionSubsystem;
  frc::PIDController m_distancePID;
  frc::PIDController m_rotationPID;

  // Target distance from the tag (meters).
  const double m_targetDistance = 1.7;
  const double m_distanceTolerance = 0.1; // meters tolerance for distance
  const double m_maxSpeed = 0.2;          // maximum translation speed (m/s)

  // Maximum rotational speed (rad/s); adjust as necessary.
  const double m_maxRotation = 0.5;
  const double m_angleTolerance = 2.0;    // degrees tolerance for the horizontal offset

  /**
   * Checks if the Limelight has a valid target.
   * It expects the "tv" entry to be 1.0 if a valid target is detected.
   */
  bool HasValidTarget() {
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double tv = table->GetNumber("tv", 0.0);
    return tv >= 1.0;
  }

  /**
   * Computes the distance to the target using the Limelight's vertical offset ("ty").
   *
   * The formula is:
   *   distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
   *
   * Adjust the constants for your robot’s configuration.
   */
  double GetDistanceFromTag() {
    return -m_visionSubsystem->GetZ();
//     const double targetHeight = 2.5;        // Height of the AprilTag (meters)
//     const double cameraHeight = 0.5;        // Height of the Limelight (meters)
//     const double cameraAngleDegrees = 30.0; // Limelight mounting angle (degrees)

//     auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
//     double ty = table->GetNumber("ty", 0.0);

//     double totalAngleDegrees = cameraAngleDegrees + ty;
//     double totalAngleRadians = totalAngleDegrees * (M_PI / 180.0);

//     // Prevent division by zero.
//     if (std::abs(std::tan(totalAngleRadians)) < 1e-6) {
//       return 0.0;
//     }

//     double distance = (targetHeight - cameraHeight) / std::tan(totalAngleRadians);
//     return distance;
    }
};