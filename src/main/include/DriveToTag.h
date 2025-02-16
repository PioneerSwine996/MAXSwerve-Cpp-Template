#pragma once1


#include <cmath>
#include <algorithm>
#include <iostream>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/PIDController.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to wrap an angle (in radians) to the range [-pi, pi].
inline double WrapAngle(double angle) {
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * Command that uses the Limelight-provided robot pose (x, y, yaw) in the target's coordinate frame
 * to drive the robot to a point 1 meter from the target (assumed at (0,0))
 * and to rotate so that the robot faces the target.
 */
class DriveToTagWithPoseCommand
    : public frc2::CommandHelper<frc2::Command, DriveToTagWithPoseCommand> {
public:
  explicit DriveToTagWithPoseCommand(DriveSubsystem* driveSubsystem, VisionSubsystem* visionSubsystem)
      : m_driveSubsystem(driveSubsystem),
        m_visionSubsystem(visionSubsystem),
        m_distancePID(0.5, 0.0, 0.1),   // Tune these PID gains as needed.
        m_rotationPID(0.03, 0.0, 0.005)   // Tune these PID gains as needed.
  {
    AddRequirements({driveSubsystem});
    m_distancePID.SetTolerance(0.1);  // 10 cm tolerance for distance.
    m_rotationPID.SetTolerance(0.05); // ~2.8° tolerance for yaw (in radians).
  }

  void Initialize() override {
    m_distancePID.Reset();
    m_rotationPID.Reset();
  }

  void Execute() override {
    // Retrieve the robot's current pose from the Limelight.
    double robotX   = m_visionSubsystem->GetX();    // in meters
    double robotY   = m_visionSubsystem->GetZ();    // in meters
    double robotYaw = m_visionSubsystem->GetYaw() * M_PI/180;  // in radians

    // 1. Compute the current distance from the target (target assumed at (0,0)).
    double currentDistance = std::sqrt(robotX * robotX + robotY * robotY);
    double distanceError = currentDistance - m_targetDistance; // m_targetDistance is 1.0 meter.

    // 2. Compute the unit vector from target to robot.
    double ux = (currentDistance > 1e-6) ? robotX / currentDistance : 0.0;
    double uy = (currentDistance > 1e-6) ? robotY / currentDistance : 0.0;

    // 3. Compute translation command magnitude using the distance PID.
    double translationCmdMagnitude = m_distancePID.Calculate(currentDistance, m_targetDistance);
    translationCmdMagnitude = std::clamp(translationCmdMagnitude, -m_maxSpeed, m_maxSpeed);

    // In the target's coordinate frame, the desired translation vector is:
    double targetFrameVx = translationCmdMagnitude * ux;
    double targetFrameVy = translationCmdMagnitude * uy;

    // 4. Convert the translation command into the robot's coordinate frame.
    // Rotate by -robotYaw to align the target vector with the robot's forward axis.
    double forwardCmd = targetFrameVx * std::cos(-robotYaw) - targetFrameVy * std::sin(-robotYaw);
    double strafeCmd  = targetFrameVx * std::sin(-robotYaw) + targetFrameVy * std::cos(-robotYaw);

    // 5. Compute desired yaw: the robot should face the target (i.e. toward the origin).
    double desiredYaw = std::atan2(-robotY, -robotX);
    double yawError = WrapAngle(desiredYaw - robotYaw);
    double rotationCmd = m_rotationPID.Calculate(robotYaw, desiredYaw);
    rotationCmd = std::clamp(rotationCmd, -m_maxRotation, m_maxRotation);

    // 6. Command the swerve drive subsystem.
    m_driveSubsystem->Drive(units::meters_per_second_t{forwardCmd}, units::meters_per_second_t{strafeCmd}, units::radians_per_second_t{rotationCmd}, false);
  }

  void End(bool interrupted) override {
    m_driveSubsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
  }

  bool IsFinished() override {
    return m_distancePID.AtSetpoint() && m_rotationPID.AtSetpoint();
  }

private:
  DriveSubsystem* m_driveSubsystem;
  VisionSubsystem* m_visionSubsystem;
  frc::PIDController m_distancePID;
  frc::PIDController m_rotationPID;

  const double m_targetDistance = 1.6;    // Desired distance from target (meters).
  const double m_distanceTolerance = 0.1;   // Tolerance for distance (meters).
  const double m_maxSpeed = 0.4;            // Maximum translation speed (m/s).
  const double m_maxRotation = 0.5;         // Maximum rotational speed (rad/s).
  const double m_yawTolerance = 0.05;       // Tolerance for yaw error (radians).
};