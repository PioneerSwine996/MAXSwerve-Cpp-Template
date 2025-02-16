//=== DriveToTagCommand.h ===
#pragma once

#include <cmath>
#include <algorithm>

#include <frc2/command/CommandHelper.h>
// #include <frc2/command/CommandBase.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>
#include "subsystems/DriveSubsystem.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Command that drives the robot toward an AprilTag until it reaches 1 meter away.
 */
class DriveToTagCommand
    : public frc2::CommandHelper<frc2::Command, DriveToTagCommand> {
public:
  explicit DriveToTagCommand(DriveSubsystem* driveSubsystem)
      : m_driveSubsystem(driveSubsystem),
        m_pidController(0.5, 0.0, 0.1)  // PID gains; adjust as needed
  {
    AddRequirements({driveSubsystem});
    m_pidController.SetTolerance(0.1);  // 10 cm tolerance
  }

  void Initialize() override {
    // Reset or initialize any state if needed.
  }

  void Execute() override {
    // Check if a valid target is detected
    if (!HasValidTarget()) {
      m_driveSubsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
      return;
    }

    // Get current distance from the AprilTag (in meters)
    double currentDistance = GetDistanceFromTag();

    // Calculate forward speed using PID (error = currentDistance - desiredDistance)
    double speed = m_pidController.Calculate(currentDistance, m_targetDistance);

    // Clamp the speed so it isn’t too fast.
    speed = std::clamp(speed, -m_maxSpeed, m_maxSpeed);

    // Command the robot to drive straight forward.
    // (If your robot is already facing the target, then forward motion is all that’s needed.)
    m_driveSubsystem->Drive(units::meters_per_second_t{speed}, 0_mps, 0_rad_per_s, false);
  }

  void End(bool interrupted) override {
    m_driveSubsystem->Drive(0_mps, 0_mps, 0_rad_per_s, false);
  }

  bool IsFinished() override {
    // Finish when the robot is within tolerance of 1 meter from the target.
    double currentDistance = GetDistanceFromTag();
    return std::abs(currentDistance - m_targetDistance) < m_distanceTolerance;
  }

private:
  DriveSubsystem* m_driveSubsystem;
  frc::PIDController m_pidController;

  // Desired distance from the target (in meters)
  const double m_targetDistance = 1.0;
  // Tolerance in meters
  const double m_distanceTolerance = 0.1;
  // Maximum forward speed command (tune as needed)
  const double m_maxSpeed = 1.0;

  /**
   * Checks if the Limelight has a valid target.
   * Expects the "tv" entry to be 1 if valid.
   */
  bool HasValidTarget() {
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double tv = table->GetNumber("tv", 0.0);
    return tv >= 1.0;
  }

  /**
   * Computes the distance to the target using the Limelight's "ty" (vertical offset).
   *
   * The formula used is:
   *   distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
   *
   * Adjust the constants for your robot's configuration.
   */
  double GetDistanceFromTag() {
    auto reading = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_targetspace",std::vector<double>(6));
    return reading[2];

    // // Constants (in meters and degrees); adjust these for your robot setup.
    // const double targetHeight = 2.5;        // Height of the AprilTag (or target) from the floor
    // const double cameraHeight = 0.5;        // Height of the Limelight from the floor
    // const double cameraAngleDegrees = 30.0; // Angle of the Limelight relative to horizontal

    // auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    // // "ty" is the vertical offset angle from crosshair to target.
    // double ty = table->GetNumber("ty", 0.0);

    // double totalAngleDegrees = cameraAngleDegrees + ty;
    // double totalAngleRadians = totalAngleDegrees * (M_PI / 180.0);

    // // Avoid division by zero if the angle is near 0.
    // if (std::abs(std::tan(totalAngleRadians)) < 1e-6) {
    //   return 0.0;
    // }

    // double distance = (targetHeight - cameraHeight) / std::tan(totalAngleRadians);
    // return distance;
  }
};