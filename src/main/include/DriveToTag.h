// DriveToAprilTagRotate.h
#pragma once

#include <cmath>
#include <algorithm>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

// Include your subsystem headers
#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/**
 * @brief Drives the robot toward an AprilTag until it is 1 meter away,
 * while rotating to face the tag head-on.
 *
 * This command uses two controllers:
 *  - A distance controller that drives forward until the distance error (distance - 1 m)
 *    is reduced.
 *  - A heading controller that rotates the robot to zero out the horizontal offset (tx)
 *    from the Limelight.
 */
class DriveToAprilTagRotate
    : public frc2::CommandHelper<frc2::Command, DriveToAprilTagRotate> {
 public:
  /**
   * @param driveSubsystem Pointer to the MAXSwerve drive subsystem.
   * @param limelight Pointer to the Limelight subsystem (or helper) that provides target data.
   */
  DriveToAprilTagRotate(DriveSubsystem* driveSubsystem, VisionSubsystem* limelight)
      : m_drive(driveSubsystem), m_limelight(limelight) {
    AddRequirements({driveSubsystem});
  }

  void Initialize() override {
    // Optionally, reset controllers or sensors.
  }

  void Execute() override {
    // Get the current distance from the Limelight (in meters)
    double distance = m_limelight->GetZ();

    // Only drive if we are farther than 1 meter away.
    if (distance > 1.0) {
      // --- Translation Controller ---
      // Calculate how far we are from 1 meter.
      double distanceError = distance - 1.0;
      constexpr double kPDistance = 0.5;  // Tune this constant as needed.
      double forwardSpeed = kPDistance * distanceError;


      // Clamp the forward speed to a maximum value.
      constexpr double kMaxSpeed = 1.0;   // Maximum speed (m/s)
      forwardSpeed = std::min(forwardSpeed, kMaxSpeed);

      // --- Rotation Controller ---
      // Get the horizontal offset (tx in degrees) from the Limelight.
      double tx = m_limelight->GetX();

      // A positive tx means the target is to the right of center.
      // To rotate the robot so that its front faces the target, we need to turn clockwise,
      // which is typically a negative angular velocity.
      constexpr double kPRotation = 0.03; // Tune this constant as needed.
      double rotationCommand = -kPRotation * tx;

      // --- Choose Translation Option ---
      // Option 1: Drive straight forward relative to the robot's current orientation.
    //   double vx = forwardSpeed;
    //   double vy = 0.0;

      // Option 2: Drive directly toward the target based on its angular offset.
      // Uncomment the lines below if you prefer this behavior.
      double angleRadians = tx * (M_PI / 180.0);
      double vx = forwardSpeed * std::cos(angleRadians);
      double vy = forwardSpeed * std::sin(angleRadians);

      // Command the drive subsystem:
      // vx and vy are translational speeds (m/s), and rotationCommand is angular speed (rad/s).
      m_drive->Drive(units::meters_per_second_t{vx}, units::meters_per_second_t{vy}, units::radians_per_second_t{rotationCommand}, false);
    } else {
      // When 1 meter or closer, stop the robot.
      m_drive->Drive(units::meters_per_second_t{0.0}, units::meters_per_second_t{0.0}, units::radians_per_second_t{0.0}, false);
    }
  }

  // The command ends once the robot is within 1 meter of the AprilTag.
  bool IsFinished() override {
    return m_limelight->GetZ() <= 1.0;
  }

  void End(bool interrupted) override {
    // Stop the robot if the command ends or is interrupted.
    m_drive->Drive(units::meters_per_second_t{0.0}, units::meters_per_second_t{0.0}, units::radians_per_second_t{0.0}, false);
  }

 private:
  DriveSubsystem* m_drive;
  VisionSubsystem* m_limelight;
};