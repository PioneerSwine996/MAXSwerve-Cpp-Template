#pragma once

// DriveToAprilTagRotate.h
#pragma once

#include <cmath>
#include <algorithm>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

// Include your subsystem headers
#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/ArmSubsystem.h"

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
 * to drive the robot to a point 1 meter from the t arget (assumed at (0,0))
 * and to rotate so that the robot faces the target.
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
class DriveToTag
    : public frc2::CommandHelper<frc2::Command, DriveToTag> {
public:
  explicit DriveToTag(DriveSubsystem* driveSubsystem, VisionSubsystem* visionSubsystem, ArmSubsystem* armSubsystem)
      : m_driveSubsystem(driveSubsystem),
        m_visionSubsystem(visionSubsystem),
        m_armSubsystem(armSubsystem),
        m_distancePID(0.5, 0.0, 0.1),   // Tune these PID gains as needed.
        m_rotationPID(0.03, 0.0, 0.005)   // Tune these PID gains as needed.
  {
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

  const double x_offset = 0.5;

  void Execute() override {
    // Retrieve the robot's current pose from the Limelight.
    double robotX   = m_visionSubsystem->GetX() - x_offset;    // in meters
    double robotY   = -m_visionSubsystem->GetZ();    // in meters
    double robotYaw = -m_visionSubsystem->GetYaw() * M_PI/180;  // in radians

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
    m_driveSubsystem->Drive(units::meters_per_second_t{-strafeCmd}, units::meters_per_second_t{forwardCmd}, units::radians_per_second_t{rotationCmd}, false);
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
  DriveSubsystem* m_driveSubsystem;
  VisionSubsystem* m_visionSubsystem;
  ArmSubsystem* m_armSubsystem;
  frc::PIDController m_distancePID;
  frc::PIDController m_rotationPID;

  const double m_targetDistance = 1.6;    // Desired distance from target (meters).
  const double m_distanceTolerance = 0.1;   // Tolerance for distance (meters).
  const double m_maxSpeed = 0.2;            // Maximum translation speed (m/s).
  const double m_maxRotation = 0.5;         // Maximum rotational speed (rad/s).
  const double m_yawTolerance = 0.05;       // Tolerance for yaw error (radians).
 private:
  DriveSubsystem* m_drive;
  VisionSubsystem* m_limelight;
};