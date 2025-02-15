#pragma once

#include <frc/DigitalInput.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <frc/filter/LinearFilter.h>

#include <frc2/command/SubsystemBase.h>


class VisionSubsystem : public frc2::SubsystemBase {
public:
  void Periodic() noexcept override {
    auto reading = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<botpose_targetspace>",std::vector<double>(6));
    x = xfilter.Calculate(reading[0]);
    y = yfilter.Calculate(reading[1]);
    yaw = yawfilter.Calculate(reading[4]);
  }

  double GetX() {
    return x;
  }
    double GetY() {
    return y;
  }
    double GetYaw() {
    return yaw;
  }

private:
  double x{};
  double y{};
  double yaw{};


  frc::LinearFilter<double> xfilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 0.02_s);
  frc::LinearFilter<double> yfilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 0.02_s);
  frc::LinearFilter<double> yawfilter = frc::LinearFilter<double>::SinglePoleIIR(0.1, 0.02_s);

};