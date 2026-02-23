// Copyright 2022 Pixel Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Tony Najjar
 */

#include "offset_tricycle_controller/odometry.hpp"

namespace offset_tricycle_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  lateral_offset_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

bool Odometry::update(double Ws, double alpha, const rclcpp::Duration & dt)
{
  // Offset tricycle kinematics (ROS: x forward, y left; wheel at (L, -A) = right side)
  //
  // Notation: psi = alpha (steering angle), L = wheelbase, A = lateral_offset, r = wheel_radius
  //
  // For left turn (omega > 0), ICR is to the left on rear axle: ICR = (0, d), d > 0.
  // No-slip at wheel (L, -A): wheel velocity perpendicular to (L, -A-d) => d = L/tan(psi) - A.
  // Robot center velocity: Vx = omega * d = Vs*sin(psi)/L * (L/tan(psi) - A).
  //
  //   Eq(1):  Vs = Ws * r;   omega = Vs * sin(psi) / L
  //   Eq(2):  Vx = Vs * (cos(psi) - A * sin(psi) / L)
  //           R  = Vx / omega = L/tan(psi) - A
  //
  // (Minus sign: with wheel offset to the right (A>0), effective turn radius is smaller.)

  const double L = wheelbase_;
  const double A = lateral_offset_;
  const double r = wheel_radius_;

  // Wheel linear (circumferential) speed
  double Vs = Ws * r;

  double sin_psi = std::sin(alpha);
  double cos_psi = std::cos(alpha);

  // Eq(1): angular velocity of the robot
  double theta_dot = Vs * sin_psi / L;

  // Eq(2): robot-center forward velocity (minus A for rightward wheel offset)
  double Vx = Vs * (cos_psi - A * sin_psi / L);

  // Integrate odometry: R = Vx/theta_dot = L/tan(psi) - A
  integrateExact(Vx * dt.seconds(), theta_dot * dt.seconds());

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(Vx);
  angular_accumulator_.accumulate(theta_dot);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Duration & dt)
{
  /// Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  /// Integrate odometry:
  integrateExact(linear * dt.seconds(), angular * dt.seconds());
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  resetAccumulators();
}

void Odometry::setWheelParams(double wheelbase, double wheel_radius, double lateral_offset)
{
  wheelbase_ = wheelbase;
  wheel_radius_ = wheel_radius;
  lateral_offset_ = lateral_offset;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace offset_tricycle_controller
