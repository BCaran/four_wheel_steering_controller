// Copyright 2024
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
 * Author: Branimir Caran
 */

#include "four_wheel_steering_controller/odometry.hpp"

namespace four_wheel_steering_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  fl_wheel_position_x_(0.0),
  fl_wheel_position_y_(0.0),
  fl_wheel_position_radius_(0.0),
  bl_wheel_position_x_(0.0),
  bl_wheel_position_y_(0.0),
  bl_wheel_position_radius_(0.0),
  br_wheel_position_x_(0.0),
  br_wheel_position_y_(0.0),
  br_wheel_position_radius_(0.0),
  fr_wheel_position_x_(0.0),
  fr_wheel_position_y_(0.0),
  fr_wheel_position_radius_(0.0),
  left_wheel_old_pos_(0.0), //<------------------- DORADITI!
  right_wheel_old_pos_(0.0), //<------------------- DORADITI!
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_x_(velocity_rolling_window_size),
  linear_accumulator_y_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double fl_wheel_pos, double fl_steering_pos, double bl_wheel_pos, double bl_steering_pos,
                      double br_wheel_pos, double br_steering_pos, double fr_wheel_pos, double fr_steering_pos,
                      const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double fl_wheel_cur_pos = fl_wheel_pos * fl_wheel_radius_;
  const double bl_wheel_cur_pos = bl_wheel_pos * br_wheel_radius_;
  const double br_wheel_cur_pos = br_wheel_pos * bl_wheel_radius_;
  const double fr_wheel_cur_pos = fr_wheel_pos * fr_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double fl_wheel_est_vel = fl_wheel_cur_pos - fl_wheel_old_pos_;
  const double bl_wheel_est_vel = bl_wheel_cur_pos - bl_wheel_old_pos_;
  const double br_wheel_est_vel = br_wheel_cur_pos - br_wheel_old_pos_;
  const double fr_wheel_est_vel = fr_wheel_cur_pos - fr_wheel_old_pos_;

  // Update old position with current:
  fl_wheel_old_pos_ = fl_wheel_cur_pos;
  bl_wheel_old_pos_ = bl_wheel_cur_pos;
  br_wheel_old_pos_ = br_wheel_cur_pos;
  fr_wheel_old_pos_ = fr_wheel_cur_pos;

  updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time);

  return true;
}

bool Odometry::updateFromVelocity(double fl_wheel_vel, double fl_steering_pos, double bl_wheel_vel, double bl_steering_pos,
                                  double br_wheel_vel, double br_steering_pos, double fr_wheel_vel, double fr_steering_pos,
                                  const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  //Compute linear x, linear y and angular according to Kinematics, dynamics and control design of 4WIS4WID mobile robots (Lee et. all)
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular = 0.0;
  const double wheel_vel[4] = {fl_wheel_vel, bl_wheel_vel, br_wheel_vel, fr_wheel_vel};
  const double steering_pos[4] = {fl_steering_pos, bl_steering_pos, br_steering_pos, fr_steering_pos};
  const double wheel_pos_x[4] = {fl_wheel_position_x_, bl_wheel_position_x_, br_wheel_position_x_, fr_wheel_position_x_};
  const double wheel_pos_y[4] = {fl_wheel_position_y_, bl_wheel_position_y_, br_wheel_position_y_, fr_wheel_position_y_};
  double W[4] = {0.0, 0.0, 0.0, 0.0};
  for(int i = 0; i <4; i++){
    linear_x = linear_x + (wheel_vel[i] * (cos(steering_pos[i]) / 4));
    linear_y = linear_y + (wheel_vel[i] * (sin(steering_pos[i]) / 4));
    W[i] = (-wheel_pos_y[i] * cos(steering_pos[i]) + wheel_pos_x[i] * sin(steering_pos[i])) / (4 * pow(wheel_pos_x[i], 2) + 4 * pow(wheel_pos_y[i], 2));
    angular = angular + W[i] * wheel_vel[i];
  }

  // Integrate odometry:
  integrateExact(linear_x, linear_y, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_x_acumulator_.accumulate(linear_x / dt);
  linear_y_acumulator_.accumulate(linear_y / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_x_ = linear_x_acumulator_.getRollingMean();
  linear_y_ = linear_y_acumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear_x * dt, linear_y * dt,  angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void setWheelParams(double fl_wheel_position_x, double fl_wheel_position_y, double fl_wheel_radius,
                      double bl_wheel_position_x, double bl_wheel_position_y, double bl_wheel_radius,
                      double br_wheel_position_x, double br_wheel_position_y, double br_wheel_radius,
                      double fr_wheel_position_x, double fr_wheel_position_y, double fr_wheel_radius)
{
  fl_wheel_position_x_ = fl_wheel_position_x;
  fl_wheel_position_y_ = fl_wheel_position_y;
  fl_wheel_radius_ = fl_wheel_radius;

  bl_wheel_position_x_ = bl_wheel_position_x;
  bl_wheel_position_y_ = bl_wheel_position_y;
  bl_wheel_radius_ = bl_wheel_radius;

  br_wheel_position_x_ = br_wheel_position_x;
  br_wheel_position_y_ = br_wheel_position_y;
  br_wheel_radius_ = br_wheel_radius;

  fr_wheel_position_x_ = fr_wheel_position_x;
  fr_wheel_position_y_ = fr_wheel_position_y;
  fr_wheel_radius_ = fr_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
{
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration:
  x_ += linear_x * cos(direction) - linear_y * sin(direction);
  y_ += linear_x * sin(direction) + linear_y * cos(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear_x, linear_y, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    heading_ += angular;
    x_ += linear_x * ((cos(heading_) - linear_y * sin(heading_)) - (cos(heading_old) - linear_y * sin(heading_old)));
    y_ += linear_x * ((sin(heading_) + linear_y * cos(heading_)) - (sin(heading_old) + linear_y * cos(heading_old)));
  }
}

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace four_wheel_steering_controller
