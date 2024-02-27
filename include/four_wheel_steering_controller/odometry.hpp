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

#ifndef FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_HPP_
#define FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace four_wheel_steering_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(double fl_wheel_pos, double fl_steering_pos, double bl_wheel_pos, double bl_steering_pos,
              double br_wheel_pos, double br_steering_pos, double fr_wheel_pos, double fr_steering_pos,
              const rclcpp::Time & time);
  bool updateFromVelocity(double fl_wheel_vel, double fl_steering_pos, double bl_wheel_vel, double bl_steering_pos,
                          double br_wheel_vel, double br_steering_pos, double fr_wheel_vel, double fr_steering_pos,
                          const rclcpp::Time & time);
  void updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double fl_wheel_position_x, double fl_wheel_position_y, double fl_wheel_radius,
                      double bl_wheel_position_x, double bl_wheel_position_y, double bl_wheel_radius,
                      double br_wheel_position_x, double br_wheel_position_y, double br_wheel_radius,
                      double fr_wheel_position_x, double fr_wheel_position_y, double fr_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear_x, double linear_y, double angular);
  void integrateExact(double linear_x, double linear_y, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_x_;   //   [m/s]
  double linear_y_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  //Front left wheel (FL)
  double fl_wheel_position_x_;
  double fl_wheel_position_y_;
  double fl_wheel_radius_;

  //Back left wheel(BL)
  double bl_wheel_position_x_;
  double bl_wheel_position_y_;
  double bl_wheel_radius_;

  //Back right wheel(BR)
  double br_wheel_position_x_;
  double br_wheel_position_y_;
  double br_wheel_radius_;

  //Front right wheel(FR)
  double fr_wheel_position_x_;
  double fr_wheel_position_y_;
  double fr_wheel_radius_;

  // Previous wheel position/state [rad]: <---------------------DORADITI!!
  double fl_wheel_old_pos_;
  double bl_wheel_old_pos_;
  double br_wheel_old_pos_;
  double fr_wheel_old_pos_;

  // Rolling mean accumulators for the linear x, linear y and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace four_wheel_steering_controller

#endif  // FOUR_WHEEL_STEERING_CONTROLLER__ODOMETRY_HPP_
