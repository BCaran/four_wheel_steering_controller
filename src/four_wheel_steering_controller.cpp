// Copyright 2020 PAL Robotics S.L.
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
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "four_wheel_steering_controller/four_wheel_steering_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace four_wheel_steering_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

FourWheelSteeringController::FourWheelSteeringController() : controller_interface::ControllerInterface() {}

const char * FourWheelSteeringController::driving_feedback_type() const
{
  return params_.driving_position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

const char * FourWheelSteeringController::steering_feedback_type() const
{
  return params_.steering_position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration FourWheelSteeringController::command_interface_configuration() const
{

  std::vector<std::string> conf_names;

  for (const auto & joint_name : params_.fl_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.bl_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.br_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.fr_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }

  for (const auto & joint_name : params_.fl_steering_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.bl_steering_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.br_steering_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : params_.fr_steering_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }


  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration FourWheelSteeringController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;

  for (const auto & joint_name : params_.fl_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + driving_feedback_type());
  }
  for (const auto & joint_name : params_.bl_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + driving_feedback_type());
  }
  for (const auto & joint_name : params_.br_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + driving_feedback_type());
  }
  for (const auto & joint_name : params_.fr_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + driving_feedback_type());
  }

  for (const auto & joint_name : params_.fl_steering_name)
  {
    conf_names.push_back(joint_name + "/" + steering_feedback_type());
  }
  for (const auto & joint_name : params_.bl_steering_name)
  {
    conf_names.push_back(joint_name + "/" + steering_feedback_type());
  }
  for (const auto & joint_name : params_.br_steering_name)
  {
    conf_names.push_back(joint_name + "/" + steering_feedback_type());
  }
  for (const auto & joint_name : params_.fr_steering_name)
  {
    conf_names.push_back(joint_name + "/" + steering_feedback_type());
  }

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type FourWheelSteeringController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & linear_x_command = command.twist.linear.x;
  double & linear_y_command = command.twist.linear.y;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  const double fl_wheel_position_x = params_.fl_wheel_position_x_multiplier * params_.fl_wheel_position_x;
  const double fl_wheel_position_y = params_.fl_wheel_position_y_multiplier * params_.fl_wheel_position_y;
  const double fl_wheel_radius = params_.fl_wheel_radius_multiplier * params_.fl_wheel_radius;

  const double bl_wheel_position_x = params_.bl_wheel_position_x_multiplier * params_.bl_wheel_position_x;
  const double bl_wheel_position_y = params_.bl_wheel_position_y_multiplier * params_.bl_wheel_position_y;
  const double bl_wheel_radius = params_.bl_wheel_radius_multiplier * params_.bl_wheel_radius;

  const double br_wheel_position_x = params_.br_wheel_position_x_multiplier * params_.br_wheel_position_x;
  const double br_wheel_position_y = params_.br_wheel_position_y_multiplier * params_.br_wheel_position_y;
  const double br_wheel_radius = params_.br_wheel_radius_multiplier * params_.br_wheel_radius;

  const double fr_wheel_position_x = params_.fr_wheel_position_x_multiplier * params_.fr_wheel_position_x;
  const double fr_wheel_position_y = params_.fr_wheel_position_y_multiplier * params_.fr_wheel_position_y;
  const double fr_wheel_radius = params_.fr_wheel_radius_multiplier * params_.fr_wheel_radius;

  if (params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_x_command, linear_y_command, angular_command, time);
  }
  else
  {
    const double fl_velocity_feedback = registered_fl_wheel_handles_[0].feedback.get().get_value() * fl_wheel_radius;
    const double bl_velocity_feedback = registered_bl_wheel_handles_[0].feedback.get().get_value() * bl_wheel_radius;
    const double br_velocity_feedback = registered_br_wheel_handles_[0].feedback.get().get_value() * br_wheel_radius;
    const double fr_velocity_feedback = registered_fr_wheel_handles_[0].feedback.get().get_value() * fr_wheel_radius;

    const double fl_steering_feedback = registered_fl_steering_handles_[0].position.get().get_value();
    const double bl_steering_feedback = registered_bl_steering_handles_[0].position.get().get_value();
    const double br_steering_feedback = registered_br_steering_handles_[0].position.get().get_value();
    const double fr_steering_feedback = registered_fr_steering_handles_[0].position.get().get_value();

    
    odometry_.updateFromVelocity(fl_velocity_feedback, fl_steering_feedback, bl_velocity_feedback, bl_steering_feedback, 
        br_velocity_feedback, br_steering_feedback, fr_velocity_feedback, fr_steering_feedback, time);
        
    /*if (params_.driving_position_feedback)
    {
      odometry_.update(fl_velocity_feedback, fl_steering_feedback, bl_velocity_feedback, bl_steering_feedback, 
        br_velocity_feedback, br_steering_feedback, fr_velocity_feedback, fr_steering_feedback, time);
    }
    else
    {      
      odometry_.updateFromVelocity(fl_velocity_feedback, fl_steering_feedback, bl_velocity_feedback, bl_steering_feedback, 
        br_velocity_feedback, br_steering_feedback, fr_velocity_feedback, fr_steering_feedback, time);
    }*/
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    // Handle exceptions when the time source changes and initialize publish timestamp
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinearX();
      odometry_message.twist.twist.linear.y = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_x_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_linear_.limit(
    linear_x_command, last_command.linear.x, second_to_last_command.linear.y, period.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Compute wheels velocities:
  double wheels_x_velocity[4] = {0.0, 0.0, 0.0, 0.0};
  double wheels_y_velocity[4] = {0.0, 0.0, 0.0, 0.0};
  double wheels_velocity[4] = {0.0, 0.0, 0.0, 0.0};
  double wheels_angles[4] = {0.0, 0.0, 0.0, 0.0};
  const double wheel_pos_x[4] = {fl_wheel_position_x, bl_wheel_position_x, br_wheel_position_x, fr_wheel_position_x};
  const double wheel_pos_y[4] = {fl_wheel_position_y, bl_wheel_position_y, br_wheel_position_y, fr_wheel_position_y};

  for(int i = 0; i < 4; i++){
    wheels_x_velocity[i] = linear_x_command - wheel_pos_y[i] * angular_command;
    wheels_y_velocity[i] = linear_y_command + wheel_pos_x[i] * angular_command;
    wheels_velocity[i] = sqrt(pow(wheels_x_velocity[i], 2) + pow(wheels_y_velocity[i], 2));
    wheels_angles[i] = atan2(wheels_y_velocity[i], wheels_x_velocity[i]);
  }

  //Set wheels velocities
  registered_fl_wheel_handles_[0].velocity.get().set_value(wheels_velocity[0]);
  registered_bl_wheel_handles_[0].velocity.get().set_value(wheels_velocity[1]);
  registered_br_wheel_handles_[0].velocity.get().set_value(wheels_velocity[2]);
  registered_fr_wheel_handles_[0].velocity.get().set_value(wheels_velocity[3]);

  //Set wheels angles
  registered_fl_steering_handles_[0].position.get().set_value(wheels_angles[0]);
  registered_bl_steering_handles_[0].position.get().set_value(wheels_angles[1]);
  registered_br_steering_handles_[0].position.get().set_value(wheels_angles[2]);
  registered_fr_steering_handles_[0].position.get().set_value(wheels_angles[3]);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.fl_wheel_name.empty() && params_.bl_wheel_name.empty() && params_.br_wheel_name.empty() && params_.fr_wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.fl_steering_name.empty() && params_.bl_steering_name.empty() && params_.br_steering_name.empty() && params_.fr_steering_name.empty())
  {
    RCLCPP_ERROR(logger, "Steering names parameters are empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  const double fl_wheel_position_x = params_.fl_wheel_position_x_multiplier * params_.fl_wheel_position_x;
  const double fl_wheel_position_y = params_.fl_wheel_position_y_multiplier * params_.fl_wheel_position_y;
  const double fl_wheel_radius = params_.fl_wheel_radius_multiplier * params_.fl_wheel_radius;

  const double bl_wheel_position_x = params_.bl_wheel_position_x_multiplier * params_.bl_wheel_position_x;
  const double bl_wheel_position_y = params_.bl_wheel_position_y_multiplier * params_.bl_wheel_position_y;
  const double bl_wheel_radius = params_.bl_wheel_radius_multiplier * params_.bl_wheel_radius;

  const double br_wheel_position_x = params_.br_wheel_position_x_multiplier * params_.br_wheel_position_x;
  const double br_wheel_position_y = params_.br_wheel_position_y_multiplier * params_.br_wheel_position_y;
  const double br_wheel_radius = params_.br_wheel_radius_multiplier * params_.br_wheel_radius;

  const double fr_wheel_position_x = params_.fr_wheel_position_x_multiplier * params_.fr_wheel_position_x;
  const double fr_wheel_position_y = params_.fr_wheel_position_y_multiplier * params_.fr_wheel_position_y;
  const double fr_wheel_radius = params_.fr_wheel_radius_multiplier * params_.fr_wheel_radius;

  odometry_.setWheelParams(fl_wheel_position_x, fl_wheel_position_y, fl_wheel_radius, bl_wheel_position_x, bl_wheel_position_y, bl_wheel_radius,
                           br_wheel_position_x, br_wheel_position_y, br_wheel_radius, fr_wheel_position_x, fr_wheel_position_y, fr_wheel_radius);
  odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  publish_limited_velocity_ = params_.publish_limited_velocity;
  use_stamped_vel_ = params_.use_stamped_vel;

  limiter_linear_ = SpeedLimiter( //<--------------------------------------------------------------------DORADITI
    params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
    params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
    params_.linear.x.max_jerk);

  limiter_angular_ = SpeedLimiter( //<--------------------------------------------------------------------DORADITI
    params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
    params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
    params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  //params_.wheels_per_side = params_.left_wheel_names.size();

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // Append the tf prefix if there is one
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    if (params_.tf_frame_prefix != "")
    {
      tf_prefix = params_.tf_frame_prefix;
    }
    else
    {
      tf_prefix = std::string(get_node()->get_namespace());
    }

    if (tf_prefix == "/")
    {
      tf_prefix = "";
    }
    else
    {
      tf_prefix = tf_prefix + "/";
    }
  }

  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id = base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto fl_wheel_result = configure_driving_side("fl_driving", params_.fl_wheel_name, registered_fl_wheel_handles_);
  const auto bl_wheel_result = configure_driving_side("bl_driving", params_.bl_wheel_name, registered_bl_wheel_handles_);
  const auto br_wheel_result = configure_driving_side("br_driving", params_.br_wheel_name, registered_br_wheel_handles_);
  const auto fr_wheel_result = configure_driving_side("fl_driving", params_.fr_wheel_name, registered_fr_wheel_handles_);

  const auto fl_steering_result = configure_steering_side("fl_steering", params_.fl_steering_name, registered_fl_steering_handles_);
  const auto bl_steering_result = configure_steering_side("bl_steering", params_.bl_steering_name, registered_bl_steering_handles_);
  const auto br_steering_result = configure_steering_side("br_steering", params_.br_steering_name, registered_br_steering_handles_);
  const auto fr_steering_result = configure_steering_side("fr_steering", params_.fr_steering_name, registered_fr_steering_handles_);

  if (
    fl_wheel_result == controller_interface::CallbackReturn::ERROR ||
    bl_wheel_result == controller_interface::CallbackReturn::ERROR ||
    br_wheel_result == controller_interface::CallbackReturn::ERROR ||
    fr_wheel_result == controller_interface::CallbackReturn::ERROR ||
    fl_steering_result == controller_interface::CallbackReturn::ERROR ||
    bl_steering_result == controller_interface::CallbackReturn::ERROR ||
    br_steering_result == controller_interface::CallbackReturn::ERROR ||
    fr_steering_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  
  if (registered_fl_wheel_handles_.empty() || registered_fl_steering_handles_.empty() ||
      registered_bl_wheel_handles_.empty() || registered_bl_steering_handles_.empty() ||
      registered_br_wheel_handles_.empty() || registered_br_steering_handles_.empty() ||
      registered_fr_wheel_handles_.empty() || registered_fr_steering_handles_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Either one interfaces are non existent");
    return controller_interface::CallbackReturn::ERROR;
  }
  

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted)
  {
    halt();
    is_halted = true;
  }
  /*
  registered_fl_wheel_handle_.clear();
  registered_bl_wheel_handle_.clear();
  registered_br_wheel_handle_.clear();
  registered_fr_wheel_handle_.clear();

  registered_fl_steering_handle_.clear();
  registered_bl_steering_handle_.clear();
  registered_br_steering_handle_.clear();
  registered_fr_steering_handle_.clear();
  */
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool FourWheelSteeringController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);
  
  /*
  registered_fl_wheel_handles_.clear();
  registered_bl_wheel_handles_.clear();
  registered_br_wheel_handles_.clear();
  registered_fr_wheel_handles_.clear();

  registered_fl_steering_handles_.clear();
  registered_bl_steering_handles_.clear();
  registered_br_steering_handles_.clear();
  registered_fr_steering_handles_.clear();

  //registered_left_wheel_handles_.clear();
  //registered_right_wheel_handles_.clear();
  */

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn FourWheelSteeringController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void FourWheelSteeringController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  const auto halt_steering = [](auto & steering_handles)
  {
    for(const auto & steering_handle : steering_handles)
    {
      steering_handle.position.get().set_value(0.0);
    }
  };

  halt_wheels(registered_fl_wheel_handles_);
  halt_wheels(registered_bl_wheel_handles_);
  halt_wheels(registered_br_wheel_handles_);
  halt_wheels(registered_fr_wheel_handles_);

  halt_steering(registered_fl_steering_handles_);
  halt_steering(registered_bl_steering_handles_);
  halt_steering(registered_br_steering_handles_);
  halt_steering(registered_fr_steering_handles_);
}

controller_interface::CallbackReturn FourWheelSteeringController::configure_driving_side(
  const std::string & side, const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = driving_feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FourWheelSteeringController::configure_steering_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<SteeringHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto interface_name = steering_feedback_type();
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&wheel_name, &interface_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface)
      {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_POSITION;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace four_wheel_steering_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  four_wheel_steering_controller::FourWheelSteeringController, controller_interface::ControllerInterface)
