/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pinpoint_application.hpp>

namespace pinpoint
{
namespace std_ph = std::placeholders;
const double PI = 3.145159265359;
inline double deg2rad(double deg) { return deg * PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / PI; }

PinPointApplication::PinPointApplication(const rclcpp::NodeOptions & options)
: carma_ros2_utils::CarmaLifecycleNode(options), updater_(this)
{
  carma_driver_msgs::msg::DriverStatus status;
  status.status = carma_driver_msgs::msg::DriverStatus::OFF;
  status.gnss = true;
  setStatus(status);

  // Create initial config
  config_ = PinPointConfig();

  declare_parameter<std::string>("address", config_.address);
  declare_parameter<std::string>("loc_port", config_.loc_port);
  declare_parameter<std::string>("odom_frame", config_.odom_frame);
  declare_parameter<std::string>("base_link_frame", config_.base_link_frame);
  declare_parameter<std::string>("sensor_frame", config_.sensor_frame);
  declare_parameter<bool>("publish_tf", config_.publish_tf);
  declare_parameter<int>("spin_rate", config_.spin_rate);
}

rcl_interfaces::msg::SetParametersResult PinPointApplication::parameter_update_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto error_string = update_params<std::string>(
    {
      {"address", config_.address},
      {"loc_port", config_.loc_port},
      {"odom_frame", config_.odom_frame},
      {"base_link_frame", config_.base_link_frame},
      {"sensor_frame", config_.sensor_frame},
    },
    parameters);
  auto error_bool = update_params<bool>({{"publish_tf", config_.publish_tf}}, parameters);
  auto error_int = update_params<int>({{"spin_rate", config_.spin_rate}}, parameters);

  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error_string && !error_bool && !error_int;

  return result;
}

carma_ros2_utils::CallbackReturn PinPointApplication::handle_on_configure(
  const rclcpp_lifecycle::State &)
{
  // Reset config
  config_ = PinPointConfig();

  // Pinpoint address
  get_parameter<std::string>("address", config_.address);
  get_parameter<std::string>("loc_port", config_.loc_port);

  // Frames
  get_parameter<std::string>("odom_frame", config_.odom_frame);
  get_parameter<std::string>("base_link_frame", config_.base_link_frame);
  get_parameter<std::string>("sensor_frame", config_.sensor_frame);

  get_parameter<bool>("publish_tf", config_.publish_tf);
  get_parameter<int>("spin_rate", config_.spin_rate);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("pinpoint"), "Loaded params: " << config_);

  // Register runtime parameter update callback
  add_on_set_parameters_callback(
    std::bind(&PinPointApplication::parameter_update_callback, this, std_ph::_1));

  updater_.setHardwareID("PinPoint");

  // Setup connection handlers
  pinpoint_.onConnect.connect([this]() { onConnectHandler(); });
  pinpoint_.onDisconnect.connect([this]() { onDisconnectHandler(); });

  // Velocity
  velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 1);

  pinpoint_.onVelocityChanged.connect(
    [this](torc::PinPointVelocity const & vel) { onVelocityChangedHandler(vel); });

  // GlobalPose
  global_pose_pub_ = create_publisher<gps_msgs::msg::GPSFix>("gps_common_fix", 1);

  pinpoint_.onGlobalPoseChanged.connect(
    [this](torc::PinPointGlobalPose const & pose) { onGlobalPoseChangedHandler(pose); });

  // LocalPose
  local_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 1);

  pinpoint_.onLocalPoseChanged.connect(
    [this](torc::PinPointLocalPose const & pose) { onLocalPoseChangedHandler(pose); });

  // Other non-published pinpoint data
  pinpoint_.onFilterAccuracyChanged.connect(
    [this](torc::PinPointFilterAccuracy const & acc) { onFilterAccuracyChangedHandler(acc); });

  pinpoint_.onQuaternionCovarianceChanged.connect(
    [this](torc::PinPointQuaternionCovariance const & quat) {
      onQuaternionCovarianceChangedHandler(quat);
    });

  pinpoint_.onStatusConditionChanged.connect(
    [this](torc::PinPointLocalizationClient::PinPointStatusCode const & code) {
      onStatusConditionChangedHandler(code);
    });

  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn PinPointApplication::handle_on_activate(
  const rclcpp_lifecycle::State &)
{
  // Initialize the transform buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize time
  last_heartbeat_time_ = now();
  pinpoint_.onHeartbeat.connect([this]() {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_heartbeat_time_ = now();
  });

  diagnostic_timer_ = create_timer(
    get_clock(), std::chrono::milliseconds(1000),
    std::bind(&PinPointApplication::diagnosticUpdate, this));

  spin_timer_ = create_timer(
    get_clock(), std::chrono::milliseconds(static_cast<int>(1.0 / config_.spin_rate * 1000)),
    std::bind(&PinPointApplication::spin_callback, this));

  return CallbackReturn::SUCCESS;
}

carma_driver_msgs::msg::DriverStatus PinPointApplication::getStatus() { return status_; }

void PinPointApplication::setStatus(carma_driver_msgs::msg::DriverStatus status)
{
  status_ = status;
}

void PinPointApplication::onConnectHandler()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("pinpoint"), "PinPoint Connected");

  carma_driver_msgs::msg::DriverStatus status = getStatus();
  status.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
  setStatus(status);
}

void PinPointApplication::onDisconnectHandler()
{
  carma_driver_msgs::msg::DriverStatus status = getStatus();
  status.status = carma_driver_msgs::msg::DriverStatus::OFF;
  setStatus(status);

  RCLCPP_WARN_STREAM(rclcpp::get_logger("pinpoint"), "PinPoint Disconnected");
}

void PinPointApplication::onVelocityChangedHandler(const torc::PinPointVelocity & vel)
{
  geometry_msgs::msg::TwistStamped msg;

  msg.header.frame_id = config_.base_link_frame;
  try {
    msg.header.stamp = rclcpp::Time(
      vel.time * static_cast<uint64_t>(1000));  // use nanoseconds constructor to get time
  } catch (const std::runtime_error& e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("pinpoint"),
      "onVelocityChangedHandler through exception in rclcpp::Time(nanoseconds), time : "
        << vel.time);
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(config_.base_link_frame, config_.sensor_frame, msg.header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("pinpoint"), *this->get_clock(), 5,
      "Exception looking up transform: " << e.what());
    return;
  }

  geometry_msgs::msg::Vector3Stamped vec_in;
  geometry_msgs::msg::Vector3Stamped vec_out;
  vec_in.vector.x = vel.forward_vel;
  vec_in.vector.y = vel.right_vel;
  vec_in.vector.z = vel.down_vel;
  tf2::doTransform(vec_in, vec_out, tf);

  msg.twist.linear.x = vec_out.vector.x;
  msg.twist.linear.y = vec_out.vector.y;
  msg.twist.linear.z = vec_out.vector.z;

  vec_in.vector.x = vel.roll_rate;
  vec_in.vector.y = vel.pitch_rate;
  vec_in.vector.z = vel.yaw_rate;

  tf2::doTransform(vec_in, vec_out, tf);

  msg.twist.angular.x = vel.roll_rate;
  msg.twist.angular.y = vel.pitch_rate;
  msg.twist.angular.z = vel.yaw_rate;

  velocity_pub_->publish(msg);
  latest_velocity_ = msg;
}

void PinPointApplication::onGlobalPoseChangedHandler(const torc::PinPointGlobalPose & pose)
{
  gps_msgs::msg::GPSFix msg;
  msg.header.frame_id = config_.sensor_frame;

  try {
    msg.header.stamp = rclcpp::Time(pose.time * static_cast<uint64_t>(1000));
  } catch (const std::runtime_error& e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("pinpoint"),
      "onGlobalPoseChangedHandler threw exception in rclcpp::Time(nanoseconds), time : "
        << pose.time);
    return;
  }

  msg.altitude = pose.altitude;
  msg.longitude = pose.longitude;
  msg.latitude = pose.latitude;

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("pinpoint"),
    "Lat: " << pose.latitude << " Lon: " << pose.longitude << " Alt: " << pose.altitude);

  msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.position_covariance = {
    latest_filter_accuracy_.position.east * latest_filter_accuracy_.position.east,   0.0, 0.0, 0.0,
    latest_filter_accuracy_.position.north * latest_filter_accuracy_.position.north, 0.0, 0.0, 0.0,
    latest_filter_accuracy_.position.down * latest_filter_accuracy_.position.down};

  msg.status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;
  msg.status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;

  // Convert yaw [-180,180] to  [0,360] degrees east of north
  msg.track = pose.yaw < 0 ? 360 + pose.yaw : pose.yaw;

  global_pose_pub_->publish(msg);
}

void PinPointApplication::onLocalPoseChangedHandler(const torc::PinPointLocalPose & pose)
{
  geometry_msgs::msg::TransformStamped tf;

  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = config_.odom_frame;
  try {
    msg.header.stamp = rclcpp::Time(pose.time * static_cast<uint64_t>(1000));
  } catch (const std::runtime_error& e) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("pinpoint"),
      "onLocalPoseChangedHandler threw exception in rclcpp::Time(nanoseconds), time: "
        << pose.time);
    return;
  }
  msg.child_frame_id = config_.base_link_frame;

  try {
    tf =
      tf_buffer_->lookupTransform(config_.base_link_frame, config_.sensor_frame, msg.header.stamp);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("pinpoint"), *this->get_clock(), 5,
      "Exception looking up transform: " << e.what());
    return;
  }

  geometry_msgs::msg::PoseStamped pinpoint_pose;
  pinpoint_pose.header.frame_id = config_.sensor_frame;
  pinpoint_pose.header.stamp = msg.header.stamp;

  pinpoint_pose.pose.position.x = pose.north;
  pinpoint_pose.pose.position.y = pose.east;
  pinpoint_pose.pose.position.z = pose.down;

  tf2::Quaternion pinpoint_quat;
  pinpoint_quat.setRPY(deg2rad(pose.roll), deg2rad(pose.pitch), deg2rad(pose.yaw));

  pinpoint_pose.pose.orientation.x = pinpoint_quat.x();
  pinpoint_pose.pose.orientation.y = pinpoint_quat.y();
  pinpoint_pose.pose.orientation.z = pinpoint_quat.z();
  pinpoint_pose.pose.orientation.w = pinpoint_quat.w();

  geometry_msgs::msg::PoseStamped out;
  tf2::doTransform(pinpoint_pose, out, tf);

  msg.pose.pose.position = out.pose.position;
  msg.pose.pose.orientation = out.pose.orientation;

  msg.pose.covariance = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    latest_quaternion_covariance_.covariance[0][0],
    0,
    0,
    0,
    0,
    0,
    0,
    latest_quaternion_covariance_.covariance[1][1],
    0,
    0,
    0,
    0,
    0,
    0,
    latest_quaternion_covariance_.covariance[2][2]};

  msg.twist.twist = latest_velocity_.twist;

  local_pose_pub_->publish(msg);

  if (config_.publish_tf) {
    static tf2_ros::TransformBroadcaster br(shared_from_this());
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header = msg.header;
    transformStamped.child_frame_id = msg.child_frame_id;
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.pose.position.z;

    transformStamped.transform.rotation = msg.pose.pose.orientation;
    br.sendTransform(transformStamped);
  }
}

void PinPointApplication::onFilterAccuracyChangedHandler(const torc::PinPointFilterAccuracy & acc)
{
  latest_filter_accuracy_ = acc;
}

void PinPointApplication::onQuaternionCovarianceChangedHandler(
  const torc::PinPointQuaternionCovariance & quat)
{
  latest_quaternion_covariance_ = quat;
}

void PinPointApplication::onStatusConditionChangedHandler(
  const torc::PinPointLocalizationClient::PinPointStatusCode & code)
{
  // Check to see if we are already tracking this code
  if (auto it = code_map_.find(code.code); it == code_map_.end()) {
    // If the code is not tracked we need to create a DiagnostHelper object for it and add it to our
    // sotre
    StatusMessageDiagnosticHelper stat;
    stat.code = code.code;
    stat.condition = code.condition;
    code_map_[code.code] = stat;

    // Then we add it to the diagnostic_updater callbacks
    updater_.add(
      code_map_[code.code].codeAsString(), &code_map_[code.code],
      &StatusMessageDiagnosticHelper::processDiagnostics);
  }

  // update the condition
  code_map_[code.code].condition = code.condition;

  // For the driver status state, we need to know if we are in an error or warning condition.
  // We maintain two sets: one for warnings and one for errors.
  if (code.condition == torc::StatusCondition::Error) {
    warning_set_.erase(code.code);
    error_set_.insert(code.code);
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("pinpoint"),
      "Error detected, code = " << static_cast<uint16_t>(code.code));
  } else if (code.condition == torc::StatusCondition::Warning) {
    warning_set_.insert(code.code);
    error_set_.erase(code.code);
  } else {
    warning_set_.erase(code.code);
    error_set_.erase(code.code);
  }

  // We assume status is unchanged if either of the sets contain items then we set status
  // accordingly
  carma_driver_msgs::msg::DriverStatus status = getStatus();
  if (!error_set_.empty()) {
    status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
    setStatus(status);
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("pinpoint"),
      "Publishing FAULT status. " << error_set_.size() << " errors.");
  } else if (!warning_set_.empty()) {
    status.status = carma_driver_msgs::msg::DriverStatus::DEGRADED;
    setStatus(status);
  } else {
    status.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
    setStatus(status);
  }
}

void PinPointApplication::spin_callback()
{
  // If we are not connected
  if (!connecting_ && !pinpoint_.connected()) {
    connecting_ = true;
    if (connect_thread_) {
      connect_thread_->join();
    }

    // We don't want to block the spin thread because the driver application maintains driver status
    // topic
    connect_thread_.reset(new std::thread([this]() {
      RCLCPP_INFO(rclcpp::get_logger("pinpoint"), "Attempting to connect pinpoint");
      boost::system::error_code ec;
      if (!pinpoint_.Connect(config_.address, config_.loc_port, ec)) {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("pinpoint"), "Failed to connect, err: " << ec.message());
      }

      connecting_ = false;
    }));
  } else if (pinpoint_.connected())  // If we are connected lets make sure we are getting updates
  {
    rclcpp::Time last;
    {
      std::lock_guard<std::mutex> lock(heartbeat_mutex_);
      last = last_heartbeat_time_;
    }

    try {
      rclcpp::Duration time = now() - last;

      if (time.seconds() > 1 && static_cast<int>(time.seconds()) % 5 == 0) {
        RCLCPP_WARN_STREAM_THROTTLE(
          rclcpp::get_logger("pinpoint"), *this->get_clock(), 5,
          "No heartbeat received from pinpoint in " << time.seconds() << " seconds");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        if (status.status != carma_driver_msgs::msg::DriverStatus::FAULT) {
          status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
          setStatus(status);
        }
      }

      if (time.seconds() > 30) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("pinpoint"), "Connection to pinpoint timeout");
        pinpoint_.Close();
      }

    } catch (const std::runtime_error& e) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("pinpoint"),
        "spin_callback threw exception in rclcpp::TimeBase::fromNSec(), ex: " << e.what());
      return;
    }
  }
}

}  // namespace pinpoint

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(pinpoint::PinPointApplication)
