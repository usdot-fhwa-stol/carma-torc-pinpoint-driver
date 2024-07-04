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

//#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_listener.h>

//#include <carma_driver_msgs/msg/heading_stamped.hpp>
//#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <gps_msgs/msg/gps_fix.hpp>
//#include <nav_msgs/msg/odometry.hpp>

#include <pinpoint_application.hpp>

namespace pinpoint
{
const double PI = 3.145159265359;
inline double deg2rad(double deg) { return deg * PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / PI; }

PinPointApplication::PinPointApplication(const rclcpp::NodeOptions & options)
: carma_ros2_utils::CarmaLifecycleNode(options)
{
  //  carma_driver_msgs::msg::DriverStatus status;
  //  status.status = carma_driver_msgs::msg::DriverStatus::OFF;
  //  status.gnss = true;
  //  setStatus(status);
  //
  //  declare_parameter<std::string>("address", config_.address);
  //  declare_parameter<std::string>("loc_port", config_.loc_port);
  //  declare_parameter<std::string>("odom_frame", odom_frame);
  //  declare_parameter<std::string>("base_link_frame", base_link_frame);
  //  declare_parameter<std::string>("sensor_frame", sensor_frame);
  //  declare_parameter<bool>("publish_tf", publish_tf);
}

carma_ros2_utils::CallbackReturn PinPointApplication::handle_on_configure(
  const rclcpp_lifecycle::State &)
{
  // // CAV platform requires that the position api falls under the /pinpoint/position namespace
  // tf_buffer_.reset(new tf2_ros::Buffer());
  // tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  // // Pinpoint address TODO
  // config_.address = "10.26.4.73";
  // config_.loc_port = "9501";
  // get_parameter<std::string>("address", config_.address);
  // get_parameter<std::string>("loc_port", config_.loc_port);

  // // Frames
  // get_parameter<std::string>("odom_frame", odom_frame);
  // get_parameter<std::string>("base_link_frame", base_link_frame);
  // get_parameter<std::string>("sensor_frame", sensor_frame);

  // get_parameter<bool>("publish_tf", publish_tf);

  // // Setup connection handlers
  // pinpoint_.onConnect.connect([this]() { onConnectHandler(); });
  // pinpoint_.onDisconnect.connect([this]() { onDisconnectHandler(); });

  // server.setCallback(
  //   [this](pinpoint::pinpointConfig & cfg, uint32_t level) { dynReconfigCB(cfg, level); });

  // // Setup the API publishers
  // std::string node_name = rclcpp::this_node::getName();
  // api_.clear();

  // // Velocity
  // velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 1);
  // api_.push_back(velocity_pub_.getTopic());

  // pinpoint_.onVelocityChanged.connect(
  //   [this](torc::PinPointVelocity const & vel) { onVelocityChangedHandler(vel); });

  // // GlobalPose
  // global_pose_pub_ = create_publisher<gps_msgs::msg::GPSFix>("gps_common_fix", 1);
  // api_.push_back(global_pose_pub_.getTopic());

  // pinpoint_.onGlobalPoseChanged.connect(
  //   [this](torc::PinPointGlobalPose const & pose) { onGlobalPoseChangedHandler(pose); });

  // // LocalPose
  // local_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
  // api_.push_back(local_pose_pub_.getTopic());

  // pinpoint_.onLocalPoseChanged.connect(
  //   [this](torc::PinPointLocalPose const & pose) { onLocalPoseChangedHandler(pose); });

  // // Other non-published pinpoint data
  // pinpoint_.onFilterAccuracyChanged.connect(
  //   [this](torc::PinPointFilterAccuracy const & acc) { onFilterAccuracyChangedHandler(acc); });

  // pinpoint_.onQuaternionCovarianceChanged.connect(
  //   [this](torc::PinPointQuaternionCovariance const & quat) {
  //     onQuaternionCovarianceChangedHandler(quat);
  //   });

  // pinpoint_.onStatusConditionChanged.connect(
  //   [this](torc::PinPointLocalizationClient::PinPointStatusCode const & code) {
  //     onStatusConditionChangedHandler(code);
  //   });

  // // Initialize time
  // last_heartbeat_time_ = rclcpp::Time::now();
  // pinpoint_.onHeartbeat.connect([this]() {
  //   std::lock_guard<std::mutex> lock(heartbeat_mutex_);
  //   last_heartbeat_time_ = rclcpp::Time::now();
  // });

  // updater_.setHardwareID("PinPoint");
  // diagnostic_timer_ =
  //   nh_->createTimer(rclcpp::Duration(1), &PinPointApplication::diagnosticUpdate, this);

  // spin_rate = 50;

  return CallbackReturn::SUCCESS;
}
/*
void PinPointApplication::onConnectHandler()
{
  ROS_INFO_STREAM("PinPoint Connected");

  carma_driver_msgs::msg::DriverStatus status = getStatus();
  status.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
  setStatus(status);
}

void PinPointApplication::onDisconnectHandler()
{
  carma_driver_msgs::msg::DriverStatus status = getStatus();
  status.status = carma_driver_msgs::msg::DriverStatus::OFF;
  setStatus(status);

  ROS_WARN_STREAM("PinPoint Disconnected");
}

void PinPointApplication::onVelocityChangedHandler(const torc::PinPointVelocity & vel)
{
  geometry_msgs::msg::TwistStamped msg;

  msg.header.frame_id = base_link_frame;
  try {
    msg.header.stamp.fromNSec(vel.time * static_cast<uint64_t>(1000));
  } catch (std::runtime_error e) {
    ROS_WARN_STREAM(
      "onVelocityChangedHandler through exception in rclcpp::TimeBase::fromNSec(), time : "
      << vel.time);
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(base_link_frame, sensor_frame, msg.header.stamp);
  } catch (tf2::TransformException e) {
    ROS_WARN_STREAM_THROTTLE(5, "Exception looking up transform: " << e.what());
    return;
  }

  geometry_msgs::msg::Vector3Stamped vec_in, vec_out;
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

  velocity_pub_.publish(msg);
  latest_velocity_ = msg;
}

void PinPointApplication::onGlobalPoseChangedHandler(const torc::PinPointGlobalPose & pose)
{
  gps_common::GPSFix msg;
  msg.header.frame_id = sensor_frame;

  try {
    msg.header.stamp.fromNSec(pose.time * static_cast<uint64_t>(1000));
  } catch (std::runtime_error e) {
    ROS_WARN_STREAM(
      "onGlobalPoseChangedHandler threw exception in rclcpp::TimeBase::fromNSec(), time : "
      << pose.time);
    return;
  }

  msg.altitude = pose.altitude;
  msg.longitude = pose.longitude;
  msg.latitude = pose.latitude;

  ROS_DEBUG_STREAM(
    "Lat: " << pose.latitude << " Lon: " << pose.longitude << " Alt: " << pose.altitude);

  msg.position_covariance_type = gps_common::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  msg.position_covariance = {
    latest_filter_accuracy_.position.east * latest_filter_accuracy_.position.east,   0.0, 0.0, 0.0,
    latest_filter_accuracy_.position.north * latest_filter_accuracy_.position.north, 0.0, 0.0, 0.0,
    latest_filter_accuracy_.position.down * latest_filter_accuracy_.position.down};

  msg.status.position_source = gps_common::GPSStatus::SOURCE_GPS;
  msg.status.status = gps_common::GPSStatus::STATUS_FIX;

  // Convert yaw [-180,180] to  [0,360] degrees east of north
  msg.track = pose.yaw < 0 ? 360 + pose.yaw : pose.yaw;

  global_pose_pub_.publish(msg);
}

void PinPointApplication::onLocalPoseChangedHandler(const torc::PinPointLocalPose & pose)
{
  geometry_msgs::msg::TransformStamped tf;

  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = odom_frame;
  try {
    msg.header.stamp.fromNSec(pose.time * static_cast<uint64_t>(1000));
  } catch (std::runtime_error e) {
    ROS_WARN_STREAM(
      "onLocalPoseChangedHandler threw exception in rclcpp::TimeBase::fromNSec(), time : "
      << pose.time);
    return;
  }
  msg.child_frame_id = base_link_frame;

  try {
    tf = tf_buffer_->lookupTransform(base_link_frame, sensor_frame, msg.header.stamp);
  } catch (tf2::TransformException e) {
    ROS_WARN_STREAM_THROTTLE(5, "Exception looking up transform: " << e.what());
    return;
  }

  geometry_msgs::msg::PoseStamped pinpoint_pose;
  pinpoint_pose.header.frame_id = sensor_frame;
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

  local_pose_pub_.publish(msg);

  if (publish_tf) {
    static tf2_ros::TransformBroadcaster br;
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
  auto it = code_map_.find(code.code);
  if (it == code_map_.end()) {
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

  // For the driverstatus state we need to know if we are in error or warning we maintain two sets
  // of the
  if (code.condition == torc::StatusCondition::Error) {
    warning_set_.erase(code.code);
    error_set_.insert(code.code);
    ROS_WARN_STREAM("Error detected, code = " << static_cast<uint16_t>(code.code));
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
    ROS_WARN_STREAM("Publishing FAULT status. " << error_set_.size() << " errors.");
  } else if (!warning_set_.empty()) {
    status.status = carma_driver_msgs::msg::DriverStatus::DEGRADED;
    setStatus(status);
  } else {
    status.status = carma_driver_msgs::msg::DriverStatus::OPERATIONAL;
    setStatus(status);
  }
}

void PinPointApplication::pre_spin()
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
      ROS_INFO("Attempting to connect pinpoint");
      boost::system::error_code ec;
      if (!pinpoint_.Connect(config_.address, config_.loc_port, ec)) {
        ROS_WARN_STREAM("Failed to connect, err: " << ec.message());
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
      rclcpp::Duration time = rclcpp::Time::now() - last;

      if (time.sec > 1 && time.sec % 5 == 0) {
        ROS_WARN_STREAM_THROTTLE(
          5, "No heartbeat received from pinpoint in " << time.sec << " seconds");
        carma_driver_msgs::msg::DriverStatus status = getStatus();
        if (status.status != carma_driver_msgs::msg::DriverStatus::FAULT) {
          status.status = carma_driver_msgs::msg::DriverStatus::FAULT;
          setStatus(status);
        }
      }

      if (time.sec > 30) {
        ROS_WARN_STREAM("Connection to pinpoint timeout");
        pinpoint_.Close();
      }

    } catch (std::runtime_error e) {
      ROS_WARN_STREAM("pre_spin threw exception in rclcpp::TimeBase::fromNSec(), ex: " << e.what());
      return;
    }
  }
}

void PinPointApplication::post_spin()
{
  // We don't have anything to do
}

void PinPointApplication::dynReconfigCB(pinpoint::pinpointConfig & cfg, uint32_t level)
{
  if (config_.address != cfg.address || config_.loc_port != cfg.loc_port) {
    ROS_INFO_STREAM("DynReconfig address " << cfg.address << " port " << cfg.loc_port);
    config_ = cfg;
    pinpoint_.Close();
  }
}

*/

}  // namespace pinpoint

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(pinpoint::PinPointApplication)
