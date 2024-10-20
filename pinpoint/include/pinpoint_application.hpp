#pragma once
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

#include <pinpoint_driver/pinpoint_localization_client.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <set>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <carma_driver_msgs/msg/driver_status.hpp>
#include <carma_driver_msgs/msg/heading_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pinpoint_config.hpp>

namespace pinpoint
{
/**
 * @class PinpointApplication
 * @brief Is the class responsible for the ROS PinPoint driver
 */
class PinPointApplication : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  /**
   * \brief PinPointApplication Node constructor
   */
  explicit PinPointApplication(const rclcpp::NodeOptions &);

  ~PinPointApplication()
  {
    if (connect_thread_) {
      connect_thread_->join();
    }
  }

private:
  // parameters
  PinPointConfig config_;

  ///////////
  // DRIVER
  ///////////
  rclcpp::TimerBase::SharedPtr spin_timer_;

  /**
   * @brief Spin callback for the node to check connection, attempt connection, and check heartbeat
   * for setting driver status
   */
  void spin_callback();

  // PinPoint event handlers
  bool connecting_ = false;
  std::shared_ptr<std::thread> connect_thread_;
  rclcpp::Time last_heartbeat_time_;
  std::mutex heartbeat_mutex_;
  torc::PinPointLocalizationClient pinpoint_;
  torc::PinPointFilterAccuracy latest_filter_accuracy_;
  torc::PinPointQuaternionCovariance latest_quaternion_covariance_;

  /**
   * @brief Handles the PinPoint onConnect Event
   *
   * Establishes status of the node
   */
  void onConnectHandler();

  /**
   * @brief Handles the PinPoint onDisconnect Event
   *
   * On Disconnect this node will enter a reconnect loop attempting to the PinPoint device
   */
  void onDisconnectHandler();

  /**
   * @brief Handles the PinPoint onVelocityChanged event
   * @param vel
   *
   * This method translates the PinPoint velocity into base_link frame ad
   * publishes the velocity as a geometry_msgs::msg::Twist message out on the
   * pinpoint/position/velocity topic
   */
  void onVelocityChangedHandler(const torc::PinPointVelocity & vel);

  /**
   * @brief Handles the PinPoint onFilterAccuracyChanged event
   * @param acc
   *
   * This method stores the filter accuracy in the local member variable to use
   * for within other callbacks
   */
  void onFilterAccuracyChangedHandler(const torc::PinPointFilterAccuracy & acc);

  /**
   * @brief Handles the PinPoint onQuaternionCovarianceChanged event
   * @param quat
   *
   * This method stores the quaternion and the covariance to local member variable to use
   * within other callbacks
   */
  void onQuaternionCovarianceChangedHandler(const torc::PinPointQuaternionCovariance & quat);

  /**
   * @brief Handles the PinPoint onGlobalPoseChanged handler
   * @param pose
   *
   * This method translates the PinPoint global pose into a sensor_msgs::NavSatFix and a
   * cav_msgs::HeadingStamped message published onto the corresponding topics
   * /pinpoint/position/nav_sat_fix and /pinpoint/position/heading
   */
  void onGlobalPoseChangedHandler(const torc::PinPointGlobalPose & pose);

  /**
   * @brief Handles the PinPoint onLocalPoseChanged event
   * @param pose
   *
   * This method translates the PinPoint local pose into a nav_msgs::Odometry on topic
   * /pinpoint/position/odometry. If publish_tf is set to true it also publishes this transform
   out
   * onto tf
   */
  void onLocalPoseChangedHandler(const torc::PinPointLocalPose & pose);

  /**
   * @brief Handles the PinPoint onStatusConditionChanged event
   * @param code
   *
   * This method translates PinPoint status codes/conditions into updating the driver status
   used by
   * the DriverApplication class as well as updating diagnost_updater info
   */
  void onStatusConditionChangedHandler(
    const torc::PinPointLocalizationClient::PinPointStatusCode & code);

  ///////////
  // ROS
  ///////////

  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  carma_driver_msgs::msg::DriverStatus getStatus();
  void setStatus(const carma_driver_msgs::msg::DriverStatus status);

  // publishers
  carma_ros2_utils::PubPtr<geometry_msgs::msg::TwistStamped> velocity_pub_;
  carma_ros2_utils::PubPtr<gps_msgs::msg::GPSFix> global_pose_pub_;
  carma_ros2_utils::PubPtr<nav_msgs::msg::Odometry> local_pose_pub_;

  // tf2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ros parameters
  carma_driver_msgs::msg::DriverStatus status_;
  geometry_msgs::msg::TwistStamped latest_velocity_;

  ////////////////////////
  // Diagnostic Updater
  ////////////////////////
  /**
   * @brief This is a helper class for the Diagnostic Updater
   *
   * This class allows us to pass state information code/condition
   * to the diagnostic_updater without forcing the main class to maintain
   * a specific reference for each of these;
   */
  struct StatusMessageDiagnosticHelper
  {
    torc::PinPointLocalizationClient::StatusCode code;
    torc::StatusCondition condition;

    /**
     * @brief Returns a string representation of the condition
     * @return
     */
    std::string conditionAsString()
    {
      switch (condition) {
        case torc::StatusCondition::Clear:
          return "Clear";
        case torc::StatusCondition::Info:
          return "Info";
        case torc::StatusCondition::Warning:
          return "Warning";
        case torc::StatusCondition::Error:
          return "Error";
        default:
          return "Unknown Condition";
      }
    }

    /**
     * @brief Returns a string representation of the code
     * @return
     */
    std::string codeAsString()
    {
      switch (code) {
        case torc::PinPointLocalizationClient::StatusCode::Aligning:
          return "Aligning";
        case torc::PinPointLocalizationClient::StatusCode::NoImuData:
          return "NoImuData";
        case torc::PinPointLocalizationClient::StatusCode::NoGpsUpdates:
          return "NoGpsUpdates";
        case torc::PinPointLocalizationClient::StatusCode::NoLeftWssUpdates:
          return "NoLeftWssUpdates";
        case torc::PinPointLocalizationClient::StatusCode::NoRightWssUpdates:
          return "NoRightWssUpdates";
        case torc::PinPointLocalizationClient::StatusCode::BadGpsPosAgreement:
          return "BadGpsPosAgreement";
        case torc::PinPointLocalizationClient::StatusCode::BadGpsVelAgreement:
          return "BadGpsVelAgreement";
        case torc::PinPointLocalizationClient::StatusCode::BadWssVelAgreement:
          return "BadWssVelAgreement";
        case torc::PinPointLocalizationClient::StatusCode::BadGyroBiasEstimate:
          return "BadGyroBiasEstimate";
        case torc::PinPointLocalizationClient::StatusCode::BadAccelBiasEstimate:
          return "BadAccelBiasEstimate";
        case torc::PinPointLocalizationClient::StatusCode::PoseSteadying:
          return "PoseSteadying";
        case torc::PinPointLocalizationClient::StatusCode::NoHeadingUpdates:
          return "NoHeadingUpdates";
        case torc::PinPointLocalizationClient::StatusCode::BadHeadingAgreement:
          return "BadHeadingAgreement";
        case torc::PinPointLocalizationClient::StatusCode::BadMeasurementTime:
          return "BadMeasurementTime";
        case torc::PinPointLocalizationClient::StatusCode::IrregularTimeStep:
          return "IrregularTimeStep";
        case torc::PinPointLocalizationClient::StatusCode::BadWssScaleFactor:
          return "BadWssScaleFactor";
        default:
          return "Unknown Code";
      }
    }

    /**
     * @brief Handles the diagnostic_updater callback to update that status associated with this
     * object
     * @param stat
     */
    void processDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
    {
      unsigned char level;
      switch (condition) {
        case torc::StatusCondition::Clear: {
          level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          break;
        }
        case torc::StatusCondition::Info: {
          level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          break;
        }
        case torc::StatusCondition::Warning: {
          level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          break;
        }
        case torc::StatusCondition::Error:
        default: {
          level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          break;
        }
      }

      stat.addf(codeAsString(), conditionAsString().c_str());
      stat.summaryf(
        level, "StatusCode: %s, Condition: %s", codeAsString().c_str(),
        conditionAsString().c_str());
    }
  };

  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  /**
   * @brief Called by the diagnostic_timer_ to fire the diagnostic_updater update
   */
  void diagnosticUpdate() { updater_.force_update(); }

  diagnostic_updater::Updater updater_;
  std::map<torc::PinPointLocalizationClient::StatusCode, StatusMessageDiagnosticHelper> code_map_;
  std::set<torc::PinPointLocalizationClient::StatusCode> error_set_;
  std::set<torc::PinPointLocalizationClient::StatusCode> warning_set_;
};
}  // namespace pinpoint
