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

#include <pinpoint_driver/pinpoint_gps_client.h>

#include <cav_driver_utils/driver_application/driver_application.h>


#include <dynamic_reconfigure/server.h>
#include <pinpoint/pinpointConfig.h>

#include <geometry_msgs/TwistStamped.h>
#include <gps_common/GPSFix.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include <map>
#include <set>
#include <tf2_ros/transform_listener.h>

/**
 * @class PinpointApplication
 * @brief Is the class responsible for the ROS PinPoint driver
 */
class PinPointApplication : public cav::DriverApplication
{
public:
    /**
     * @brief constructor
     * @param argc - command line argument count
     * @param argv - command line arguments
     */
    PinPointApplication(int argc, char** argv);

    ~PinPointApplication() 
    {
        if(connect_thread_) 
        {
            connect_thread_->join();
        }
    }

private:

    /**
     * @brief Initializes ROS context for this node
     *
     * Establishes the connection to the PinPoint hardware. Sets up pertinent events and corresponding topics
     */
    virtual void initialize() override;

    /**
     * @brief Called by the base DriverApplication class after spin
     *
     * The PinPointApplication has nothing it needs to do here
     */
    virtual void post_spin() override;

    /**
     * @brief Called by the base DriverApplication class prior to Spin
     *
     * Manages local state of pinpoint device reconnecting and monitoring
     * heartbeat as needed
     */
    virtual void pre_spin() override;

    /**
     * @brief Called by the base DriverApplication class to fetch this implementation's api
     *
     * The API is a list of fully scoped names to topics and services specified by the
     * CAV Platform architecture
     *
     * @return list of api
     */
    inline virtual std::vector<std::string>& get_api() override  { return api_; }
    std::vector<std::string> api_;

    //ROS
    ros::Publisher gps_data_pub_;
    std::shared_ptr<ros::NodeHandle> position_api_nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //parameters
    std::string base_link_frame, odom_frame, sensor_frame;
    bool publish_tf;

    pinpoint::pinpointConfig config_;
    dynamic_reconfigure::Server<pinpoint::pinpointConfig> server;
    void dynReconfigCB(pinpoint::pinpointConfig& cfg, uint32_t level);
    bool connecting_ = false;
    std::shared_ptr<std::thread> connect_thread_;

    ros::Time last_heartbeat_time_;
    std::mutex heartbeat_mutex_;

    //PinPoint event handlers
    torc::PinPointGPSClient pinpoint_;

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
     * @brief Translates status from the torc gps server into a ros gps status
     * @param fixType
     *
     */
    uint8_t PinpointGPSInfoToROSGPSStatus(torc::FixType fixType);

    /**
     * @brief Handles the PinPoint onRawGPSDataChanged handler
     * @param pose
     *
     * This method translates the PinPoint global pose into a sensor_msgs::NavSatFix and a cav_msgs::HeadingStamped
     * message published onto the corresponding topics /pinpoint/position/nav_sat_fix and /pinpoint/position/heading
     */
    void onRawGPSDataChangedHandler(const torc::PinPointRawGPSData& position);

    /**
     * @brief Handles the PinPoint onRawGPSHeadingChanged handler
     * @param pose
     *
     * This method stores the gps heading and heading accuracy in the local member variable to use
     * for within other callbacks
     */
    void onRawGPSHeadingChangedHandler(const torc::PinPointRawGPSHeading& heading);

    /**
     * @brief Handles the PinPoint onGPSFixInfoChanged event
     * @param acc
     *
     * This method stores the gps status and satellite information in the local member variable to use
     * for within other callbacks
     */
    void onGPSFixInfoChangedHandler(const torc::PinPointGPSFixInfo& info);

    /**
     * @brief Handles the PinPoint onStatusConditionChanged event
     * @param code
     *
     * This method translates PinPoint status codes/conditions into updating the driver status used by the DriverApplication
     * class as well as updating diagnost_updater info
     */
    void onStatusConditionChangedHandler(const torc::PinPointGPSClient::PinPointStatusCode& code );

    torc::PinPointRawGPSHeading latest_heading_;
    torc::PinPointGPSFixInfo latest_info_;

    //Diagnostic Updater
    /**
     * @brief This is a helper class for the Diagnostic Updater
     *
     * This class allows us to pass state information code/condition
     * to the diagnostic_updater without forcing the main class to maintain
     * a specific reference for each of these;
     */
    struct StatusMessageDiagnosticHelper
    {
        torc::PinPointGPSClient::StatusCode code;
        torc::StatusCondition condition;

        /**
         * @brief Returns a string representation of the condition
         * @return
         */
        std::string conditionAsString()
        {
            switch(condition)
            {
                case torc::StatusCondition::Clear: return "Clear";
                case torc::StatusCondition::Info: return "Info";
                case torc::StatusCondition::Warning: return "Warning";
                case torc::StatusCondition::Error: return "Error";
                default: return "Unknown Condition";
            }
        }

        /**
         * @brief Returns a string representation of the code
         * @return
         */
        std::string codeAsString()
        {
            switch(code)
            {
                case torc::PinPointGPSClient::StatusCode::CommsFailure: return "CommsFailure";
                case torc::PinPointGPSClient::StatusCode::ReceiverFailure: return "ReceiverFailure";
                case torc::PinPointGPSClient::StatusCode::BadTemperature: return "BadTemperature";
                case torc::PinPointGPSClient::StatusCode::BadVoltage: return "BadVoltage";
                case torc::PinPointGPSClient::StatusCode::AntennaOpen: return "AntennaOpen";
                case torc::PinPointGPSClient::StatusCode::AntennaShort: return "AntennaShort";
                case torc::PinPointGPSClient::StatusCode::CpuOverload: return "CpuOverload";
                case torc::PinPointGPSClient::StatusCode::InvalidAlmanac: return "InvalidAlmanac";
                case torc::PinPointGPSClient::StatusCode::InvalidClock: return "InvalidClock";
                case torc::PinPointGPSClient::StatusCode::InvalidPosition: return "InvalidPosition";
                default: return "Unknown Code";
            }
        }

        /**
         * @brief Handles the diagnostic_updater callback to update that status associated with this object
         * @param stat
         */
        void processDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
        {
            unsigned char level;
            switch(condition)
            {
                case torc::StatusCondition::Clear:
                {
                    level = diagnostic_msgs::DiagnosticStatus::OK;
                    break;
                }
                case torc::StatusCondition::Info:
                {
                    level = diagnostic_msgs::DiagnosticStatus::OK;
                    break;
                }
                case torc::StatusCondition::Warning:
                {
                    level = diagnostic_msgs::DiagnosticStatus::WARN;
                    break;
                }
                case torc::StatusCondition::Error:
                default:
                {
                    level = diagnostic_msgs::DiagnosticStatus::ERROR;
                    break;
                }
            }

            stat.addf(codeAsString(), conditionAsString().c_str());
            stat.summaryf(level, "StatusCode: %s, Condition: %s", codeAsString().c_str(), conditionAsString().c_str());
            //std::cout << "Code: " << codeAsString().c_str() << ", Condition: " << conditionAsString().c_str() << std::endl;
        }
    };

    ros::Timer diagnostic_timer_;

    /**
     * @brief Called by the diagnostic_timer_ to fire the diagnostic_updater update
     */
    void diagnosticUpdate(const ros::TimerEvent&)
    {
        updater_.update();
    }

    diagnostic_updater::Updater updater_;
    std::map<torc::PinPointGPSClient::StatusCode, StatusMessageDiagnosticHelper> code_map_;
    std::set<torc::PinPointGPSClient::StatusCode> error_set_;
    std::set<torc::PinPointGPSClient::StatusCode> warning_set_;

};