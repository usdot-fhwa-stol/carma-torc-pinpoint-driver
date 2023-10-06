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

#include <pinpoint_application_gps.h>
#include <gps_common/GPSFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cav_msgs/HeadingStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

const double PI = 3.145159265359;
inline double deg2rad(double deg) { return deg*PI/180.0;}
inline double rad2deg(double rad) { return rad*180.0/PI;}

PinPointApplication::PinPointApplication(int argc, char **argv) : cav::DriverApplication(argc, argv, "pinpoint"),
                                                                  latest_heading_(), latest_info_()
{
    cav_msgs::DriverStatus status;
    status.status = cav_msgs::DriverStatus::OFF;
    status.gnss = true;
    setStatus(status);
}

void PinPointApplication::initialize() 
{

    // CAV platform requires that the position api falls under the /pinpoint/position namespace
    position_api_nh_.reset(new ros::NodeHandle("/raw_gps"));

    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    // Pinpoint address
    pnh_->param<std::string>("address", config_.address, "10.26.4.73");
    pnh_->param<std::string>("gps_port", config_.loc_port, "9503");

    // Frames
    pnh_->param<std::string>("odom_frame", odom_frame, "odom");
    pnh_->param<std::string>("base_link_frame", base_link_frame, "base_link");
    pnh_->param<std::string>("sensor_frame", sensor_frame, "pinpoint");

    pnh_->param<bool>("publish_tf", publish_tf, false);

    // Setup connection handlers
    pinpoint_.onConnect.connect([this]() { onConnectHandler(); });
    pinpoint_.onDisconnect.connect([this]() { onDisconnectHandler(); });

    server.setCallback([this](pinpoint::pinpointConfig& cfg, uint32_t level){ dynReconfigCB(cfg,level);});

    // Setup the API publishers
    std::string node_name = ros::this_node::getName();
    api_.clear();

    // RawGPSData
    gps_data_pub_ = position_api_nh_->advertise<gps_common::GPSFix>("pinpoint_fix", 1);
    api_.push_back(gps_data_pub_.getTopic());

    pinpoint_.onRawGPSDataChanged
            .connect([this](torc::PinPointRawGPSData const &pose) { onRawGPSDataChangedHandler(pose); });

    // RawGPSHeading
    pinpoint_.onRawGPSHeadingChanged
            .connect([this](torc::PinPointRawGPSHeading const &pose) { onRawGPSHeadingChangedHandler(pose); });

    // Other non-published pinpoint data
    pinpoint_.onGPSFixInfoChanged
            .connect([this](torc::PinPointGPSFixInfo const &acc) { onGPSFixInfoChangedHandler(acc); });

    pinpoint_.onStatusConditionChanged
            .connect([this](torc::PinPointGPSClient::PinPointStatusCode const &code) {
                onStatusConditionChangedHandler(code);
            });

    // Initialize time
    last_heartbeat_time_ = ros::Time::now();
    pinpoint_.onHeartbeat.connect([this](){
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        last_heartbeat_time_ = ros::Time::now();
    });

    updater_.setHardwareID("PinPoint");
    diagnostic_timer_ = nh_->createTimer(ros::Duration(1), &PinPointApplication::diagnosticUpdate, this);

    spin_rate = 50;
}

void PinPointApplication::onConnectHandler() 
{
    ROS_INFO_STREAM("PinPoint Connected");

    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
}

void PinPointApplication::onDisconnectHandler() 
{
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OFF;
    setStatus(status);

    ROS_WARN_STREAM("PinPoint Disconnected");
}


/**
 * Converts from a torc pinpoint status to a ROS gps status
 * We don't have access to whether or not the pinpoint has RTK corrections active
 *
 */
gps_common::GPSStatus PinPointApplication::PinpointGPSInfoToROSGPSStatus(torc::FixType fixType)
{
    switch (fixType)
    {
        case torc::FixType::Unknown:
        case torc::FixType::None:
        {
            return gps_common::GPSStatus::STATUS_NO_FIX;
        }
        case torc::FixType::twoD:
        case torc::FixType::threeD:
        {
            return gps_common::GPSStatus::STATUS_FIX;
        }
        case torc::FixType::SBAS:
        {
            return gps_common::GPSStatus::STATUS_SBAS_FIX;
        }
        case torc::FixType::OmniSTARVBS:
        case torc::FixType::OmniSTARXP:
        case torc::FixType::OmniSTARHP:
        case torc::FixType::Terrastar:
        {
            return gps_common::GPSStatus::STATUS_DGPS_FIX;
        }
    }
}

/**
 * Publishes nav_sat_fix messages from PinPoint globalPose structure
 *
 * Publishes cav_msgs/HeadingStamped msg from the PinPoint globalPose
 *
 */
void PinPointApplication::onRawGPSDataChangedHandler(const torc::PinPointRawGPSData &position) 
{
   
    gps_common::GPSFix msg; 
    msg.header.frame_id = sensor_frame;

    try 
    {
        msg.header.stamp.fromNSec(position.time * static_cast<uint64_t>(1000));
        msg.status.header.stamp.fromNSec(latest_heading_.time * static_cast<uint64_t>(1000));
    }
    catch(std::runtime_error e)
    {
        ROS_WARN_STREAM("onRawGPSDataChangedHandler threw exception in ros::TimeBase::fromNSec(), time : " << position.time);
        return;
    }

    msg.latitude = position.latitude;
    msg.longitude = position.longitude;
    msg.altitude = position.altitude;

    ROS_DEBUG_STREAM("Lat: " << position.latitude << " Lon: " << position.longitude << " Alt: " << position.altitude);

    msg.track = latest_heading_.heading < 0 ? 360 + latest_heading_.heading : latest_heading_.heading;
    msg.speed = math.sqrt(math.pow(position.lat_vel, 2) + math.pow(position.lon_vel, 2));
    msg.climb = -position.down_vel;

    msg.err = position.pos_acc;
    msg.err_horz = position.pos_acc;
    msg.err_vert = position.pos_acc;
    msg.err_track = latest_heading_.heading_acc;
    msg.error_speed = position.vel_acc;
    msg.error_climb = position.vel_acc;

    msg.status.satellites_used = latest_info_.sat_used_pos;
    msg.status.satellites_visible = latest_info_.sat_primary;
    msg.status.position_source = gps_common::GPSStatus::SOURCE_GPS;
    msg.status.motion_source = gps_common::GPSStatus::SOURCE_GPS;
    msg.status.status = PinPointApplication::PinpointGPSInfoToROSGPSStatus(latest_info_.fix_type);

    gps_data_pub_.publish(msg);
}

void PinPointApplication::onRawGPSHeadingChangedHandler(const torc::PinPointRawGPSHeading &heading) 
{
    latest_heading_ = heading;
}

void PinPointApplication::onGPSFixInfoChangedHandler(const torc::PinPointGPSFixInfo &info) 
{
    latest_info_ = info;
}

void PinPointApplication::onStatusConditionChangedHandler(const torc::PinPointGPSClient::PinPointStatusCode &code) 
{
    // Check to see if we are already tracking this code
    auto it = code_map_.find(code.code);
    if (it == code_map_.end()) 
    {
        // If the code is not tracked we need to create a DiagnostHelper object for it and add it to our sotre
        StatusMessageDiagnosticHelper stat;
        stat.code = code.code;
        stat.condition = code.condition;
        code_map_[code.code] = stat;

        // Then we add it to the diagnostic_updater callbacks
        updater_.add(code_map_[code.code].codeAsString(), &code_map_[code.code],
                     &StatusMessageDiagnosticHelper::processDiagnostics);
    }

    // update the condition
    code_map_[code.code].condition = code.condition;

    // For the driverstatus state we need to know if we are in error or warning we maintain two sets of the
    if (code.condition == torc::StatusCondition::Error) 
    {
        warning_set_.erase(code.code);
        error_set_.insert(code.code);
        ROS_WARN_STREAM("Error detected, code = " << static_cast<uint16_t>(code.code));
    } 
    else if (code.condition == torc::StatusCondition::Warning) 
    {
        warning_set_.insert(code.code);
        error_set_.erase(code.code);
    } 
    else 
    {
        warning_set_.erase(code.code);
        error_set_.erase(code.code);
    }


    // We assume status is unchanged if either of the sets contain items then we set status accordingly
    cav_msgs::DriverStatus status = getStatus();
    if (!error_set_.empty()) 
    {
        status.status = cav_msgs::DriverStatus::FAULT;
        setStatus(status);
        ROS_WARN_STREAM("Publishing FAULT status. " << error_set_.size() << " errors.");
    } 
    else if (!warning_set_.empty())
    {
        status.status = cav_msgs::DriverStatus::DEGRADED;
        setStatus(status);
    } 
    else
    {
        status.status = cav_msgs::DriverStatus::OPERATIONAL;
        setStatus(status);
    }
}

void PinPointApplication::pre_spin() 
{
    // If we are not connected
    if(!connecting_ && !pinpoint_.connected())
    {
        connecting_ = true;
        if(connect_thread_)
        {
            connect_thread_->join();
        }

        // We don't want to block the spin thread because the driver application maintains driver status topic
        connect_thread_.reset( new std::thread([this]()
                                               {

                                                   ROS_INFO("Attempting to connect pinpoint");
                                                   boost::system::error_code ec;
                                                   if(!pinpoint_.Connect(config_.address,config_.loc_port,ec))
                                                   {
                                                       ROS_WARN_STREAM("Failed to connect, err: "<<ec.message());
                                                   }

                                                   connecting_ = false;
                                               }));
    }
    else if(pinpoint_.connected()) // If we are connected lets make sure we are getting updates
    {
        ros::Time last;
        {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            last = last_heartbeat_time_;
        }

        try
        {
            ros::Duration time = ros::Time::now() - last;

            if(time.sec > 1 && time.sec % 5 == 0)
            {
                ROS_WARN_STREAM_THROTTLE(5, "No heartbeat received from pinpoint in " << time.sec << " seconds");
                cav_msgs::DriverStatus status = getStatus();
                if(status.status != cav_msgs::DriverStatus::FAULT)
                {
                    status.status = cav_msgs::DriverStatus::FAULT;
                    setStatus(status);
                }
            }

            if(time.sec > 30)
            {
                ROS_WARN_STREAM("Connection to pinpoint timeout");
                pinpoint_.Close();
            }

        }
        catch(std::runtime_error e)
        {
            ROS_WARN_STREAM("pre_spin threw exception in ros::TimeBase::fromNSec(), ex: " << e.what());
            return;
        }
    }
}

void PinPointApplication::post_spin() 
{
    // We don't have anything to do
}

void PinPointApplication::dynReconfigCB(pinpoint::pinpointConfig& cfg, uint32_t level)
{
    if(config_.address != cfg.address || config_.loc_port != cfg.loc_port)
    {
        ROS_INFO_STREAM("DynReconfig address " << cfg.address << " port " << cfg.loc_port);
        config_ = cfg;
        pinpoint_.Close();
    }
}
