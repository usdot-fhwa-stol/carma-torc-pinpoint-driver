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

#include "pinpoint_driver.h"
#include <sstream>

namespace torc
{

/**
 * @brief PinPoint rawgpsdata structure
 *
 * @var time            microseconds since 1970
 * @var latitude        latitude (deg)
 * @var longitude       longitude (deg)
 * @var altitude        altitude (m)
 * @var pos_acc         position accuracy (m)
 * @var lat_vel         north velocity (m/s)
 * @var lon_vel         east velocity (m/s)
 * @var down_vel        down velocity (m/s)
 * @var vel_acc         velocity accuracy (m/s)
 *
 */
struct PinPointRawGPSData {
    uint64_t time;
    double latitude;
    double longitude;
    float altitude;
    float pos_acc;
    float lat_vel;
    float lon_vel;
    float down_vel;
    float vel_acc;
};


/**
 * @brief PinPoint rawgpsheading structure
 *
 * @var time            microseconds since 1970
 * @var heading         antenna heading (deg)
 * @var pitch           antenna heading (deg)
 * @var heading_acc     heading accuracy (deg)
 * @var pitch_acc       pitch accuracy (deg)
 *
 */
struct PinPointRawGPSHeading {
    uint64_t time;
    float heading;
    float pitch;
    float heading_acc;
    float pitch_acc;
};


/**
 * @enum FixType
 * @brief GPS fix type provided by the PinPoint gps server
 */
enum class FixType : uint8_t
{
    Unknown = 0,
    None = 1,
    twoD = 2,
    threeD = 3,
    SBAS = 4,
    OmniSTARVBS = 5,
    OmniSTARXP = 6,
    OmniSTARHP = 7,
    Terrastar = 8
};


/**
 * @brief PinPoint gpsfixinfo structure
 *
 * @var fix_type            enum fix type
 * @var sat_primary         latitude (deg)
 * @var sat_secondary       number of satellites seen by the 
 * @var sat_used_pos        number of satellited used in the heading computation
 * @var sat_used_heading    number of satellited used in the heading computation
 *
 */
struct PinPointGPSFixInfo {
    FixType fix_type;
    uint8_t sat_primary;
    uint8_t sat_secondary;
    uint8_t sat_used_pos;
    uint8_t sat_used_heading;
};


template <typename T>
struct VectorNED
{
    T north;
    T east;
    T down;
};


/**
 * @brief Implements the PinPoint driver connecting to the PinPoint gps server
 */
class PinPointGPSClient : public PinPoint
{
public:
    
    // /**
    //  * @brief Initializes the base PinPointGPSClient class
    //  *
    //  * The underlying io service is setup
    //  */
    // PinPointGPSClient();
    // virtual ~PinPointGPSClient();

    /**
     * @enum StatusCode
     * @brief Status Codes provided by the PinPoint gps server
     */
    enum class StatusCode : uint16_t
    {
        CommsFailure = 1,
        ReceiverFailure = 2,
        BadTemperature = 3,
        BadVoltage = 4,
        AntennaOpen = 5,
        AntennaShort = 6,
        CpuOverload = 7,
        InvalidAlmanac = 8,
        InvalidClock = 9,
        InvalidPosition = 10
    };

    struct PinPointStatusCode
    {
        torc::StatusCondition condition;
        PinPointGPSClient::StatusCode code;
    };

    PinPointGPSClient();

    /**
     * @brief Signaled onRawGPSDataChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointRawGPSData const &)> onRawGPSDataChanged;


    /**
     * @brief Signaled onRawGPSHeadingChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointRawGPSHeading const &)> onRawGPSHeadingChanged;


    /**
     * @brief Signaled onGPSFixInfoChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointGPSFixInfo const &)> onGPSFixInfoChanged;


    /**
     * @brief Signaled onStatusConditionChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointGPSClient::PinPointStatusCode const &)> onStatusConditionChanged;


protected:

    /**
     * @brief Processes messages received from PinPoint device
     * @param msg_type
     * @param msg_id
     * @param msg
     */
    virtual void processMessage(MessageType msg_type, uint8_t msg_id, std::vector<uint8_t>& msg) override;

private:

    /**
     * @brief Connect the signals PinPointGPSClient is interested in
     */
    void connectSignals();

};

}
