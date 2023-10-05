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
#include <iomanip>
#include <pinpoint_driver/unpack_macros.h>


const double rad_to_deg = 57.295779513
const long long two_to_thirty_one = 2147483648;
const long long two_to_fifteenth  = 32768;

torc::PinPointGPSClient::PinPointGPSClient() 
{
    onConnect.connect(boost::bind(&PinPointGPSClient::connectSignals,this));
}

void torc::PinPointGPSClient::processMessage(torc::MessageType msg_type, uint8_t msg_id, std::vector<uint8_t> &msg)
{
    switch (msg_type) 
    {
        case MessageType::ReturnValue: 
        {
            switch (msg_id) 
            {
                case 2: // get status return
                {
                    uint16_t number = UNPACK_UINT16(&msg[0]);
                    for (uint16_t i = 0; i < number; i++) 
                    {
                        struct PinPointGPSClient::PinPointStatusCode status;

                        status.condition = static_cast<StatusCondition>(UNPACK_UINT8(&msg[2 + i * 3]));
                        status.code = static_cast<StatusCode>(UNPACK_UINT16(&msg[3 + i * 3]));
                        onStatusConditionChanged(status);
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case MessageType::SignalEmitted:
        {
            switch (msg_id) 
            {
                case 0: // status messages
                {
                    struct PinPointGPSClient::PinPointStatusCode status;

                    status.condition = static_cast<StatusCondition>(UNPACK_UINT8(&msg[0]));
                    status.code = static_cast<StatusCode>(UNPACK_UINT16(&msg[1]));

                    onStatusConditionChanged(status);
                    break;
                }
                case 6: // position signal emit
                {
                    struct PinPointRawGPSData position;
                    position.time = UNPACK_UINT64(&msg[0]);
                    position.latitude = rad_to_deg * (double) UNPACK_DOUBLE(&msg[8]);
                    position.longitude = rad_to_deg * (double) UNPACK_DOUBLE(&msg[16]);
                    position.altitude = (float) UNPACK_DOUBLE(&msg[24]);
                    position.pos_acc = (float) UNPACK_FLOAT(&msg[32]);
                    position.lat_vel = (float) UNPACK_FLOAT(&msg[36]);
                    position.lon_vel = (float) UNPACK_FLOAT(&msg[40]);
                    position.down_vel = (float) UNPACK_FLOAT(&msg[44]);
                    position.vel_acc = (float) UNPACK_FLOAT(&msg[48]);

                    onRawGPSDataChanged(position);
                    break;
                }
                case 7:// heading signal emit
                {
                    struct PinPointRawGPSHeading heading;
                    heading.time = UNPACK_UINT64(&msg[0]);
                    heading.heading = rad_to_deg * (float) UNPACK_FLOAT(&msg[8]);
                    heading.pitch = rad_to_deg * (float) UNPACK_FLOAT(&msg[12]);
                    heading.heading_acc = rad_to_deg * (float) UNPACK_FLOAT(&msg[16]);
                    heading.pitch_acc = rad_to_deg * (float) UNPACK_FLOAT(&msg[20]);

                    onRawGPSHeadingChanged(heading);
                    break;
                }
                case 8: // info signal emit
                {
                    struct PinPointGPSFixInfo info;

                    info.fix_type = static_cast<FixType>(UNPACK_UINT8(&msg[0]));
                    info.sat_primary = UNPACK_UINT8(&msg[1]);
                    info.sat_secondary = UNPACK_UINT8(&msg[2]);
                    info.sat_used_pos = UNPACK_UINT8(&msg[3]);
                    info.sat_used_heading = UNPACK_UINT8(&msg[4]);

                    onGPSFixInfoChanged(info);
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void torc::PinPointGPSClient::connectSignals() 
{
    std::vector<uint8_t> signalIDs =
            {
                    0, // statusChanged
                    6, // globalPoseChanged
                    7, // localPoseChanged
                    8, // velocityStateChanged
                    9, // quaternionCovarianceChanged
                    10 // filterAccuracyChanged
            };

    std::for_each(signalIDs.begin(),signalIDs.end(),[this](uint8_t n){connectSignal(n);});

    //We need to get the status because if they aren't changing we wont get the initial state
    getStatus();
}