#pragma once
/*
 * Copyright (C) 2024 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <iostream>

/**
 * \brief Struct containing configuration parameters for pinpoint
 */
struct PinPointConfig
{
  std::string base_link_frame = "base_link";
  std::string odom_frame = "odom";
  std::string sensor_frame = "pinpoint";
  std::string address = "10.26.4.73";
  std::string loc_port = "9501";
  bool publish_tf = false;
  int spin_rate = 50;

  friend std::ostream & operator<<(std::ostream & output, const PinPointConfig & c)
  {
    output << "PinpointConfig { " << std::endl
           << "base_link_frame: " << c.base_link_frame << std::endl
           << "odom_frame: " << c.odom_frame << std::endl
           << "sensor_frame: " << c.sensor_frame << std::endl
           << "address: " << c.address << std::endl
           << "loc_port: " << c.loc_port << std::endl
           << "publish_tf: " << c.publish_tf << std::endl
           << "spin_rate: " << c.spin_rate << std::endl
           << "}" << std::endl;
    return output;
  }
};
