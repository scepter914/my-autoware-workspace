// Copyright 2022 TIER IV, Inc.
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

#include "radar_scan_to_pointcloud2/radar_scan_to_pointcloud2.hpp"

namespace radar_scan_to_pointcloud2
{
RadarScanToPointcloud2::Output RadarScanToPointcloud2::update(const RadarScanToPointcloud2::Input & input)
{
  RadarScanToPointcloud2::Output output;

  // Sample
  output.data = input.data + param_.data;
  RCLCPP_INFO(rclcpp::get_logger("radar_scan_to_pointcloud2"), "Debug: %d", 0);
  return output;
}

}  // namespace radar_scan_to_pointcloud2
