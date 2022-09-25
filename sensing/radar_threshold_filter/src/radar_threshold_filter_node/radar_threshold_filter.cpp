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

#include "radar_threshold_filter/radar_threshold_filter.hpp"

namespace radar_threshold_filter
{
RadarThresholdFilter::Output RadarThresholdFilter::update(const RadarThresholdFilter::Input & input)
{
  RadarThresholdFilter::Output output;

  // Sample
  output.data = input.data + param_.data;
  RCLCPP_INFO(rclcpp::get_logger("radar_threshold_filter"), "Debug: %d", 0);
  return output;
}

}  // namespace radar_threshold_filter
