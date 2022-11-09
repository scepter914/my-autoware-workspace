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

#ifndef RADAR_THRESHOLD_FILTER__RADAR_THRESHOLD_FILTER_NODE_HPP__
#define RADAR_THRESHOLD_FILTER__RADAR_THRESHOLD_FILTER_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include <radar_msgs/msg/radar_scan.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_threshold_filter
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

class RadarThresholdFilterNode : public rclcpp::Node
{
public:
  explicit RadarThresholdFilterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarScan>::SharedPtr sub_data_{};

  // Callback
  void onData(const RadarScan::ConstSharedPtr msg);

  // Data Buffer
  RadarScan::ConstSharedPtr radar_data_{};

  // Publisher
  rclcpp::Publisher<RadarScan>::SharedPtr pub_data_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  bool isDataReady();
  void onTimer();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};
};

}  // namespace radar_threshold_filter

#endif  // RADAR_THRESHOLD_FILTER__RADAR_THRESHOLD_FILTER_NODE_HPP__
