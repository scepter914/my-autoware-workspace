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

#ifndef RADAR_SCAN_TO_POINTCLOUD2__RADAR_SCAN_TO_POINTCLOUD2_NODE_HPP__
#define RADAR_SCAN_TO_POINTCLOUD2__RADAR_SCAN_TO_POINTCLOUD2_NODE_HPP__

#include "example_interfaces/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_scan_to_pointcloud2
{
using example_interfaces::msg::Int32;

class RadarScanToPointcloud2Node : public rclcpp::Node
{
public:
  explicit RadarScanToPointcloud2Node(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
  };

private:
  // Subscriber
  rclcpp::Subscription<Int32>::SharedPtr sub_data_{};

  // Callback
  void onData(const Int32::ConstSharedPtr msg);

  // Data Buffer
  Int32::ConstSharedPtr data_{};

  // Publisher
  rclcpp::Publisher<Int32>::SharedPtr pub_data_{};

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

  // Core
  RadarScanToPointcloud2::Input input_{};
  RadarScanToPointcloud2::Output output_{};
  RadarScanToPointcloud2::Param core_param_{};
  std::unique_ptr<RadarScanToPointcloud2> radar_scan_to_pointcloud2_{};
};

}  // namespace radar_scan_to_pointcloud2

#endif  // RADAR_SCAN_TO_POINTCLOUD2__RADAR_SCAN_TO_POINTCLOUD2_NODE_HPP__
