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

#ifndef RADAR_OBJECT_TRACKING__RADAR_OBJECT_TRACKING_NODE_HPP__
#define RADAR_OBJECT_TRACKING__RADAR_OBJECT_TRACKING_NODE_HPP__

#include "radar_object_tracking/radar_object_tracking.hpp"

#include <example_interfaces/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_object_tracking
{
using example_interfaces::msg::Int32;

class RadarObjectTrackingNode : public rclcpp::Node
{
public:
  explicit RadarObjectTrackingNode(const rclcpp::NodeOptions & node_options);

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
  int time_frame_ = 0;

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
  RadarObjectTracking::Input input_{};
  RadarObjectTracking::Output output_{};
  RadarObjectTracking::Param core_param_{};
  std::unique_ptr<RadarObjectTracking> radar_object_tracking_{};
};

}  // namespace radar_object_tracking

#endif  // RADAR_OBJECT_TRACKING__RADAR_OBJECT_TRACKING_NODE_HPP__
