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

#include "rclcpp/rclcpp.hpp"

#include <radar_msgs/msg/radar_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_scan_to_pointcloud2
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;
using sensor_msgs::msg::PointCloud2;

class RadarScanToPointcloud2Node : public rclcpp::Node
{
public:
  explicit RadarScanToPointcloud2Node(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    std::string intensity_value_mode{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarScan>::SharedPtr sub_radar_{};

  // Callback
  void onData(const RadarScan::ConstSharedPtr msg);

  // Data Buffer
  RadarScan::ConstSharedPtr radar_data_{};

  // Publisher
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_{};

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

}  // namespace radar_scan_to_pointcloud2

#endif  // RADAR_SCAN_TO_POINTCLOUD2__RADAR_SCAN_TO_POINTCLOUD2_NODE_HPP__
