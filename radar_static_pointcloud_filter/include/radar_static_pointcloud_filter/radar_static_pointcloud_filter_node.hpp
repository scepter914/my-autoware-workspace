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

#ifndef RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_NODE_HPP__
#define RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_NODE_HPP__

#include "radar_static_pointcloud_filter/radar_static_pointcloud_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <radar_msgs/msg/radar_scan.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_static_pointcloud_filter
{
using nav_msgs::msg::Odometry;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

class RadarStaticPointcloudFilterNode : public rclcpp::Node
{
public:
  explicit RadarStaticPointcloudFilterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    double min_sd{};
    double magnification_sd{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarScan>::SharedPtr sub_radar_{};
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_{};

  // Callback
  void onRadar(const RadarScan::ConstSharedPtr msg);
  void onOdometory(const Odometry::ConstSharedPtr msg);

  // Data Buffer
  RadarReturn::ConstSharedPtr radar_data_{};
  Odometory::ConstSharedPtr odometry_data_{};

  // Publisher
  rclcpp::Publisher<RadarScan>::SharedPtr pub_static_radar_{};
  rclcpp::Publisher<RadarScan>::SharedPtr pub_dynamic_radar_{};

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

}  // namespace radar_static_pointcloud_filter

#endif  // RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_NODE_HPP__
