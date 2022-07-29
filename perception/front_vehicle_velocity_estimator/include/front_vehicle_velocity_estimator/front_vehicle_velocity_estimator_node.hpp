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

#ifndef FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_NODE_HPP__
#define FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_NODE_HPP__

#include "front_vehicle_velocity_estimator/front_vehicle_velocity_estimator.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace front_vehicle_velocity_estimator
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

class FrontVehicleVelocityEstimatorNode : public rclcpp::Node
{
public:
  explicit FrontVehicleVelocityEstimatorNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
  };

private:
  // Subscriber
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_{};
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_{};
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_{};

  // Callback
  void onPointcloud(const PointCloud2::ConstSharedPtr msg);
  void onObjects(const DetectedObjects::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);

  // Data Buffer
  PointCloud2::ConstSharedPtr pointcloud_data_{};
  DetectedObjects::ConstSharedPtr objects_data_{};
  Odometry::ConstSharedPtr odometry_data_{};

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};

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
  FrontVehicleVelocityEstimator::Input input_{};
  FrontVehicleVelocityEstimator::Output output_{};
  FrontVehicleVelocityEstimator::Param core_param_{};
  std::unique_ptr<FrontVehicleVelocityEstimator> front_vehicle_velocity_estimator_{};
};

}  // namespace front_vehicle_velocity_estimator

#endif  // FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_NODE_HPP__
