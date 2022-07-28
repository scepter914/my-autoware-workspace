// SPDX-License-Identifier: Apache-2.0
// Copyright 2022 Satoshi Tanaka
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

#ifndef FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_HPP__
#define FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_HPP__

#include "rclcpp/logger.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <memory>
#include <string>
#include <vector>

namespace front_vehicle_velocity_estimator
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using sensor_msgs::msg::PointCloud2;

class FrontVehicleVelocityEstimator
{
public:
  explicit FrontVehicleVelocityEstimator(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Input
  {
    PointCloud2::ConstSharedPtr pointcloud{};
    DetectedObjects::ConstSharedPtr objects{};
  };

  struct Output
  {
    DetectedObjects::SharedPtr objects{};
  };

  struct Param
  {
    int moving_average_num{};
  };

  void setParam(const Param & param) { param_ = param; }

  Output update(const Input & input);
  DetectedObjects::SharedPtr addFrontVehicleVelocity(
    DetectedObjects::ConstSharedPtr objects_data_, PointCloud2::ConstSharedPtr pointcloud_data_);

private:
  rclcpp::Logger logger_;
  Param param_{};
};

}  // namespace front_vehicle_velocity_estimator

#endif  // FRONT_VEHICLE_VELOCITY_ESTIMATOR__FRONT_VEHICLE_VELOCITY_ESTIMATOR_HPP__
