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

#include "front_vehicle_velocity_estimator/front_vehicle_velocity_estimator.hpp"

namespace front_vehicle_velocity_estimator
{
FrontVehicleVelocityEstimator::Output FrontVehicleVelocityEstimator::update(
  const FrontVehicleVelocityEstimator::Input & input)
{
  // Initialize output
  FrontVehicleVelocityEstimator::Output output{};
  output.objects.header = input.objects->header;

  LinearRing2d front_area = createBoxArea(50.0, 4.0);
  DetectedObject front_vehicle = filterFrontVehicle(input.objects, front_area);
  // RCLCPP_INFO(rclcpp::get_logger("front_vehicle_velocity_estimator"), "Debug: %d", 0);

  double velocity = estimateVelocity(front_object, input.pointcloud, input.odometry);

  // Sample
  // RCLCPP_INFO(rclcpp::get_logger("front_vehicle_velocity_estimator"), "Debug: %d", 0);
  return output;
}

// Create front area
LinearRing2d FrontVehicleVelocityEstimator::createBoxArea(const double x_size, const double y_size)
{
  LinearRing2d box{};
  box.push_back(Point2d{0.0, y_size / 2.0});
  box.push_back(Point2d{x_size, y_size / 2.0});
  box.push_back(Point2d{x_size, -y_size / 2.0});
  box.push_back(Point2d{0.0, -y_size / 2.0});
  box.push_back(Point2d{0.0, y_size / 2.0});
  return box;
}

// Filter for a front vehicle.
// Objects except the front vehicle are pushed to output objects
DetectedObject FrontVehicleVelocityEstimator::filterFrontVehicle(
  DetectedObjects::ConstSharedPtr objects, LinearRing2d & front_area)
{
  DetectedObject front_vehicle;
  bool is_initialized = false;

  for (const auto & object : objects) {
    auto position = object.pose_with_covariance.pose.position;
    Point2d object_point{position.x, position.y};

    if (!boost::geometry::within(object_point, front_area)) {
      outputs.objects.objects.emplace_back(object);
    } else if (!is_initialized) {
      front_vehicle = object;
      is_initialized = true;
    } else {
      auto front_vehicle_position = front_vehicle.kinematics.pose_with_covariance.pose.position;
      if (position.x < front_vehicle_position.x) {
        outputs.objects.objects.emplace_back(front_vehicle);
        front_vehicle = object;
      } else {
        outputs.objects.objects.emplace_back(object);
      }
    }
  }
  return front_vehicle;
}

double FrontVehicleVelocityEstimator::estimateVelocity(
  DetectedObject & object, PointCloud2::SharedPtr pointcloud, Odometry::ConstSharedPtr odometry)
{
}

}  // namespace front_vehicle_velocity_estimator
