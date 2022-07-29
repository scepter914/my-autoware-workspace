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
  // Filter front vehicle
  LinearRing2d front_area = createBoxArea(50.0, 4.0);
  // Output output_{};
  // DetectedObject front_vehicle{};
  auto [output_, front_vehicle] = filterFrontVehicle(input.objects, front_area);
  // RCLCPP_INFO(rclcpp::get_logger("front_vehicle_velocity_estimator"), "Debug: %d", 0);

  // Get nearest neighbor pointcloud
  pcl::PointXYZ nearest_neighbor_point = getNearestNeighbor(front_vehicle, input.pointcloud);

  // Set output
  pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
  output_pointcloud.points.push_back(nearest_neighbor_point);
  auto pointcloud_msg = output_pointcloud.makeShared();
  // pcl::toROSMsg(output_pointcloud, *(output_.nearest_neighbor_pointcloud));
  output_.nearest_neighbor_pointcloud = pointcloud_msg;

  // Set queue of nearest_neighbor_point
  if ((int)nearest_neighbor_point_queue.size() >= param_.moving_average_num) {
    auto _old_point = nearest_neighbor_point_queue.pop_front();
  }
  nearest_neighbor_point_queue.push_back(*output_.nearest_neighbor_pointcloud);

  // Estimate velocity
  double velocity = estimateVelocity(input.odometry);

  front_vehicle.kinematics.has_twist = true;
  front_vehicle.kinematics.twist_with_covariance.twist.linear.x = velocity;
  front_vehicle.kinematics.has_twist_covariance = true;
  front_vehicle.kinematics.twist_with_covariance.covariance.at(0) = 1.0;
  output_.objects->objects.emplace_back(front_vehicle);

  RCLCPP_INFO(rclcpp::get_logger("front_vehicle_velocity_estimator"), "Velocity: %f", velocity);

  return output_;
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

std::pair<FrontVehicleVelocityEstimator::Output, DetectedObject>
FrontVehicleVelocityEstimator::filterFrontVehicle(
  DetectedObjects::ConstSharedPtr objects, const LinearRing2d & front_area)
{
  // Initialize output
  Output output_{};
  output_.objects->header = objects->header;

  DetectedObject front_vehicle;
  bool is_initialized = false;

  for (const auto & object : objects->objects) {
    auto position = object.kinematics.pose_with_covariance.pose.position;
    Point2d object_point{position.x, position.y};

    if (!boost::geometry::within(object_point, front_area)) {
      output_.objects->objects.emplace_back(object);
    } else if (!is_initialized) {
      front_vehicle = object;
      is_initialized = true;
    } else {
      auto front_vehicle_position = front_vehicle.kinematics.pose_with_covariance.pose.position;
      if (position.x < front_vehicle_position.x) {
        output_.objects->objects.emplace_back(front_vehicle);
        front_vehicle = object;
      } else {
        output_.objects->objects.emplace_back(object);
      }
    }
  }
  return std::pair(output_, front_vehicle);
}

pcl::PointXYZ FrontVehicleVelocityEstimator::getNearestNeighbor(
  const DetectedObject & object, PointCloud2::ConstSharedPtr pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointcloud, *pcl_msg);

  // Initialize
  pcl::PointXYZ nearest_neighbor_point;
  bool is_initialized = false;

  for (const auto & point : *pcl_msg) {
    Point2d point_{point.x, point.y};
    if (boost::geometry::within(point_, object_ring_2d)) {
      if (!is_initialized) {
        nearest_neighbor_point = point;
      } else if (point.x < nearest_neighbor_point.x) {
        nearest_neighbor_point = point;
      }
    }
  }

  return nearest_neighbor_point;
}

double FrontVehicleVelocityEstimator::estimateVelocity(Odometry::ConstSharedPtr odometry)
{
  return 0.0;
}

}  // namespace front_vehicle_velocity_estimator
