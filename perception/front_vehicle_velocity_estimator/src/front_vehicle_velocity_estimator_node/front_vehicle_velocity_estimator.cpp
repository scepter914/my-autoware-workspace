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

#include <boost/geometry.hpp>

#include <utility>

namespace front_vehicle_velocity_estimator
{
FrontVehicleVelocityEstimator::Output FrontVehicleVelocityEstimator::update(
  const FrontVehicleVelocityEstimator::Input & input)
{
  // Filter front vehicle
  LinearRing2d front_area = createBoxArea(50.0, 4.0);
  Output output_{};
  DetectedObject front_vehicle{};
  auto p_ = filterFrontVehicle(input.objects, front_area);
  output_ = p_.first;
  front_vehicle = p_.second;
  // RCLCPP_INFO(rclcpp::get_logger("front_vehicle_velocity_estimator"), "Debug: %d", 0);

  // Get nearest neighbor pointcloud
  pcl::PointXYZ nearest_neighbor_point = getNearestNeighborPoint(front_vehicle, input.pointcloud);

  // Estimate velocity
  double velocity = estimateVelocity(input.odometry, input.pointcloud->header.stamp);
  RCLCPP_INFO(rclcpp::get_logger("front_vehicle_velocity_estimator"), "Velocity: %f", velocity);

  // Set queue of nearest_neighbor_point
  if ((int)velocity_queue_.size() >= param_.moving_average_num) {
    double _old_velocity = velocity_queue_.front();
  }
  velocity_queue_.push_back(velocity);

  // Set prev_time
  prev_time_ = input.pointcloud->header.stamp;

  // Set objects output
  front_vehicle.kinematics.has_twist = true;
  front_vehicle.kinematics.twist_with_covariance.twist.linear.x = velocity;
  front_vehicle.kinematics.has_twist_covariance = true;
  front_vehicle.kinematics.twist_with_covariance.covariance.at(0) = 1.0;
  output_.objects->objects.emplace_back(front_vehicle);

  // Set nearest_neighbor_pointcloud output for debug
  pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
  output_pointcloud.points.push_back(nearest_neighbor_point);
  pcl::toROSMsg(output_pointcloud, *(output_.nearest_neighbor_pointcloud));
  output_.nearest_neighbor_pointcloud->header = input.pointcloud->header;

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

// Create object area
LinearRing2d FrontVehicleVelocityEstimator::createObjectArea(const DetectedObject & object)
{
  Point2d object_size{object.shape.dimensions.x, object.shape.dimensions.y};
  const double x_front = object_size.x() / 2.0;
  const double x_rear = -object_size.x() / 2.0;
  const double y_left = object_size.y() / 2.0;
  const double y_right = -object_size.y() / 2.0;

  LinearRing2d box{};
  box.push_back(Point2d{x_front, y_left});
  box.push_back(Point2d{x_front, y_right});
  box.push_back(Point2d{x_rear, y_right});
  box.push_back(Point2d{x_rear, y_left});
  box.push_back(Point2d{x_front, y_left});

  LinearRing2d transformed_box = tier4_autoware_utils::transformVector(
    box, tier4_autoware_utils::pose2transform(object.kinematics.pose_with_covariance.pose));

  return transformed_box;
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
  return std::pair<FrontVehicleVelocityEstimator::Output, DetectedObject>(output_, front_vehicle);
}

pcl::PointXYZ FrontVehicleVelocityEstimator::getNearestNeighborPoint(
  const DetectedObject & object, PointCloud2::ConstSharedPtr pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointcloud, *pcl_msg);

  // Initialize
  pcl::PointXYZ nearest_neighbor_point;
  bool is_initialized = false;

  for (const auto & point : *pcl_msg) {
    LinearRing2d object_ring_2d = createObjectArea(object);
    Point2d point_{point.x, point.y};
    if (
      boost::geometry::within(point_, object_ring_2d) && point.z > param_.threshold_pointcloud_z) {
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
  dt = return 0.0;
}

}  // namespace front_vehicle_velocity_estimator
