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

#include <memory>
#include <numeric>
#include <utility>

namespace front_vehicle_velocity_estimator
{
FrontVehicleVelocityEstimator::Output FrontVehicleVelocityEstimator::update(
  const FrontVehicleVelocityEstimator::Input & input)
{
  Output output_{};
  ObjectsWithFrontVehicle objects_with_front_vehicle{};

  // Filter front vehicle
  LinearRing2d front_area = createBoxArea(100.0, 6.0);
  objects_with_front_vehicle = filterFrontVehicle(input.objects, front_area);

  // Get nearest neighbor pointcloud
  pcl::PointXYZ nearest_neighbor_point =
    getNearestNeighborPoint(objects_with_front_vehicle.front_vehicle, input.pointcloud);

  // Set objects output
  output_.objects = *(objects_with_front_vehicle.objects_without_front_vehicle);

  // If there is no front vehicle, return output
  if (!objects_with_front_vehicle.is_front_vehicle) {
    return output_;
  }

  // Estimate relative velocity
  double now_relative_velocity =
    estimateRelativeVelocity(nearest_neighbor_point, input.pointcloud->header.stamp);

  // Estimate absolute velocity
  double now_absolute_velocity = estimateAbsoluteVelocity(now_relative_velocity, input.odometry);

  // Validate velocity
  if (
    !isfinite(now_relative_velocity) ||
    now_relative_velocity > param_.threshold_relative_velocity ||
    now_relative_velocity < -param_.threshold_relative_velocity) {
    output_.objects.objects.emplace_back(objects_with_front_vehicle.front_vehicle);
    return output_;
  }
  now_absolute_velocity = std::min(param_.threshold_absolute_velocity, now_absolute_velocity);

  // Set queue of nearest_neighbor_point
  if ((int)velocity_queue_.size() >= param_.moving_average_num) {
    velocity_queue_.pop_front();
  }

  // Estimate average velocity
  velocity_queue_.push_back(now_absolute_velocity);
  double velocity = std::accumulate(std::begin(velocity_queue_), std::end(velocity_queue_), 0.0) /
                    velocity_queue_.size();
  // RCLCPP_INFO(
  //   rclcpp::get_logger("front_vehicle_velocity_estimator"), "x=%f, v=%f km/h, v_a=%fkm/h",
  //   objects_with_front_vehicle.front_vehicle.kinematics.pose_with_covariance.pose.position.x,
  //   now_absolute_velocity * 3.6, velocity * 3.6);

  // Set kinematics information for front vehicle
  objects_with_front_vehicle.front_vehicle.kinematics.has_twist = true;
  objects_with_front_vehicle.front_vehicle.kinematics.twist_with_covariance.twist.linear.x =
    velocity;
  objects_with_front_vehicle.front_vehicle.kinematics.has_twist_covariance = true;
  objects_with_front_vehicle.front_vehicle.kinematics.twist_with_covariance.covariance.at(0) = 1.0;
  output_.objects.objects.emplace_back(objects_with_front_vehicle.front_vehicle);

  // Set previous data buffer
  prev_time_ = input.pointcloud->header.stamp;
  prev_point_ = nearest_neighbor_point;

  // Set nearest_neighbor_pointcloud output for debug
  pcl::PointCloud<pcl::PointXYZ> output_pointcloud;
  output_pointcloud.points.push_back(nearest_neighbor_point);
  pcl::toROSMsg(output_pointcloud, output_.nearest_neighbor_pointcloud);
  output_.nearest_neighbor_pointcloud.header = input.pointcloud->header;

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
FrontVehicleVelocityEstimator::ObjectsWithFrontVehicle
FrontVehicleVelocityEstimator::filterFrontVehicle(
  DetectedObjects::ConstSharedPtr objects, const LinearRing2d & front_area)
{
  // Initialize output
  FrontVehicleVelocityEstimator::ObjectsWithFrontVehicle output{};
  output.is_front_vehicle = false;

  DetectedObjects output_objects_{};
  output_objects_.header = objects->header;

  for (const auto & object : objects->objects) {
    if (!isFrontVehicle(object, front_area)) {
      output_objects_.objects.emplace_back(object);
    } else if (!output.is_front_vehicle) {
      output.front_vehicle = object;
      output.is_front_vehicle = true;
    } else {
      auto front_vehicle_position =
        output.front_vehicle.kinematics.pose_with_covariance.pose.position;
      if (object.kinematics.pose_with_covariance.pose.position.x < front_vehicle_position.x) {
        output_objects_.objects.emplace_back(output.front_vehicle);
        output.front_vehicle = object;
      } else {
        output_objects_.objects.emplace_back(object);
      }
    }
  }
  output.objects_without_front_vehicle = std::make_shared<DetectedObjects>(output_objects_);
  return output;
}

bool FrontVehicleVelocityEstimator::isFrontVehicle(
  const DetectedObject & object, const LinearRing2d & front_area)
{
  auto position = object.kinematics.pose_with_covariance.pose.position;
  auto label = object.classification.at(0).label;
  Point2d object_point{position.x, position.y};
  if (
    !(label == ObjectClassification::UNKNOWN) && !(label == ObjectClassification::PEDESTRIAN) &&
    !(label == ObjectClassification::BICYCLE) && !(label == ObjectClassification::MOTORCYCLE) &&
    boost::geometry::within(object_point, front_area)) {
    return true;
  } else {
    return false;
  }
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
      boost::geometry::within(point_, object_ring_2d) &&
      point.z > param_.threshold_pointcloud_z_low && point.z < param_.threshold_pointcloud_z_high) {
      if (!is_initialized) {
        nearest_neighbor_point = point;
      } else if (point.x < nearest_neighbor_point.x) {
        nearest_neighbor_point = point;
      }
    }
  }

  return nearest_neighbor_point;
}

double FrontVehicleVelocityEstimator::estimateRelativeVelocity(
  const pcl::PointXYZ & point, const rclcpp::Time & header_time)
{
  if (velocity_queue_.size() == 0) {
    return 0.0;
  }
  const double dt = (header_time - prev_time_).seconds();
  const double relative_velocity = (point.x - prev_point_.x) / dt;
  return relative_velocity;
}

double FrontVehicleVelocityEstimator::estimateAbsoluteVelocity(
  const double relative_velocity, Odometry::ConstSharedPtr odometry)
{
  const double velocity = relative_velocity + odometry->twist.twist.linear.x;

  return velocity;
}

}  // namespace front_vehicle_velocity_estimator
