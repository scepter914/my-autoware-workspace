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

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace perception_utils
{
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

/*
LinearRing2d getLinearRing2d(DetectedObject object)
{
  return tier4_autoware_utils::transformVector(
    createObject2d(object.shape.dimensions.x, object.shape.dimensions.y),
    tier4_autoware_utils::pose2transform(object.kinematics.pose_with_covariance.pose));
}

LinearRing2d getLinearRing2dWithMargin(DetectedObject object, float margin_x, float margin_y)
{
  auto size_x = object.shape.dimensions.x + margin_x;
  auto size_y = object.shape.dimensions.y + margin_y;
  linear_ring = tier4_autoware_utils::transformVector(
    createObject2d(size_x, size_y),
    tier4_autoware_utils::pose2transform(object.kinematics.pose_with_covariance.pose));
  return linear_ring;

tier4_autoware_utils::Polygon2d toPolygon2d(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  return tier4_autoware_utils::toPolygon2d(
    object.kinematics.initial_pose_with_covariance.pose, object.shape);
}
*/

LinearRing2d createObject2d(const float x, const float y)
{
  const double x_front = x / 2.0;
  const double x_rear = x / 2.0;
  const double y_left = y / 2.0;
  const double y_right = y / 2.0;

  LinearRing2d box{};
  box.push_back(Point2d{x_front, y_left});
  box.push_back(Point2d{x_front, y_right});
  box.push_back(Point2d{x_rear, y_right});
  box.push_back(Point2d{x_rear, y_left});
  box.push_back(Point2d{x_front, y_left});

  return box;
}

}  // namespace perception_utils

namespace tier4_autoware_utils
{

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;

inline double getTwistNorm(const Twist & twist)
{
  auto v = twist.linear;
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/// Judge whether twist covariance is available.
inline bool isTwistCovarianceAvailable(const TwistWithCovariance & twist_with_covariance)
{
  using IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto covariance = twist_with_covariance.covariance;
  if (covariance[IDX::X_X] == 0.0 && covariance[IDX::Y_Y] == 0.0 && covariance[IDX::Z_Z] == 0.0) {
    return false;
  } else {
    return true;
  }
}

inline TwistWithCovariance getTwistWithCovariance(const Twist & twist)
{
  TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist = twist;
  return twist_with_covariance;
}

inline TwistWithCovariance getTwistWithCovariance(const geometry_msgs::msg::Vector3 & velocity)
{
  TwistWithCovariance twist_with_covariance;
  twist_with_covariance.twist.linear = velocity;
  return twist_with_covariance;
}

inline Twist getTwist(const geometry_msgs::msg::Vector3 & velocity)
{
  Twist twist;
  twist.linear = velocity;
  return twist;
}
}  // namespace tier4_autoware_utils

namespace sensing_utils
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;
using geometry_msgs::msg::Vector3;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;
using sensor_msgs::msg::PointCloud2;

inline Point getPoint(const RadarReturn & radar)
{
  const auto x = radar.range * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const auto y = radar.range * std::cos(radar.azimuth) * std::cos(radar.elevation);
  const auto z = radar.range * std::sin(radar.elevation);
  return geometry_msgs::build<Point>().x(x).y(y).z(z);
}

inline Vector3 getVelocity(const RadarReturn & radar)
{
  const auto vx = radar.doppler_velocity * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const auto vz = radar.doppler_velocity * std::sin(radar.elevation);
  return geometry_msgs::build<Vector3>().x(vx).y(0.0).z(vz);
}

inline Twist getTwist(const RadarReturn & radar)
{
  return tier4_autoware_utils::getTwist(getVelocity(radar));
}

inline TwistWithCovariance getTwistWithCovariance(const RadarReturn & radar)
{
  return tier4_autoware_utils::getTwistWithCovariance(getVelocity(radar));
}

inline PointCloud2 toAmplitudePointcloud2(const RadarScan & radar_scan)
{
  PointCloud2 pointcloud_msg;
  auto pcl_pointcloud = toAmplitudePCL(radar_scan);
  pcl::toROSMsg(pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}

inline pcl::PointCloud<pcl::PointXYZI> toAmplitudePCL(const RadarScan & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> output;
  for (const auto & radar : radar_scan.returns) {
    auto point = getPoint(radar);
    output.push_back(pcl::PointXYZI{point.x, point.y, point.z, radar.amplitude});
  }
  return output;
}

}  // namespace sensing_utils
