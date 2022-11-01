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

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

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
  auto angular = tier4_autoware_utils::createVector3(0.0, 0.0, 0.0);
  return tier4_autoware_utils::getTwist(getVelocity(radar), angular);
}

inline Vector3 getVelocity(const RadarReturn & radar)
{
  const auto vx = radar.doppler_velocity * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const auto vz = radar.doppler_velocity * std::sin(radar.elevation);
  return geometry_msgs::build<Vector3>().x(vx).y(0.0).z(vz);
}

inline TwistWithCovariance getTwistWithCovariance(const RadarReturn & radar)
{
  auto angular = tier4_autoware_utils::createVector3(0.0, 0.0, 0.0);
  return tier4_autoware_utils::getTwistWithCovariance(getVelocity(radar), angular);
}

/// @brief Compensate ego vehicle twist. Doppler velocity compensated by ego vehicle in sensor
/// coordinate.
/// @param radar: Radar return
/// @param ego_vehicle_twist_with_covariance: The twist of ego vehicle
/// @param transform: transform
/// @return
inline Vector3 compensateEgoVehicleTwist(
  const RadarReturn & radar, const TwistWithCovariance & ego_vehicle_twist_with_covariance,
  const geometry_msgs::msg::TransformStamped & transform)
{
  // transform to sensor coordinate
  geometry_msgs::msg::Vector3Stamped velocity_stamped{};
  velocity_stamped.vector = ego_vehicle_twist_with_covariance.twist.linear;
  geometry_msgs::msg::Vector3Stamped transformed_velocity_stamped{};
  tf2::doTransform(velocity_stamped, transformed_velocity_stamped, transform);

  // Compensate doppler velocity with ego vehicle twist
  const auto v_e = ego_vehicle_twist_with_covariance.twist.linear;
  const auto v_r = getVelocity(radar);
  return tf2::Vector3(v_r.x - v_e.x, v_r.y - v_e.y, v_r.z - v_e.z);
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
