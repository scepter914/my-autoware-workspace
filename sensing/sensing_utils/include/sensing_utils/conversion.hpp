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

#include <cmath>

#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "radar_msgs/msg/radar_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace tier4_autoware_utils
{

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;

inline double getTwistNorm(const Twist & twist)
{
  return std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);
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
