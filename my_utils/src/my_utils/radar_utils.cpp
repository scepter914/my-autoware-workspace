
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

namespace
{

geometry_msgs::msg::Point getPoint(const radar_msgs::msg::RadarReturn & radar)
{
  const float x = radar.range * std::cos(radar.azimuth) * std::cos(radar.elevation);
  const float y = radar.range * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const float z = radar.range * std::sin(radar.elevation);
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(z);
}

geometry_msgs::msg::Vector3 getVelocity(const radar_msgs::msg::RadarReturn & radar)
{
  const auto vx = radar.doppler_velocity * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const auto vz = radar.doppler_velocity * std::sin(radar.elevation);
  return geometry_msgs::build<geometry_msgs::msg::Vector3>().x(vx).y(0.0).z(vz);
}

geometry_msgs::msg::Twist getTwist(const radar_msgs::msg::RadarReturn & radar)
{
  auto angular = tier4_autoware_utils::createVector3(0.0, 0.0, 0.0);
  return tier4_autoware_utils::createTwist(getVelocity(radar), angular);
}

/// @brief Compensate ego vehicle twist. Doppler velocity compensated by ego vehicle in sensor
/// coordinate.
/// @param radar: Radar return
/// @param ego_vehicle_twist_with_covariance: The twist of ego vehicle
/// @param transform: transform
/// @return
geometry_msgs::msg::Vector3 compensateEgoVehicleTwist(
  const radar_msgs::msg::RadarReturn & radar,
  const geometry_msgs::msg::TwistWithCovariance & ego_vehicle_twist_with_covariance,
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
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(v_r.x - v_e.x)
    .y(v_r.y - v_e.y)
    .z(v_r.z - v_e.z);
}

pcl::PointXYZ getPointXYZ(const radar_msgs::msg::RadarReturn & radar)
{
  const float x = radar.range * std::cos(radar.azimuth) * std::cos(radar.elevation);
  const float y = radar.range * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const float z = radar.range * std::sin(radar.elevation);
  return pcl::PointXYZ{x, y, z};
}

pcl::PointXYZI getPointXYZI(const radar_msgs::msg::RadarReturn & radar, float intensity)
{
  const float x = radar.range * std::cos(radar.azimuth) * std::cos(radar.elevation);
  const float y = radar.range * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const float z = radar.range * std::sin(radar.elevation);
  return pcl::PointXYZI{x, y, z, intensity};
}

pcl::PointCloud<pcl::PointXYZI> toAmplitudePCL(const radar_msgs::msg::RadarScan & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> pcl;
  for (const auto & radar : radar_scan.returns) {
    pcl.push_back(getPointXYZI(radar, radar.amplitude));
  }
  return pcl;
}

sensor_msgs::msg::PointCloud2 toAmplitudePointcloud2(const radar_msgs::msg::RadarScan & radar_scan)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  auto pcl_pointcloud = toAmplitudePCL(radar_scan);
  pcl::toROSMsg(pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}

pcl::PointCloud<pcl::PointXYZI> toDopplerPCL(const radar_msgs::msg::RadarScan & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> pcl;
  for (const auto & radar : radar_scan.returns) {
    pcl.push_back(getPointXYZI(radar, radar.doppler_velocity));
  }
  return pcl;
}

sensor_msgs::msg::PointCloud2 toDopplerPointcloud2(const radar_msgs::msg::RadarScan & radar_scan)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  auto pcl_pointcloud = toDopplerPCL(radar_scan);
  pcl::toROSMsg(pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}
}  // namespace
