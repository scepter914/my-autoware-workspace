
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace
{

Eigen::Vector3d getVector3d(const radar_msgs::msg::RadarReturn & radar)
{
  const float r_xy = radar.range * std::cos(radar.elevation);
  const float x = r_xy * std::cos(radar.azimuth);
  const float y = r_xy * std::sin(radar.azimuth);
  const float z = radar.range * std::sin(radar.elevation);
  return Eigen::Vector3d{x, y, z};
}

tier4_autoware_utils::Point3d getPoint3d(const radar_msgs::msg::RadarReturn & radar)
{
  const float r_xy = radar.range * std::cos(radar.elevation);
  const float x = r_xy * std::cos(radar.azimuth);
  const float y = r_xy * std::sin(radar.azimuth);
  const float z = radar.range * std::sin(radar.elevation);
  return tier4_autoware_utils::Point3d(x, y, z);
}

geometry_msgs::msg::Point getPoint(const radar_msgs::msg::RadarReturn & radar)
{
  return tier4_autoware_utils::toMsg(getPoint3d(radar));
}

geometry_msgs::msg::Vector3 getVelocity(const radar_msgs::msg::RadarReturn & radar)
{
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(radar.doppler_velocity * std::cos(radar.azimuth))
    .y(radar.doppler_velocity * std::sin(radar.azimuth))
    .z(0.0);
}

geometry_msgs::msg::Twist getTwist(const radar_msgs::msg::RadarReturn & radar)
{
  auto angular = tier4_autoware_utils::createVector3(0.0, 0.0, 0.0);
  return tier4_autoware_utils::createTwist(getVelocity(radar), angular);
}

pcl::PointXYZ getPointXYZ(const radar_msgs::msg::RadarReturn & radar)
{
  const auto p = getVector3d(radar);
  return pcl::PointXYZ{p.x(), p.y(), p.z()};
}

pcl::PointXYZI getPointXYZI(const radar_msgs::msg::RadarReturn & radar, float intensity)
{
  const auto p = getVector3d(radar);
  return pcl::PointXYZI{p.x(), p.y(), p.z()};
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

geometry_msgs::msg::Vector3 getTransformedVelocity(
  const geometry_msgs::msg::Vector3 velocity,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  geometry_msgs::msg::Vector3Stamped velocity_stamped{};
  velocity_stamped.vector = velocity;
  geometry_msgs::msg::Vector3Stamped transformed_velocity_stamped{};
  tf2::doTransform(velocity_stamped, transformed_velocity_stamped, *transform);
  return velocity_stamped.vector;
}

geometry_msgs::msg::Vector3 compensateEgoVehicleTwist(
  const radar_msgs::msg::RadarReturn & radar,
  const geometry_msgs::msg::TwistWithCovariance & ego_vehicle_twist_with_covariance,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  const geometry_msgs::msg::Vector3 radar_velocity = getVelocity(radar);
  const geometry_msgs::msg::Vector3 v_r = getTransformedVelocity(radar_velocity, transform);

  const auto v_e = ego_vehicle_twist_with_covariance.twist.linear;
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(v_r.x + v_e.x)
    .y(v_r.y + v_e.y)
    .z(v_r.z + v_e.z);
}

}  // namespace
