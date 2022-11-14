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

#include "radar_scan_to_pointcloud2/radar_scan_to_pointcloud2_node.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::placeholders::_1;

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}

geometry_msgs::msg::Point getPoint(const radar_msgs::msg::RadarReturn & radar)
{
  const auto x = radar.range * std::sin(radar.azimuth) * std::cos(radar.elevation);
  const auto y = radar.range * std::cos(radar.azimuth) * std::cos(radar.elevation);
  const auto z = radar.range * std::sin(radar.elevation);
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(z);
}

pcl::PointCloud<pcl::PointXYZI> toAmplitudePCL(const radar_msgs::msg::RadarScan & radar_scan)
{
  pcl::PointCloud<pcl::PointXYZI> output;
  for (const auto & radar : radar_scan.returns) {
    auto point = getPoint(radar);
    output.push_back(pcl::PointXYZI{point.x, point.y, point.z, radar.amplitude});
  }
  return output;
}

sensor_msgs::msg::PointCloud2 toAmplitudePointcloud2(const radar_msgs::msg::RadarScan & radar_scan)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  auto pcl_pointcloud = toAmplitudePCL(radar_scan);
  pcl::toROSMsg(pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header = radar_scan.header;
  return pointcloud_msg;
}
}  // namespace

namespace radar_scan_to_pointcloud2
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;
using sensor_msgs::msg::PointCloud2;

RadarScanToPointcloud2Node::RadarScanToPointcloud2Node(const rclcpp::NodeOptions & node_options)
: Node("radar_scan_to_pointcloud2", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarScanToPointcloud2Node::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz", 10.0);

  // Subscriber
  sub_radar_ = create_subscription<RadarScan>(
    "~/input/radar", rclcpp::QoS{1}, std::bind(&RadarScanToPointcloud2Node::onData, this, _1));

  // Publisher
  pub_pointcloud_ = create_publisher<PointCloud2>("~/output/pointcloud", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&RadarScanToPointcloud2Node::onTimer, this));
}

void RadarScanToPointcloud2Node::onData(const RadarScan::ConstSharedPtr msg) { radar_data_ = msg; }

rcl_interfaces::msg::SetParametersResult RadarScanToPointcloud2Node::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    auto & p = node_param_;
    update_param(params, "node_params.update_rate_hz", p.update_rate_hz);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

bool RadarScanToPointcloud2Node::isDataReady()
{
  if (!radar_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for radar msg...");
    return false;
  }
  return true;
}

void RadarScanToPointcloud2Node::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  sensor_msgs::msg::PointCloud2 output = toAmplitudePointcloud2(*radar_data_);
  pub_pointcloud_->publish(output);
}

}  // namespace radar_scan_to_pointcloud2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_scan_to_pointcloud2::RadarScanToPointcloud2Node)
