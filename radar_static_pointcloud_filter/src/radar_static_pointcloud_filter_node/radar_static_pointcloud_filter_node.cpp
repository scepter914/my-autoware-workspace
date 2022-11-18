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

#include "radar_static_pointcloud_filter/radar_static_pointcloud_filter_node.hpp"

#include <memory>
#include <string>
#include <vector>

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

}  // namespace

namespace radar_static_pointcloud_filter
{
using nav_msgs::msg::Odometry;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

RadarStaticPointcloudFilterNode::RadarStaticPointcloudFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("radar_static_pointcloud_filter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarStaticPointcloudFilterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.doppler_velocity_sd = declare_parameter<double>("doppler_velocity_sd", 2.0);

  // Subscriber
  sub_radar_.subscribe(this, "~/input/radar", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_odometry_.subscribe(this, "~/input/odometry", rclcpp::QoS{1}.get_rmw_qos_profile());

  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(10), sub_radar_, sub_odometry_);
  sync_ptr_->registerCallback(std::bind(&RadarStaticPointcloudFilterNode::onData, this, _1, _2));

  // Publisher
  pub_static_radar_ = create_publisher<RadarScan>("~/output/static_radar_scan", 1);
  pub_dynamic_radar_ = create_publisher<RadarScan>("~/output/dynamic_radar_scan", 1);
}

rcl_interfaces::msg::SetParametersResult RadarStaticPointcloudFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    auto & p = node_param_;
    update_param(params, "doppler_velocity_sd", p.magnification_sd);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

void RadarStaticPointcloudFilterNode::onData(
  const RadarScan::ConstSharedPtr radar_msg, const Odometry::ConstSharedPtr odom_msg)
{
  RadarScan static_radar_{};
  RadarScan dynamic_radar_{};
  static_radar_.header = radar_msg->header;
  dynamic_radar_.header = radar_msg->header;

  for (const auto & radar_return : radar_msg->returns) {
    if (isStaticPointcloud(radar_return)) {
      static_radar_.returns.emplace_back(radar_return);
    } else {
      dynamic_radar_.returns.emplace_back(radar_return);
    }
  }

  pub_static_radar->publish(static_radar_);
  pub_dynamic_radar->publish(dynamic_radar_);
}

bool RadarStaticPointcloudFilterNode::isStaticPointcloud(const RadarReturn & radar_return)
{
  const auto v = radar_return.doppler_velocity;
  node_param_.doppler_velocity_sd;
  return -node_param_.doppler_velocity_sd < v && v < node_param_.doppler_velocity_sd;

  // if ego_v - magnification_sd * sd < doppler_velocity < ego_v + magnification_sd * sd
  // then return true (This means static point)
  //  return (
  //    (ego_vel_doppler - node_param_.magnification_sd * sd < pointcloud.doppler_velocity) &&
  //    (pointcloud.doppler_velocity < ego_vel_doppler + node_param_.magnification_sd * sd));
}

}  // namespace radar_static_pointcloud_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_static_pointcloud_filter::RadarStaticPointcloudFilterNode)
