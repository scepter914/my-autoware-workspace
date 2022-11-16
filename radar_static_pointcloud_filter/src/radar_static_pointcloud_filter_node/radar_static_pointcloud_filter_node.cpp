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
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 10.0);
  node_param_.min_sd = declare_parameter<double>("min_sd", 1.0);
  node_param_.magnification_sd = declare_parameter<double>("magnification_sd", 1.0);

  // Subscriber
  sub_radar_ = create_subscription<RadarScan>(
    "~/input/radar", rclcpp::QoS{1},
    std::bind(&RadarStaticPointcloudFilterNode::onRadar, this, _1));
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    std::bind(&RadarStaticPointcloudFilterNode::onOdometry, this, _1));

  // Publisher
  pub_static_radar_ = create_publisher<RadarScan>("~/output/static_radar_scan", 1);
  pub_dynamic_radar_ = create_publisher<RadarScan>("~/output/dynamic_radar_scan", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns,
    std::bind(&RadarStaticPointcloudFilterNode::onTimer, this));
}

void RadarStaticPointcloudFilterNode::onRadar(const RadarReturn::ConstSharedPtr msg)
{
  radar_data_ = msg;
}

void RadarStaticPointcloudFilterNode::onOdometory(const RadarReturn::ConstSharedPtr msg)
{
  odometry_data_ = msg;
}

rcl_interfaces::msg::SetParametersResult RadarStaticPointcloudFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    auto & p = node_param_;
    update_param(params, "update_rate_hz", p.update_rate_hz);
    update_param(params, "min_sd", p.min_sd);
    update_param(params, "magnification_sd", p.magnification_sd);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

bool RadarStaticPointcloudFilterNode::isDataReady()
{
  if (!radar_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for radar msg...");
    return false;
  }
  if (!odometry_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for odometry msg...");
    return false;
  }
  return true;
}

void RadarStaticPointcloudFilterNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  // pub_data->publish(hoge_msgs);
}

}  // namespace radar_static_pointcloud_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_static_pointcloud_filter::RadarStaticPointcloudFilterNode)
