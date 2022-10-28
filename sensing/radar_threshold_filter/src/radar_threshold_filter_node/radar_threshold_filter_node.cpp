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

#include "radar_threshold_filter/radar_threshold_filter_node.hpp"

#include <radar_msgs/msg/radar_scan.hpp>

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

namespace radar_threshold_filter
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

RadarThresholdFilterNode::RadarThresholdFilterNode(const rclcpp::NodeOptions & node_options)
: Node("radar_threshold_filter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarThresholdFilterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz", 10.0);

  // Core Parameter
  core_param_.data = declare_parameter<int>("core_params.data");

  // Core
  radar_threshold_filter_ = std::make_unique<RadarThresholdFilter>(get_logger());
  radar_threshold_filter_->setParam(core_param_);

  // Subscriber
  sub_data_ = create_subscription<RadarScan>(
    "~/input/radar", rclcpp::QoS{1}, std::bind(&RadarThresholdFilterNode::onData, this, _1));

  // Publisher
  pub_data_ = create_publisher<RadarScan>("~/output/radar", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&RadarThresholdFilterNode::onTimer, this));
}

void RadarThresholdFilterNode::onData(const Int32::ConstSharedPtr msg) { data_ = msg; }

rcl_interfaces::msg::SetParametersResult RadarThresholdFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "node_params.update_rate_hz", p.update_rate_hz);
    }

    // Core Parameter
    {
      auto & p = core_param_;

      // Update params
      update_param(params, "core_params.data", p.data);

      // Set parameter to instance
      if (radar_threshold_filter_) {
        radar_threshold_filter_->setParam(core_param_);
      }
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

bool RadarThresholdFilterNode::isDataReady()
{
  if (!data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for data msg...");
    return false;
  }

  return true;
}

void RadarThresholdFilterNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Set input data
  RadarThresholdFilter::Input input;

  input.data = data_->data;

  // Update
  output_ = radar_threshold_filter_->update(input);

  // Sample
  pub_data_->publish(example_interfaces::build<Int32>().data(output_.data));
  // pub_data->publish(hoge_msgs);

  RCLCPP_INFO(get_logger(), "input, output: %d, %d", input_.data, output_.data);
}

}  // namespace radar_threshold_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_threshold_filter::RadarThresholdFilterNode)
