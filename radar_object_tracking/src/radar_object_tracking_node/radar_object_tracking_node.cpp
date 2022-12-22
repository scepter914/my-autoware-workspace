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

#include "radar_object_tracking/radar_object_tracking_node.hpp"

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

namespace radar_object_tracking
{
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

RadarObjectTrackingNode::RadarObjectTrackingNode(const rclcpp::NodeOptions & node_options)
: Node("radar_object_tracking", node_options)
{
  using std::placeholders::_1;

  // Parameter Server
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&RadarObjectTrackingNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz", 20.0);

  // Core Parameter
  core_param_.num_frame = declare_parameter<int>("core_params.num_frame", 5);
  core_param_.clustering_range = declare_parameter<double>("core_params.clustering_range", 5.0);
  core_param_.min_object_x = declare_parameter<double>("core_params.min_object_x", 4.5);
  core_param_.min_object_y = declare_parameter<double>("core_params.min_object_y", 2.0);
  core_param_.min_object_z = declare_parameter<double>("core_params.min_object_z", 1.4);
  core_param_.min_sigma_doppler = declare_parameter<double>("core_params.min_sigma_doppler", 2.0);
  core_param_.min_sigma_range = declare_parameter<double>("core_params.min_sigma_range", 3.0);
  core_param_.max_object_acc = declare_parameter<double>("core_params.max_object_acc", 3.0);
  core_param_.noise_thereshold_frame =
    declare_parameter<int>("core_params.noise_thereshold_frame", 4);
  core_param_.object_confidence = declare_parameter<double>("core_params.object_confidence", 0.30);

  // Core
  radar_object_tracking_ = std::make_unique<RadarObjectTracking>(get_logger());
  radar_object_tracking_->setParam(core_param_);

  // Subscriber
  sub_data_ = create_subscription<RadarScan>(
    "~/input/radar", rclcpp::QoS{1}, std::bind(&RadarObjectTrackingNode::onData, this, _1));

  // Publisher
  pub_data_ = create_publisher<TrackedObjects>("~/output/objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&RadarObjectTrackingNode::onTimer, this));
}

void RadarObjectTrackingNode::onData(const RadarScan::ConstSharedPtr msg) { data_ = msg; }

rcl_interfaces::msg::SetParametersResult RadarObjectTrackingNode::onSetParam(
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
      update_param(params, "core_params.num_frame", p.num_frame);
      update_param(params, "core_params.clustering_range", p.clustering_range);
      update_param(params, "core_params.min_object_x", p.min_object_x);
      update_param(params, "core_params.min_object_y", p.min_object_y);
      update_param(params, "core_params.min_object_z", p.min_object_z);
      update_param(params, "core_params.min_sigma_doppler", p.min_sigma_doppler);
      update_param(params, "core_params.min_sigma_range", p.min_sigma_range);
      update_param(params, "core_params.max_object_acc", p.max_object_acc);
      update_param(params, "core_params.noise_thereshold_frame", p.noise_thereshold_frame);
      update_param(params, "core_params.object_confidence", p.object_confidence);

      // Set parameter to instance
      if (radar_object_tracking_) {
        radar_object_tracking_->setParam(core_param_);
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

bool RadarObjectTrackingNode::isDataReady()
{
  if (!data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for data msg...");
    return false;
  }

  return true;
}

void RadarObjectTrackingNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Set input data
  RadarObjectTracking::Input input;
  input.radar_scan = data_;

  // Update
  radar_object_tracking_->resisterPointcloud(input);
  output_ = radar_object_tracking_->update(input);

  pub_data_->publish(output_.output_objects);
}

}  // namespace radar_object_tracking

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_object_tracking::RadarObjectTrackingNode)
