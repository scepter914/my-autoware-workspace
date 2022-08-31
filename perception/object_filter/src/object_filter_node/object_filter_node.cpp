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

#include "object_filter/object_filter_node.hpp"

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

namespace object_filter
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

ObjectFilterNode::ObjectFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_filter", node_options)
{
  // Parameter Server
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&ObjectFilterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz", 10.0);
  node_param_.probability_threshold =
    declare_parameter<double>("node_params.probability_threshold", 0.40);

  // Subscriber
  sub_objects_ = create_subscription<DetectedObjects>(
    "~/input/objects", rclcpp::QoS{1}, std::bind(&ObjectFilterNode::onObjects, this, _1));

  // Publisher
  pub_objects_ = create_publisher<DetectedObjects>("~/output/objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&ObjectFilterNode::onTimer, this));
}

void ObjectFilterNode::onObjects(const DetectedObjects::ConstSharedPtr msg) { objects_data_ = msg; }

rcl_interfaces::msg::SetParametersResult ObjectFilterNode::onSetParam(
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

  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

bool ObjectFilterNode::isDataReady()
{
  if (!objects_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for data msg...");
    return false;
  }

  return true;
}

void ObjectFilterNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  DetectedObjects output_objects = update(objects_data_);

  // publish
  pub_objects_->publish(output_objects);
}

DetectedObjects ObjectFilterNode::update(DetectedObjects::ConstSharedPtr objects)
{
  DetectedObjects output_objects;
  output_objects.header = objects->header;
  for (const auto & object : objects->objects) {
    if (object.classification.at(0).probability > node_param_.probability_threshold) {
      output_objects.objects.emplace_back(object);
    }
  }
  return output_objects;
}

}  // namespace object_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(object_filter::ObjectFilterNode)
