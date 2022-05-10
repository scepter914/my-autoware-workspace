// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Kenji Miyake
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

#include "radar_tracks_msgs_converter/radar_tracks_msgs_converter_node.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using namespace std::placeholders;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

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

namespace radar_tracks_msgs_converter
{
RadarTracksMsgsConverterNode::RadarTracksMsgsConverterNode(const rclcpp::NodeOptions & node_options)
: Node("radar_tracks_msgs_converter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarTracksMsgsConverterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("node_params.update_rate_hz", 20.0);
  node_param_.use_twist_compensation =
    declare_parameter<bool>("node_params.use_twist_compensation", false);

  // Subscriber
  sub_data_ = create_subscription<RadarTracks>(
    "~/input/radar_objects", rclcpp::QoS{1},
    std::bind(&RadarTracksMsgsConverterNode::onData, this, _1));

  // Publisher
  pub_data_ = create_publisher<TrackedObjects>("~/output/radar_objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&RadarTracksMsgsConverterNode::onTimer, this));
}

void RadarTracksMsgsConverterNode::onData(const RadarTracks::ConstSharedPtr msg)
{
  radar_data_ = msg;
}

rcl_interfaces::msg::SetParametersResult RadarTracksMsgsConverterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      // Copy to local variable
      auto p = node_param_;

      // Update params
      update_param(params, "node_params.update_rate_hz", p.update_rate_hz);
      update_param(params, "node_params.use_twist_compensation", p.use_twist_compensation);

      // Copy back to member variable
      node_param_ = p;
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

bool RadarTracksMsgsConverterNode::isDataReady()
{
  if (!radar_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for data msg...");
    return false;
  }

  return true;
}

void RadarTracksMsgsConverterNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  TrackedObjects tracked_objects = convertRadarTrackToTrackedObjects(radar_data_);
  pub_data_->publish(tracked_objects);
}

TrackedObjects RadarTracksMsgsConverterNode::convertRadarTrackToTrackedObjects(
  const RadarTracks::ConstSharedPtr radar_tracks)
{
  TrackedObjects tracked_objects;
  tracked_objects.header = radar_tracks->header;
  for (const auto & radar_track : radar_tracks->tracks) {
    TrackedObject tracked_object;

    tracked_object.object_id = radar_track.uuid;
    tracked_object.existence_probability = 1.0;

    //kinematics

    // classification
    ObjectClassification classification;
    classification.probability = 1.0;
    classification.label = convertClassification(radar_track.classification);
    tracked_object.classification.emplace_back(classification);

    tracked_objects.objects.emplace_back(tracked_object);
  }
  /*
  - autoware_auto_perception_msgs::msg::TrackedObjectKinematics kinematics;
    - geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
    - uint8 orientation_availability;
    - geometry_msgs::msg::TwistWithCovariance twist_with_covariance;
    - geometry_msgs::msg::AccelWithCovariance acceleration_with_covariance;
    - boolean is_stationary;
  - autoware_auto_perception_msgs::msg::Shape shape;
  */
}

uint8_t RadarTracksMsgsConverterNode::convertClassification(const uint16_t classification)
{
  if (classification == 32000) {
    return ObjectClassification::UNKNOWN;
  } else if (classification == 32001) {
    return ObjectClassification::CAR;
  } else if (classification == 32002) {
    return ObjectClassification::TRUCK;
  } else if (classification == 32003) {
    return ObjectClassification::BUS;
  } else if (classification == 32004) {
    return ObjectClassification::TRAILER;
  } else if (classification == 32005) {
    return ObjectClassification::MOTORCYCLE;
  } else if (classification == 32006) {
    return ObjectClassification::BICYCLE;
  } else if (classification == 32007) {
    return ObjectClassification::PEDESTRIAN;
  } else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Receive unknown label for RadarTracks");
    return ObjectClassification::UNKNOWN;
  }  // namespace radar_tracks_msgs_converter
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_tracks_msgs_converter::RadarTracksMsgsConverterNode)
