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

#include "radar_tracks_noise_filter/radar_tracks_noise_filter_node.hpp"

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

namespace radar_tracks_noise_filter
{
using radar_msgs::msg::RadarTrack;
using radar_msgs::msg::RadarTracks;

RadarTrackCrossingNoiseFilterNode::RadarTrackCrossingNoiseFilterNode(
  const rclcpp::NodeOptions & node_options)
: Node("radar_tracks_noise_filter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarTrackCrossingNoiseFilterNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.velocity_angle_threshold =
    declare_parameter<double>("velocity_angle_threshold", 1.0472);

  // Subscriber
  sub_tracks_ = create_subscription<RadarTracks>(
    "~/input/tracks", rclcpp::QoS{1},
    std::bind(&RadarTrackCrossingNoiseFilterNode::onTracks, this, _1));

  // Publisher
  pub_noise_tracks_ = create_publisher<RadarTracks>("~/output/noise_tracks", 1);
  pub_filtered_tracks_ = create_publisher<RadarTracks>("~/output/filtered_tracks", 1);
}

void RadarTrackCrossingNoiseFilterNode::onTracks(const RadarTracks::ConstSharedPtr msg)
{
  RadarTracks noise_tracks;
  RadarTracks filtered_tracks;
  high_speed_tracks.header = msg->header;
  low_speed_tracks.header = msg->header;

  for (const auto & radar_track : msg->tracks) {
    if (isNoise(radar_track)) {
      noise_tracks.tracks.emplace_back(radar_track);
    } else {
      filtered_tracks.tracks.emplace_back(radar_track);
    }
  }
  // publish
  pub_high_speed_tracks_->publish(high_speed_tracks);
  pub_low_speed_tracks_->publish(low_speed_tracks);
}

rcl_interfaces::msg::SetParametersResult RadarTrackCrossingNoiseFilterNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "velocity_threshold", p.velocity_threshold);
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

bool RadarTrackCrossingNoiseFilterNode::isNoise(RadarTrack & track) {}

}  // namespace radar_tracks_noise_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode)
