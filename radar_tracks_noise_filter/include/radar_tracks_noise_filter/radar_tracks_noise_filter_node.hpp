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

#ifndef radar_tracks_noise_filter__radar_tracks_noise_filter_NODE_HPP__
#define radar_tracks_noise_filter__radar_tracks_noise_filter_NODE_HPP__

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_tracks.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_tracks_noise_filter
{
using radar_msgs::msg::RadarTrack;
using radar_msgs::msg::RadarTracks;

class RadarTrackCrossingNoiseFilterNode : public rclcpp::Node
{
public:
  explicit RadarTrackCrossingNoiseFilterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double velocity_y_threshold{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarTracks>::SharedPtr sub_objects_{};

  // Callback
  void onObjects(const RadarTracks::ConstSharedPtr msg);

  // Data Buffer
  RadarTracks::ConstSharedPtr objects_data_{};

  // Publisher
  rclcpp::Publisher<RadarTracks>::SharedPtr pub_high_speed_objects_{};
  rclcpp::Publisher<RadarTracks>::SharedPtr pub_low_speed_objects_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  bool isDataReady();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  bool isNoise(RadarTracks & object);
};

}  // namespace radar_tracks_noise_filter

#endif  // radar_tracks_noise_filter__radar_tracks_noise_filter_NODE_HPP__
