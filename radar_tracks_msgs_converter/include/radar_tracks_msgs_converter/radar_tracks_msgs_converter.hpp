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

#ifndef RADAR_TRACKS_MSGS_CONVERTER__RADAR_TRACKS_MSGS_CONVERTER_HPP__
#define RADAR_TRACKS_MSGS_CONVERTER__RADAR_TRACKS_MSGS_CONVERTER_HPP__

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"

namespace radar_tracks_msgs_converter
{
class RadarTracksMsgsConverter
{
public:
  explicit RadarTracksMsgsConverter(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Input
  {
    int data{};
  };

  struct Output
  {
    int data{};
  };

  struct Param
  {
    int data{};
  };

  void setParam(const Param & param) { param_ = param; }

  Output update(const Input & input);

private:
  rclcpp::Logger logger_;
  Param param_{};
};

}  // namespace radar_tracks_msgs_converter

#endif  // RADAR_TRACKS_MSGS_CONVERTER__RADAR_TRACKS_MSGS_CONVERTER_HPP__
