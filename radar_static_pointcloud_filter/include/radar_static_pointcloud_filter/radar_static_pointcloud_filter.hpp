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

#ifndef RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_HPP__
#define RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_HPP__

#include <rclcpp/logger.hpp>

#include <memory>
#include <string>
#include <vector>

namespace radar_static_pointcloud_filter
{
class RadarStaticPointcloudFilter
{
public:
  explicit RadarStaticPointcloudFilter(const rclcpp::Logger & logger) : logger_(logger) {}

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

}  // namespace radar_static_pointcloud_filter

#endif  // RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_HPP__
