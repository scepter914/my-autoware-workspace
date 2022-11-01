
#include "my_utils/my_utils.hpp"

namespace my_utils
{
MyUtils::Output MyUtils::update(const MyUtils::Input & input)
{
  MyUtils::Output output;

  // Sample
  output.data = input.data + param_.data;
  RCLCPP_INFO(rclcpp::get_logger("my_utils"), "Debug: %d", 0);
  return output;
}

}  // namespace my_utils
