cmake_minimum_required(VERSION 3.5)
project(sensing_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(Boost REQUIRED)

# Targets
ament_auto_add_library(sensing_utils SHARED
  src/conversion.cpp
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(test_sensing_utils ${test_files})
  target_link_libraries(test_sensing_utils
    sensing_utils
  )
endif()
