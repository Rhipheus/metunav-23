cmake_minimum_required(VERSION 3.5)
project(metu_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY launch params src
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/robot_navigator.py src/logged_waypoint_follower.py src/utils/gps_utils.py src/gps_converter.py src/utils/geo_to_cartesian.py src/example_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
