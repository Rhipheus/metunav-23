
cmake_minimum_required(VERSION 3.5)
project(metu_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rviz2 REQUIRED)

find_package(metu_description REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include launch launch/include worlds src models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()