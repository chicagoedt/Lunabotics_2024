cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(interfaces_pkg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(sparkcan REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lunabot_msgs REQUIRED)

add_executable(drivetrain src/drivetrain.cpp)
add_executable(scoop src/scoop.cpp)
add_executable(excavation_node src/excavation.cpp)

ament_target_dependencies(drivetrain 
  rclcpp
  interfaces_pkg
  sparkcan
)

ament_target_dependencies(scoop 
  rclcpp
  interfaces_pkg
  sparkcan
)

ament_target_dependencies(excavation_node 
  rclcpp
  rclcpp_action
  geometry_msgs
  lunabot_msgs
  sparkcan
)

install(TARGETS
  drivetrain
  scoop
  excavation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/testcontrol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
