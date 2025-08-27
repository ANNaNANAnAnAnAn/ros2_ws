cmake_minimum_required(VERSION 3.10)
project(jetracer_control)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Include headers
include_directories(include)

# -----------------------
# C++ Libraries
# -----------------------

add_library(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp)

add_library(steering_controller src/steering_controller.cpp)
ament_target_dependencies(steering_controller rclcpp)

# -----------------------
# C++ Nodes
# -----------------------

add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp geometry_msgs)
target_link_libraries(motor_node motor_controller steering_controller)

add_executable(followme_node nodes/followme_node.cpp)
ament_target_dependencies(followme_node rclcpp geometry_msgs)

# -----------------------
# Install C++ executables
# -----------------------

install(TARGETS
  motor_node
  motor_controller
  steering_controller
  followme_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Python Package
# -----------------------

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  jetracer_control/aoa_node.py
  jetracer_control/lidar_node.py
  jetracer_control/gamepad_cmd_vel.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Resource & Launch
# -----------------------

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
