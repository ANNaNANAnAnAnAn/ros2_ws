cmake_minimum_required(VERSION 3.10)
project(jetracer_control)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Motor Controller
add_library(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp)

# Steering Controller
add_library(steering_controller src/steering_controller.cpp)
ament_target_dependencies(steering_controller rclcpp)

# Main node
add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp geometry_msgs)
target_link_libraries(motor_node motor_controller steering_controller)


# Install targets
install(TARGETS
  motor_node
  motor_controller
  steering_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS 
        scripts/gamepad_cmd_vel.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

