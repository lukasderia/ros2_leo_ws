cmake_minimum_required(VERSION 3.5)
project(leo_teleop)

# Default to C99 / C++14
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Executable
add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)

add_executable(leo_mode src/leo_mode.cpp)
ament_target_dependencies(leo_mode rclcpp sensor_msgs std_msgs)  # Add whatever dependencies you need

# Install binary
install(TARGETS 
  teleop
  leo_mode
  DESTINATION lib/${PROJECT_NAME})

# Optional: install headers (nice practice)
install(DIRECTORY include/
  DESTINATION include)

# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()