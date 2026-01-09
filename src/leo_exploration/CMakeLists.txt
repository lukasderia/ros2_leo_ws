cmake_minimum_required(VERSION 3.5)
project(leo_exploration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


# Declare messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrontierCluster.msg"
  "msg/FrontierClusters.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(frontier_detector src/frontier_detector.cpp)
ament_target_dependencies(frontier_detector 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  visualization_msgs
)

# Link the custom messages to the executable
rosidl_target_interfaces(frontier_detector ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  frontier_detector
  DESTINATION lib/${PROJECT_NAME})

ament_package()