cmake_minimum_required(VERSION 3.5)
project(eps_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build the EPS Core Node
add_executable(eps_core_node src/eps_core_node.cpp)
ament_target_dependencies(eps_core_node rclcpp std_msgs)

# Install EPS Core Node
install(TARGETS
  eps_core_node
  DESTINATION lib/${PROJECT_NAME})

# Install the visualizer script
install(PROGRAMS
  scripts/eps_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
