cmake_minimum_required(VERSION 3.5)
project(stage_control)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(stage_control_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(stage_control_node src/stage_control_node.cpp)
ament_target_dependencies(stage_control_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "stage_control_interfaces")

add_executable(stage_hardware_node src/stage_hardware_node.cpp)
ament_target_dependencies(stage_hardware_node rclcpp stage_control_interfaces)
target_link_libraries(stage_hardware_node usb-1.0)

add_executable(stage_emulated_node src/stage_emulated_node.cpp)
ament_target_dependencies(stage_emulated_node rclcpp)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    stage_control_node
    stage_hardware_node
    stage_emulated_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
