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
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs)

add_executable(stage_control_node src/stage_control_node.cpp)
ament_target_dependencies(stage_control_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "stage_control_interfaces"
  "control_msgs"
  )

add_executable(stage_hardware_node src/stage_hardware_node.cpp)
ament_target_dependencies(stage_hardware_node rclcpp stage_control_interfaces)
target_link_libraries(stage_hardware_node usb-1.0)

add_executable(stage_emulated_node src/stage_emulated_node.cpp)
ament_target_dependencies(stage_emulated_node rclcpp)

add_executable(stage_virtual_node src/stage_virtual_node.cpp)
ament_target_dependencies(stage_virtual_node rclcpp sensor_msgs std_msgs control_msgs rclcpp_action)

add_executable(run_experiment_node scripts/run_experiment_node.cpp)
ament_target_dependencies(run_experiment_node rclcpp stage_control_interfaces std_msgs rclcpp_action)

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
    stage_virtual_node
    run_experiment_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
