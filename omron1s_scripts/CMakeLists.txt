cmake_minimum_required(VERSION 3.5)
project(omron1s_scripts)

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
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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

ament_package()

# Generate the executable
add_executable(test_trajectory
    src/test_trajectory.cpp)
target_include_directories(test_trajectory
    PUBLIC include)
ament_target_dependencies(test_trajectory
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)


add_executable(test_trajectory_multi
  src/test_trajectory_multi.cpp)
target_include_directories(test_trajectory_multi
  PUBLIC include)
ament_target_dependencies(test_trajectory_multi
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)

# Install the executable
install(TARGETS 
	test_trajectory
  test_trajectory_multi
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)
