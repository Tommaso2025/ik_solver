cmake_minimum_required(VERSION 3.8)
project(ik_test_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(kdl_kinematics REQUIRED)

find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(
  include
  ${kdl_kinematics_INCLUDE_DIRS}
)

#add_executable(ik_test_node src/ik_test_node.cpp)
#ament_target_dependencies(ik_test_node
#  rclcpp
#  xacro
#  moveit_core
#  moveit_ros_planning_interface
#  pluginlib
#  kdl_parser
#  kdl_kinematics
#)

add_executable(ik_test_node_2 src/ik_test_node_2.cpp)
ament_target_dependencies(ik_test_node_2
  rclcpp
  rcl
  geometry_msgs
  xacro
  moveit_core
  moveit_ros_planning
  pluginlib
  kdl_parser
  kdl_kinematics
)

#install(TARGETS ik_test_node
#  DESTINATION lib/${PROJECT_NAME}
#  )

install(TARGETS ik_test_node_2
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
