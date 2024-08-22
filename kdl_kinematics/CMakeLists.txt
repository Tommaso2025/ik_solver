cmake_minimum_required(VERSION 3.8)
project(kdl_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(random_numbers REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_ros_planning REQUIRED) 

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(
  include
  ${pluginlib_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(kdl_kinematics SHARED
  src/kdl_kinematics.cpp
  src/chainiksolver_vel_mimic_svd.cpp
)

# Link the dependencies
ament_target_dependencies(kdl_kinematics
  pluginlib
  rclcpp
  rclpy
  random_numbers
  moveit_msgs
  moveit_core
  moveit_kinematics
  kdl_parser
  tf2_kdl
  class_loader
  geometry_msgs
  generate_parameter_library
  moveit_ros_planning
  orocos_kdl
  Eigen3
)

#add_executable(ik_load src/ik_solver_loader.cpp)
#ament_target_dependencies(ik_load
#  rclcpp
#  moveit_core
#  moveit_kinematics
#  geometry_msgs
#  moveit_msgs
#  tf2_kdl
#  kdl_parser
#  pluginlib
#  moveit_ros_planning
#)


# Install the library
install(TARGETS kdl_kinematics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the test node executable
#install(TARGETS ik_load
#  DESTINATION lib/${PROJECT_NAME}
#)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

install(FILES kdl_kinematics_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} kdl_kinematics_description.xml)

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

# Export package dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(pluginlib rclcpp)
ament_package()
