cmake_minimum_required(VERSION 3.8)
project(action_tutorials_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

#add_library(action_client SHARED
#  src/followjointtrajectory_action_client.cpp)
#target_include_directories(action_client PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_definitions(action_client
#  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
#ament_target_dependencies(action_client
#  "action_tutorials_interfaces"
#  "rclcpp"
#  "rclcpp_action"
#  "rclcpp_components")
#rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FollowJointTrajectoryActionClient" EXECUTABLE followjointtrajectory_action_client)
#install(TARGETS
#  action_client
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)


add_executable(example_position src/followjointtrajectory_action_client.cpp)
ament_target_dependencies(example_position
  rcl
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  std_msgs
)

## Install
install(TARGETS example_position
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
