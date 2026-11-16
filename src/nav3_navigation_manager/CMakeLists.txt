cmake_minimum_required(VERSION 3.22)
project(nav3_navigation_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pct_planner_interfaces REQUIRED)
find_package(scan_planner_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  DEPENDENCIES geometry_msgs)

add_executable(navigation_manager src/navigation_manager.cpp)
ament_target_dependencies(navigation_manager
  rclcpp rclcpp_action geometry_msgs nav_msgs std_msgs tf2 tf2_ros
  pct_planner_interfaces scan_planner_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigation_manager "${cpp_typesupport_target}")

install(TARGETS navigation_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
