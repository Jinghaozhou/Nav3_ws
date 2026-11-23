cmake_minimum_required(VERSION 3.16)
project(nav3_visualization)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/map_publisher.py scripts/prepare_visual_map.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
