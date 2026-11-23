cmake_minimum_required(VERSION 3.16)
project(nav3_bag_recorder)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_package()
