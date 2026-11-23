cmake_minimum_required(VERSION 3.16)
project(nav3_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/prewarm_pct.py scripts/bringup_status.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
