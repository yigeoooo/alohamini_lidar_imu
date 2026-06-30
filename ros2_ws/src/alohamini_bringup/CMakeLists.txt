cmake_minimum_required(VERSION 3.8)
project(alohamini_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/alohamini_mapping_session
    scripts/scan_sector_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
