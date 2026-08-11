cmake_minimum_required(VERSION 3.8)
project(alohamini_bringup)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(head_camera_publisher src/head_camera_publisher.cpp)
target_compile_features(head_camera_publisher PUBLIC cxx_std_17)
target_include_directories(head_camera_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(head_camera_publisher rclcpp sensor_msgs)
target_link_libraries(head_camera_publisher ${OpenCV_LIBS})

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

install(
  TARGETS head_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
