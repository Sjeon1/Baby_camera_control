cmake_minimum_required(VERSION 3.8)
project(baby_camera_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rtsp_image_publisher src/rtsp_image_publisher.cpp)
ament_target_dependencies(rtsp_image_publisher
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(rtsp_image_publisher ${OpenCV_LIBRARIES})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  rtsp_image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/calibrator
  scripts/palm_tracker
  scripts/ptz_controller
  scripts/video_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
