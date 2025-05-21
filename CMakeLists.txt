cmake_minimum_required(VERSION 3.10)
project(cis_scm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(tof_node src/tof_node.cpp src/tof_driver.cpp src/Device.cpp)
ament_target_dependencies(tof_node rclcpp sensor_msgs OpenCV camera_info_manager tf2)

add_executable(rgbd_node src/rgbd_node.cpp src/rgbd_driver.cpp src/tof_driver.cpp src/Device.cpp)
ament_target_dependencies(rgbd_node rclcpp sensor_msgs OpenCV camera_info_manager tf2)

install(TARGETS tof_node rgbd_node DESTINATION lib/${PROJECT_NAME})
install(FILES cam_param.yaml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
