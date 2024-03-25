cmake_minimum_required(VERSION 3.8)
project(tof1_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(tof_node src/tof_node.cpp src/tof_driver.cpp src/Device.cpp)
ament_target_dependencies(tof_node rclcpp sensor_msgs OpenCV)

install(TARGETS tof_node DESTINATION lib/${PROJECT_NAME})

ament_package()
