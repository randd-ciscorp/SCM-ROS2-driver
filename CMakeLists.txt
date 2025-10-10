cmake_minimum_required(VERSION 3.10)
project(cis_scm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wno-stringop-overflow -Wno-array-bounds)


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

if(INTERNAL_DRIVER)
    add_definitions(-DINTERNAL_DRIVER)

    find_package(image_transport REQUIRED)
    find_package(point_cloud_transport REQUIRED)

    add_library(internal_device
        src/Device.cpp
        src/InternalDevice.cpp
        src/InternalDevices/ToFInternalDevice.cpp
        src/InternalDevices/RGBInternalDevice.cpp
        src/InternalDevices/RGBDInternalDevice.cpp
        src/Params.cpp
        src/Controls.cpp
    )
    target_link_libraries(internal_device scmcap scmtof1)
    ament_target_dependencies(internal_device rclcpp)

    add_executable(tof_node src/tof_node.cpp src/tof_driver.cpp)
    target_link_libraries(tof_node internal_device)
    ament_target_dependencies(tof_node rclcpp sensor_msgs OpenCV camera_info_manager tf2 point_cloud_transport)

    add_executable(rgbd_node src/rgbd_node.cpp src/rgbd_driver.cpp src/tof_driver.cpp)
    target_link_libraries(rgbd_node internal_device)
    ament_target_dependencies(rgbd_node rclcpp sensor_msgs OpenCV camera_info_manager tf2 point_cloud_transport)

    add_executable(rgb_node src/rgb_node.cpp src/rgb_driver.cpp)
    target_link_libraries(rgb_node internal_device)
    ament_target_dependencies(rgb_node rclcpp sensor_msgs OpenCV camera_info_manager tf2 image_transport point_cloud_transport)

    install(TARGETS tof_node rgbd_node rgb_node DESTINATION lib/${PROJECT_NAME})
else()
    #External driver
    add_library(external_device
        src/Device.cpp
        src/ExternalDevice.cpp
        src/Params.cpp
        src/Controls.cpp
    )
    ament_target_dependencies(external_device rclcpp)

    add_executable(tof_node src/tof_node.cpp src/tof_driver.cpp)
    target_link_libraries(tof_node external_device)
    ament_target_dependencies(tof_node rclcpp sensor_msgs OpenCV camera_info_manager tf2)

    add_executable(rgbd_node src/rgbd_node.cpp src/rgbd_driver.cpp src/tof_driver.cpp)
    target_link_libraries(rgbd_node external_device)
    ament_target_dependencies(rgbd_node rclcpp sensor_msgs OpenCV camera_info_manager tf2)

    add_executable(rgb_node src/rgb_node.cpp src/rgb_driver.cpp)
    target_link_libraries(rgb_node external_device)
    ament_target_dependencies(rgb_node rclcpp sensor_msgs OpenCV camera_info_manager tf2)

    install(TARGETS tof_node rgbd_node rgb_node DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING AND NOT INTERNAL_DRIVER)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_uncrustify)
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE "src/InternalDevices/*" "include/cis_scm/InternalDevice.hpp")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
