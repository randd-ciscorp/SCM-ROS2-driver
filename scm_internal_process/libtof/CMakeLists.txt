cmake_minimum_required(VERSION 3.10)
project(cistof CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(cistof STATIC
	tof.cpp
	tof_calib.cpp)

target_link_libraries(cistof
	${OpenCV_LIBS}
    OpenMP::OpenMP_CXX)

