cmake_minimum_required(VERSION 3.10)

project(scm1camera CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(OpenCV REQUIRED)

add_library(scm1camera STATIC
	camerabase_color.cpp
	camerabase_raw.cpp
	cameraoutput.cpp
	)
