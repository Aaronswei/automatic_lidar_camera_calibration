cmake_minimum_required(VERSION 3.10)

project(sensors_calib)
set (CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(LIBRARY_NAME ${PROJECT_NAME})

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include(Findrapidjson)
if(NOT RAPIDJSON_FOUND)
  message(STATUS "rapidjson not found")
  return()
endif()

add_subdirectory(src)

if(BUILD_TEST)
  enable_testing()
  add_subdirectory(tests)
endif()

if(BUILD_APPS)
  add_subdirectory(examples)
endif(BUILD_APPS)
