cmake_minimum_required(VERSION 3.10)

set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/Dummy.cpp
)

add_library(${LIBRARY_NAME}
  SHARED
    ${SOURCE_FILES}
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(${LIBRARY_NAME}
  PRIVATE
     $<$<CONFIG:Debug>:-O0 -g -Wall -Werror>
     $<$<CONFIG:Release>:-O3>
)
