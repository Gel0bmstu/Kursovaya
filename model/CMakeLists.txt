cmake_minimum_required(VERSION 3.10)

project(BINS)

add_executable(main main.cpp)

set(CMAKE_CXX_STANDARD 11)
set(INCLUDE_PATH src/)
set(SRC_PATH src/)

set(GCC_COVERAGE_COMPILE_FLAGS "-w -Wall -Werror -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

include_directories(src)

add_library( bins
  ${INCLUDE_PATH}/namespace/bins_namespace.h
  ${INCLUDE_PATH}/bins.h
  ${INCLUDE_PATH}/lib/quaternions/include/quaternion.h
  ${INCLUDE_PATH}/logger/logger.h

  ${SRC_PATH}/bins.cpp
  ${SRC_PATH}/logger/logger.cpp
)
