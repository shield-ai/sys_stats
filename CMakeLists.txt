cmake_minimum_required(VERSION 2.8.3)
project(sys_stats)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sys_stats
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Sys stats library
add_library(sys_stats
  src/sys_stats.cpp
  include/sys_stats/sys_stats.h
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

