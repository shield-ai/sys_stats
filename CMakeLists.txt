cmake_minimum_required(VERSION 2.8.3)
project(sys_stats)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  shield_msgs
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
  include/sys_stats/serialization.h
)

find_package(CUDA)

if (CUDA_FOUND)
    link_directories(
      ${CUDA_TOOLKIT_ROOT_DIR}/lib64
      ${CUDA_TOOLKIT_ROOT_DIR}/lib64/stubs
      ${CUDA_TOOLKIT_ROOT_DIR}/lib)

    include_directories(
        ${CUDA_INCLUDE_DIRS}
    )

    target_link_libraries(sys_stats nvidia-ml)

    add_definitions(-DHAVE_CUDA)
ENDIF()

# Wrap with node - the usual defaults
add_executable(${PROJECT_NAME}_node src/sys_stats_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  sys_stats
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

