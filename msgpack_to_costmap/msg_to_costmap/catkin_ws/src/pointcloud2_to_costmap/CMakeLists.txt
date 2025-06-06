cmake_minimum_required(VERSION 3.0.2)
project(pointcloud2_to_costmap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# 查找库文件
find_library(MY_LIBRARY NAMES MyLibrary PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib)
# 查找头文件目录
find_path(MY_LIBRARY_INCLUDE_DIR msgpack.hpp msgpack-functions.hpp PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
if(NOT MY_LIBRARY_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find msgpack.hpp.")
endif()

if(NOT MY_LIBRARY)
    message(FATAL_ERROR "Could not find the MyLibrary library.")
endif()

link_directories(/opt/ros/noetic/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  rosbag
)
find_package( PCL REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${MY_LIBRARY_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sb_test.cpp
# )
add_executable(${PROJECT_NAME} src/pointcloud2_to_costmap.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  rosbag
  ${MY_LIBRARY}
)

# INSTALL 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}        
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


