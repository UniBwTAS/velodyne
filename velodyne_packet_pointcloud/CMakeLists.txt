cmake_minimum_required(VERSION 3.0.2)
project(velodyne_packet_pointcloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS " -Wall -Wextra -fPIC ")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    roscpp
    roslib
    sensor_msgs
    tf2_ros
    velodyne_driver
    velodyne_msgs
    velodyne_pointcloud
    ethernet_msgs
    dynamic_reconfigure
    diagnostic_updater
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             )



# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})
#generate_dynamic_reconfigure_options()
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES velodyne_rawdata data_containers
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

add_subdirectory(src)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)




