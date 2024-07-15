cmake_minimum_required(VERSION 3.0.2)
project(velodyne_configuration)


find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        velodyne_msgs
)

catkin_package(

)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        src/configuration_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)