cmake_minimum_required(VERSION 2.8.3)
project(velodyne_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs)

add_message_files(
        DIRECTORY msg
        FILES
        VelodynePacket.msg
        VelodyneScan.msg
        VelodyneReturnMode.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        VelodyneSetConfiguration.srv
        VelodyneRequestConfiguration.srv
        VelodyneSpecialCommands.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
