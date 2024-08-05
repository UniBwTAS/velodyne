add_executable(packetpc_node packetpc_node.cpp unpacker.cpp)
target_link_libraries(packetpc_node
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)
add_dependencies(packetpc_node ${PROJECT_NAME}_gencfg)


add_library(packetpc_nodelet packetpc_nodelet.cpp unpacker.cpp)
target_link_libraries(packetpc_nodelet
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES})
add_dependencies(packetpc_nodelet ${PROJECT_NAME}_gencfg)
