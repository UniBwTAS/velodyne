add_library(data_containers pointcloudXYZIRT.cc organized_cloudXYZIRT.cc pointcloud_extended.cc)
add_dependencies(data_containers ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(data_containers velodyne_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(transform_node transform_node.cc transform.cc)
add_dependencies(transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_node velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_library(transform_nodelet transform_nodelet.cc transform.cc)
add_dependencies(transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transform_nodelet velodyne_rawdata data_containers
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(ego_motion_correction_node ego_motion_correction_node.cc ego_motion_correction.cc)
add_dependencies(ego_motion_correction_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ego_motion_correction_node ${catkin_LIBRARIES})

add_library(ego_motion_correction_nodelet ego_motion_correction_nodelet.cc ego_motion_correction.cc)
add_dependencies(ego_motion_correction_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ego_motion_correction_nodelet ${catkin_LIBRARIES})

install(TARGETS data_containers transform_node transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS data_containers ego_motion_correction_node ego_motion_correction_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
