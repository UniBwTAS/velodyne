#!/usr/bin/env python3

import rospy
import time

from velodyne_curl_communication_lib.velodyne_curl_config import Configurator
from velodyne_msgs.msg import VelodyneReturnMode
import os
from velodyne_msgs.srv import VelodyneRequestConfiguration
from velodyne_msgs.srv import VelodyneRequestConfigurationRequest
from velodyne_msgs.srv import VelodyneRequestConfigurationResponse
from velodyne_msgs.srv import VelodyneSetConfiguration
from velodyne_msgs.srv import VelodyneSetConfigurationRequest
from velodyne_msgs.srv import VelodyneSetConfigurationResponse
from velodyne_msgs.srv import VelodyneSpecialCommands
from velodyne_msgs.srv import VelodyneSpecialCommandsRequest
from velodyne_msgs.srv import VelodyneSpecialCommandsResponse




class ConfiguratorNode:
    def __init__(self):
        rospy.init_node("velodyne_configuration_test_node", anonymous=True)
        rospy.loginfo("Velodyne configuration test node initializing")

        self._set_config_srv_proxy = rospy.ServiceProxy('/sensor/lidar/vls128_roof/set_configuration',
                                                        VelodyneSetConfiguration)
        self._request_config_srv_proxy = rospy.ServiceProxy('/sensor/lidar/vls128_roof/request_configuration',
                                                            VelodyneRequestConfiguration)
        self._special_config_srv_proxy = rospy.ServiceProxy('/sensor/lidar/vls128_roof/special_command',
                                                            VelodyneSpecialCommands)

        rospy.wait_for_service('/sensor/lidar/vls128_roof/set_configuration')
        rospy.wait_for_service('/sensor/lidar/vls128_roof/request_configuration')
        rospy.wait_for_service('/sensor/lidar/vls128_roof/special_command')

        rospy.loginfo("All Velodyne configuration services available")

        # Get current conf and change the desired settings
        get_config_request = VelodyneRequestConfigurationRequest()
        get_config_request.stamp = rospy.Time.now()
        print(get_config_request.stamp.secs)
        print(get_config_request.stamp.nsecs)
        print(get_config_request.stamp)
        print(type(get_config_request))
        current_config = self._request_config_srv_proxy(get_config_request)

        if current_config.success:
            print("received current configuration")
            print("gps_pps_state")
            print(current_config.gps_pps_state)
            print("gps_position")
            print(current_config.gps_position)
            print("tod_time")
            print(current_config.tod_time)
            print("usectoh")
            print(current_config.usectoh)
            print("motor_state")
            print(current_config.motor_state)
            print("ns_per_rev")
            print(current_config.ns_per_rev)
            print("rpm")
            print(current_config.rpm)
            print("fov_start")
            print(current_config.fov_start)
            print("fov_end")
            print(current_config.fov_end)
            print("returns")
            print(current_config.returns.return_mode)
            print("phase")
            print(current_config.phase)
            print("phaselock")
            print(current_config.phaselock)
            print("laser")
            print(current_config.laser)
            print("host_addr")
            print(current_config.host_addr)
            print("host_dport")
            print(current_config.host_dport)
            print("host_tport")
            print(current_config.host_tport)
            print("net_addr")
            print(current_config.net_addr)
            print("net_mask")
            print(current_config.net_mask)
            print("net_gateway")
            print(current_config.net_gateway)
        else:
            print("fail to get configuration")
            raise RuntimeError

        set_config_request = self.request_config_from_current_config(current_config)
        set_config_request.returns.return_mode = VelodyneReturnMode.DUAL
        set_response = self._set_config_srv_proxy(set_config_request)

        spc_cmd_config_request = VelodyneSpecialCommandsRequest()
        # spc_cmd_config_request.stamp = rospy.Time.now()
        spc_cmd_config_request.command = VelodyneSpecialCommandsRequest.DIAGNOSTICS

        spc_cmd_response = self._special_config_srv_proxy(spc_cmd_config_request)

        if spc_cmd_response.success:
            print("all parameters within the operational range")
        else:
            print("parameter outside its operational range!")
            for p in spc_cmd_response.parameters:
                print(p)

        spc_cmd_config_request = VelodyneSpecialCommandsRequest()
        # spc_cmd_config_request.stamp = rospy.Time.now()
        spc_cmd_config_request.command = VelodyneSpecialCommandsRequest.DOWNLOAD_SNAPSHOT

        spc_cmd_response = self._special_config_srv_proxy(spc_cmd_config_request)

        spc_cmd_config_request = VelodyneSpecialCommandsRequest()
        # spc_cmd_config_request.stamp = rospy.Time.now()
        spc_cmd_config_request.command = VelodyneSpecialCommandsRequest.RESET_SENSOR

        spc_cmd_response = self._special_config_srv_proxy(spc_cmd_config_request)

        # Get current conf and change the desired settings
        get_config_request = VelodyneRequestConfigurationRequest()
        get_config_request.stamp = rospy.Time.now()
        current_config = self._request_config_srv_proxy(get_config_request)
        set_config_request = self.request_config_from_current_config(current_config)
        set_config_request.returns.return_mode = VelodyneReturnMode.STRONGEST
        set_config_request.rpm = 300
        set_config_request.fov_start = 90
        set_config_request.fov_end = 180
        set_response = self._set_config_srv_proxy(set_config_request)

    def request_config_from_current_config(self, current):
        set_config_request = VelodyneSetConfigurationRequest()
        set_config_request.stamp = rospy.Time.now()

        set_config_request.rpm = current.rpm
        set_config_request.fov_start = current.fov_start
        set_config_request.fov_end = current.fov_end
        set_config_request.returns = current.returns
        set_config_request.phase = current.phase
        set_config_request.phaselock = current.phaselock
        set_config_request.laser = current.laser
        set_config_request.host_addr = current.host_addr
        set_config_request.host_dport = current.host_dport
        set_config_request.host_tport = current.host_tport
        set_config_request.net_addr = current.net_addr
        set_config_request.net_mask = current.net_mask
        set_config_request.net_gateway = current.net_gateway

        return set_config_request


if __name__ == "__main__":
    try:
        ConfiguratorNode()
    except Exception as inst:
        print(type(inst))  # the exception type

        print(inst.args)  # arguments stored in .args

        print(inst)

    rospy.spin()
