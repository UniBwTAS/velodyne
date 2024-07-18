#!/usr/bin/env python3

import rospy
import time

from velodyne_curl_communication_lib.velodyne_curl_config import Configurator
from velodyne_msgs.msg import VelodyneReturnMode
from velodyne_msgs.srv import VelodyneRequestConfiguration
from velodyne_msgs.srv import VelodyneRequestConfigurationResponse
from velodyne_msgs.srv import VelodyneSetConfiguration
from velodyne_msgs.srv import VelodyneSetConfigurationResponse
from velodyne_msgs.srv import VelodyneSpecialCommands
from velodyne_msgs.srv import VelodyneSpecialCommandsRequest
from velodyne_msgs.srv import VelodyneSpecialCommandsResponse

import os


class ConfiguratorNode:
    return_modes = {
        'Strongest': VelodyneReturnMode.STRONGEST,
        'Last': VelodyneReturnMode.LAST,
        'Dual': VelodyneReturnMode.DUAL,
        'Dual_conf': VelodyneReturnMode.DUAL_CONF,
    }

    return_mode_to_str_cmd = {
        VelodyneReturnMode.STRONGEST: 'Strongest',
        VelodyneReturnMode.LAST: 'Last',
        VelodyneReturnMode.DUAL: 'Dual',
        VelodyneReturnMode.DUAL_CONF: 'Dual_conf',
    }

    def __init__(self):
        rospy.loginfo("Velodyne configuration node initializing")
        rospy.init_node("velodyne_configuration_node", anonymous=False)

        ns = '/sensor/lidar/vls128_roof'
        parameter_name = rospy.resolve_name('~ns_transform')
        if rospy.has_param(parameter_name):
            ns = rospy.get_param(parameter_name)
        else:
            rospy.loginfo("name space  parameter not found, using default ip %s" % ns)

        self._set_service = rospy.Service('/set_configuration', VelodyneSetConfiguration,
                                          self.set_configuration)
        self._get_service = rospy.Service('/request_configuration',
                                          VelodyneRequestConfiguration,
                                          self.get_configuration)
        self._cmd_service = rospy.Service('/special_command', VelodyneSpecialCommands,
                                          self.send_command)

        velodyne_ip = '192.168.3.201'
        parameter_name = rospy.resolve_name('~velodyne_ip')
        if rospy.has_param(parameter_name):
            velodyne_ip = rospy.get_param(parameter_name)
        else:
            rospy.loginfo("velodyne_ip parameter not found, using default ip %s" % velodyne_ip)

        rospy.loginfo("Using sensor IP: %s" % velodyne_ip)

        home = os.environ.get('HOME')
        self.snapshot_path = home + "/Downloads"
        parameter_name = rospy.resolve_name('~snapshot_path')
        if rospy.has_param(parameter_name):
            self.snapshot_path = rospy.get_param(parameter_name)
        else:
            rospy.loginfo("parameter_name parameter not found, using default ip %s" % self.snapshot_path)

        print("Create configuration Object")
        self.configurator = Configurator(velodyne_ip)  # Loads config object with current state
        print("Configuration node ready")

    def send_command(self, request):
        response = VelodyneSpecialCommandsResponse()
        try:
            if request.command == VelodyneSpecialCommandsRequest.DIAGNOSTICS:
                diag = self.configurator.get_diagnostics()
                response.all_in_range = True
                outside_range = []
                for b in diag:
                    print(b)
                    for n in diag[b]:
                        print(n)
                        try:
                            in_range = self.configurator.check_diagnostics_parameter(b, n, diag[b][n])
                        except NameError as e:
                            print("No ranges for parameter", b, n)
                            in_range = True
                        if not in_range:
                            print("parameter %s %s is outside operational ranges [%f]" % (b, n, diag[b][n]))
                            response.all_in_range = False
                            outside_range.append(b + "_" + n)
                response.parameters = outside_range
                response.success = True

            elif request.command == VelodyneSpecialCommandsRequest.RESET_SENSOR:
                self.configurator.reset_sensor()
                response.success = True
            elif request.command == VelodyneSpecialCommandsRequest.DOWNLOAD_SNAPSHOT:
                self.configurator.download_snapshot(self.snapshot_path)
                response.success = True
            elif request.command == VelodyneSpecialCommandsRequest.SAVE_CONFIG:
                self.configurator.save_current_config_to_sensor()
                response.success = True
            else:
                response.success = False
        except RuntimeError as e:
            response.success = False
            print("Caught Runtime Error while getting current configuration")
            print(e.args)

        return response

    def get_configuration(self, request):
        print("Request to get current configuration")
        response = VelodyneRequestConfigurationResponse()
        try:
            current_conf = self.configurator.get_current_configuration().conf
            response.gps_pps_state = current_conf["gps_pps_state"]
            response.gps_position = current_conf["gps_position"]
            response.tod_time.secs = current_conf["tod_sec"]
            response.tod_time.nsecs = current_conf["tod_nsec"]
            response.usectoh = current_conf["usectoh"]
            response.motor_state = current_conf["motor_state"]
            response.rpm = current_conf["rpm"]
            response.phaselock = current_conf["phaselock"]
            response.phase = current_conf["phase"]
            response.ns_per_rev = current_conf["ns_per_rev"]
            response.laser = current_conf["laser"]
            rt_msg = VelodyneReturnMode()
            rt_msg.return_mode = ConfiguratorNode.return_modes[current_conf["returns"]]
            response.returns = rt_msg
            response.fov_start = current_conf["fov_start"]
            response.fov_end = current_conf["fov_end"]
            response.host_addr = current_conf["host_addr"]
            response.host_dport = current_conf["host_dport"]
            response.host_tport = current_conf["host_tport"]
            response.net_addr = current_conf["net_addr"]
            response.net_mask = current_conf["net_mask"]
            response.net_gateway = current_conf["net_gateway"]
            #response.net_dhcp = current_conf["net_dhcp"]
            #response.net_mac_addr = current_conf["net_mac_addr"]

            response.success = True
        except RuntimeError as e:
            print("Caught Runtime Error while getting current configuration")
            print(e.args)

            response.success = False

        response.stamp = rospy.Time.now()

        return response

    def set_configuration(self, request):
        print("Request to set configuration")
        # Find what is different
        changes = {}
        if request.rpm != self.configurator.get_setting("rpm"):
            changes["rpm"] = request.rpm
        if request.fov_start != self.configurator.get_setting("fov_start"):
            changes["fov_start"] = request.fov_start
        if request.fov_end != self.configurator.get_setting("fov_end"):
            changes["fov_end"] = request.fov_end
        if request.returns.return_mode != ConfiguratorNode.return_modes[self.configurator.get_setting("returns")]:
            changes["returns"] = request.returns.return_mode
        if request.phaselock != self.configurator.get_setting("phaselock"):
            changes["phaselock"] = request.phaselock
        if request.phase != self.configurator.get_setting("phase"):
            changes["phase"] = request.phase
        if request.laser != self.configurator.get_setting("laser"):
            changes["laser"] = request.laser
        if request.host_addr != self.configurator.get_setting("host_addr"):
            changes["host_addr"] = request.host_addr
        if request.host_dport != self.configurator.get_setting("host_dport"):
            changes["host_dport"] = request.host_dport
        if request.host_tport != self.configurator.get_setting("host_tport"):
            changes["host_tport"] = request.host_tport
        if request.net_addr != self.configurator.get_setting("net_addr"):
            changes["net_addr"] = request.net_addr
        if request.net_mask != self.configurator.get_setting("net_mask"):
            changes["net_mask"] = request.net_mask
        if request.net_gateway != self.configurator.get_setting("net_gateway"):
            changes["net_gateway"] = request.net_gateway

        response = VelodyneSetConfigurationResponse()
        for k in changes:
            try:
                self.configurator.set_setting(k, changes[k])
            except ValueError as e:
                print("Caught ValueError Error while setting current configuration")
                print(e.args)
                response.success = False
                return response

            except NameError as e:
                print("Caught NameError Error while setting current configuration")
                print(e.args)
                response.success = False
                return response
            except Exception as e:
                print("Caught Exception Error while setting current configuration")
                print(e.args)
                response.success = False
                return response

        response.stamp = rospy.Time.now()
        response.success = True
        return response


if __name__ == "__main__":
    try:
        ConfiguratorNode()
    except Exception as inst:
        print(type(inst))  # the exception type

        print(inst.args)  # arguments stored in .args

        print(inst)

    rospy.spin()
