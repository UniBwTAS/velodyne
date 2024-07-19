#!/usr/bin/env python3

import rospy
from velodyne_curl_communication_lib.velodyne_curl_config import Configurator
from velodyne_msgs.msg import VelodyneReturnMode
from velodyne_msgs.srv import VelodyneRequestConfiguration
from velodyne_msgs.srv import VelodyneRequestConfigurationRequest
from velodyne_msgs.srv import VelodyneRequestConfigurationResponse
from velodyne_msgs.srv import VelodyneSetConfiguration
from velodyne_msgs.srv import VelodyneSetConfigurationRequest
from velodyne_msgs.srv import VelodyneSetConfigurationResponse
from velodyne_msgs.srv import VelodyneSpecialCommands
from velodyne_msgs.srv import VelodyneSpecialCommandsRequest

return_options = {
    'Strongest': VelodyneReturnMode.STRONGEST,
    'strongest': VelodyneReturnMode.STRONGEST,
    'Last': VelodyneReturnMode.LAST,
    'last': VelodyneReturnMode.LAST,
    'Dual': VelodyneReturnMode.DUAL,
    'dual': VelodyneReturnMode.DUAL,
    'Dual_conf': VelodyneReturnMode.DUAL_CONF,
    'dual_conf': VelodyneReturnMode.DUAL_CONF,

}


def request_config_from_current_config(current):
    request = VelodyneSetConfigurationRequest()
    request.stamp = rospy.Time.now()

    request.rpm = current.rpm
    request.fov_start = current.fov_start
    request.fov_end = current.fov_end
    request.returns = current.returns
    request.phase = current.phase
    request.phaselock = current.phaselock
    request.laser = current.laser
    request.host_addr = current.host_addr
    request.host_dport = current.host_dport
    request.host_tport = current.host_tport
    request.net_addr = current.net_addr
    request.net_mask = current.net_mask
    request.net_gateway = current.net_gateway

    return request

if __name__ == "__main__":
    try:
        rospy.init_node("velodyne_initialization_node", anonymous=True)
        rospy.loginfo("Velodyne initialization node starting")

        # Define and wait for the services
        set_config_srv_proxy = rospy.ServiceProxy('set_configuration',
                                                  VelodyneSetConfiguration)
        request_config_srv_proxy = rospy.ServiceProxy('request_configuration',
                                                      VelodyneRequestConfiguration)
        # Get the desired parameters from the launch file

        return_mode = VelodyneReturnMode.STRONGEST
        parameter_name = rospy.resolve_name('~return_mode')
        if rospy.has_param(parameter_name):
            soll = rospy.get_param(parameter_name)
            if not isinstance(soll,str):
                rospy.logerr("The desired return should be a string")
                rospy.logerr(soll)
                raise TypeError

            if soll in return_options:
                return_mode = return_options[soll]
            else:
                rospy.logerr("The desired return mode was not found: %s" % soll)
                raise NameError
        else:
            rospy.loginfo("return_mode parameter not found, using default Strongest")
        rpm = 600
        parameter_name = rospy.resolve_name("~rpm")
        if rospy.has_param(parameter_name):
            rpm = rospy.get_param(parameter_name)
            if not isinstance(rpm,int):
                rospy.logerr("The desired rpm should be an integer, and a multiple of 60")
                rospy.logerr(rpm)
                raise TypeError
            if not rpm % 60 == 0:
                rospy.logerr("The desired rpm should be an integer, and a multiple of 60")
                rospy.logerr(rpm)
                raise ValueError

        else:
            rospy.loginfo("rpm parameter not found, using default value %i" % rpm)
        fov_start = 0
        parameter_name = rospy.resolve_name("~fov_start")
        if rospy.has_param(parameter_name):
            fov_start = rospy.get_param(parameter_name)
            if not isinstance(fov_start,int):
                rospy.logerr("The desired fov_start should be an integer")
                rospy.logerr(fov_start)
                raise TypeError
        else:
            rospy.loginfo("fov_start parameter not found, using default value %i" % fov_start)
        fov_end = 359
        parameter_name = rospy.resolve_name("~fov_end")
        if rospy.has_param(parameter_name):
            fov_end = rospy.get_param(parameter_name)
            if not isinstance(fov_end,int):
                rospy.logerr("The desired fov_end should be an integer")
                rospy.logerr(fov_end)
                raise TypeError
        else:
            rospy.loginfo("fov_end parameter not found, using default value %i" % fov_end)
        phaselock = False
        parameter_name = rospy.resolve_name("~phaselock")
        if rospy.has_param(parameter_name):
            phaselock = rospy.get_param(parameter_name)
            if not isinstance(phaselock,bool):
                rospy.logerr("The desired phaselock should be a bool")
                rospy.logerr(phaselock)
                raise TypeError
        else:
            rospy.loginfo("phaselock parameter not found, using default value %i" % phaselock)
        phase = 0
        parameter_name = rospy.resolve_name("~phase")
        if rospy.has_param(parameter_name):
            phase = rospy.get_param(parameter_name)
            if not isinstance(phase, float):
                rospy.logerr("The desired phase should be a float value")
                rospy.logerr(phase)
                raise TypeError
            if not phase*100 <= 99999:
                rospy.logerr("The maximum allowed precision of the phase parameter is 2 decimal digits")
                rospy.logerr(phase)
                raise ValueError
        else:
            rospy.loginfo("phase parameter not found, using default value %i" % phase)

        # Wait for the services to become available

        rospy.loginfo("Velodyne Initialization node: Waiting for services to become available")
        rospy.wait_for_service(set_config_srv_proxy.resolved_name)
        rospy.wait_for_service(request_config_srv_proxy.resolved_name)
        rospy.loginfo("Velodyne Initialization node: Services Found, Initializing")

        # Try to get current confing and set the desired confing
        done = False
        while not done:
            current_done = False
            while not current_done:
                # Request current configuration
                get_config_request = VelodyneRequestConfigurationRequest()
                get_config_request.stamp = rospy.Time.now()
                current_config = request_config_srv_proxy(get_config_request)
                current_done = current_config.success
                if not current_config:
                    rospy.logerr("Failed to get current configuration from sensor. Trying again.")

            set_config_request = request_config_from_current_config(current_config)
            set_config_request.returns.return_mode = return_mode
            set_config_request.rpm = rpm
            set_config_request.fov_start = fov_start
            set_config_request.fov_end = fov_end
            set_config_request.phaselock = phaselock
            set_config_request.phase = int(phase*100)
            set_response = set_config_srv_proxy(set_config_request)
            done = set_response.success
            if not done:
                rospy.logerr("Failed to set desired configuration in the sensor. Trying again.")

        rospy.loginfo("Velodyne configuration successful, shutting down node")




    except Exception as inst:
        rospy.logerr(type(inst))  # the exception type
        rospy.logerr(inst.args)  # arguments stored in .args
        rospy.logerr(inst)


