#!/usr/bin/env python3

import rospy
import velodyne_curl_config
from velodyne_msgs.msg import VelodyneReturnMode

class ConfiguratorNode:
    def __init__(self):
        rospy.init_node("velodyne_configuration_node", anonymous=True)
        rospy.loginfo("Velodyne configuration node initializing")

        velodyne_ip = '192.168.103.201'
        if rospy.has_param('velodyne_ip'):
            velodyne_ip = rospy.get_param('~velodyne_ip')
        else:
            rospy.loginfo("velodyne_ip parameter not found, using default ip %s", velodyne_ip )

        rospy.loginfo("Using sensor IP: %s", velodyne_ip)

        self.configurator = velodyne_curl_config.Configurator(velodyne_ip)

        self.configurator.set_setting("returns", VelodyneReturnMode.DOUBLE)
        d = self.configurator.get_diagnostics()
        print("diagnostics:")
        print(d)
        state = self.configurator.get_status()
        print("state")
        print(state)
        self.configurator.set_setting("laser","off")
        rospy.sleep(rospy.Duration.from_sec(5))
        self.configurator.set_setting("laser","on")
        self.configurator.set_setting("returns",VelodyneReturnMode.STRONGEST)

        diagnost = self.configurator.get_diagnostics()
        print("diagnost")
        print(diagnost)

        self.configurator.reset_sensor()

        rospy.sleep(rospy.Duration.from_sec(10))



if __name__ == "__main__":
    try:
        ConfiguratorNode()
    except Exception as inst:
        print(type(inst))  # the exception type

        print(inst.args)  # arguments stored in .args

        print(inst)

    rospy.spin()
