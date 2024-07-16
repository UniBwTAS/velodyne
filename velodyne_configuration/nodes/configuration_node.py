#!/usr/bin/env python3

import rospy
import time

from velodyne_curl_communication_lib.velodyne_curl_config import Configurator
from velodyne_msgs.msg import VelodyneReturnMode
import os


class ConfiguratorNode:
    def __init__(self):
        rospy.init_node("velodyne_configuration_node", anonymous=True)
        rospy.loginfo("Velodyne configuration node initializing")

        velodyne_ip = '192.168.3.201'
        if rospy.has_param('velodyne_ip'):
            velodyne_ip = rospy.get_param('~velodyne_ip')
        else:
            rospy.loginfo("velodyne_ip parameter not found, using default ip %s", velodyne_ip)

        rospy.loginfo("Using sensor IP: %s", velodyne_ip)

        self.configurator = Configurator(velodyne_ip)
        print("Changing return mode")
        self.configurator.set_setting("returns", VelodyneReturnMode.DUAL)
        time.sleep(10)
        print("get diagnostics")
        d = self.configurator.get_diagnostics()
        print("diagnostics:")
        print(d)
        time.sleep(10)
        state = self.configurator.get_status()
        print("state")
        print(state)
        time.sleep(10)
        print("Turning the lasers off")
        self.configurator.set_setting("laser", "off")
        time.sleep(10)
        print("Turning the laser on")
        self.configurator.set_setting("laser", "on")
        time.sleep(10)
        print("Setting return mode Strong")
        self.configurator.set_setting("returns", VelodyneReturnMode.STRONGEST)
        time.sleep(10)
        print("Getting diagnostics")
        diagnostics = self.configurator.get_diagnostics()
        print("diagnostics")
        print(diagnostics)
        print("Resetting sensor")
        self.configurator.reset_sensor()
        time.sleep(15)
        print("Downloading snapshot")
        home = os.environ.get('HOME')
        self.configurator.download_snapshot(home+"/Downloads")

        rospy.sleep(rospy.Duration.from_sec(10))
        print("done")

if __name__ == "__main__":
    try:
        ConfiguratorNode()
    except Exception as inst:
        print(type(inst))  # the exception type

        print(inst.args)  # arguments stored in .args

        print(inst)

    rospy.spin()
