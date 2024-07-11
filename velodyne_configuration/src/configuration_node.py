#!/usr/bin/env python3
#
# curl --data “returns=Strongest” http://192.168.1.201/cgi/setting

import rospy
import velodyne_curl_config

class Configurator_node:
    def __init__(self):
        rospy.init_node("velodyne_configuration_node", anonymous=True)
        rospy.loginfo("Velodyne configuration node initializing")

        velodyne_ip = '192.168.1.201'
        if rospy.has_param('velodyne_ip'):
            velodyne_ip = rospy.get_param('~velodyne_ip')
        else:
            rospy.loginfo("velodyne_ip parameter not found, using default ip %s", velodyne_ip )

        rospy.loginfo("Using sensor IP: %s", velodyne_ip)

        self.configurator = velodyne_curl_config.Configurator(velodyne_ip)






        rc = self.sensor_do(self.sensor, Base_URL + 'reset', urlencode({'data': 'reset_system'}), self.buffer)
        if rc:
            time.sleep(10)
            rc = self.sensor_do(self.sensor, Base_URL + 'setting', urlencode({'rpm': '300'}), self.buffer)
            if rc:
                time.sleep(1)
                rc = self.sensor_do(self.sensor, Base_URL + 'setting', urlencode({'laser': 'on'}), self.buffer)
                if rc:
                    time.sleep(10)


if __name__ == "__main__":
    try:
        Configurator()
    except Exception as inst:
        print(type(inst))  # the exception type

        print(inst.args)  # arguments stored in .args

        print(inst)

    rospy.spin()
