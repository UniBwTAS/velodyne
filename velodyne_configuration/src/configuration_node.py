#!/usr/bin/env python3
#
# curl --data “returns=Strongest” http://192.168.1.201/cgi/setting

import rospy
import pycurl
from io import BytesIO
from urllib.parse import urlencode
import urllib.request
import json
import time
from urllib.error import HTTPError, URLError
import socket
from velodyne_msgs.msg import VelodyneReturnMode
import math


# It is recommended to issue at most one curl command per second to the sensor.

class VelodyneConfiguration:

    def __init__(self):
        # Configuration of the sensor
        self.conf = {
            "gps": {"pps_state": "Absent",
                    "position": ""},
            "motor": {"state": "On",
                      "rpm": 600,
                      "lock": "Off",
                      "phase": 0},
            "laser": {"state": "On"},
            "returns": "Strongest",
            "fov": {"start": 0,
                    "end": 359},
            "host": {"addr": "255.255.255.255",
                     "host_dport": 2368,
                     "host_tport": 8309, },
            "net": {"addr": "192.168.1.200",
                    "mask": "255.255.255.0",
                    "gateway": "192.168.1.2",
                    "dhcp": "on"},
            #"": ,
            # "": ,
        }

    # list of configurable parameter, probably there are more e.g. laser on/off Gps pps lock delay
    conf_ids = ['rpm', "fov_start", "fov_end", "returns", "phaselock", "host_addr", "host_dport", "host_tport",
                "net_ip",
                "net_mask", "net_gateway", ]
    # Take care when turning DHCP ON. Before doing so, ensure a DHCP server on the network is available to provide the
    # sensor with an IP address. If you turn DHCP ON and lose contact with the sensor, follow the Turned DHCP On, Lost
    # Contact With Sensor on page 92 procedure to get it back.
    conf_banned = ["net_dhcp"]

    # Commands translation to url endings
    __commands = {
        'get_status': 'status.json',
        'get_diagnostic': 'diag.json',
        'get_configuration': 'snapshot.json',
        'set_setting': 'setting',
        'reset': 'reset',
        'save_cfg': 'save',
    }

    reset = "reset_system"
    submit = "submit"

    # available return modes
    __return_modes = {
        VelodyneReturnMode.STRONGEST: 'Strongest',
        VelodyneReturnMode.LAST: 'Last',
        VelodyneReturnMode.DUAL: 'Dual',
        VelodyneReturnMode.DUAL_CONF: 'Dual',
    }

    # Dict with the functions to interpret the diagnostig data to human readable units
    __diagnostic_data_interpreter = \
        {'top:hv': (lambda raw_val: 101.0 * (VelodyneConfiguration.std_voltage_conversion(raw_val) - 5.0)),
         'top:lm20_temp': (lambda raw_val: VelodyneConfiguration.std_temperature_conversion(raw_val)),
         'top:pwr_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val) * 2.0),
         'top:pwr_2_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'top:pwr_3_3v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'top:pwr_raw': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'top:pwr_vccint': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'bot:i_out': (lambda raw_val: VelodyneConfiguration.std_current_conversion(raw_val)),
         'bot:lm20_temp': (lambda raw_val: VelodyneConfiguration.std_temperature_conversion(raw_val)),
         'bot:pwr_1_2v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'bot:pwr_1_25v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'bot:pwr_1_2_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'bot:pwr_3_3v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
         'bot:pwr_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val) * 2.0),
         'bot:pwr_v_in': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val) * 11.0)
         }
    # Operational ranges according to manual
    operational_ranges = \
        {
            'top:hv': [-150.0, -132.0],
            'top:lm20_temp': [25.0, 90.0],
            'top:pwr_5v': [4.8, 5.2],
            'top:pwr_2_5v': [2.3, 2.7],
            'top:pwr_3_3v': [3.1, 3.5],
            'top:pwr_raw': [-math.inf, math.inf],
            'top:pwr_vccint': [1.0, 1.4],
            'bot:i_out': [0.3, 1.0],
            'bot:lm20_temp': [-25.0, 90.0],
            'bot:pwr_1_2v': [1.0, 1.4],
            'bot:pwr_1_25v': [1.0, 1.4],
            'bot:pwr_1_2_5v': [2.3, 2.7],
            'bot:pwr_3_3v': [3.1, 3.5],
            'bot:pwr_5v': [4.8, 5.2],
            'bot:pwr_v_in': [8.0, 19.0],
        }

    def config(self, name):
        return self.conf[name]

    def set(self, name, value):
        if name in self.conf:
            self.conf[name] = value
        else:
            raise NameError("Name not accepted in set() method")

    @staticmethod
    def get_command_url(cmd_id, base_url):
        if cmd_id in VelodyneConfiguration.__commands:
            return base_url + VelodyneConfiguration.__commands[cmd_id]
        else:
            raise NameError("Command name not accepted in get_command_url() method")

    @staticmethod
    def std_voltage_conversion(raw_val):
        return raw_val * 5.0 / 4096

    @staticmethod
    def std_current_conversion(raw_val):
        return 10.0 * (raw_val * 5.0 / 4096 - 2.5)

    @staticmethod
    def std_temperature_conversion(raw_val):
        return -1481.96 + math.sqrt(2.1962e6 + (1.8639 - (raw_val * 5.0 / 4096)) / 3.88e-6)

    @staticmethod
    def interpret_diagnostic_data(name, raw_val):
        if name in VelodyneConfiguration.__diagnostic_data_interpreter:
            op = VelodyneConfiguration.__diagnostic_data_interpreter[name]
            return op(raw_val)
        else:
            raise NameError("Diagnostic data %s name not accepted", name)

    @staticmethod
    def check_diagnostic_data_in_range(name, interpreted_val):
        if name in VelodyneConfiguration.__diagnostic_data_interpreter:
            val_range = VelodyneConfiguration.__operational_ranges[name]
            return val_range[0] < interpreted_val < val_range[1]
        else:
            raise NameError("Diagnostic data %s name not accepted", name)

    @staticmethod
    def check_rpm_soll_val(soll_val):
        return (300 <= soll_val <= 1200) and (soll_val % 60 == 0)

    @staticmethod
    def check_fov_soll_val(soll_val):
        return 0 <= soll_val <= 359

    @staticmethod
    def check_return_mode_soll_val(soll_val):
        return soll_val in VelodyneConfiguration.__return_modes

    @staticmethod
    def check_phase_lock_soll_val(soll_val):
        return 0 <= soll_val <= 359


class Configurator:
    def __init__(self, velodyne_ip):
        self.config = VelodyneConfiguration()
        self.Base_URL = 'http://' + velodyne_ip + '/cgi/'
        self.sensor = pycurl.Curl()
        self.buffer = BytesIO()

    def test_connection(self):
        pass

    def set_setting(self, setting_id, value):
        #phaselock
        #offset = offsetInput*100
        #enabled={on|off}&offset=27025&offsetInput=270.25

        #network
        #   save and restart needed
        pass

    def get_setting(self, setting_id):
        pass

    def sensor_do(self, url, encoded_command):
        self.sensor.setopt(self.sensor.URL, url)
        self.sensor.setopt(self.sensor.POSTFIELDS, encoded_command)
        self.sensor.setopt(self.sensor.WRITEDATA, self.buffer)
        self.sensor.perform()
        rcode = self.sensor.getinfo(self.sensor.RESPONSE_CODE)
        success = rcode in range(200, 207)
        print('%s %s: %d (%s)' % (url, encoded_command, rcode, 'OK' if success else 'ERROR'))
        # It is recommended to issue at most one curl command per second to the sensor.
        time.sleep(1.2)
        return success

    def reset_sensor(self):
        rc = self.sensor_do(self.config.get_command_url("reset", self.Base_URL),
                            urlencode({'data': self.config.reset}))

    def safe_current_config_to_sensor(self):
        rc = self.sensor_do(self.config.get_command_url("save_cfg", self.Base_URL),
                            urlencode({'data': self.config.submit}))

    def __del__(self):
        self.sensor.close()


class Configurator_node:
    def __init__(self):
        rospy.init_node("velodyne_configuration_node", anonymous=True)
        rospy.loginfo("Velodyne configuration node initializing")

        velodyne_ip = '192.168.1.201'
        if rospy.has_param('to_delete'):
            velodyne_ip = rospy.get_param('~velodyne_ip')
        else:
            rospy.loginfo("velodyne_ip parameter not found", )

        rospy.loginfo("Using sensor IP: %s", velodyne_ip)
        Base_URL = 'http://' + velodyne_ip + '/cgi/'

        self.sensor = pycurl.Curl()
        self.buffer = BytesIO()

        # Test connection to sensor
        try:

            url = VelodyneConfiguration.get_command_url('get_status', Base_URL)
            response = urllib.request.urlopen(url, timeout=5)
            time.sleep(1.2)
        except HTTPError as error:
            rospy.logerr('HTTP Error: Data not retrieved because %s\nURL: %s', error, url)
            raise Exception('Fail to communicate with sensor')
        except URLError as error:
            if isinstance(error.reason, socket.timeout):
                rospy.logerr('socket timed out - URL %s', url)
                raise Exception('Fail to communicate with sensor')
            else:
                rospy.logerr('some other error happened: %s', error)
                raise Exception('Fail to communicate with sensor')

        if response:
            status = json.loads(response.read())
            print('Sensor laser is %s, motor rpm is %s',
                  status['laser']['state'], status['motor']['rpm'])
        else:
            rospy.logerr('Error Response from sensor is empty')
            raise Exception('Fail to communicate with sensor')

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
