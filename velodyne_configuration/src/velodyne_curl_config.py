import pycurl
from io import BytesIO
from urllib.parse import urlencode
import urllib.request
import json
import time
from urllib.error import HTTPError, URLError
import socket
import math


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
            # "": ,
            # "": ,
        }

    # list of configurable parameter, probably there are more e.g. laser on/off Gps pps lock delay
    conf_ids = ['rpm', "fov_start", "fov_end", "returns", "phaselock", "phaselock_off", "laser",
                "host_addr", "host_dport", "host_tport",
                "net_addr", "net_mask", "net_gateway"]
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
    return_modes = {
        55: 'Strongest',
        56: 'Last',
        57: 'Dual',
        59: 'Dual_conf',
    }

    # Dict with the functions to interpret the diagnostic data to human-readable units
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
        return soll_val in VelodyneConfiguration.return_modes

    @staticmethod
    def check_phase_lock_soll_val(soll_val):
        return 0 <= soll_val <= 359

    @staticmethod
    def check_on_off_soll_val(soll_val):
        return soll_val == 'on' or soll_val == 'off'


class Configurator:
    def __init__(self, velodyne_ip):
        self.config = VelodyneConfiguration()
        self.Base_URL = 'http://' + velodyne_ip + '/cgi/'
        self.sensor = pycurl.Curl()
        self.buffer = BytesIO()
        self.test_connection()

    def test_connection(self):
        # Test connection to sensor
        status = self._request_json('get_status')
        print('Sensor laser is %s, motor rpm is %s, Return mode is %s',
              status['laser']['state'], status['motor']['rpm'], status['returns'])

    def _request_json(self,cmd):
        try:
            url = VelodyneConfiguration.get_command_url(cmd, self.Base_URL)
            response = urllib.request.urlopen(url, timeout=5)
            time.sleep(1.2)
        except HTTPError as error:
            print('HTTP Error: Data not retrieved because %s\nURL: %s', error, url)
            raise Exception('Fail to communicate with sensor')
        except URLError as error:
            if isinstance(error.reason, socket.timeout):
                print('socket timed out - URL %s', url)
                raise Exception('Fail to communicate with sensor')
            else:
                print('some other error happened: %s', error)
                raise Exception('Fail to communicate with sensor')

        if response:
            js = json.loads(response.read())
            return js

        else:
            print('Error Response from sensor is empty')
            raise Exception('Fail to communicate with sensor')

    def set_setting(self, setting_id, value):
        if setting_id in VelodyneConfiguration.conf_ids:
            rc = None
            if setting_id == 'rpm':
                if VelodyneConfiguration.check_rpm_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL),
                                        urlencode({'rpm': str(value)}))
                else:
                    raise ValueError("The desired value for the rpm is outside the allowed range")

            elif setting_id == 'fov_start':
                if VelodyneConfiguration.check_fov_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + '/fov',
                                        urlencode({'start': str(value)}))
                else:
                    raise ValueError("The desired value for the start of the fov is outside the allowed range")

            elif setting_id == 'fov_end':
                if VelodyneConfiguration.check_fov_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + '/fov',
                                        urlencode({'end': str(value)}))
                else:
                    raise ValueError("The desired value for the end of the fov is outside the allowed range")

            elif setting_id == 'returns':
                if VelodyneConfiguration.check_return_mode_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL),
                                        urlencode({'returns': VelodyneConfiguration.return_modes[value]}))
                else:
                    raise ValueError("The desired return mode is not permitted")

            elif setting_id == 'phaselock':
                if VelodyneConfiguration.check_phase_lock_soll_val(value):
                    cmd = {"enabled": "on", "offset": str(value * 100), "offsetInput": str(value)}
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + 'phaselock',
                                        urlencode(cmd))
                else:
                    raise ValueError("The desired phase lock value %s is invalid", str(value))

            elif setting_id == 'phaselock_off':
                cmd = {"enabled": "off", "offset": str(0 * 100), "offsetInput": str(0)}
                rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + 'phaselock',
                                    urlencode(cmd))

            elif setting_id == 'laser':
                if VelodyneConfiguration.check_on_off_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL),
                                        urlencode({'laser': value}))
                else:
                    raise ValueError("Wrong option for laser on/off")

            else:
                # Network configs

                cmd = setting_id.split("_")

                if cmd[0] == "host":
                    url = self.config.get_command_url("set_setting", self.Base_URL) + "/host"
                    rc = self.sensor_do(url, urlencode({cmd[1]: value}))
                elif cmd[0] == "net":
                    url = self.config.get_command_url("set_setting", self.Base_URL) + "/net"
                    rc = self.sensor_do(url, urlencode({cmd[1]: value}))
                    self.safe_current_config_to_sensor()
                    self.reset_sensor()

                else:
                    raise NameError("The passed setting is not recognised", setting_id)

            if not rc:
                raise Exception('Fail to set setting in the sensor')

        else:
            raise NameError("Parameter name not found. Name:%s ", setting_id)

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

    def get_diagnostics(self):

        diagnostics = self._request_json('get_diagnostic')
        interpret_diagnostic_data(name,raw_data)
        pass

    def get_status(self):
        'get_status'
        status = self._request_json('get_status')
        pass

    def get_configuration(self):

        configuration = self._request_json('get_configuration')
        pass





    def reset_sensor(self):
        rc = self.sensor_do(self.config.get_command_url("reset", self.Base_URL),
                            urlencode({'data': self.config.reset}))
        if not rc:
            raise Exception('Fail to restet the sensor')
        else:
            time.sleep(10)

    def safe_current_config_to_sensor(self):
        rc = self.sensor_do(self.config.get_command_url("save_cfg", self.Base_URL),
                            urlencode({'data': self.config.submit}))
        if not rc:
            raise Exception('Fail to save current confing in the sensor')
        else:
            time.sleep(10)

    def __del__(self):
        self.sensor.close()
