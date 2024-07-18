import os

import pycurl
from io import BytesIO
from urllib.parse import urlencode
import urllib.request
import json
import time
from urllib.error import HTTPError, URLError
import socket
import math
import tempfile


# TODO fill initial config with snapshot

class VelodyneConfiguration:

    def __init__(self):
        # Configuration of the sensor
        self.conf = {
            # These are in the status msg.
            "gps_pps_state": "Absent",
            "gps_position": "",
            "tod_sec": 0,
            "tod_nsec": 0,
            "usectoh": 411093848,
            "motor_state": True,
            "rpm": 600,
            "phaselock": False,
            "phase": 0,  # in *100
            "ns_per_rev": 100040448,
            "laser": True,
            # These have to be read from other source
            "returns": "Strongest",
            "fov_start": 0,
            "fov_end": 359,
            "host_addr": "255.255.255.255",
            "host_dport": 2368,
            "host_tport": 8309,
            "net_addr": "192.168.1.200",
            "net_mask": "255.255.255.0",
            "net_gateway": "192.168.1.2",
            "net_dhcp": "on",
            "net_mac_addr": ""
        }

    # list of configurable parameter, probably there are more e.g. laser on/off Gps pps lock delay
    conf_ids = ['rpm',
                "fov_start",
                "fov_end",
                "returns",
                "phaselock",
                "phase",
                "laser",
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
        'get_snapshot': 'snapshot.hdl',
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
        {
            'top': {
                'hv': (lambda raw_val: 101.0 * (VelodyneConfiguration.std_voltage_conversion(raw_val) - 5.0)),
                'lm20_temp': (lambda raw_val: VelodyneConfiguration.std_temperature_conversion(raw_val)),
                'pwr_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val) * 2.0),
                'pwr_2_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_3_3v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_raw': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_vccint': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val))},
            'bot': {
                'i_out': (lambda raw_val: VelodyneConfiguration.std_current_conversion(raw_val)),
                'lm20_temp': (lambda raw_val: VelodyneConfiguration.std_temperature_conversion(raw_val)),
                'pwr_1_2v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_1_25v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_1_2_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_3_3v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val)),
                'pwr_5v': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val) * 2.0),
                'pwr_v_in': (lambda raw_val: VelodyneConfiguration.std_voltage_conversion(raw_val) * 11.0)}
        }
    # Operational ranges according to manual
    __operational_ranges = \
        {'top': {
            'hv': [-150.0, -132.0],
            'lm20_temp': [25.0, 90.0],
            'pwr_5v': [4.8, 5.2],
            'pwr_2_5v': [2.3, 2.7],
            'pwr_3_3v': [3.1, 3.5],
            'pwr_raw': [-math.inf, math.inf],
            'pwr_vccint': [1.0, 1.4]
        },
            'bot': {
                'i_out': [0.3, 1.0],
                'lm20_temp': [-25.0, 90.0],
                'pwr_1_2v': [1.0, 1.4],
                'pwr_1_25v': [1.0, 1.4],
                'pwr_1_2_5v': [2.3, 2.7],
                'pwr_3_3v': [3.1, 3.5],
                'pwr_5v': [4.8, 5.2],
                'pwr_v_in': [8.0, 19.0]}
        }

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
    def interpret_diagnostic_data(board, name, raw_val):
        if board not in VelodyneConfiguration.__diagnostic_data_interpreter:
            raise NameError("board position mus be top or bot not %s " % board)
        if name in VelodyneConfiguration.__diagnostic_data_interpreter[board]:
            op = VelodyneConfiguration.__diagnostic_data_interpreter[board][name]
            return op(raw_val)
        else:
            raise NameError("Diagnostic data %s name not accepted" % name)

    @staticmethod
    def check_diagnostic_data_in_range(board, name, interpreted_val):
        if board not in VelodyneConfiguration.__operational_ranges:
            raise NameError("board position mus be top or bot not %s " % board)
        if name in VelodyneConfiguration.__diagnostic_data_interpreter[board]:
            val_range = VelodyneConfiguration.__operational_ranges[board][name]
            return val_range[0] < interpreted_val < val_range[1]
        else:
            raise NameError("Diagnostic data %s name not accepted" % name)

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
        return 0 <= soll_val <= 36000

    @staticmethod
    def check_on_off_soll_val(soll_val):
        return soll_val == 'on' or soll_val == 'off'


class Configurator:
    def __init__(self, velodyne_ip):
        self.config = VelodyneConfiguration()
        self.Base_URL = 'http://' + velodyne_ip + '/cgi/'
        self.sensor = pycurl.Curl()
        self.buffer = BytesIO()
        print("Testing connection to sensor")
        self.test_connection()
        print("Connection OK")

    def test_connection(self):
        # Test connection to sensor
        try:
            cfg = self.get_current_configuration()

        except RuntimeError as e:
            print(e)
            raise RuntimeError from e

        print('Sensor laser is %b, motor rpm is %i' % (cfg.conf['laser'], cfg.conf['rpm']))

    def _update_conf_from_snapshot(self):
        # Download a temporal snapshot to get the current state of the sensor
        url = VelodyneConfiguration.get_command_url("get_snapshot", self.Base_URL)
        with tempfile.NamedTemporaryFile() as temp_file:
            temp_file_path = temp_file.name
            urllib.request.urlretrieve(url, temp_file_path)
            with open(temp_file_path, 'r') as file:
                son = json.load(file)

                self.config.conf["returns"] = son["config"]["returns"]
                self.config.conf["fov_start"] = son["config"]["fov"]["start"]
                self.config.conf["fov_end"] = son["config"]["fov"]["end"]
                self.config.conf["host_addr"] = son["config"]["host"]["addr"]
                self.config.conf["host_dport"] = son["config"]["host"]["dport"]
                self.config.conf["host_tport"] = son["config"]["host"]["tport"]
                self.config.conf["net_addr"] = son["config"]["net"]["addr"]
                self.config.conf["net_mask"] = son["config"]["net"]["mask"]
                self.config.conf["net_gateway"] = son["config"]["net"]["gateway"]
                self.config.conf["net_dhcp"] = son["config"]["net"]["dhcp"]
                self.config.conf["net_mac_addr"] = son["config"]["net"]["mac_addr"]

    def _request_json(self, cmd):
        try:
            url = VelodyneConfiguration.get_command_url(cmd, self.Base_URL)
            response = urllib.request.urlopen(url, timeout=5)
            time.sleep(1.2)
        except HTTPError as error:
            print('HTTP Error: Data not retrieved because %s\nURL: %s' % (error, url))
            raise Exception('Fail to communicate with sensor') from error
        except URLError as error:
            if isinstance(error.reason, socket.timeout):
                print('socket timed out - URL %s' % url)
                raise Exception('Fail to communicate with sensor') from error
            else:
                print('some other error happened: %s' % error)
                raise Exception('Fail to communicate with sensor') from error

        if response:
            js = json.loads(response.read())
            return js

        else:
            print('Error Response from sensor is empty')
            raise Exception('Fail to communicate with sensor')

    def bool_to_on_off(self, b_val):
        if b_val:
            return 'on'
        else:
            return 'off'

    def set_setting(self, setting_id, value):
        if setting_id in VelodyneConfiguration.conf_ids:
            rc = None
            if setting_id == 'rpm':
                if VelodyneConfiguration.check_rpm_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL),
                                        urlencode({'rpm': str(value)}))
                    if rc:
                        self.config.conf[setting_id] = value
                else:
                    raise ValueError(
                        "The desired value for the rpm is outside the allowed range and it must be a multiple of 60")

            elif setting_id == 'fov_start':
                if VelodyneConfiguration.check_fov_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + '/fov',
                                        urlencode({'start': str(value)}))
                    if rc:
                        self.config.conf[setting_id] = value
                else:
                    raise ValueError("The desired value for the start of the fov is outside the allowed range")

            elif setting_id == 'fov_end':
                if VelodyneConfiguration.check_fov_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + '/fov',
                                        urlencode({'end': str(value)}))
                    if rc:
                        self.config.conf[setting_id] = value
                else:
                    raise ValueError("The desired value for the end of the fov is outside the allowed range")

            elif setting_id == 'returns':
                if VelodyneConfiguration.check_return_mode_soll_val(value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL),
                                        urlencode({'returns': VelodyneConfiguration.return_modes[value]}))
                    if rc:
                        self.config.conf[setting_id] = VelodyneConfiguration.return_modes[value]
                else:
                    raise ValueError("The desired return mode is not permitted")

            elif setting_id == 'phase':
                current_phaselock = self.bool_to_on_off(self.config.conf["phaselock"])
                if VelodyneConfiguration.check_phase_lock_soll_val(value):
                    cmd = {"enabled": current_phaselock, "offset": str(value), "offsetInput": str(value / 100.0)}
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + 'phaselock',
                                        urlencode(cmd))
                    if rc:
                        self.config.conf[setting_id] = value

                else:
                    raise ValueError("The desired phase lock value %s is invalid" % str(value))

            elif setting_id == 'phaselock':
                current_phase_value = self.config.conf["phase"]
                str_value = self.bool_to_on_off(value)
                if VelodyneConfiguration.check_on_off_soll_val(str_value):
                    cmd = {"enabled": str_value, "offset": str(current_phase_value),
                           "offsetInput": str(current_phase_value / 100)}
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL) + 'phaselock',
                                        urlencode(cmd))
                    if rc:
                        self.config.conf[setting_id] = value
                else:
                    raise ValueError("Wrong option for phaselock, correct options are on/off")

            elif setting_id == 'laser':
                str_value = self.bool_to_on_off(value)
                if VelodyneConfiguration.check_on_off_soll_val(str_value):
                    rc = self.sensor_do(self.config.get_command_url("set_setting", self.Base_URL),
                                        urlencode({'laser': str_value}))
                    if rc:
                        self.config.conf[setting_id] = value
                else:
                    raise ValueError("Wrong option for laser on/off")

            else:
                # Network configs

                cmd = setting_id.split("_")

                if cmd[0] == "host":
                    url = self.config.get_command_url("set_setting", self.Base_URL) + "/host"
                    rc = self.sensor_do(url, urlencode({cmd[1]: value}))
                    if rc:
                        self.config.conf[setting_id] = value
                elif cmd[0] == "net":
                    url = self.config.get_command_url("set_setting", self.Base_URL) + "/net"
                    rc = self.sensor_do(url, urlencode({cmd[1]: value}))
                    if rc:
                        self.config.conf[setting_id] = value
                        self.safe_current_config_to_sensor()
                        self.reset_sensor()

                else:
                    raise NameError("The passed setting is not recognised Name:%s " % setting_id)

            if not rc:
                raise Exception('Fail to set setting in the sensor Name:%s ' % setting_id)

        else:
            raise NameError("Parameter name not found. Name:%s " % setting_id)

    def get_setting(self, setting_id):
        try:
            value = self.config.conf[setting_id]
        except KeyError:
            print(" %s is not a valid setting" % setting_id)
            return None
        return value

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
        interpreted_diagnostics = {}
        try:
            diagnostics = self._request_json('get_diagnostic')
        except Exception as e:
            raise RuntimeError from e

        volt_temp = diagnostics['volt_temp']
        interpreted_diagnostics['volt_temp'] = {}
        for t_b in volt_temp:
            interpreted_diagnostics[t_b] = {}
            for name in volt_temp[t_b]:
                value = volt_temp[t_b][name]
                interp_value = value
                try:
                    interp_value = VelodyneConfiguration.interpret_diagnostic_data(t_b, name, value)
                except NameError as e:
                    print("Value of %s %s can not be interpreted, leaving it as it is" % (t_b, name))

                interpreted_diagnostics[t_b][name] = interp_value

        return interpreted_diagnostics

    def check_diagnostics_parameter(self, t_b, diag_par, value):
        try:
            in_range = VelodyneConfiguration.check_diagnostic_data_in_range(t_b, diag_par, value)
        except NameError:
            print("Value of %s %s can not be checked, no defined ranges" % (t_b, diag_par))
            raise NameError("Parameters ranges not defined for check")
        return in_range

    def get_current_configuration(self):
        try:
            print("Get status")
            status = self._request_json('get_status')
            print("Status received")
        except Exception as e:
            print(e)
            raise RuntimeError from e

        # Only this information is in the status response

        self.config.conf["gps_pps_state"] = status["gps"]["pps_state"]
        self.config.conf["gps_position"] = status["gps"]["position"]
        self.config.conf["tod_sec"] = status["tod"]["sec"]
        self.config.conf["tod_nsec"] = status["tod"]["nsec"]
        self.config.conf["usectoh"] = status["usectoh"]
        self.config.conf["motor_state"] = True if status["motor"]["state"] == "On" else False
        self.config.conf["rpm"] = status["motor"]["rpm"]
        self.config.conf["phaselock"] = False if status["motor"]["lock"] == "Off" else True
        self.config.conf["phase"] = status["motor"]["phase"]
        self.config.conf["ns_per_rev"] = status["motor"]["ns_per_rev"]
        self.config.conf["laser"] = True if status["laser"]["state"] == "On" else False

        print("Get Snapshot")
        self._update_conf_from_snapshot()
        print("Snapshot received")
        return self.config

    def download_snapshot(self, folder_path, file_name=None):

        url = VelodyneConfiguration.get_command_url("get_snapshot", self.Base_URL)
        # Ensure the folder exists
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # Get the file name from the URL if not provided
        if not file_name:
            file_name = os.path.basename(url)

        # Full path to save the file
        file_path = os.path.join(folder_path, file_name)

        try:
            urllib.request.urlretrieve(url, file_path)
            print(f'File downloaded and saved to {file_path}')
        except Exception as e:
            print(f'Failed to download the file. Error: {e}')
            raise RuntimeError from e

    def reset_sensor(self):
        rc = self.sensor_do(self.config.get_command_url("reset", self.Base_URL),
                            urlencode({'data': self.config.reset}))
        if not rc:
            raise RuntimeError('Fail to reset the sensor')
        else:
            time.sleep(10)

    def save_current_config_to_sensor(self):
        rc = self.sensor_do(self.config.get_command_url("save_cfg", self.Base_URL),
                            urlencode({'data': self.config.submit}))
        if not rc:
            raise RuntimeError('Fail to save current confing in the sensor')
        else:
            time.sleep(10)

    def __del__(self):
        self.sensor.close()
