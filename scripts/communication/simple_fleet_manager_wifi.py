#!/usr/bin/env python3
import rospy, time, netifaces, signal, math, socket
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int64, Float32
from dhi_amr.msg import simple_in
from dhi_amr.msg import task_data
from dhi_amr.msg import simple_confirmations, encoder_data, ethernet_devices_status
from dhi_amr.msg import flexi_data_out, log_messages, plc_error_codes, plc_error_status
from dhi_amr.msg import scangrid, sensors_data, task_data, teb_config
from dhi_amr.msg import visionary_data_out, workstate_read


""" Server setting class """
class Server:
    """ Server initialization """
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(5)

    """ Accepting client method """
    def accept_connection(self):
        client_socket = None
        try:
            client_socket, addr = self.server_socket.accept()
            rospy.loginfo(f'AMR SERVER: Connection accepted from -  {addr[0]}:{addr[1]}')
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while accepting connection from client: {e}')
        return client_socket

    """ Server closing method """
    def close(self):
        if self.server_socket:
            self.server_socket.close()


""" Data exchange handling class """
class ConnectionHandler:

    """ Initialization  """
    def __init__(self):
        """ Variables """
        """ options """
        self.server_startup = True
        self.interface_name = 'wlp0s20f3'
        # self.interface_name = 'eno1'
        self.server_address = self.get_my_ip(self.interface_name)
        self.server_port = 8000
        self.server_refresh_rate = 30
        self.data_in_queue = 1
        self.data_in_latch = True
        """ Connection """
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server = Server(self.server_address, self.server_port)

        """ Others """
        self.rate = rospy.Rate(self.server_refresh_rate)
        self.log_action_time = True
        # signal.signal(signal.SIGINT, self.signal_handler)
        # signal.signal(signal.SIGTSTP, self.signal_handler)

        """ Data """
        self.actual_pose = Pose2D()
        self.sensors = sensors_data()
        self.pwm = 0
        self.encoder = encoder_data()
        self.safety = flexi_data_out()
        self.scangrid_left = scangrid()
        self.scangrid_right = scangrid()
        self.plc_error_codes = plc_error_codes()
        self.plc_error_status = plc_error_status()
        self.ethernet = ethernet_devices_status()
        self.config_teb_actual = teb_config()
        self.task_actual = task_data()
        self.data_in = simple_in()
        self.amr_confirmations = simple_confirmations()
        self.workstates_actual = workstate_read()
        self.log = log_messages()

        """ Main connection handler loop """
        while not rospy.is_shutdown():
            try:
                """ Get client if server is running """
                if self.server_startup:
                    rospy.logdebug('AMR SERVER: Trying to accept client connection...')
                    self.client = self.server.accept_connection()
                    if self.client is not None:
                        self.server_startup = False
                """ ROS Subscriber's definitions """
                actual_pose_sub = rospy.Subscriber('amr/odom_pub/sickLidarPose2D', Pose2D, self.actual_pose_callback)
                sensors_sub = rospy.Subscriber('amr/plc/sensors_converted', sensors_data, self.sensors_callback)
                encoder_sub = rospy.Subscriber('amr/encoder', encoder_data, self.encoder_callback)
                safety_sub = rospy.Subscriber('amr/safety', flexi_data_out, self.safety_callback)
                scangrid_left_sub = rospy.Subscriber('amr/scangrids/scangrid_left', scangrid, self.scangrid_left_callback)
                scangrid_right_sub = rospy.Subscriber('amr/scangrids/scangrid_right', scangrid, self.scangrid_right_callback)
                plc_error_status_sub = rospy.Subscriber('amr/plc/errors/statuses', plc_error_status, self.plc_error_status_callback)
                plc_error_codes_sub = rospy.Subscriber('amr/plc/errors/codes', plc_error_codes, self.plc_error_codes_callback)
                ethernet_sub = rospy.Subscriber('amr/tests/ethernet_devices', ethernet_devices_status, self.ethernet_callback)
                config_teb_actual_sub = rospy.Subscriber('amr/config/actual/teb', teb_config, self.config_teb_actual_callback)
                task_actual_sub = rospy.Subscriber('amr/task/actual', task_data, self.task_actual_callback)
                amr_confirmations_sub = rospy.Subscriber('simple_fleet_manager/data_out/confirmations',simple_confirmations, self.amr_confirmations_callback)
                log_sub = rospy.Subscriber('amr/log/current', log_messages, self.log_callback)
                workstates_actual_sub = rospy.Subscriber('amr/state_machine', workstate_read, self.workstates_actual_callback)

                """ ROS Publisher's definitions """
                self.data_in_pub = rospy.Publisher('simple_fleet_manager/data_in', simple_in, queue_size=self.data_in_queue, latch=self.data_in_latch)
            except Exception as e:
                rospy.logfatal(f'AMR SERVER: Error detected in init: {e}')
            self.communication_sequence()

    """ Callback for actual workstates subscriber """
    def workstates_actual_callback(self, msg):
        self.workstates_actual = msg

    """ callback for log messages subscriber """
    def log_callback(self, msg):
        self.log = msg

    """ Callback for AMR confirmations to SimpleFleetManager subscriber """
    def amr_confirmations_callback(self, msg):
        self.amr_confirmations = msg

    """ Callback for actual task subscriber """
    def task_actual_callback(self, msg):
        self.task_actual = msg

    """ Callback for actual teb config subscriber """
    def config_teb_actual_callback(self, msg):
        self.config_teb_actual = msg

    """ Callback for actual position subscriber """
    def actual_pose_callback(self, msg):
        self.actual_pose = msg

    """ Callback for sensors data converted from PLC subscriber """
    def sensors_callback(self, msg):
        self.sensors = msg

    """ Callback for encoder data subscriber """
    def encoder_callback(self, msg):
        self.encoder = msg

    """ Callback for safety statuses subscriber """
    def safety_callback(self, msg):
        self.safety = msg

    """ Callback for left scangrid data subscriber """
    def scangrid_left_callback(self, msg):
        self.scangrid_left = msg

    """ Callback for right scangrid data subscriber """
    def scangrid_right_callback(self, msg):
        self.scangrid_right = msg

    """ Callback for PLC error status subscriber """
    def plc_error_status_callback(self, msg):
        self.plc_error_status = msg

    """ Callback for PLC error status subscriber """
    def plc_error_codes_callback(self, msg):
        self.plc_codes_status = msg 

    """ Callback for ethernet test statuses """
    def ethernet_callback(self, msg):
        self.ethernet = msg

    """ Get ip from current interface - automatic config for server """
    def get_my_ip(self, interface_name):
        try:
            interfaces = netifaces.interfaces()
            if interface_name in interfaces:
                addresses = netifaces.ifaddresses(interface_name)
                if netifaces.AF_INET in addresses:
                    return addresses[netifaces.AF_INET][0]['addr']
                else:
                    return "No ip address for selected interface"
            else:
                return "Selected interface does not exists"
        except Exception as e:
            return f"Error detected in get_my_ip method: {e}"

    """ Renew connection to client after lost connection method """
    def reconnect_client(self):
        rospy.logwarn(f'AMR SERVER: Lost connection to client: {self.client}. Trying to reconnect')
        self.client.close()
        self.client = self.server.accept_connection()

    """ Creating pose data table method """
    def create_pose_data(self, data):
        try:
            data.append(round(self.actual_pose.x, 3))
            data.append(round(self.actual_pose.y, 3))
            data.append(round((self.actual_pose.theta * (180 / math.pi)), 3))
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating pose data: {e}')
            for i in range(0, 3):
                data.append(0)
        return data

    """ Creating PLC error status data table method """
    def create_plc_error_status_data(self, data):
        try:
            data.append(self.plc_error_status.battery_sensor)
            data.append(self.plc_error_status.forks_height_sensor)
            data.append(self.plc_error_status.manual_speed_regulator)
            data.append(self.plc_error_status.pressure_sensor)
            data.append(self.plc_error_status.pwm_to_curtis_write)
            data.append(self.plc_error_status.scangrid_left)
            data.append(self.plc_error_status.scangrid_right)
            data.append(self.plc_error_status.servo_halt)
            data.append(self.plc_error_status.servo_move)
            data.append(self.plc_error_status.servo_position_read)
            data.append(self.plc_error_status.tilt_sensor_axis_1)
            data.append(self.plc_error_status.tilt_sensor_axis_2)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating plc error status table: {e}')
            for i in range(12):
                data.append(0)
        return data

    """ Creating PLC error codes data table method """
    def create_plc_error_codes_data(self, data):
        try:
            data.append(self.plc_error_codes.battery_sensor)
            data.append(self.plc_error_codes.forks_height_sensor)
            data.append(self.plc_error_codes.manual_speed_regulator)
            data.append(self.plc_error_codes.pressure_sensor)
            data.append(self.plc_error_codes.pwm_to_curtis_write)
            data.append(self.plc_error_codes.scangrid_left)
            data.append(self.plc_error_codes.scangrid_right)
            data.append(self.plc_error_codes.servo_halt)
            data.append(self.plc_error_codes.servo_move)
            data.append(self.plc_error_codes.servo_position_read)
            data.append(self.plc_error_codes.tilt_sensor_axis_1)
            data.append(self.plc_error_codes.tilt_sensor_axis_2)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating plc error status table: {e}')
            for i in range(12):
                data.append(0)
        return data

    """ Creating sensors data table method """
    def create_sensors_data(self, data):
        try:
            data.append(round(self.sensors.battery_voltage, 2))
            data.append(round(self.sensors.battery_percentage, 2))
            data.append(self.sensors.battery_critical)
            data.append(self.sensors.forks_height)
            data.append(self.sensors.forks_height_limiter)
            data.append(self.sensors.weight)
            data.append(self.sensors.weight_saved)
            data.append(self.sensors.tilt_axis_1)
            data.append(self.sensors.tilt_axis_2)
            data.append(round(self.sensors.servo_angle, 2))
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating sensors data: {e}')
            for i in range(10):
                data.append(0)
        return data

    """ Creating encoder data table method """
    def create_encoder_data(self, data):
        try:
            data.append(self.encoder.speed)
            data.append(self.encoder.direction_forward)
            data.append(self.encoder.direction_backward)
            data.append(self.encoder.standstill)
            data.append(self.encoder.current_position)
            data.append(self.encoder.distance_mm)
            data.append(self.encoder.distance_m)
            data.append(self.encoder.distance_km)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating encoder data: {e}')
            for i in range(8):
                data.append(0)
        return data

    """ Creating safety data table method """
    def create_safety_data(self, data):
        try:
            data.append(self.safety.cpu_ok)
            data.append(self.safety.encoder_ok)
            data.append(self.safety.speed_ok)
            data.append(self.safety.standstill)
            data.append(self.safety.left_emergency_stop_button)
            data.append(self.safety.right_emergency_stop_button)
            data.append(self.safety.left_scanner.is_active)
            data.append(self.safety.left_scanner.device_error)
            data.append(self.safety.left_scanner.app_error)
            data.append(self.safety.left_scanner.contamination_error)
            data.append(self.safety.left_scanner.contamination_warning)
            data.append(self.safety.left_scanner.monitoring_case_valid)
            data.append(self.safety.left_scanner.emergency_stop_zone_status)
            data.append(self.safety.left_scanner.soft_stop_zone_status)
            data.append(self.safety.left_scanner.reduced_speed_zone_status)
            data.append(self.safety.right_scanner.is_active)
            data.append(self.safety.right_scanner.device_error)
            data.append(self.safety.right_scanner.app_error)
            data.append(self.safety.right_scanner.contamination_error)
            data.append(self.safety.right_scanner.contamination_warning)
            data.append(self.safety.right_scanner.monitoring_case_valid)
            data.append(self.safety.right_scanner.emergency_stop_zone_status)
            data.append(self.safety.right_scanner.soft_stop_zone_status)
            data.append(self.safety.right_scanner.reduced_speed_zone_status)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating safety data table: {e}')
            for i in range(24):
                data.append(0)
        return data

    """ Creating scangrid status data table method """
    def create_scangrid_status(self, data):
        try:
            data.append(self.scangrid_left.status.work_status)
            data.append(self.scangrid_left.status.voltage_error)
            data.append(self.scangrid_left.status.resistance_to_external_light_error)
            data.append(self.scangrid_left.status.contamination_error)
            data.append(self.scangrid_left.status.contamination_warning)
            data.append(self.scangrid_left.status.sleep_mode_status)
            data.append(self.scangrid_left.status.monitoring_case_switch_input_status)
            data.append(self.scangrid_left.status.monitoring_case_switch_can_input_status)
            data.append(self.scangrid_left.status.safety_output)
            data.append(self.scangrid_left.status.protection_field_status)
            data.append(self.scangrid_left.status.warning_field_status)
            data.append(self.scangrid_right.status.work_status)
            data.append(self.scangrid_right.status.voltage_error)
            data.append(self.scangrid_right.status.resistance_to_external_light_error)
            data.append(self.scangrid_right.status.contamination_error)
            data.append(self.scangrid_right.status.contamination_warning)
            data.append(self.scangrid_right.status.sleep_mode_status)
            data.append(self.scangrid_right.status.monitoring_case_switch_input_status)
            data.append(self.scangrid_right.status.monitoring_case_switch_can_input_status)
            data.append(self.scangrid_right.status.safety_output)
            data.append(self.scangrid_right.status.protection_field_status)
            data.append(self.scangrid_right.status.warning_field_status)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating scangrid statuses table: {e}')
            for i in range(22):
                data.append(0)
        return data

    """ Creating scangrid measuring data table method """
    def create_scangrid_measuring_data(self, data):
        try:
            for i in range(32):
                data.append(getattr(self.scangrid_left.data, f'range_{i}'))
            for i in range(32):
                data.append(getattr(self.scangrid_right.data, f'range_{i}'))
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating scangrids measuring data table: {e}')
            for i in range(64):
                data.append(0)
        return data

    """ Creating actual workstates data table method """
    def create_actual_workstates_data(self, data):
        try:
            data.append(self.workstates_actual.S0_1)
            data.append(self.workstates_actual.S0_2)
            data.append(self.workstates_actual.S0_3)
            data.append(self.workstates_actual.S1)
            data.append(self.workstates_actual.S2)
            data.append(self.workstates_actual.S3)
            data.append(self.workstates_actual.S4)
            data.append(self.workstates_actual.S4_0)
            data.append(self.workstates_actual.S4_1)
            data.append(self.workstates_actual.S4_2)
            data.append(self.workstates_actual.S4_3)
            data.append(self.workstates_actual.S4_4)
            data.append(self.workstates_actual.S4_5)
            data.append(self.workstates_actual.S4_6)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creaating actual workstates data table: {e}')
            for i in range(14):
                data.append(0)
        return data

    """ Creating ethernet status data table method """
    def create_ethernet_status_data(self, data):
        try:
            data.append(self.ethernet.lidarloc_is_avaible)
            data.append(self.ethernet.plc_is_avaible)
            data.append(self.ethernet.visionary_is_avaible)
            data.append(self.ethernet.safety_modbus_is_avaible)
            data.append(self.ethernet.lan_gateway_is_avaible)
            data.append(self.ethernet.microscan3_left_is_avaible)
            data.append(self.ethernet.microscan3_right_is_avaible)
            data.append(self.ethernet.safety_ethernet_is_avaible)
            data.append(self.ethernet.wifi_gateway_is_avaible)
            data.append(self.ethernet.server_is_avaible)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating ethernet test status data table: {e}')
            for i in range(10):
                data.append(0)
        return data

    """ Creating actual teb config data table method """
    def create_actual_teb_config_data(self, data):
        try:
            data.append(self.config_teb_actual.max_vel_forward)
            data.append(self.config_teb_actual.max_vel_backward)
            data.append(self.config_teb_actual.max_vel_theta)
            data.append(self.config_teb_actual.acc_lim_x)
            data.append(self.config_teb_actual.acc_lim_theta)
            data.append(self.config_teb_actual.turning_radius)
            data.append(self.config_teb_actual.wheelbase)
            data.append(self.config_teb_actual.goal_tolerance_xy)
            data.append(self.config_teb_actual.goal_tolerance_yaw)
            data.append(self.config_teb_actual.min_obstacle_distance)
            data.append(self.config_teb_actual.obstacle_inflation_radius)
            data.append(self.config_teb_actual.dynamic_obstacle_inflation_radius)
            data.append(self.config_teb_actual.dt_ref)
            data.append(self.config_teb_actual.dt_hysteresis)
            data.append(self.config_teb_actual.include_dynamic_obstacles)
            data.append(self.config_teb_actual.include_costmap_obstacles)
            data.append(self.config_teb_actual.oscillation_recovery)
            data.append(self.config_teb_actual.allow_init_with_backward_motion)
            data.append(self.config_teb_actual.save_settings)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating actual teb config data table: {e}')
            for i in range(19):
                data.append(0)
        return data

    """ Creating actual task data table method """
    def create_actual_task_data(self, data):
        try:
            data.append(self.task_actual.id)
            data.append(self.task_actual.type)
            data.append(self.task_actual.coord.point_x)
            data.append(self.task_actual.coord.point_y)
            data.append(self.task_actual.coord.point_r)
            data.append(self.task_actual.is_running)
            data.append(self.task_actual.done)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating actual task data table: {e}')
            for i in range(7):
                data.append(0)
        return data
    
    """ Creating log data table method"""
    def create_log_data(self, data):
        try:
            data.append(self.log.date)
            data.append(self.log.level)
            data.append(self.log.node_name)
            data.append(self.log.message)
            data.append(self.log.code_line)
            data.append(self.log.file)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating log data table: {e}')
            for i in range(6):
                data.append(0)
        return data

    """ Creating confirmations data table method """
    def create_confirmations_data(self, data):
        try:
            data.append(self.amr_confirmations.config_saved)
            data.append(self.amr_confirmations.cancelled)
            data.append(self.amr_confirmations.started)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while creating cofirmations data table: {e}')
            for i in range(3):
                data.append(0)
        return data

    """ Validating, converting and ending data set method """
    def validate_and_convert_data(self, data):
        try:
            data = [str(item) if item is not None else '0' for item in data]
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while validating, converting and ending one data set: {e}')
        return data

    """ Divide every item with special character method """
    def divide_data_to_send(self, data):
        try:
            divided_message = ''
            for item in data:
                divided_message = divided_message + item + '#'
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while dividing data to send with special character: {e}')
        divided_message = divided_message + '!'
        return divided_message

    """ Creating data to SimpleFleetManager """
    def get_data_to_send(self, data):
        try:
            data = self.create_pose_data(data)
            data.append('&')
            data = self.create_sensors_data(data)
            data.append('&')
            data = self.create_encoder_data(data)
            data.append('&')
            data = self.create_plc_error_status_data(data)
            data.append('&')
            data = self.create_plc_error_codes_data(data)
            data.append('&')
            data = self.create_safety_data(data)
            data.append('&')
            data = self.create_scangrid_status(data)
            data.append('&')
            data = self.create_scangrid_measuring_data(data)
            data.append('&')
            data = self.create_actual_workstates_data(data)
            data.append('&')
            data = self.create_ethernet_status_data(data)
            data.append('&')
            data = self.create_actual_teb_config_data(data)
            data.append('&')
            data = self.create_actual_task_data(data)
            data.append('&')
            data = self.create_log_data(data)

        except Exception as e:
            rospy.logfatal(f'AMR SERVER: Error in get data to send method: {e}. Creating empty table')
        return data

    """ Sending message trought TCP/IP protocol method """
    def send_data(self, message):
        try:
            byte_to_send = bytes(message + '$', 'utf-8')
            self.client.send(byte_to_send)
            rospy.logdebug(byte_to_send)
        except Exception as e:
            rospy.logwarn(f'AMR SERVER: Send bytes error - {e}. Starting reconnect method!')
            self.reconnect_client()

    """ Reading data from client method """
    def read_data(self):
        try:
            data = self.client.recv(65535)
            return data
        except Exception as e:
            rospy.logwarn(f'AMR SERVER: Read data from client error - {e}. Starting reconnect method.')
            self.reconnect_client()
            return None

    """ Decoding received message and splitting in topics """
    def decode_and_split_data(self, message):
        try:
            data = message.decode('utf-8')
            data_splitted = data.split('&')
            if len(data_splitted) > 0:
                commands = data_splitted[0]
            else:
                commands = None
            if len(data_splitted) > 1:
                task = data_splitted[1]
            else:
                task = None
            return commands, task
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while decoding and splitting received message: {e}')
            return None, None

    """ Assigning commands data method"""
    def set_commands(self, data):
        try:
            if len(data) == 6:
                bool_data = []
                for i in range(len(data)):
                    bool_data.append(bool(data))
                self.data_in.commands.manual_mode_override = bool_data[0]
                self.data_in.commands.start_task = bool_data[1]
                self.data_in.commands.pause_task = bool_data[2]
                self.data_in.commands.continue_task = bool_data[3]
                self.data_in.commands.cancel_task = bool_data[4]
                self.data_in.commands.emergency_stop = bool_data[5]
            else:
                rospy.logerr(f'AMR SERVER: Commands data to short: {len(data)}')
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while setting commands data: {e}')

    """ Assign task data method """
    def set_task(self, data):
        try:
            if len(data) == 5:
                data_int = []
                data_float = []
                data_int_len = 2
                data_float_len = 3
                for i in range (data_int_len):
                    data_int = data[i]
                for i in range (data_float_len):
                    data_float = data[i + data_int_len]
                self.data_in.task.id = data_int[0]
                self.data_in.task.type = data_int[1]
                self.data_in.task.coord.point_x = data_float[0]
                self.data_in.task.coord.point_y = data_float[1]
                self.data_in.task.coord.point_r = data_float[2]
            else:
                rospy.logerr(f'AMR SERVER: Task data to short: {len(data)}')
        except Exception as e:
            rospy.logerr(f'AMR SERVER: Error while setting task data: {e}')
    """ Publish received data to ROS topic """

    def publish_message(self):
        try:
            self.data_in_pub.publish(self.data_in)
        except Exception as e:
            rospy.logerr(f'AMR SERVER: error while publishing readed messages to ROS topic: {e}')

    """ Main communication with SimpleFleetManager sequence"""
    def communication_sequence(self):
        action_start_time = time.time()
        data_to_send = []
        message_to_send = None
        data_to_send = self.get_data_to_send(data_to_send)
        data_to_send = self.validate_and_convert_data(data_to_send)
        message_to_send = self.divide_data_to_send(data_to_send)
        self.send_data(message_to_send)
        if self.log_action_time:
            action_duration = (time.time() - action_start_time) * 1000
            rospy.logdebug(f'AMR SERVER: Send data action time: {action_duration} ms.')
        # action_start_time = time.time()
        # message_received = self.read_data()
        # if message_received is not None:
        #     commands, task = self.decode_and_split_data(message_received)
        #     if commands is not None:
        #         self.set_commands(commands)
        #     if task is not None:
        #         self.set_task(task)
        #     self.publish_message()
        #     if self.log_action_time:
        #         action_duration = (time.time() - action_start_time) * 1000
        #         rospy.logdebug(f'AMR SERVER: Read data and publish action time: {action_duration} ms.')

if __name__ == '__main__':
    try:
        rospy.init_node('simple_fleet_manager_wifi', log_level=rospy.DEBUG)
        connection_handler = ConnectionHandler()
        rospy.spin()
    except Exception as e:
        rospy.logfatal(f'AMR SERVER: Error detected in main: {e}')
    finally:
        rospy.logfatal('AMR SERVER: SHUTTING DOWN')