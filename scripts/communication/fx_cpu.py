#!/usr/bin/env python3
import rospy, time
from pyModbusTCP.client import ModbusClient
from bitstring import BitArray
from std_msgs.msg import Bool
from dhi_amr.msg import workstate_read, flexi_data_in, flexi_data_out


class Communication:
    def __init__(self):
        """Variables definitions"""

        """Options"""
        self.fx_ip_address = '192.168.1.11'
        self.fx_port = 502
        self.fx_connection_timeout = 0.5
        self.fx_connection_retry_timeout = 10.0
        self.refresh_rate = 10
        self.lidars_reset_retries_count = 2
        self.data_out_pub_queue = 1
        self.safety_status_pub_queue = 1
        self.data_out_pub_latch = True
        self.safety_status_pub_latch = True
        self.log_action_times = True
        self.lidar_reset_time = 3.0
        self.auto_reset_option = True
        self.test_mode = True

        """Main"""
        self.fx_cpu = ModbusClient()
        self.rate = rospy.Rate(self.refresh_rate)
        self.fx_cpu_is_connected = False
        self.user_options_changed = False
        self.safety_status = False
        self.is_workstates_changed = False

        """Custom"""
        self.data_in = flexi_data_in()
        self.external_commands = flexi_data_in()
        self.data_out = flexi_data_out()
        self.actual_workstates = workstate_read()
        self.last_workstates = workstate_read()

        """Main program loop"""
        while not rospy.is_shutdown():
            try:
                """ROS Subsrciber's definitions"""
                actual_workstates_sub = rospy.Subscriber('amr/state_machine', workstate_read, self.actual_workstates_callback)
                external_commands_sub = rospy.Subscriber('amr/commands/safety', flexi_data_in, self.external_commands_callback)
                """ROS Publisher's definitions"""
                self.data_out_pub = rospy.Publisher('amr/safety', flexi_data_out, queue_size=self.data_out_pub_queue, latch=self.data_out_pub_latch)
                self.safety_status_pub = rospy.Publisher('amr/safety/lidars_ok', Bool, queue_size=self.safety_status_pub_queue, latch=self.safety_status_pub_latch)
            except Exception as e:
                rospy.logerr(f'Error detected in fx cpu communication at main program loop: {e}')
            if not self.fx_cpu_is_connected or self.user_options_changed:
                self.connect_fx()
            if self.fx_cpu_is_connected:
                self.communication()

    """ROS topic's callbacks"""
    def actual_workstates_callback(self, msg):
        self.actual_workstates = msg

    def external_commands_callback(self, msg):
        self.external_commands = msg
        rospy.loginfo(f'Fx cpu communication: External command received: {self.external_commands}')

    """ Optional features """
    def colorize(self, text, color_code):
        return "\033[{}m{}\033[0m".format(color_code, text)

    def print_green(self, message):
        print(self.colorize(message, '92'))

    """ Connection method """
    def connect_fx(self):
        action_start_time = time.time()
        try:
            if self.fx_cpu.is_open:
                self.fx_cpu.close()
            self.fx_cpu = ModbusClient(self.fx_ip_address, self.fx_port, timeout=self.fx_connection_timeout, auto_open=True, auto_close=True)
            start_time = time.time()
            while (time.time() - start_time) < self.fx_connection_retry_timeout:
                self.fx_cpu.open()
                self.fx_cpu_is_connected = self.fx_cpu.is_open
                self.fx_cpu.close()
                if (time.time() - start_time) > self.fx_connection_retry_timeout:
                    self.fx_cpu_is_connected = False
                    rospy.logerr(f'FX CPU connection status: {self.fx_cpu_is_connected}')
                    break
                if self.fx_cpu_is_connected:
                    self.print_green(f'FX CPU connection status: {self.fx_cpu_is_connected}')
                    break
            if self.log_action_times:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'FX CPU communication - connect action duration time: {action_duration}')
        except ValueError as e:
            rospy.logerr(f'Error detected in fx cpu communication at connect - value error: {e}')
        except Exception as e:
            rospy.logerr(f'Error detected in fx cpu communication at connect - other error: {e}')

    """ Checking for lidar alarms """
    def check_lidar_alarms(self):
        result = False
        result = self.data_out.left_scanner.contamination_error
        if result:
            return result
        result = self.data_out.left_scanner.monitoring_case_valid
        if result:
            return result
        result = self.data_out.left_scanner.app_error
        if result:
            return result
        result = self.data_out.left_scanner.device_error
        if result:
            return result
        result = self.data_out.right_scanner.contamination_error
        if result:
            return result
        result = self.data_out.right_scanner.monitoring_case_valid
        if result:
            return result
        result = self.data_out.right_scanner.app_error
        if result:
            return result
        result = self.data_out.right_scanner.device_error
        if result:
            return result
        return result    

    """ Check that last workstate was different than actual """
    def check_workstate_changed(self):
        if self.actual_workstates.S0_1 != self.last_workstates.S0_1:
            return True
        if self.actual_workstates.S0_2 != self.last_workstates.S0_2:
            return True
        if self.actual_workstates.S0_3 != self.last_workstates.S0_3:                   
            return True
        if self.actual_workstates.S1 != self.last_workstates.S1:
            return True
        if self.actual_workstates.S2 != self.last_workstates.S2:
            return True
        if self.actual_workstates.S3 != self.last_workstates.S3:
            return True
        if self.actual_workstates.S4 != self.last_workstates.S4:
            return True
        if self.actual_workstates.S4_0 != self.last_workstates.S4_0:
            return True
        if self.actual_workstates.S4_1 != self.last_workstates.S4_1:
            return True
        if self.actual_workstates.S4_2 != self.last_workstates.S4_2:
            return True
        if self.actual_workstates.S4_3 != self.last_workstates.S4_3:
            return True
        if self.actual_workstates.S4_4 != self.last_workstates.S4_4:
            return True
        if self.actual_workstates.S4_5 != self.last_workstates.S4_5:
            return True
        if self.actual_workstates.S4_6 != self.last_workstates.S4_6:
            return True
        return False

    """ Set no monitoring field for monitoring case change """
    def prepare_lidar_for_change(self):
        self.data_in.monitoring_case_signal_a1 = False
        self.data_in.monitoring_case_signal_a2 = False
        self.data_in.monitoring_case_signal_b1 = False
        self.data_in.monitoring_case_signal_b2 = False

    """ Set monitoring case relevant to actual selected workstate """
    def set_monitoring_case(self):
        if not self.test_mode:
            if self.actual_workstates.S0_1 or self.actual_workstates.S4_0:
                self.data_in.monitoring_case_signal_a1 = True
                self.data_in.monitoring_case_signal_a2 = False
                self.data_in.monitoring_case_signal_b1 = True
                self.data_in.monitoring_case_signal_b2 = False
            if self.actual_workstates.S0_3:
                self.data_in.monitoring_case_signal_a1 = True
                self.data_in.monitoring_case_signal_a2 = False
                self.data_in.monitoring_case_signal_b1 = False
                self.data_in.monitoring_case_signal_b2 = True
            if self.actual_workstates.S1 or self.actual_workstates.S4_1:
                self.data_in.monitoring_case_signal_a1 = False
                self.data_in.monitoring_case_signal_a2 = True
                self.data_in.monitoring_case_signal_b1 = True
                self.data_in.monitoring_case_signal_b2 = False
            if self.actual_workstates.S4 and not self.actual_workstates.S4_0 and not self.actual_workstates.S4_1:
                self.data_in.monitoring_case_signal_a1 = False
                self.data_in.monitoring_case_signal_a2 = True
                self.data_in.monitoring_case_signal_b1 = False
                self.data_in.monitoring_case_signal_b2 = True
        if self.test_mode:
            self.data_in.monitoring_case_signal_a1 = True
            self.data_in.monitoring_case_signal_a2 = False
            self.data_in.monitoring_case_signal_b1 = False
            self.data_in.monitoring_case_signal_b2 = True

    """Set global safety status"""
    def set_safety_status(self, lidar_alarm):
        if not lidar_alarm and self.data_out.cpu_ok and self.data_out.encoder_ok and self.data_out.left_emergency_stop_button and self.data_out.right_emergency_stop_button and self.data_out.speed_ok:
            return True
        else:
            return False

    """ Global reset method """
    def lidar_reset_global(self):
        action_start_time = time.time()
        self.data_in.left_scanner.reset_global = True
        self.data_in.right_scanner.reset_global = True
        self.write_data()
        time.sleep(self.lidar_reset_time)
        self.data_in.left_scanner.reset_global = False
        self.data_in.right_scanner.reset_global = False
        self.write_data()
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - lidar global reset action duration time: {action_duration}')

    """Converting BitArray to write"""
    def bit_array_conversion(self, bit_array):
        action_start_time = time.time()
        result = 0
        for bit in bit_array:
            result = (result << 1) | bit
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - bit array conversion action duration time: {action_duration}')
        return result

    """Converting readed data to binary list"""
    def convert_to_binary_list(self, data):
        action_start_time = time.time()
        result = []
        if data is not None:
            for item in data:
                result.append(bin(int(item)))
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - conver to binary list action duration time: {action_duration}')
        return result

    """Converting binary list"""
    def process_data(self, data):
        action_start_time = time.time()
        tmp_list = []
        tmp_string = ''
        for i in range (0, len(data)):
            data[i] = data[i][2:]
            difference = 16 - len(data[i])
            for j in range (0, difference):
                data[i] = "0" + data[i]
            data[i] = data[i][::-1]
            tmp_list.append(data[i])
        for i in range(0, len(tmp_list)):
            tmp_string = tmp_string + tmp_list[i]
        result = list(tmp_string)
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - process data action duration time: {action_duration}')
        return result

    """Creating bool list"""
    def create_bool_list(self, data):
        action_start_time = time.time()
        result = []
        for i in range(0, len(data)):
            result.append(bool(int(data[i])))
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - creating bool list action duration time: {action_duration}')
        return result

    """ Data write to flexi modbus gateway"""
    def write_data(self):
        action_start_time = time.time()
        bits = BitArray(16)
        bits[0] = self.data_in.monitoring_case_signal_a1
        bits[1] = self.data_in.monitoring_case_signal_a2
        bits[2] = self.data_in.monitoring_case_signal_b1
        bits[3] = self.data_in.monitoring_case_signal_b2
        bits[4] = self.data_in.right_scanner.reset_connection
        bits[5] = self.data_in.right_scanner.reset_global
        bits[6] = self.data_in.left_scanner.reset_connection
        bits[7] = self.data_in.left_scanner.reset_global
        bits[8] = self.data_in.activate_lidars
        bits[9] = self.data_in.right_scanner.reset_reduced_speed_zone
        bits[10] = self.data_in.right_scanner.reset_soft_stop_zone
        bits[11] = self.data_in.right_scanner.reset_emergency_zone
        bits[12] = self.data_in.left_scanner.reset_reduced_speed_zone
        bits[13] = self.data_in.left_scanner.reset_soft_stop_zone
        bits[14] = self.data_in.left_scanner.reset_emergency_zone
        bits[15] = False
        bits_set = [0] * 5
        bits_set[0] = self.bit_array_conversion(bits)
        if bits_set[0] is not None:
            try:
                status = self.fx_cpu.write_multiple_registers(2099, bits_set)
                if not status:
                    self.fx_cpu_is_connected = False
            except Exception as e:
                rospy.logerr(f'Error detected in fx cpu communication at writing data to gateway: {e}')
        else:
            rospy.logerr(f'Error detected in fx cpu communication at writing data to gateway: data set is NONE: {bits_set}')
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - write data to gmod action duration time: {action_duration}')
        

    """ Data read from flexi modbus gateway """
    def read_data(self):
        action_start_time = time.time()
        data = bytearray()
        data = self.fx_cpu.read_holding_registers(999, 25)
        if data is None:
            self.fx_cpu_is_connected = False
        data_bin_list = self.convert_to_binary_list(data)
        data_tmp_list = self.process_data(data_bin_list)
        data_bool_list = self.create_bool_list(data_tmp_list)
        if len(data_bool_list) > 0:
            try:
                self.data_out.left_emergency_stop_button = data_bool_list[0]
                self.data_out.right_emergency_stop_button = data_bool_list[1]
                self.data_out.left_scanner.is_active = data_bool_list[2]
                self.data_out.right_scanner.is_active = data_bool_list[3]
                self.data_out.speed_ok = data_bool_list[4]
                self.data_out.encoder_ok = data_bool_list[5]
                self.data_out.cpu_ok = data_bool_list[6]
                self.data_out.standstill = data_bool_list[7]
                self.data_out.left_scanner.emergency_stop_zone_status = data_bool_list[8]
                self.data_out.left_scanner.soft_stop_zone_status = data_bool_list[9]
                self.data_out.left_scanner.reduced_speed_zone_status = data_bool_list[10]
                self.data_out.right_scanner.emergency_stop_zone_status = data_bool_list[11]
                self.data_out.right_scanner.soft_stop_zone_status = data_bool_list[12]
                self.data_out.right_scanner.reduced_speed_zone_status = data_bool_list[13]
                self.data_out.left_scanner.contamination_warning = data_bool_list[14]
                self.data_out.left_scanner.contamination_error = data_bool_list[15]
                self.data_out.right_scanner.contamination_warning = data_bool_list[16]
                self.data_out.right_scanner.contamination_error = data_bool_list[17]
                self.data_out.left_scanner.app_error = data_bool_list[18]
                self.data_out.left_scanner.device_error = data_bool_list[19]
                self.data_out.right_scanner.app_error = data_bool_list[20]
                self.data_out.right_scanner.device_error = data_bool_list[21]
            except Exception as e:
                rospy.logerr(f'Error detected in fx cpu communication at reading data from gateway: data set length < 0: {data_bool_list}')
                self.fx_cpu_is_connected = False
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - read data from GMOD action duration time: {action_duration}')

    """Publishing ROS topics"""
    def publish_topics(self):
        try:
            self.data_out_pub.publish(self.data_out)
            self.safety_status_pub.publish(self.safety_status)
        except Exception as e:
            rospy.logerr(f'Error detected in fx cpu communication at publishing ROS topics: {e}')

    """ Main communication sequence"""
    def communication(self):
        sequence_start_time = time.time()
        self.read_data()
        action_start_time = time.time()
        lidar_status = self.check_lidar_alarms()
        self.set_safety_status(lidar_status)
        if self.log_action_times:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'FX CPU communication - lidar alarm check action duration time: {action_duration}')
        if lidar_status and self.auto_reset_option:
            self.lidar_reset_global()
            lidar_status = self.check_lidar_alarms()
            self.set_safety_status(lidar_status)
            if self.log_action_times:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'FX CPU communication - lidar alarm check action duration time: {action_duration}')
        if not lidar_status:
            is_workstate_changed = self.check_workstate_changed()
            if is_workstate_changed:
                self.last_workstates = self.actual_workstates
                self.prepare_lidar_for_change()
                self.write_data()
                time.sleep(0.3)
                self.set_monitoring_case()
        self.write_data()
        self.publish_topics()
        if self.log_action_times:
            sequence_duration = time.time() - sequence_start_time
            rospy.loginfo(f'FX CPU communication - communication sequence action duration time: {sequence_duration}')
        self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('communication_fx_cpu', log_level=rospy.DEBUG)
        rospy.loginfo('FX CPU communication: Program started')
        fx = Communication()
        
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in fx cpu communication at main init: {e}')
    finally:
        rospy.logwarn('FX CPU communication: Program shutting down')




