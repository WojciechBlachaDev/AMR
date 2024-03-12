#!/usr/bin/env python3
import rospy, time, threading, os
from pyModbusTCP.client import ModbusClient
from dhi_amr.msg import plc_data_in, plc_data_out, plc_error_codes, plc_error_status
from dhi_amr.msg import workstate_read, workstate_request, scangrid
from dhi_amr.msg import commands_curtis, commands_distance_drive, commands_forks, commands_scangrids, commands_servo


class PLC:
    def __init__(self):
        """ Variables declarations """
        """ Options """

        self.plc_data_out_pub_queue = 1
        self.plc_error_statuses_pub_queue = 1
        self.plc_error_codes_pub_queue = 1
        self.scangrid_left_pub_queue = 1
        self.scangrid_right_pub_queue = 1
        self.workstates_active_pub_queue = 1
        self.plc_data_out_pub_latch = True
        self.plc_error_statuses_pub_latch = True
        self.plc_error_codes_pub_latch = True
        self.scangrid_left_pub_latch = True
        self.scangrid_right_pub_latch = True
        self.workstates_active_pub_latch = True
        self.plc_connection_timeout = 0.5
        self.plc_connection_retry_timeout = 10.0
        self.log_actions_durations = True

        """ Custom messages """

        self.mutex = threading.Lock()
        self.plc_data_in = plc_data_in()
        self.plc_data_out = plc_data_out()
        self.plc_error_statuses = plc_error_status()
        self.plc_error_codes = plc_error_codes()
        self.scangrid_left = scangrid()
        self.scangrid_right = scangrid()
        self.workstates_active = workstate_read()
        self.workstates_requested = workstate_request()
        self.curtis_commands = commands_curtis()
        self.distance_drive_commands = commands_distance_drive()
        self.forks_commands = commands_forks()
        self.scangrids_commands = commands_scangrids()
        self.servo_commands = commands_servo()

        """Communication variables"""

        self.plc_ip_address = '192.168.1.4'
        self.plc_port = 502
        self.plc_is_connected = False
        self.plc = ModbusClient()
        self.refresh_rate = rospy.Rate(30)
        self.pwm_value = 0
        self.servo_angle  = 0
        self.servo_direction = 0
        self.drive_forward = False
        self.drive_backward = False
        self.servo_power = False
        self.plc_watchdog = 1
        self.plc_watchdog_old = 0
        
        """ Main program loop """

        while not rospy.is_shutdown():
            try:
                """ Ros Publisher's declarations"""
                self.plc_data_out_pub = rospy.Publisher('amr/plc/data_out', plc_data_out, queue_size=self.plc_data_out_pub_queue, latch=self.plc_data_out_pub_latch)
                self.plc_error_statuses_pub = rospy.Publisher('amr/plc/errors/statuses', plc_error_status, queue_size=self.plc_error_statuses_pub_queue, latch=self.plc_error_statuses_pub_latch)
                self.plc_error_codes_pub = rospy.Publisher('amr/plc/errors/codes', plc_error_codes, queue_size=self.plc_error_codes_pub_queue, latch=self.plc_error_codes_pub_latch)
                self.scangrid_left_pub = rospy.Publisher('amr/scangrids/scangrid_left', scangrid, queue_size=self.scangrid_left_pub_queue, latch=self.scangrid_left_pub_latch)
                self.scangrid_right_pub = rospy.Publisher('amr/scangrids/scangrid_right', scangrid, queue_size=self.scangrid_right_pub_queue, latch=self.scangrid_right_pub_latch)
                self.workstates_active_pub = rospy.Publisher('amr/state_machine', workstate_read, queue_size=self.workstates_active_pub_queue, latch=self.workstates_active_pub_latch)
                
                """ Ros Subscriber's declarations"""
                curtis_commands_sub = rospy.Subscriber('amr/plc/commands/curtis', commands_curtis, self.curtis_commands_callback)
                distance_drive_commands_sub = rospy.Subscriber('amr/plc/commands/distance_drive', commands_distance_drive, self.distance_drive_commands_callback)
                forks_commands_sub = rospy.Subscriber('amr/plc/commands/forks', commands_forks, self.forks_commands_callback)
                scangrids_commands_sub = rospy.Subscriber('amr/plc/commands/scangrids', commands_scangrids, self.scangrids_commands_callback)
                servo_commands_sub = rospy.Subscriber('amr/plc/commands/servo', commands_servo, self.servo_commands_callback)
                workstates_request_sub = rospy.Subscriber('amr/plc/commands/workstate_request', workstate_request, self.workstates_request_callback)
            except Exception as e:
                rospy.logerr(f'Error detected in plc communication at main loop: {e}')
            if not self.plc_is_connected:
                self.plc_connect()
            if self.plc_is_connected:
                self.communication()

    """ Ros Subsriber's callbacks """
    def curtis_commands_callback(self, msg):
        self.curtis_commands = msg

    def distance_drive_commands_callback(self, msg):
        self.distance_drive_commands = msg

    def forks_commands_callback(self, msg):
        self.forks_commands = msg

    def scangrids_commands_callback(self, msg):
        self.scangrids_commands = msg

    def servo_commands_callback(self, msg):
        self.servo_commands = msg

    def workstates_request_callback(self, msg):
        self.workstates_requested = msg

    """ Optional features """
    def colorize(self, text, color_code):
        return "\033[{}m{}\033[0m".format(color_code, text)

    def print_green(self, message):
        print(self.colorize(message, '92'))

    """Connection def with waiting for avaible connection function"""
    def plc_connect(self):
        action_start_time = time.time()
        try:
            if self.plc.is_open:
                self.plc.close()
            self.plc = ModbusClient(self.plc_ip_address, self.plc_port, auto_open=True, auto_close=True, timeout=self.plc_connection_timeout)
            start_time = time.time()
            while time.time() - start_time < self.plc_connection_retry_timeout:
                self.plc.open()
                self.plc_is_connected = self.plc.is_open
                self.plc.close()
                if time.time() - start_time > self.plc_connection_retry_timeout:
                    self.plc_is_connected = False
                    rospy.logerr(f'PLC connection status: {self.plc_is_connected}')
                    break
                if self.plc_is_connected:
                    self.print_green(f'PLC connection status: {self.plc_is_connected}')
                    break
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'Plc communication connect method action time: {action_duration} s.')
        except ValueError as e:
            rospy.logfatal(f'Error detected in plc communication at connect method - Value Error: {e}')
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at connect method - Other exception: {e}')
    
    """Watchdog function for PLC connection monitoring"""
    def watchdog(self):
        action_start_time = time.time()
        self.plc_watchdog_old = self.plc_watchdog
        if self.plc_watchdog_old == 1:
            self.plc_watchdog = 2
        elif self.plc_watchdog_old == 2:
            self.plc_watchdog = 1
        else:
            rospy.logfatal(f'Error detected in plc communication at watchdog method - watchdog value should be 1 or 2. Current value: {self.plc_watchdog_old}')
        if self.log_actions_durations:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'PLC communication watchdog generation action time: {action_duration} s.')
        return self.plc_watchdog

    """Assign values from ROS to PLC data """
    def assign_data_from_ros(self):
        action_start_time = time.time()
        try:
            self.plc_data_in.curtis_power = self.curtis_commands.pwm
            self.plc_data_in.curtis_forward = self.curtis_commands.forward
            self.plc_data_in.curtis_backward = self.curtis_commands.backward
            self.plc_data_in.servo_direction = self.servo_commands.direction
            self.plc_data_in.servo_power = self.servo_commands.power
            self.plc_data_in.servo_angle = self.servo_commands.angle
            self.plc_data_in.distance_drive_start = self.distance_drive_commands.start
            self.plc_data_in.distance_drive_cancel = self.distance_drive_commands.cancel
            self.plc_data_in.distance_drive_reset = self.distance_drive_commands.reset
            self.plc_data_in.requested_distance = self.distance_drive_commands.distance
            self.plc_data_in.forks_up = self.forks_commands.lift
            self.plc_data_in.forks_down = self.forks_commands.drop
            self.plc_data_in.save_weight = self.forks_commands.save_weight
            self.plc_data_in.scangrids_activate_all = self.scangrids_commands.all
            self.plc_data_in.scangrid_activate_left = self.scangrids_commands.left
            self.plc_data_in.scangrids_activate_right = self.scangrids_commands.right
            self.plc_data_in.watchdog = self.watchdog()
            print(self.plc_data_in.servo_direction)
            print(self.servo_commands.direction)
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC communication assign ROS data action time: {action_duration} s.')
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at assigning ros data: {e}')
    
    """ Writing data to plc registers """
    def write_plc_registers(self):
        action_start_time = time.time()
        table = [0] * 100
        try:
            table[0] = self.plc_data_in.servo_direction
            table[1] = self.plc_data_in.servo_angle
            table[2] = self.plc_data_in.curtis_power
            table[3] = self.plc_data_in.requested_distance
            table[98] = self.plc_data_in.watchdog
            status = self.plc.write_multiple_registers(100, table)
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC Communication write plc registers action time: {action_duration} s.')
            if not status:
                rospy.logerr(f'Error detected in plc communication at writing plc registers - write status: {status}')
                self.plc_is_connected = False
        except Exception as e:
            rospy.logfatal(f'Error detected in plc communication at writing plc registers: {e}')
            self.plc_is_connected = False

    """Writing data to PLC virtual digital inputs"""
    def write_plc_virtual_digital_inputs(self):
        action_start_time = time.time()
        table = [False] * 80
        try:
            table[8] = self.workstates_requested.select_auto_mode
            table[9] = self.workstates_requested.initial_diag_complete
            table[10] = self.workstates_requested.start_navigation_ride
            table[11] = self.workstates_requested.finish_navigation_ride
            table[12] = self.workstates_requested.start_charging_sequence
            table[13] = self.workstates_requested.finish_charging_sequence
            table[14] = self.workstates_requested.start_magazine_get_palette
            table[15] = self.workstates_requested.finish_magazine_get_palette
            table[16] = self.workstates_requested.start_magazine_leave_palette
            table[17] = self.workstates_requested.finish_magazine_leave_palette
            table[18] = self.workstates_requested.start_nest_get_palette
            table[19] = self.workstates_requested.finish_nest_get_palette
            table[20] = self.workstates_requested.start_nest_leave_palette
            table[21] = self.workstates_requested.finish_nest_leave_palette
            table[22] = False #reserved for authorized operator login status
            table[23] = False #reserved (free signal )
            table[24] = self.plc_data_in.distance_drive_start
            table[25] = self.plc_data_in.distance_drive_reset
            table[26] = self.plc_data_in.distance_drive_cancel
            table[27] = self.plc_data_in.scangrid_activate_left
            table[28] = self.plc_data_in.scangrids_activate_right
            table[29] = self.plc_data_in.scangrids_activate_all
            table[30] = self.plc_data_in.servo_power
            table[31] = self.plc_data_in.curtis_forward
            table[32] = self.plc_data_in.curtis_backward
            table[33] = self.plc_data_in.forks_up
            table[34] = self.plc_data_in.forks_down
            table[35] = self.plc_data_in.save_weight
            """ Secure data when table value is NONE"""
            for i in range(0, len(table)):
                if table[i] is None:
                    table[i] = False
            status = self.plc.write_multiple_coils(41120, table)
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC Communication write plc virtual inputs action time: {action_duration} s.')
            if not status:
                rospy.logerr(f'Error detected in plc communication at writing plc virtual inputs - write status: {status}')
                self.plc_is_connected = False
        except Exception as e:
            rospy.logfatal(f'Error detected in plc communication at writing plc virtual inputs: {e}')
            self.plc_is_connected = False

    """Read PLC registers data"""
    def read_plc_registers(self):
        action_start_time = time.time()
        try:
            table = self.plc.read_holding_registers(200, 99)
            if table is not None:
                self.plc_data_out.actual_forks_height = table[0]
                self.plc_data_out.weight = table[1]
                self.plc_data_out.tilt_axis_1 = table[2]
                self.plc_data_out.tilt_axis_2 = table[3]
                self.plc_data_out.battery_voltage = table[4]
                self.plc_data_out.speed_value = table[5]
                self.plc_data_out.speed_direction = table[6]
                self.plc_data_out.steering_direction = table[7]
                self.plc_data_out.steering_angle = table[8]
                self.plc_data_out.position = table[9]
                self.plc_data_out.weight_saved = table[10]
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC Communication read plc registers action time: {action_duration} s.')
            if table is None:
                rospy.logerr(f'Error detected in plc communication at reading plc registers - table status is None')
                self.plc_is_connected = False
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at reading plc registers: {e}')
            self.plc_is_connected = False

    """ Read PLC scangrids data table """
    def read_plc_scangrids_data(self):
        action_start_time = time.time()
        table = [0] * 100
        try:
            if not self.scangrids_commands.all and not self.scangrids_commands.left and not self.scangrids_commands.right:
                if table is not None:
                    for i in range (0,63):
                        table[i] = 0
            else:
                table = self.plc.read_holding_registers(600, 99)
            for i in range(32):
                setattr(self.scangrid_left.data, f'range_{i}', table[i])
            for i in range(32):
                setattr(self.scangrid_right.data, f'range_{i}', table[i + 32])
            if table is None:
                rospy.logerr(f'Error detected in plc communication at reading plc scangrid list - table is NONE')
                self.plc_is_connected = False
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC communication read plc scangrids data action time: {action_duration} s.')
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at reading plc scangrids data: {e}')
            self.plc_is_connected = False

    """ Read PLC virtual digital outputs"""
    def read_plc_virtual_digital_outputs(self):
        action_start_time = time.time()
        try:
            table = self.plc.read_coils(41200, 80)
            if table is not None:
                self.workstates_active.S0_1 = table[8]
                self.workstates_active.S0_2 = table[9]
                self.workstates_active.S0_3 = table[10]
                self.workstates_active.S1 = table[11]
                self.workstates_active.S2 = table[12]
                self.workstates_active.S3 = table[13]
                self.workstates_active.S4 = table[14]
                self.workstates_active.S4_0 = table[15]
                self.workstates_active.S4_1 = table[16]
                self.workstates_active.S4_2 = table[17]
                self.workstates_active.S4_3 = table[18]
                self.workstates_active.S4_4 = table[19]
                self.workstates_active.S4_5 = table[20]
                self.workstates_active.S4_6 = table[21]
                self.plc_data_out.base_position_saved = table[22]
                self.plc_data_out.position_reached = table[23]
                self.plc_error_statuses.scangrid_left = table[24]
                self.plc_error_statuses.scangrid_right = table[25]
                self.plc_error_statuses.pressure_sensor = table[26]
                self.plc_error_statuses.forks_height_sensor = table[27]
                self.plc_error_statuses.tilt_sensor_axis_1 = table[28]
                self.plc_error_statuses.tilt_sensor_axis_2 = table[29]
                self.plc_error_statuses.battery_sensor = table[30]
                self.plc_error_statuses.manual_speed_regulator = table[31]
                self.scangrid_left.status.safety_output = table[32]
                self.scangrid_left.status.protection_field_status = table[33]
                self.scangrid_left.status.work_status = table[34]
                self.scangrid_left.status.warning_field_status = table[40]
                self.scangrid_left.status.contamination_warning = table[41]
                self.scangrid_left.status.contamination_error = table[42]
                self.scangrid_left.status.monitoring_case_switch_input_status = table[43]
                self.scangrid_left.status.monitoring_case_switch_can_input_status = table[44]
                self.scangrid_left.status.voltage_error = table[45]
                self.scangrid_left.status.resistance_to_external_light_error = table[46]
                self.scangrid_left.status.sleep_mode_status = table[47]
                self.scangrid_right.status.safety_output = table[48]
                self.scangrid_right.status.protection_field_status = table[49]
                self.scangrid_right.status.work_status = table[50]
                self.scangrid_right.status.warning_field_status = table[56]
                self.scangrid_right.status.contamination_warning = table[57]
                self.scangrid_right.status.contamination_error = table[58]
                self.scangrid_right.status.monitoring_case_switch_input_status = table[59]
                self.scangrid_right.status.monitoring_case_switch_can_input_status = table[60]
                self.scangrid_right.status.voltage_error = table[61]
                self.scangrid_right.status.resistance_to_external_light_error = table[62]
                self.scangrid_right.status.sleep_mode_status = table[63]
                self.plc_error_statuses.pwm_to_curtis_write = table[64]
                self.plc_error_statuses.servo_position_read = table[65]
                self.plc_error_statuses.servo_move = table[66]
                self.plc_error_statuses.servo_halt = table[67]
            if table is None:
                rospy.logerr(f'Error detected in plc communication at reading virtual digital outputs - list is NONE')
                self.plc_is_connected = False
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC communication read plc virtual digital outputs action time: {action_duration} s.')
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at reading virtual outputs: {e}')
            self.plc_is_connected = False
            
    """ Read PLC error registers"""
    def read_plc_error_registers(self):
        action_start_time = time.time()
        try:
            table = self.plc.read_holding_registers(500, 30)
            if table is not None:
                self.plc_error_codes.scangrid_left = table[0]
                self.plc_error_codes.scangrid_right = table[1]
                self.plc_error_codes.forks_height_sensor = table[2]
                self.plc_error_codes.pwm_to_curtis_write = table[3]
                self.plc_error_codes.tilt_sensor_axis_1 = table[4]
                self.plc_error_codes.tilt_sensor_axis_2 = table[5]
                self.plc_error_codes.battery_sensor = table[6]
                self.plc_error_codes.servo_position_read = table[7]
                self.plc_error_codes.manual_speed_regulator = table[8]
                self.plc_error_codes.servo_move = table[9]
                self.plc_error_codes.pressure_sensor = table[10]
                self.plc_error_codes.servo_halt = table[11]
            if table is None:
                rospy.logerr(f'Error detected in plc communication at reading plc error registers = list is NONE')
                self.plc_is_connected = False
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'PLC communication read plc virtual digital outputs action time: {action_duration} s.')
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at reading plc error registers: {e}')
            self.plc_is_connected = False
    
    """Publish readed data to ROS topics"""
    def publish_topics(self):
        self.plc_data_out_pub.publish(self.plc_data_out)
        self.plc_error_statuses_pub.publish(self.plc_error_statuses)
        self.plc_error_codes_pub.publish(self.plc_error_codes)
        self.scangrid_left_pub.publish(self.scangrid_left)
        self.scangrid_right_pub.publish(self.scangrid_right)
        self.workstates_active_pub.publish(self.workstates_active)

    """Main communication sequence with ros data publishing"""
    def communication(self):
        action_start_time = time.time()
        try:
            self.read_plc_registers()
            self.read_plc_virtual_digital_outputs()
            self.read_plc_scangrids_data()
            self.read_plc_error_registers()
            self.publish_topics()
            self.assign_data_from_ros()
            self.write_plc_registers()
            self.write_plc_virtual_digital_inputs()
            if self.log_actions_durations:
                action_duration = time.time() - action_start_time 
                rospy.loginfo(f'PLC communication overall time: {action_duration} s.')
            self.refresh_rate.sleep()
            # os.system('clear')
        except Exception as e:
            rospy.logerr(f'Error detected in plc communication at main communication sequence: {e}')

if __name__ == '__main__':
    try:
        rospy.init_node('communication_plc', log_level=rospy.DEBUG)
        plc = PLC()
        rospy.loginfo('Communication with PLC script started')
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in plc communication at main launch: {e}')
    finally:
        rospy.loginfo('Communication with PLC script shuted down!')

