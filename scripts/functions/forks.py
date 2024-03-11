#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Bool
from dhi_amr.msg import flexi_data_out, sensors_data, commands_forks


class ForksController:
    def __init__(self):
        """ Variables """

        """ Options """
        self.max_forks_height = 207
        self.min_forks_height = 107
        self.forks_height_tolerance = 10
        self.forks_action_timeout = 15
        self.forklift_tilt_toleration_axis1 = 5
        self.forklift_tilt_toleration_axis2 = 5
        self.command_publish_queue = 1
        self.command_publish_latch = True
        self.action_status_queue = 1
        self.action_status_latch = True

        """ Custom """
        self.data_safety = flexi_data_out()
        self.data_sensors = sensors_data()
        self.command_received = commands_forks()
        self.command_publish = commands_forks()
        self.command_received_last = commands_forks()

        """ Other """

        self.tilt_axis1_max = 0.0
        self.tilt_axis1_min = 0.0
        self.tilt_axis2_max = 0.0
        self.tilt_axis2_min = 0.0
        self.forks_active = False
        self.lidars_ok = False
        self.action_confirmation = False
        self.log_action_time = True

        """ Main program loop """
        while not rospy.is_shutdown():
            try:
                """ ROS Subscriber's definitions """
                command_received_sub = rospy.Subscriber('amr/steering/forks', commands_forks, self.command_received_callback)
                action_confirmation = rospy.Subscriber('amr/steering/forks/action_confirmation', Bool, self.action_confirmation_callback)
                data_safety_sub = rospy.Subscriber('amr/safety', flexi_data_out, self.data_safety_callback)
                data_sensors_sub = rospy.Subscriber('amr/plc/sensors_converted', sensors_data, self.data_sensors_callback)
                lidars_ok_sub = rospy.Subscriber('amr/safety/lidars_ok', Bool, self.lidars_ok_callback)
                """ ROS Publisher's definitions """
                self.action_status_pub = rospy.Publisher('amr/forks/action_result', Bool, queue_size=self.action_status_queue, latch=self.action_status_latch)
                self.command_publish_pub = rospy.Publisher('amr/plc/commands/forks', commands_forks, queue_size=self.command_publish_queue, latch=self.command_publish_latch)
            except Exception as e:
                rospy.logerr(f'Error detected in Forks controller at main program loop: {e}')
    
    """ Callback for forks command subscriber """
    def command_received_callback(self, msg):
        self.command_received = msg
    
    """ Callback for safety data subscriber """
    def data_safety_callback(self, msg):
        self.data_safety = msg
    
    """ Callback for sensors data subscriber """
    def data_sensors_callback(self, msg):
        self.data_sensors = msg
    
    """ Callback for lidar ok status subscriber """
    def lidars_ok_callback(self, msg):
        self.lidars_ok = msg.data
    
    """ Callback for action edn confirmation subscriber """
    def action_confirmation_callback(self, msg):
        self.action_confirmation = msg.data
    
    """ Check for command received changed """
    def check_for_command_change(self):
        if self.command_received.lift != self.command_received_last.lift:
            return True
        if self.command_received.drop != self.command_received_last.drop:
            return True
        if self.command_received.save_weight != self.command_received_last.save_weight:
            return True
        return False
    
    """ Check for E-STOP button status"""
    def check_emergency_stop_buttons(self):
        if self.data_safety.left_emergency_stop_button and self.data_safety.right_emergency_stop_button:
            return True
        return False
    
    """ Check for lidars reduced speed zone status """
    def check_lidar_reduced_speed_zone(self):
        if self.data_safety.left_scanner.reduced_speed_zone_status and self.data_safety.right_scanner.reduced_speed_zone_status:
            return True
        return False
    
    """ Check for lidars soft stop zone status """
    def check_lidar_soft_stop_zone(self):
        if self.data_safety.left_scanner.soft_stop_zone_status and self.data_safety.right_scanner.soft_stop_zone_status:
            return True
        return False
    
    """ Check for lidars emergency stop zone status """
    def check_lidar_emergency_stop_zone(self):
        if self.data_safety.left_scanner.emergency_stop_zone_status and self.data_safety.right_scanner.emergency_stop_zone_status:
            return True
        return False
    
    """ Check for errors in safety system """
    def check_for_safety_errors(self):
        if self.lidars_ok and self.data_safety.cpu_ok and self.data_safety.encoder_ok:
            return False
        return True
    
    """ Check for safety to do forks motion"""
    def safety_check(self):
        check_0 = self.check_emergency_stop_buttons()
        check_1 = self.check_lidar_emergency_stop_zone()
        check_2 = self.check_lidar_soft_stop_zone()
        check_3 = self.check_lidar_reduced_speed_zone()
        check_4 = self.check_for_safety_errors()
        return check_0, check_1, check_2, check_3, check_4
    
    """ Set stop from safety """
    def safety_stop_action(self):
        safety_result = self.safety_check()
        if safety_result[0] and safety_result[1] and safety_result[2] and safety_result[3] and not safety_result[4]:
            return False
        return True
    
    """ Check forks current height with tolerance """
    def verify_forks_height(self):
        if not (self.min_forks_height - self.forks_height_tolerance) > self.data_sensors.forks_height > (self.min_forks_height + self.forks_height_tolerance):
            return True
        return False
        
    """ Verification is received command adequate to current forks status """
    def verify_received_command(self):
        try:
            if self.command_received.lift and not self.command_received.drop and not self.command_received.save_weight:
                if self.data_sensors.forks_height_limiter:
                    return True
            if self.command_received.drop and not self.command_received.lift and not self.command_received.save_weight:
                if self.verify_forks_height():
                    return True
            if self.command_received.save_weight and not self.command_received.lift and not self.command_received.drop:
                return True
            return False
        except Exception as e:
            rospy.logerr(f'Error detected in Forks controller at verify command received: {e}')
            return False

    """ Check for overturn risk method """
    def is_stable(self):
        if self.tilt_axis1_min > self.data_sensors.tilt_axis_1 > self.tilt_axis1_max and self.tilt_axis2_min > self.data_sensors.tilt_axis_2 > self.tilt_axis2_max:
            return True
        return False
    
    """ Set max and min tilt tolerance to actual surface tilt"""
    def set_tilt_tolerance(self):
        self.tilt_axis1_min = self.data_sensors.tilt_axis_1 - self.forklift_tilt_toleration_axis1
        self.tilt_axis1_max = self.data_sensors.tilt_axis_1 + self.forklift_tilt_toleration_axis1
        self.tilt_axis2_min = self.data_sensors.tilt_axis_2 - self.forklift_tilt_toleration_axis2
        self.tilt_axis2_max = self.data_sensors.tilt_axis_2 + self.forklift_tilt_toleration_axis2

    """ Stopping all forks module actions in PLC """
    def set_stop_action(self):
        self.command_publish.drop = False
        self.command_publish.lift = False
        self.command_publish.save_weight = False

    """ Execute lifting command with overturn protection """
    def forks_up(self):
        try:
            action_start_time = time.time()
            self.forks_active = True
            self.command_publish = self.command_received
            self.command_publish_pub.publish(self.command_publish)
            while self.forks_active:
                if not self.is_stable():
                    rospy.logerr('Forks Controller: Overturning risk detected - stopping action')
                    return False
                if (time.time() - action_start_time) > self.forks_action_timeout:
                    rospy.logerr('FOrks Controller:Action timeout overflow detected!')
                    return False
                if self.safety_stop_action():
                    rospy.logerr('Forks controller: Action stopped by safety systems')
                    return False
                if not self.data_sensors.forks_height_limiter:
                    return True
        except Exception as e:
            rospy.logerr(f'Error detected in Forks controller at forks up command execute: {e}')
            return False
    
    """ Execute forks drop command with overturn protection """
    def forks_down(self):
        try:
            action_start_time = time.time()
            self.forks_active = True
            self.command_publish = self.command_received
            self.command_publish_pub.publish(self.command_publish)
            while self.forks_active:
                if not self.is_stable():
                    rospy.logerr('Forks Controller: Overturning risk detected - stopping action')
                    return False
                if (time.time() - action_start_time) > self.forks_action_timeout:
                    rospy.logerr('FOrks Controller:Action timeout overflow detected!')
                    return False
                if self.safety_stop_action():
                    rospy.logerr('Forks controller: Action stopped by safety systems')
                    return False
                if self.verify_forks_height():
                    return True
        except Exception as e:
            rospy.logerr(f'Error detected in Forks controller at forks up command execute: {e}')
            return False
    
    """ Execute save weight command """
    def save_weight(self):
        self.command_publish = self.command_received
        self.command_publish_pub.publish(self.command_publish)
        return True
    
    """ Main logic for fork control """
    def controller(self):
        sequence_start_time = time.time()
        status = None
        if self.check_for_command_change():
            safety_check_result = self.safety_check()
            if safety_check_result[0] and safety_check_result[1] and safety_check_result[2] and safety_check_result[3] and not safety_check_result[4]:
                if self.verify_received_command():
                    self.set_tilt_tolerance()

                    if self.command_received.lift:
                        status = self.forks_up()
                        self.set_stop_action()
                        self.command_publish_pub.publish(self.command_publish)
                    elif self.command_received.drop():
                        status = self.forks_down()
                        self.set_stop_action()
                        self.command_publish_pub.publish(self.command_publish)
                    elif self.command_received.save_weight():
                        status = self.save_weight()
                        self.set_stop_action()
                        self.command_publish_pub.publish(self.command_publish)
            else:
                if not safety_check_result[0]:
                    rospy.logerr('Fork controller: E-Stop button is pressed.')
                if not safety_check_result[1]:
                    rospy.logerr('Fork controller: Object detected in reduced speed zone')
                if not safety_check_result[2]:
                    rospy.logerr('Fork controller: Object detected in soft stop zone')
                if not safety_check_result[3]:
                    rospy.logerr('Fork controller: Object detected in emergency stop zone')
                if safety_check_result[4]:
                    rospy.logerr('Fork controller: Error detected in safety module. Please check safety device')
        self.action_status_pub.publish(status)
        if status is not None:
            while not self.action_confirmation:
                if self.action_confirmation:
                    break
                time.sleep(0.2)
            if self. action_confirmation:
                status = None
                self.command_received_last = self.command_received
        if self.log_action_time:
            sequence_duration = time.time() - sequence_start_time
            rospy.loginfo(f'Fork controller action time: {sequence_duration} s.')


if __name__ == '__main__':
    try:
        rospy.init_node('forks_controller', log_level=rospy.DEBUG)
        rospy.loginfo('Forks controller started')
        forks_controller = ForksController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in fork controller at initialize: {e}')
    finally:
        rospy.logwarn('Forks controller shuted down')
    
