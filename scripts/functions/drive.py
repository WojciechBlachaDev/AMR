#!/usr/bin/env python3
import rospy, math, time
from std_msgs.msg import Int64, Float32, Bool
from dhi_amr.msg import commands_curtis, commands_servo, flexi_data_out


class DriveController:
    def __init__(self):
        """Variables"""

        """Options"""
        self.curtis_min_pwm = 360
        self.curtis_max_pwm = 2000
        self.servo_angle_limit = 90.0
        self.auto_correct_command = True
        self.curtis_queue = 1
        self.servo_queue = 1
        self.curtis_latch = True
        self.servo_latch = True
        self.log_action_duration = True
        self.tilt_tolerance_axis1 = 5.0
        self.tilt_tolerance_axis2 = 5.0

        """Custom"""
        self.curtis = commands_curtis()
        self.servo = commands_servo()
        self.safety_data = flexi_data_out()

        """Others"""
        self.curtis_request = 0
        self.servo_request = 0.0
        self.lidars_ok = False
        self.axis1_min_tilt = 0.0
        self.axis1_max_tilt = 0.0
        self.axis2_min_tilt = 0.0
        self.axis2_max_tilt = 0.0
        self.servo_power = 1000
        self.servo_off = 2000

        try:
            """ ROS Subscriber's definitions """
            curtis_request_sub = rospy.Subscriber('amr/steering/curtis', Int64, self.curtis_request_callback)
            servo_request_sub = rospy.Subscriber('amr/steering/servo', Float32, self.servo_request_callback)
            safety_ok_sub = rospy.Subscriber('amr/safety/lidars_ok', Bool, self.lidars_ok_callback)
            safety_data_sub = rospy.Subscriber('amr/safety', flexi_data_out, self.safety_data_callback)

            """ ROS Publisher's definitions """
            self.servo_pub = rospy.Publisher('amr/plc/commands/servo', commands_servo, queue_size=self.servo_queue, latch=self.servo_latch)
            self.curtis_pub = rospy.Publisher('amr/plc/commands/curtis', commands_curtis, queue_size=self.curtis_queue, latch=self.curtis_latch)
        except Exception as e:
            rospy.logerr(f'Error detected in drive controller at init(self): {e}')
    
    """ Callback for curtis request subscriber """
    def curtis_request_callback(self, msg):
        self.curtis_request = msg.data
        curtis_start_time = time.time()
        self.curtis_move(self.curtis_request)
        if self.log_action_duration:
            curtis_duration = time.time() - curtis_start_time
            rospy.loginfo(f'Drive controller: Curtis movement action time: {curtis_duration} s.')
    
    """ Callback for servo request subscriber """
    def servo_request_callback(self, msg):
        self.servo_request = msg.data
        servo_start_time = time.time()
        self.servo_move(self.servo_request)
        if self.log_action_duration:
            servo_duration = time.time() - servo_start_time
            rospy.loginfo(f'Drive controller: Servo movement action time: {servo_start_time} s.')

    """ Callback for lidars_ok signal subscriber"""
    def lidars_ok_callback(self, msg):
        self.lidars_ok = msg.data
    
    """ Callback for safety data subscriber """
    def safety_data_callback(self, msg):
        self.safety_data = msg

    """ Check for lidars soft stop zone """
    def soft_stop_check(self):
        if self.safety_data.left_scanner.soft_stop_zone_status and self.safety_data.right_scanner.soft_stop_zone_status:
            return True
        return False
    
    """ Check for lidars emergency stop zone"""
    def emergency_stop_check(self):
        if self.safety_data.left_scanner.emergency_stop_zone_status and self.safety_data.right_scanner.emergency_stop_zone_status:
            return True
        return False
    
    """ Check for emergency stop button status"""
    def emergency_buttons_check(self):
        if self.safety_data.left_emergency_stop_button and self.safety_data.right_emergency_stop_button:
            return True
        return False
    
    """ Check is forklift safe to move """
    def is_forklift_safe(self):
        if not self.lidars_ok and self.emergency_buttons_check() and self.emergency_stop_check() and self.soft_stop_check():
            return True
        return False
    
    """ Validate and eventually reduce requested turn angle of servomotor"""
    def validate_servo_angle(self, angle):
        if angle > self.servo_angle_limit:
            return self.servo_angle_limit
        if angle < -self.servo_angle_limit:
            return -self.servo_angle_limit
        if -self.servo_angle_limit < angle < self.servo_angle_limit:
            return angle
        return 9999
    
    """ Converts angle for servo """
    def convert_angle(self, angle):
        if angle < 0:
            angle = -angle
        return math.trunc(round(angle, 2) * 100)
    """ Enable servo power """
    def enable_servo(self):
        self.servo.power = True
        self.servo_pub.publish(self.servo)
    
    """ Disable servo power"""
    def disable_servo(self):
        self.servo.power = False
        self.servo_pub.publish(self.servo)
    
    """ Set servo direction value for PLC"""
    def set_servo_direction(self, angle):
        if angle >= 0.0:
            return 2
        if angle < 0.0:
            return 1
    
    """ Set curtis direction from pwm value """
    def set_curtis_direction(self, pwm):
        if pwm > self.curtis_min_pwm:
            self.curtis.forward = True
            self.curtis.backward = False
        elif pwm < -self.curtis_min_pwm:
            self.curtis.forward = False
            self.curtis.backward = True
        else:
            self.curtis.forward = False
            self.curtis.backward = False
    
    """ safety stop servo function"""
    def servo_safety_stop(self):
        self.disable_servo()
        self.servo.angle = 0
        self.servo.direction = 0
        self.servo_pub.publish(self.servo)
    
    """ Safety stop for curtis drive"""
    def curtis_safety_stop(self):
        self.curtis.forward = False
        self.curtis.backward = False
        self.curtis.pwm = 0
        self.curtis_pub.publish(self.curtis)

    """ Check if requested pwm is not above limit. If its return limit."""
    def pwm_validation(self, pwm):
        if pwm > self.curtis_max_pwm:
            return self.curtis_max_pwm
        if pwm > self.curtis_min_pwm:
            return pwm
        return 0
    
    """ Converts pwm value from ros to PLC value"""
    def pwm_converter(self, pwm):
        if pwm < 0:
            return -pwm
        return pwm
    
    """ Execute servo move method """
    def servo_move(self, requested_angle):
        print(requested_angle)
        if self.is_forklift_safe():
            if requested_angle != self.servo_power and requested_angle != self.servo_off:
                angle = self.validate_servo_angle(requested_angle)
                print(angle)
                if angle != 9999:
                    self.servo.direction = self.set_servo_direction(angle)
                    
                    self.servo.angle = self.convert_angle(angle)
                    self.servo.power = True
                    print(self.servo)
                    self.servo_pub.publish(self.servo)
            if requested_angle == self.servo_power:
                self.enable_servo()
            if requested_angle == self.servo_off:
                self.disable_servo()
        else:
            self.servo_safety_stop()

    """Execute curtis move method"""
    def curtis_move(self, requested_pwm):
        if self.is_forklift_safe():
            self.set_curtis_direction(requested_pwm)
            pwm = self.pwm_converter(requested_pwm)
            self.curtis.pwm = self.pwm_validation(pwm)
            self.curtis_pub.publish(self.curtis)
        else:
            self.curtis_safety_stop()

if __name__ == '__main__':
    try:
        rospy.init_node('drive_controller', log_level=rospy.DEBUG)
        rospy.loginfo('Drive controller started.')
        drive_controller = DriveController()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Exception detected in drive controller at initialize: {e}')
    finally:
        rospy.logwarn('Drive controller shuted down!!!')
