#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Int64, Float32
from dhi_amr.msg import encoder_data, flexi_data_out, sensors_data

class Testemergency:
    def __init__(self):
        self.encoder = encoder_data()
        self.safety = flexi_data_out()
        self.sensors = sensors_data()
        self.pwm = 0
        self.angle = 0.0
        self.pwm_step_up = 20
        self.pwm_step_down = 10
        self.speed_limit = 0.3
        self.startup = True
        self.rate = rospy.Rate(10)
        rospy.loginfo('Test angle setted!')
        time.sleep(3.0)
        while not rospy.is_shutdown():
            try:
                safety_sub = rospy.Subscriber('amr/safety', flexi_data_out, self.safety_callback)
                encoder_sub = rospy.Subscriber('amr/encoder', encoder_data, self.encoder_callback)
                sensors_sub = rospy.Subscriber('amr/plc/sensors_converted', sensors_data, self.sensors_callback) 
                self.pwm_pub = rospy.Publisher('amr/steering/curtis', Int64, queue_size=1)
                self.angle_pub = rospy.Publisher('amr/steering/servo', Float32, queue_size=1)
            
            except Exception as e:
                rospy.logerr(f'Exception at init: {e}')
            if self.startup:
                self.set_angle()
                time.sleep(3.0)
                self.startup = False
            self.set_pwm()
            self.pwm_pub.publish(self.pwm)
            self.rate.sleep()
    def encoder_callback(self, msg):
        self.encoder = msg

    def safety_callback(self, msg):
        self.safety = msg

    def sensors_callback(self, msg):
        self.sensors = msg

    def set_pwm(self):
        if self.encoder.speed < self.speed_limit:
            self.pwm = self.pwm + self.pwm_step_up
            if self.pwm > 3999:
                self.pwm = 3999
        elif self.encoder.speed > self.speed_limit:
            self.pwm = self.pwm - self.pwm_step_down
            self.pwm_step_up = 10
            if self.pwm <= 350:
                self.pwm = 350
        else:
            self.pwm = self.pwm
        if not self.safety.left_scanner.emergency_stop_zone_status or not self.safety.left_scanner.soft_stop_zone_status or not self.safety.right_scanner.emergency_stop_zone_status or not self.safety.right_scanner.soft_stop_zone_status:
            self.pwm = 0
        

    def set_angle(self):
        self.angle_pub.publish(self.angle)
        while True:
            if ((self.angle - 0.1) < self.sensors.servo_angle < (self.angle + 0.1)):
                break

if __name__ == '__main__':
    try:
        rospy.init_node('emergency_brake_test')
        test = Testemergency()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Eception at main: {e}')
    finally:
        rospy.logwarn('Test shutdown!')