#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float32
from dhi_amr.msg import sensors_data, plc_data_out


class SensorsData:
    def __init__(self):
        """ Variables """
        
        """ Options """
        self.data_queue = 1
        self.data_latch = True
        self.critical_battery_percentage = 20.0
        self.log_action_duration = True
        """ Custom messages """
        self.data = sensors_data()
        self.plc_data = plc_data_out()
        
        try:
            """ ROS Subscriber's declarations"""
            plc_data_sub = rospy.Subscriber('amr/plc/data_out', plc_data_out, self.plc_data_callback)
            
            """ ROS Publisher's declarations"""
            self.data_pub = rospy.Publisher('amr/plc/sensors_converted', sensors_data, queue_size=self.data_queue, latch=self.data_latch)
            self.angle_to_base_controller_pub = rospy.Publisher('amr/base_controller/data/angle', Float32, queue_size=1, latch=True)
        except Exception as e:
            rospy.logwarn(f'Error detected in sensors data converter service at main loop: {e}')
        except KeyboardInterrupt as e:
                rospy.logwarn(f'User stop detected in sensors converter !')
            
    def plc_data_callback(self, msg):
        action_start_time = time.time()
        self.plc_data = msg
        self.convert_data()
        self.data.forks_height = self.plc_data.actual_forks_height
        self.data.weight = self.plc_data.weight
        self.data.tilt_axis_1 = self.plc_data.tilt_axis_1
        self.data.tilt_axis_2 = self.plc_data.tilt_axis_2
        self.data.weight_saved = self.plc_data.weight_saved
        self.data.forks_height_limiter = self.plc_data.forks_height_limiter
        self.data_pub.publish(self.data)
        self.angle_to_base_controller_pub.publish(self.data.servo_angle)
        if self.log_action_duration:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'Sensors data converter action duration time: {action_duration}')
    
    def convert_data(self):
        self.set_battery_info()
        self.set_steering_angle_readings()
        
    """ Converting PLC readings into battery info """
    def set_battery_info(self):
        self.data.battery_voltage = (self.plc_data.battery_voltage / 100)
        self.data.battery_percentage = round((100 - ((28.0 - self.data.battery_voltage) / 0.0743)), 2)
        if self.data.battery_percentage > 100.0:
            self.data.battery_percentage = 100.0
        if self.data.battery_percentage <= 0.0:
            self.data.battery_percentage = 0.0
        if self.data.battery_percentage <= self.critical_battery_percentage:
            self.data.battery_critical = True
        else:
            self.data.battery_critical = False
    
    """ Converting PLC readings of direction and angle as steering info"""
    def set_steering_angle_readings(self):
        if self.plc_data.steering_direction == 1:
            self.data.servo_angle = -(self.plc_data.steering_angle / 100)
            if self.data.servo_angle == -0.0:
                self.data.servo_angle = 0.0
        if self.plc_data.steering_direction == 2:
            self.data.servo_angle = self.plc_data.steering_angle / 100
            
if __name__ == '__main__':
    try:
        rospy.init_node('sensors_converter', log_level=rospy.DEBUG)
        rospy.loginfo('Sensors converting service started!')
        sensors = SensorsData()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in sensors data converter service at initialize: {e}')
    finally:
        rospy.logwarn('Sensors converting service shuted down :(')
    