#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float32
from dhi_amr.msg import plc_data_out, encoder_data


class EncoderData:
    def __init__(self):
        """ Variables """
        """ Options """
        self.log_action_duration = True
        self.data_queue = 10
        self.data_latch = True
        
        """ Custom """
        self.data = encoder_data()
        self.plc_data = plc_data_out()
        
        """ Other """
        self.last_encoder_position = 0.0
        self.service_startup = True
        self.distance_m = 0.0
        self.distance_km = 0.0
        self.int32_max_value = 2147483647
        self.int32_max_value = 10000
        """ Main loop """
        
        # while not rospy.is_shutdown():
        try:
            """ ROS Subscriber's declarations"""
            plc_data_sub = rospy.Subscriber('amr/plc/data_out', plc_data_out, self.plc_data_callback)
                
            """ ROS Publisher's declarations"""
            self.data_pub = rospy.Publisher('amr/encoder', encoder_data, queue_size=self.data_queue, latch=self.data_latch)
            self.speed_to_base_controller_pub = rospy.Publisher('amr/base_controller/data/speed', Float32, queue_size=1, latch=True)
        except Exception as e:
            rospy.logerr(f'Error detected in encoder data converter service at main loop: {e}')

    def plc_data_callback(self, msg):
        action_start_time = time.time()
        self.plc_data = msg
        self.convert_speed_data()
        self.create_distance_data()
        self.data_pub.publish(self.data)
        self.speed_to_base_controller_pub.publish(self.data.speed)
        if self.log_action_duration:
            action_duration = time.time() - action_start_time
            rospy.loginfo(f'Encoder data converter action duration time: {action_duration}')

    """ Converting received encoder about speed and direction into ROS speed data"""
    def convert_speed_data(self):
        if self.plc_data.speed_direction == 1 or self.plc_data.speed_direction == 0:
            self.data.speed = round((self.plc_data.speed_value / 100), 2)
        elif self.plc_data.speed_direction == 2:
            self.data.speed = -round((self.plc_data.speed_value / 100), 2)
            if self.data.speed == -0.0:
                self.data.speed = 0.0
        else:
            rospy.logerr(f'Error detected in encoder data converter at convert speed data. Wrong speed direction from PLC: {self.plc_data.speed_direction} ')
        if self.data.speed == 0.0:
            self.data.standstill = True
            self.data.direction_forward = False
            self.data.direction_backward = False
        else:
            self.data.standstill = False
        if self.data.speed > 0.0:
            self.data.direction_forward = True
            self.data.direction_backward = False
        if self.data.speed < 0.0:
            self.data.direction_forward = False
            self.data.direction_backward = True
    
    """ Creating distance data from encoder position in mm """
    def create_distance_data(self):
        if self.service_startup:
            self.last_encoder_position = self.plc_data.position
            self.service_startup = False
        position_difference = self.last_encoder_position - self.plc_data.position
        print(position_difference)
        if position_difference < 0:
            position_difference = -position_difference
        self.data.current_position = self.plc_data.position
        if self.int32_max_value <= (self.data.distance_mm + position_difference):
            difference_to_value = self.int32_max_value - self.data.distance_mm
            position_difference = position_difference - difference_to_value
            self.data.distance_mm = position_difference
        else:
            self.data.distance_mm = self.data.distance_mm + position_difference
        self.distance_m += position_difference / 1000
        self.data.distance_m = round(self.distance_m, 2)
        self.distance_km += position_difference / 1000000
        self.data.distance_km = round(self.distance_km, 3)
        self.last_encoder_position = self.plc_data.position
        
if __name__ == '__main__':
    try:
        rospy.init_node('encoder_converter', log_level=rospy.DEBUG)
        rospy.loginfo('Encoder converting service started!')
        encoder = EncoderData()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in encoder converting service at initialize: {e}')
    finally:
        rospy.logwarn('Encoder converting service shuted down!!')
        
        