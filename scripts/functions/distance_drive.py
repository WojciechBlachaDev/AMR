#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float32, Int64
from dhi_amr.msg import commands_distance_drive as cdd
from dhi_amr.msg import sensors_data, encoder_data


class DistanceDrive:
    def __init__(self):
        """ Variables """
        """ Options """
        self.curtis_queue = 1
        self.curtis_latch = True
        self.servo_queue = 1
        self.servo_latch = True
        self.log_action_duration = True
        self.servo_tolerance = 0.1
        self.servo_timeout = 5.0

        """Custom"""
        self.command = cdd()
        self.sensors = sensors_data()
        self.encoder = encoder_data()

        """ Others """

        while not rospy.is_shutdown():
            try:
                """ ROS Subscriber's definitions """
                command_sub = rospy.Subscriber('amr/steering/distance_drive', cdd, self,command_callback)
                encoder_sub = rospy.Subscriber('', encoder_data, self.encoder_callback)
                sensors_sub = rospy.Subscriber('amr/plc/data/sensors_converted', sensors_data, self.sensors_callback)
                """ ROS Publisher's definitions """
                self.servo_pub = rospy.Publisher('amr/steering/servo_request', Float32, queue_size=self.servo_queue, latch = self.servo_latch)
                self.curtis_pub = rospy.Publisher('amr/steering/curtis_request', Int64, queue_size=self.curtis_queue, latch=self.curtis_latch)
            except Exception as e:
                rospy.logerr(f'Error detected in distance drive at init: {e}')







if __name__ == '__main__':
    try:
        rospy.init_node('distance_drive')
        rospy.loginfo('Distance drive function started')
        distance_drive = DistanceDrive()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in distance drive at main: {e}')
    finally:
        rospy.logwarn('Distance drive function shuted down!')