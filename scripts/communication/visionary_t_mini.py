#!/usr/bin/env python3
import rospy, requests, json, time, math
from std_msgs.msg import Bool
from dhi_amr.msg import visionary_data_out


class VisionaryConnection:
    def __init__(self):
        """ Variables """

        """ Options """
        self.visionary_ip_address = '192.168.1.10'
        self.montage_height = 840.0
        self.montage_offset = 140.0
        self.data_queue = 1
        self.data_latch = True
        self.log_action_time = True

        """ Others """
        self.get_data = False
        self.api_link = self.create_api_link(self.visionary_ip_address)

        """ Custom """
        self.data = visionary_data_out()

        """ Main program loop """
        while not rospy.is_shutdown():
            try:
                """ ROS Subscriber's definitions"""
                get_data_sub = rospy.Subscriber('amr/visionary/get_data', Bool, self.get_data_callback)

                """ ROS Publisher's definitions """
                self.data_pub = rospy.Publisher('amr/visionary/data', visionary_data_out, queue_size=self.data_queue, latch=self.data_latch)
            except Exception as e:
                rospy.logerr(f'Error detected in Visionary communication at main loop: {e}')
            rospy.loginfo_once(self.api_link)

    """ ROS topic's callbacks """
    def get_data_callback(self, msg):
        self.get_data = msg.data
    
    """ Creating api link from ip address """
    def create_api_link(self, ip_address):
        api_link = 'http://' + ip_address + '/api/detectionResult'
        return api_link
    
    """ Estimating distance to palette with taking into account the camera installation data """
    def estimate_distance(self):
        result = (math.sqrt(self.data.center_point_z**2 - self.montage_height**2)) + self.montage_offset
        return result

    """ Publishing data to ROS topic's"""
    def publish_data_to_ros(self):
        self.data_pub.publish(self.data)

    """ Get data from camera """
    def get_data_from_api(self):
        try:
            action_start_time = time.time()
            raw_data = requests.get(self.api_link)
            raw_data.raise_for_status()
            json_data = json.loads(raw_data.text)
            data = json_data.get('data', {}).get('detectionResult', {})
            left_pocekt = data.get('leftPocket', {})
            center = data.get('centerPoint', {})
            right_pocket = data.get('rightPocket', {})
            self.data.palette_found = bool(data.get('paletteFound', False))
            self.data.angle = float(data.get('Angle', 0))
            self.data.left_pocket_x = float(left_pocekt.get('X', 0))
            self.data.left_pocket_y = float(left_pocekt.get('Y', 0))
            self.data.left_pocket_z = float(left_pocekt.get('Z', 0))
            self.data.center_point_x = float(center.get('X', 0))
            self.data.center_point_y = float(center.get('Y', 0))
            self.data.center_point_z = float(center.get('Z', 0))
            self.data.right_pocket_x = float(right_pocket.get('X', 0))
            self.data.right_pocket_y = float(right_pocket.get('Y', 0))
            self.data.right_pocket_z = float(right_pocket.get('Z', 0))
            if self.data.palette_found:
                self.data.estimated_distance = self.estimate_distance()
                self.publish_data_to_ros()
            if self.log_action_time:
                action_duration = time.time() - action_start_time
                rospy.loginfo(f'Communication visionary get visionary data from api action time: {action_duration} s.')
        except Exception as e:
            rospy.logerr(f'Error detected in Visionary communication at getting data from api: {e}')

if __name__ == '__main__':
    try:
        rospy.init_node('visionary_communication', log_level=rospy.DEBUG)
        rospy.loginfo('Visionary communication started!')
        visionary = VisionaryConnection()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in Visionary communication at initialize: {e}')
    finally:
        rospy.logwarn('Visionary communication shuted down!!!')
