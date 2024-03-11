#!/usr/bin/env python3
import rospy, actionlib, tf_conversions, time, math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from dhi_amr.msg import point

class PointConverter:
    def __init__(self):
        """Variables"""

        """Custom"""
        self.point_received = point()
        self.point_last = point()

        """Other"""
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.current_goal = MoveBaseGoal()
        self.start = False
        self.cancel = False

        """Main program loop"""
        while not rospy.is_shutdown():
            try:
                """ROS Subscriber's definitions"""
                start_sub = rospy.Subscriber('amr/steering/nav_ride/start', Bool, self.start_callback)
                cancel_sub = rospy.Subscriber('amr/steering/nav_ride/cancel', Bool, self.cancel_callback)
                point_received_sub = rospy.Subscriber('amr/steering/nav_ride/point', point, self.point_received_callback)

                """ROS Publisher's definitions"""
                self.goal_status_pub = rospy.Publisher('amr/status/nav_ride/goal_status', Bool, queue_size=1, latch=True)
                self.start_confirmation_pub = rospy.Publisher('amr/status/nav_ride/start', Bool, queue_size=1, latch=True)
                self.cancel_confirmation_pub = rospy.Publisher('amr/status/navride/cancel', Bool, queue_size=1, latch=True)
            except Exception as e:
                rospy.logerr(f'Error detected in point converter at main program loop: {e}')
            self.logic()

    """ callback for point subscriber """
    def point_received_callback(self, msg):
        self.point_received = msg
    
    """ Callback for start subscriber """
    def start_callback(self, msg):
        self.start = msg.data
    
    """ Callback for cancel subscriber """
    def cancel_callback(self, msg):
        self.cancel = msg.data
    
    """ Normalize angle function for converting -180 - 180 degrees to 0 - 360 degrees"""
    def normalize_angle(self, angle):
        result = angle % 360
        if result > 0:
            result += 360
        return result

    """ Move base goal creating from x, y, rotation"""
    def create_goal(self):
        self.current_goal.target_pose.header.frame_id = 'map'
        self.current_goal.target_pose.header.stamp = rospy.Time.now()
        self.current_goal.target_pose.pose.position.x = self.point_received.point_x
        self.current_goal.target_pose.pose.position.y = self.point_received.point_y
        quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, math.radians((180 + self.point_received.point_r)))
        self.current_goal.target_pose.pose.orientation.x = quaternion[0]
        self.current_goal.target_pose.pose.orientation.y = quaternion[1]
        self.current_goal.target_pose.pose.orientation.z = quaternion[2]
        self.current_goal.target_pose.pose.orientation.w = quaternion[3]
    
    """ Sending goal to navigation package trough move_base"""
    def send_goal(self):
        self.client.wait_for_server()
        if self.client.get_state() != actionlib.GoalStatus.ACTIVE:
            self.client.send_goal(self.current_goal)
        while True:
            if self.cancel:
                if self.client.get_state() == actionlib.GoalStatus.ACTIVE or self.client.get_state() == actionlib.GoalStatus.PENDING:
                    self.client.cancel_goal()
                    if self.client.get_state() == actionlib.GoalStatus.PREEMPTED:
                        self.cancel_confirmation_pub.publish(True)
                        while True:
                            if not self.cancel:
                                self.cancel_confirmation_pub.publish(False)
                                break
                            time.sleep(0.1)
                            break
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal reached')
                self.goal_status_pub.publish(True)
                break
                    
    """ Main service logic """
    def logic(self):
        if self.start:
            self.start_confirmation_pub.publish(True)
            while self.start:
                if not self.start:
                    break
                time.sleep(0.1)
            self.create_goal()
            self.send_goal()


if __name__ == '__main__':
    try:
        rospy.init_node('point_converter')
        rospy.loginfo('Point converter service started')
        point_converter = PointConverter()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in point converter at initialize: {e}')
    finally:
        rospy.logwarn('Point converter service shuted down!')
