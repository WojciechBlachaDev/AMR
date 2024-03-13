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
        self.goal_was_canceled = False

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
    
    """ Checking is received goal is a new goal """
    def check_is_new_goal(self):
        check_x = self.point_received.point_x != self.point_last.point_x
        check_y = self.point_received.point_y != self.point_last.point_y
        check_r = self.point_received.point_r != self.point_last.point_r
        if check_x and check_y and check_r:
            return False
        else:
            self.point_last = self.point_received
            return True

    """ Sending goal to navigation package trough move_base"""
    def send_goal(self):
        server_check = self.client.wait_for_server(timeout=rospy.Duration(10))
        if self.client.get_state() != actionlib.GoalStatus.ACTIVE:
            self.client.send_goal(self.current_goal)
        if not server_check:
            print('server error')
        while True:
            while True:
                if self.client.get_state() == actionlib.GoalStatus.ACTIVE:
                    rospy.logwarn("Goal was accepted")
                    break
            while self.cancel:
                rospy.loginfo_once('Goal cancel')
                try:
                    self.client.cancel_goal()
                    if self.client.get_state() == actionlib.GoalStatus.PREEMPTED:
                        self.cancel_confirmation_pub.publish(True)
                        while True:
                            if not self.cancel:
                                rospy.loginfo('Confirmed goal cancel')
                                self.cancel_confirmation_pub.publish(False)
                                self.goal_was_canceled = True
                                break
                            time.sleep(0.1)
                        break
                except Exception as e:
                    rospy.logerr(f'Cancel goal exception: {e}')
                            
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Goal reached')
                self.goal_status_pub.publish(True)
                break
            if self.goal_was_canceled:
                self.goal_was_canceled = False
                break
                    
    """ Main service logic """
    def logic(self):
        if self.start:
            rospy.loginfo('Point to goal: start command received')
            self.start_confirmation_pub.publish(True)
            while self.start:
                if not self.start:
                    rospy.loginfo('Point to goal: Start confirmation readed')
                    break
                time.sleep(0.1)
            if self.check_is_new_goal:
                rospy.loginfo('Point to goal: new goal')
                self.create_goal()
                self.send_goal()
            else:
                rospy.loginfo(f'Received point {self.point_received} == {self.point_last}')


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
