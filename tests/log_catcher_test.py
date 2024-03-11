#!/usr/bin/env python3
import time, rospy
from dhi_amr.msg import plc_data_out

rospy.init_node('test', log_level=rospy.DEBUG)
data = plc_data_out()
data.position = 20

while not rospy.is_shutdown():
    rospy.logdebug('DEBUGGING')
    time.sleep(0.1)
    rospy.loginfo("INFORMING")
    time.sleep(0.1)
    rospy.logwarn("WARNING")
    time.sleep(0.1)
    rospy.logerr("ERROR")
    time.sleep(0.1)
    rospy.logfatal("FATAL LAST LOG")
    