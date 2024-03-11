#!/usr/bin/env python3
import time, rospy
from dhi_amr.msg import plc_data_out

rospy.init_node('test')
data = plc_data_out()
data.position = 20

while not rospy.is_shutdown():
    plc_pub = rospy.Publisher('amr/plc/data_out', plc_data_out, queue_size=1, latch=False)
    plc_pub.publish(data)
    if data.position == 10:
        data.position = 20
    else:
        data.position = 10
    