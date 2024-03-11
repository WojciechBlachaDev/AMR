#!/usr/bin/env python3
import time, subprocess, rospy
from dhi_amr.msg import ethernet_devices_status as eds


class EthernetTester:
    def __init__(self):
        """Variables"""
        """Options"""
        self.lidarloc_ip = '192.168.1.1'
        self.plc_ip = '192.168.1.4'
        self.visionary_ip = '192.168.1.10'
        self.gmod_ip = '192.168.1.11'
        self.lan_gateway = '192.168.1.25'
        self.lidar_left_ip = '192.168.1.30'
        self.lidar_right_ip = '192.168.1.32'
        self.gepr_ip = '192.168.1.34'
        self.wifi_gateway = '192.168.2.25'
        self.server_ip = '192.168.2.60'

        """Custom"""
        self.status = eds()

        """Other"""
        self.responce_all = 1
        self.responces_list = []
        self.retry_counter = 0

        """ROS Publisher definition"""
        self.status_pub = rospy.Publisher('amr/tests/ethernet_devices', eds, queue_size=1, latch=True)

        while not self.check_results():
            self.test()
            if self.retry_counter == 3:
                break
            if self.check_results():
                break
            else:
                self.retry_counter += 1
        self.status_pub.publish(self.status)


    """ Overal test result check """
    def check_results(self):
        if self.status.lidarloc_is_avaible and self.status.plc_is_avaible and self.status.visionary_is_avaible and self.status.safety_modbus_is_avaible and self.status.microscan3_left_is_avaible and self.status.microscan3_right_is_avaible and self.status.safety_ethernet_is_avaible and self.status.lan_gateway_is_avaible and self.status.wifi_gateway_is_avaible and self.status.server_is_avaible:
            return True
        return False
    
    """ Converting responce data to bool value"""
    def responce_to_bool(self, responce):
        if responce == 0:
            return True
        return False
    
    """ Converting bool list to ros statuses"""
    def convert_test_to_status(self):
        results = []
        try:
            for i in range(0, len(self.responces_list)):
                results.append(self.responce_to_bool(self.responces_list[i]))
            self.status.lidarloc_is_avaible = results[0]
            self.status.plc_is_avaible = results[1]
            self.status.visionary_is_avaible = results[2]
            self.status.safety_modbus_is_avaible = results[3]
            self.status.microscan3_left_is_avaible = results[4]
            self.status.microscan3_right_is_avaible = results[5]
            self.status.safety_ethernet_is_avaible = results[6]
            self.status.lan_gateway_is_avaible = results[7]
            self.status.wifi_gateway_is_avaible = results[8]
            self.status.server_is_avaible = results[9]
        except Exception as e:
            rospy.logerr(f'Error detected in Ethernet tester at converting test to status: {e}')
    
    """Ethernet avaible test"""
    def test(self):
        self.responces_list.clear()
        rospy.loginfo('Starting avability test of LidarLoc PC:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.lidarloc_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of PLC:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.plc_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of Visionary 3D camera:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.visionary_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of FX CPU Modbus Gateway:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.gmod_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of left lidar scanner:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.lidar_left_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of right lidar scanner:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.lidar_right_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of FX CPU ethernet gateway:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.gepr_ip]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of Local network microtik gateway:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.lan_gateway]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of Wi-Fi network microtik gateway:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.wifi_gateway]))
        time.sleep(0.1)
        rospy.loginfo('Starting avability test of server PC:')
        self.responces_list.append(subprocess.call(['ping', '-c', '3', self.server_ip]))
        time.sleep(0.1)
        self.convert_test_to_status()


if __name__ == '__main__':
    try:
        rospy.init_node('ethernet_tester', log_level=rospy.DEBUG)
        rospy.loginfo('Ethernet tester started')
        test = EthernetTester()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in Ethernet tester at initialize: {e}')
    finally:
        rospy.logwarn('Ethernet tester shuted down')