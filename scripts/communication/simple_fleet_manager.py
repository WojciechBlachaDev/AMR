#!/usr/bin/env python3
import rospy, time, netifaces, signal, math, socket
from socket import socket as sock
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int64, Float32
from dhi_amr.msg import simple_in, simple_out
from dhi_amr.msg import simple_commands, task_data
from dhi_amr.msg import simple_confirmations, encoder_data, ethernet_devices_status
from dhi_amr.msg import flexi_data_out, log_messages, plc_data_out
from dhi_amr.msg import scangrid, sensors_data, task_data, teb_config
from dhi_amr.msg import visionary_data_out, workstate_read


""" Server setting class """
class Server:

    """ Server initialization """
    def __init__(self, host, port):
        try:
            self.server_socket = sock(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(host, port)
            self.server_socket.listen(5)
            rospy.logdebug(f'AMR_SERVER: Server started now!')
        except (Exception, socket.error) as e:
            if e.errno == socket.errno.EADDRINUSE:
                rospy.logwarn(f'Port or address already in use: {e}. Retrying server startup after 5 seconds..')
            else:
                rospy.logfatal(f'AMR SERVER: Server startup failed: {e}')

    """ Accepting client method """
    def accept_connection(self):
        client = None
        try:
            client, addr = self.server_socket.accept()
            rospy.loginfo(f'AMR SERVER: Connection accepted from -  {addr[0]}:{addr[1]}')
        except socket.error as e:
            if e.errno == socket.errno.EINVAL:
                rospy.logwarn(f'AMR SERVER: Invalid argument while connecting client to server: {e}. Ignoring and continuing now....')
            else:
                rospy.logerr(f'AMR SERVER: Error while accepting connection from client: {e}')
        return client

    """ Server closing method """
    def close(self):
        self.server_socket.close()

""" Data exchange handling class """
class ConnectionHandler:

    """ Initialization  """
    def __init__(self):
        """ Variables """
        """ options """
        self.server_startup = True
        self.interface_name = 'wlp0s20f3'
        self.server_address = self.get_my_ip(self.interface_name)
        self.server_port = 2137
        self.server_refresh_rate = 30
        """ Connection """
        self.client = sock(socket.AF_INET, socket.SOCK_STREAM)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server = Server(self.server_address, self.server_port)

        """ Others """
        self.rate = rospy.Rate(self.server_refresh_rate)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTSTP, self.signal_handler)

        """ Data """
        self.actual_pose = Pose2D()
        self.sensors = sensors_data()
        self.pwm = 0
        self.encoder = encoder_data()
        self.safety = flexi_data_out()
        self.scangrids = scangrid()
        self.plc = plc_data_out()
        self.ethernet = ethernet_devices_status()
        self.config_teb_actual = teb_config()
        self.task_actual = task_data()
        self.data_in = simple_in()

        """ Main connection handler loop """
        while not rospy.is_shutdown():
            try:
                """ Get client if server is running """
                if self.server_startup:
                    rospy.logdebug('AMR SERVER: Trying to accept client connection...')
                    self.client = self.server.accept_connection()
                    if self.client is not None:
                        self.server_startup = False
                actual_pose_sub = rospy.Subscriber('amr/odom_pub/sickLidarPose2D', Pose2D, self.actual_pose_callback)
                sensors_sub = rospy.Subscriber()

    def get_my_ip(self, interface_name):
        try:
            interfaces = netifaces.interfaces()
            if interface_name in interfaces:
                addresses = netifaces.ifaddresses(interface_name)
                if netifaces.AF_INET in addresses:
                    return addresses[netifaces.AF_INET][0]['addr']
                else:
                    return "No ip address for selected interface"
            else:
                return "Selected interface does not exists"
        except Exception as e:
            return f"Error detected in get_my_ip method: {e}"

if __name__ == '__main__':
    try:
        rospy.init_node('simple_fleet_manager')
        connection_handler = ConnectionHandler()
        rospy.spin()
    except Exception as e:
        rospy.logfatal(f'AMR SERVER: Error detected in main: {e}')
    finally:
        rospy.logfatal('AMR SERVER: SHUTTING DOWN')