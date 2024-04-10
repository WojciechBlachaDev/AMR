#!/usr/bin/env python3
import rospy, requests, json, time, math, os, traceback
from std_msgs.msg import Bool
from dhi_amr.msg import visionary_data_out

class SettingsHandler():
    def __init__(self):
        self.default_settings = "visionary_program_options" ,{
            "visionary_ip_address": '192.168.1.10',
            "montage_height": 840.0,
            "montage_offset": 140.0,
            "data_queue": 1,
            "data_latch": True,
            "log_action_time": True
        }

    def check_file(self, file_path):
        return os.path.exists(file_path)

    def create_file(self, file_path):
        try:
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w') as file:
                pass
            rospy.logdebug('Visionary(SettingsHandler) - create file path: File path created')
            return True
        except OSError as e:
            rospy.logfatal(f'Visionary(SettingsHandler) - create file path: Error {e}')
            return False

    def load_settings(self, file_path):
        try:
            if self.check_file(file_path):
                with open(file_path, 'r') as file:
                    settings = json.load(file)
                if "visionary_program_options" in settings:
                    startup_settings = settings["visionary_program_options"]
                    rospy.logdebug('Visionary(SettingsHandler) - load settings: SUCCESS')
                    return startup_settings
                else:
                    rospy.logwarn('Visionary(SettingsHandler) - load settings: startup program options not found. Loading default options and saving to file')
                    self.update_settings(file_path, self.default_settings)
                    return self.default_settings
            else:
                if self.create_file(file_path):
                    rospy.logdebug('Visionary(SettingsHandler) - load settings: settings file not found, creating new one empty file and saving default program settings')
                    self.update_settings(file_path, self.default_settings)
                    return self.default_settings
                else:
                    rospy.logerr('Visionary(SettingsHandler) - load settings: Settings file not found. Creating new file failed - please check log and file permissions. Returning program default settings')
                    return self.default_settings
        except Exception as e:
            rospy.logerr(f'Visionary(SettingsHandler) - load settings: Error detected: {e}')
            rospy.logerr(traceback.format_exc())  # Dodajemy informacje o śladzie stosu
            return self.default_settings

    def update_settings(self, file_path, new_settings):
        try:
            with open(file_path, 'r') as file:
                settings = json.load(file)
            if "visionary_program_options" in settings:
                settings["visionary_program_options"] = new_settings
                with open(file_path, 'w') as file:
                    json.dump(settings, file, indent=4)
                    rospy.logdebug('Visionary(SettingsHandler) - update settings: SUCCESS')
            else:
                rospy.logwarn('Visionary(SettingsHandler) - update settings: No startup_program_options section in JSON file')
        except Exception as e:
            rospy.logerr(f'Visionary(SettingsHandler) - update settings: Error detected: {e}')

class VisionaryConnection:
    def __init__(self):
        """ Variables """
        self.file = 'catkin_ws/src/AMR/settings/amrsettings.json'
        self.settings_handler = SettingsHandler()
        self.settings = self.settings_handler.load_settings(self.file)
        """ Options """
        self.visionary_ip_address = self.settings["visionary_ip_address"]
        self.montage_height = self.settings["montage_height"]
        self.montage_offset = self.settings["montage_offset"]
        self.data_queue = self.settings["data_queue"]
        self.data_latch = self.settings["data_latch"]
        self.log_action_time = self.settings["log_action_time"]

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
            except KeyboardInterrupt as e:
                rospy.logwarn(f'User stop detected in Visionary communication !')
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
                action_duration = (time.time() - action_start_time) * 1000
                rospy.loginfo(f'Communication visionary get visionary data from api action time: {action_duration} ms.')
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
