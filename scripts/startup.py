#!/usr/bin/env python3
import rospy, subprocess, time, os, json, traceback 
from dhi_amr.msg import workstate_read, workstate_request, simple_in, task_data

class SettingsHandler():
    def __init__(self):
        self.default_settings = "startup_progrma_options" ,{
            "process_options_timeout": 10.0,
            "process_options_max_retries": 3,
            "process_options_interval": 1.0,
            "log_action_time": True,
            "workstates_requests_queue_size": 1,
            "workstates_requests_latch": True,
            "refresh_rate": 10
        }

    def check_file(self, file_path):
        return os.path.exists(file_path)

    def create_file(self, file_path):
        try:
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w') as file:
                pass
            rospy.logdebug('AMR(SettingsHandler) - create file path: File path created')
            return True
        except OSError as e:
            rospy.logfatal(f'AMR(SettingsHandler) - create file path: Error {e}')
            return False

    def load_settings(self, file_path):
        try:
            if self.check_file(file_path):
                with open(file_path, 'r') as file:
                    settings = json.load(file)
                if "startup_program_options" in settings:
                    startup_settings = settings["startup_program_options"]
                    rospy.logdebug('AMR(SettingsHandler) - load settings: SUCCESS')
                    return startup_settings
                else:
                    rospy.logwarn('AMR(SettingsHandler) - load settings: startup program options not found. Loading default options and saving to file')
                    self.update_settings(file_path, self.default_settings)
                    return self.default_settings
            else:
                if self.create_file(file_path):
                    rospy.logdebug('AMR(SettingsHandler) - load settings: settings file not found, creating new one empty file and saving default program settings')
                    self.update_settings(file_path, self.default_settings)
                    return self.default_settings
                else:
                    rospy.logerr('AMR(SettingsHandler) - load settings: Settings file not found. Creating new file failed - please check log and file permissions. Returning program default settings')
                    return self.default_settings
        except Exception as e:
            rospy.logerr(f'AMR(SettingsHandler) - load settings: Error detected: {e}')
            rospy.logerr(traceback.format_exc())  # Dodajemy informacje o śladzie stosu
            return self.default_settings

    def update_settings(self, file_path, new_settings):
        try:
            with open(file_path, 'r') as file:
                settings = json.load(file)
            if "startup_program_options" in settings:
                settings["startup_program_options"] = new_settings
                with open(file_path, 'w') as file:
                    json.dump(settings, file, indent=4)
                    rospy.logdebug('AMR(SettingsHandler) - update settings: SUCCESS')
            else:
                rospy.logwarn('AMR(SettingsHandler) - update settings: No startup_program_options section in JSON file')
        except Exception as e:
            rospy.logerr(f'AMR(SettingsHandler) - update settings: Error detected: {e}')

    def save_settings(self, file_path, settings):
        try:
            if self.validate_settings_keys(settings):
                with open(file_path, 'w') as file:
                    json.dump(settings, file, indent=4)
                rospy.logdebug('AMR(SettingsHandler) - save settings: SUCCESS')
                return True
            return False
        except OSError as e:
            rospy.logfatal(f'AMR(SettingsHandler) - save settings: Error: {e}')
            return False

    def validate_settings_keys(self, settings):
        expected_keys = [
            "process_options_max_retries",
            "process_options_interval",
            "log_action_time",
            "workstates_requests_queue_size",
            "workstates_requests_latch",
            "refresh_rate"]
        if not isinstance(settings, dict):
            return False
        for key in expected_keys:
            if key not in settings:
                return False
        return True

class AMR:
    def __init__(self):
        '''Variables - custom messages'''
        self.workstates_active = workstate_read()
        self.file = 'catkin_ws/src/AMR/settings/amrsettings.json'
        self.workstates_requests = workstate_request()
        self.actual_task = task_data()
        self.last_task = task_data()
        self.simple_data = simple_in()
        self.settings_handler = SettingsHandler()
        self.settings = self.settings_handler.load_settings(self.file)
        
        '''Variables - user options'''
        self.process_options_timeout = self.settings['process_options_timeout']
        self.process_options_max_retries = self.settings['process_options_max_retries']
        self.process_options_interval = self.settings['process_options_interval']
        self.log_action_time = self.settings['log_action_time']
        self.workstates_requests_queue_size = self.settings['workstates_requests_queue_size']
        self.workstates_requests_latch = self.settings['workstates_requests_latch']
        self.refresh_rate = self.settings['refresh_rate']
        rospy.logerr(type(self.refresh_rate))
        rospy.logwarn(self.refresh_rate)

        '''Variables - common'''
        self.package = 'dhi_amr'
        self.communication_file = 'communication.launch'
        self.diagnostic_file = 'diagnostic_launch'
        self.nav_ride_file = 'nav_ride.launch'

        self.is_communication_active = False
        self.is_diagnostic_active = False
        self.is_nav_ride_active = False
        
        self.active_processes = []
        self.rate = rospy.Rate(self.refresh_rate)
        '''MAIN LOOP'''
        while not rospy.is_shutdown():
            try:
                self.workstates_active_sub = rospy.Subscriber('amr/state_machine', workstate_read, self.workstates_active_callback)
                self.simple_data_sub = rospy.Subscriber('simple_fleet_manager/data_in', simple_in, self.simple_data_callback)

                self.workstates_requests_pub = rospy.Publisher('amr/plc/commands/workstate_request', workstate_request, queue_size=self.workstates_requests_queue_size, latch=self.workstates_requests_latch)
            
            except Exception as e:
                if e == rospy.ROSException:
                    rospy.logerr(f'AMR - init error: Wrong publisher/subscriber parameters: {e}')
                    pass
                elif e == KeyboardInterrupt:
                    rospy.logwarn(f'AMR - init error: User keyboard interrupt detected: {e}')
                    break
                else:
                    rospy.logerr(f'AMR - init error: Other error occured: {e}')
                    break
            # self.process_handler()
            self.rate.sleep()
        
    def workstates_active_callback(self, msg):
        self.workstates_active = msg
    
    def simple_data_callback(self, msg):
        self.simple_data = msg

    def action_duration(self, start_time):
        try:
            return (time.time() - start_time) * 1000
        except Exception as e:
            rospy.logwarn(f'AMR - action duration: error detected while calculations: {e}. Returning 0')
            return 0

    def process_watchdog(self):
        if len(self.active_processes) > 0:
            try:
                for process in self.active_processes:
                    if process[0].poll() is None:
                        rospy.logdebug(f'AMR - process watchdog: {process[1]} is active')
                        return True
                    else:
                        rospy.logerr(f'AMR - process watchdog: {process[1]} is not active as expected!')
                        #Tu wywłaj funkcę zmiany statusu procesu
                        self.active_processes.remove(process)
                        return False
            except Exception as e:
                rospy.logerr(f'AMR - process watchodg: error occured {e}')
                return None

    def process_status_changer(self, file_name, action):
        if file_name == self.communication_file:
            if action == 1:
                self.is_communication_active = True
            elif action == 2:
                self.is_communication_active = False
        if file_name == self.diagnostic_file:
            if action == 1:
                self.is_diagnostic_active = True
            elif action == 2:
                self.is_diagnostic_active = False
        if file_name == self.nav_ride_file:
            if action == 1:
                self.is_nav_ride_active = True
            elif action == 2:
                self.is_nav_ride_active = False

    def process_launcher(self, package, file_name):
        start = time.time()
        attempts = 0
        try:
            process = subprocess.Popen(['roslaunch', package, file_name])
            while attempts < self.process_options_max_retries:
                time.sleep(2)
                if process.poll() is None:
                    self.active_processes.append((process, file_name))
                    self.process_status_changer(file_name, 1)
                    rospy.logdebug(f'AMR - process launcher: New process launched succesfully: {file_name}')
                    if self.log_action_time:
                        rospy.logdebug(f'AMR - process launcher: Action time: {self.action_duration(start)} ms.')
                    return True
                time.sleep(self.process_options_interval)
                attempts += 1
            rospy.logerr(f'AMR - process launcher: Failed to launch requested process {file_name}')
            if self.log_action_time:
                rospy.logdebug(f'AMR - process launcher: Action time: {self.action_duration(start)} ms.')
            return False
        except Exception as e:
            if e == FileNotFoundError:
                rospy.logerr(f'AMR - process launcher: Process file not found: {file_name}, {e}')
            else:
                rospy.logerr(f'AMR - process launcher: Other exception occured: {e}')
            return None

    def launch_common_processes(self):
        while not self.is_communication_active:
            launch_result = self.process_launcher(self.package, self.communication_file)
            if launch_result:
                break

    def process_handler(self):
        self.launch_common_processes()
        self.process_watchdog()



    def shutdown(self):
        if len(self.active_processes) > 0:
            for process in self.active_processes:
                try:
                    process[0].terminate()
                    process[0].wait(timeout=10)
                except subprocess.TimeoutExpired:
                    rospy.logwarn(f'AMR - shutdown process: Timeout exceeded while shutting down process {process[1]}. Killing the process now')
                    process[0].kill()
                    time.sleep(1)
        self.workstates_active_sub.unregister()
        self.simple_data_sub.unregister()
        self.workstates_requests_pub.unregister()
        rospy.signal_shutdown('AMR - shutdown process: Closing node')

if __name__ == '__main__':
    try:
        rospy.init_node('AMR', log_level=rospy.DEBUG)
        amr = AMR()
        rospy.spin()
    except Exception as e:
        if e == rospy.ROSInitException:
            rospy.logfatal(f'AMR - main: Node initialization / registration error: {e}')
        elif e == ValueError:
            rospy.logfatal(f'AMR - main: Value error occured in node name: {e}')
        elif e == KeyboardInterrupt:
            rospy.logwarn(f'AMR - main: User keyboard interruption detected: {e}')
        else:
            rospy.logfatal(f'AMR - main: Other exception detected: {e}')
    finally:
        amr.shutdown()
        rospy.logwarn(f'AMR SHUT DOWN')
