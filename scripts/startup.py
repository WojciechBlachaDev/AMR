#!/usr/bin/env python3
import rospy, subprocess, time, os, json
from dhi_amr.msg import workstate_read, workstate_request, simple_in, task_data

class SettingsHandler():
    def __init__(self):
        self.default_settings = {
            "process_options_timeout": 10.0,
            "process_options_max_retries": 3,
            "process_options_interval": 1.0,
            "log_action_time": True,
            "workstates_requests_queue_size": 1,
            "workstates_requests_latch": True,
            "refresh_rate": 10
        }

    def check_file(self, file_path):
        if not os.path.exists(file_path):
            return False
        else:
            return True
    def create_file(self, file_path):
        try:
            os.makedirs(os.path.dirname(file_path), exist_ok = True)
            with open(file_path, 'w') as file:
                pass
            rospy.logdebug('AMR(SettingsHandler) - create file path: Fiel path created')
            return True
        except OSError as e:
            rospy.logfatal(f'AMR(SettingsHandler) - create file path: Error {e}')
            return False
    def save_settings(self, file_path, settings):
        try:
            with open(file_path, 'w') as file:
                json.dump(settings, file, indent=4)
            rospy.logdebug('AMR(SettingsHandler) - sace settings: SUCCESS')
            return True
        except OSError as e:
            rospy.logfatal(f'AMR(SettingsHandler) - save settings error: {e}')
            return False
    
    def get_default_settings(self):
        return self.default_settings
        
    def validate_settings(self, settings):
        expected_keys = ["process_options_max_retries",
            "process_options_interval",
            "log_action_time",
            "workstates_requests_queue_size",
            "workstates_requests_latch",
            "refresh_rate"]
        if not isinstance(settings, dict):
            return False, "Setting needs to be dictionary type"
        for key in expected_keys:
            if key not in settings:
                return False, f"Missing expected dictionary key: {key}"
        return True, "Settings ok!"

class AMR:
    def __init__(self):
        '''Variables - custom messages'''
        self.workstates_active = workstate_read()
        self.file_path = 'catkin_ws/src/AMR/settings/startup.json'
        self.workstates_requests = workstate_request()
        self.actual_task = task_data()
        self.last_task = task_data()
        self.simple_data = simple_in()
        self.settings_handler = SettingsHandler()
        if not self.settings_handler.check_file(self.file_path):
            self.settings_handler.create_file(self.file_path)
        self.default = self.settings_handler.get_default_settings()
        self.settings_handler.save_settings(self.file_path, self.default)
        '''Variables - user options'''
        self.log_action_time = True
        self.process_options_timeout = 10.0
        self.process_options_max_retries = 3
        self.process_options_interval = 1.0
        self.workstates_requests_queue_size = 1
        self.workstates_requests_latch = True
        self.refresh_rate = 10
    
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
