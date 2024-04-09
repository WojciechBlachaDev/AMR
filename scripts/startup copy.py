#!/usr/bin/env python3
import rospy
import subprocess
import time
import asyncio
from threading import Thread
from dhi_amr.msg import workstate_request, workstate_read, task_data

class AMR:
    def __init__(self):
        self.workstates_active = workstate_read()
        self.workstates_requests = workstate_request()
        self.new_task = task_data()
        self.last_task = task_data()

        self.log_action_timeout = True
        self.process_launch_timeout = 10.0
        self.process_launch_max_retries = 3
        self.process_launch_interval = 1.0
        self.workstates_requests_queue_size = 1
        self.workstates_requests_latch = True
        self.process_monitor_interval = 1.0

        self.package_name = 'dhi_amr'
        self.communication_file = 'communication.launch'
        self.diagnostic_file = 'diagnostic.launch'
        self.communication_process = None
        self.diagnostic_process = None
        self.roscore_running = False
        self.is_communication_ok = False
        self.active_processes = []
        self.process_monitor_active = False
        self.workstates_active_sub = rospy.Subscriber('amr/state_machine', workstate_read, self.workstates_active_callback)
        self.workstates_reqest_pub = rospy.Publisher('amr/plc/commands/workstate_request', workstate_request, queue_size=self.workstates_requests_queue_size, latch=self.workstates_requests_latch)

        self.process_logic()

    def workstates_active_callback(self, msg):
        self.workstates_active = msg

    def new_task_callback(self, msg):
        self.new_task = msg
        if self.new_task.id != self.last_task.id:
            self.task_service()

    def action_time_monitor(self, start_time):
        try:
            return (time.time() - start_time) * 1000
        except Exception as e:
            rospy.logwarn(f'AMR: Exception at action timeout monitor: {e}')
            return 0

    async def process_monitor(self):
        self.process_monitor_active = True
        while not rospy.is_shutdown():
            process_count = len(self.active_processes)
            if process_count > 0:
                try:
                    for process in self.active_processes:
                        if process[0].poll() is None:
                            rospy.logdebug(f'AMR - process monitor: {process} is active')
                        else:
                            rospy.logwarn(f'AMR - process monitor: Inactive process detected - {process}. Deleting from active process list!')
                            self.process_status_changer(process[1], 2)
                            self.active_processes.remove(process)
                    await asyncio.sleep(self.process_monitor_interval)
                except Exception as e:
                    rospy.logerr(f'AMR - process monitor: Other exception occured: {e}')
            else:
                await asyncio.sleep(self.process_monitor_interval)

    def process_status_changer(self, file_name, action):
        if file_name == self.communication_file:
            if action == 1:
                self.is_communication_ok = True
                rospy.logdebug('AMR - process status changer: communication is ok')
            elif action == 2:
                self.is_communication_ok = False
                rospy.logdebug('AMR - process status changer: communication is not ok')

    def process_launcher(self, package, file_name):
        action_start_time = time.time()
        attempts = 0
        try:
            new_process = subprocess.Popen(['roslaunch', package, file_name])
            while attempts < self.process_launch_max_retries:
                time.sleep(5)
                if new_process.poll() is None:
                    self.active_processes.append((new_process, file_name))
                    self.process_status_changer(file_name, 1)
                    rospy.logdebug(f'AMR: Process {file_name} launched succesfully and added to active processes! Action time: {self.action_time_monitor(action_start_time)} ms.')
                    return True
                time.sleep(self.process_launch_interval)
                attempts += 1
            rospy.logwarn(f'AMR: Process launcher: Process {file_name} failed to launch. Action time: {self.action_time_monitor(action_start_time)}')
            return False
        except FileNotFoundError as e:
            rospy.logfatal(f'AMR: Process Launcher:  Launch file {file_name} can not be found in the system: {e}')
            return None
        except Exception as e:
            rospy.logfatal(f'AMR: Other exception detected at process launcher: {e}')
            return None

    def shutdown(self):
        for process in self.active_processes:
            try:
                process[0].terminate()
                process[0].wait(timeout=10)
            except subprocess.TimeoutExpired:
                rospy.logerr(f'AMR: Timeout while shutting down process {process[1]}, killing now!')
                process[0].kill()
                process[0].wait()
            self.active_processes.remove(process)
        rospy.loginfo('AMR: All processes shutted down!')

    def process_logic(self):
        if not self.is_communication_ok:
            self.process_launcher(self.package_name, self.communication_file)
        rospy.loginfo("TUTUTUTUTUTU")

    def task_service(self):
        rospy.loginfo('Task')

def main():
    try:
        rospy.init_node('AMR', log_level=rospy.DEBUG)
        rospy.logdebug('AMR: Started')
        amr = AMR()

        rospy.loginfo('Starting event loop...')
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(amr.process_monitor())

    except Exception as e:
        rospy.logerr(f'AMR: Exception occured at __main__: {e}')
    except KeyboardInterrupt as e:
        rospy.logwarn(f'AMR: User keyboard interruption detected!')
    finally:
        rospy.logwarn('AMR: Shutting down!')
        amr.shutdown()

if __name__ == '__main__':
    main()
