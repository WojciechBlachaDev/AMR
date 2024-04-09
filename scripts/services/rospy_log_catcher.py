#!/usr/bin/env python3
import rospy, datetime, os, logging
from rosgraph_msgs.msg import Log
from dhi_amr.msg import log_messages


class LogCatcher:
    def __init__(self):
        """ Variables """
        """ Options """
        self.debug_enabled = True
        self.save_log_to_file = False
        self.log_file_max_size = 256
        """ Custom """
        self.log = log_messages()
        
        """ Others """
        self.received_log = Log()
        self.last_received_log = Log()
        # self.create_log_directory()
        # self.set_file_handling()
        try:
            """ ROS Subscriber definition """
            received_log_sub = rospy.Subscriber('rosout_agg', Log, self.received_log_callback)
            
            """ ROS Publisher definition """
            self.log_pub = rospy.Publisher('amr/log/current', log_messages, queue_size=1, latch=True)
            
        except Exception as e:
            rospy.logerr(f'Error detected in log catcher at init: {e}')
        except KeyboardInterrupt as e:
                rospy.logwarn(f'User stop detected in LOG CATCHER!')

    """ Callback section for log subscriber"""
    def received_log_callback(self, msg):
        self.received_log = msg
        is_new = self.is_log_new(self.received_log)
        if is_new:
            # if self.save_log_to_file:
            #     self.create_log_line()
            #     self.check_file_size()
            self.create_published_topic()
            self.log_pub.publish(self.log)
            self.last_received_log == self.received_log

    """ Setting loggers and file handlers to separate files depending on log level"""
    def set_file_handling(self):
        self.debug_log_file = os.path.join(os.getcwd(), 'amr/data', 'amr_log_debug.txt')
        self.info_log_file = os.path.join(os.getcwd(), 'amr/data', 'amr_log_info.txt')
        self.warning_log_file = os.path.join(os.getcwd(), 'amr/data', 'amr_log_warning.txt')
        self.error_log_file = os.path.join(os.getcwd(), 'amr/data', 'amr_log_error.txt')
        self.fatal_log_file = os.path.join(os.getcwd(), 'amr/data', 'amr_log_fatal.txt')
        
        self.debug_logger = logging.getLogger("debug_logger")
        self.info_logger = logging.getLogger("info_logger")
        self.warning_logger = logging.getLogger("warning_logger")
        self.error_logger = logging.getLogger("error_logger")
        self.fatal_logger = logging.getLogger("fatal_logger")
        
        self.debug_logger.setLevel(logging.DEBUG)
        self.info_logger.setLevel(logging.INFO)
        self.warning_logger.setLevel(logging.WARNING)
        self.error_logger.setLevel(logging.ERROR)
        self.fatal_logger.setLevel(logging.CRITICAL)
        
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s - %(filename)s-%(funcName)s-%(lineno)d')
        
        debug_file_handler = logging.FileHandler(self.debug_log_file)
        info_file_handler = logging.FileHandler(self.info_log_file)
        warning_file_handler = logging.FileHandler(self.warning_log_file)
        error_file_handler = logging.FileHandler(self.error_log_file)
        fatal_file_handler = logging.FileHandler(self.fatal_log_file)
        
        debug_file_handler.setLevel(logging.DEBUG)
        info_file_handler.setLevel(logging.INFO)
        warning_file_handler.setLevel(logging.WARNING)
        error_file_handler.setLevel(logging.ERROR)
        fatal_file_handler.setLevel(logging.CRITICAL)
        
        debug_file_handler.setFormatter(formatter)
        info_file_handler.setFormatter(formatter)
        warning_file_handler.setFormatter(formatter)
        error_file_handler.setFormatter(formatter)
        fatal_file_handler.setFormatter(formatter)

        self.debug_logger.addHandler(debug_file_handler)
        self.info_logger.addHandler(info_file_handler)
        self.warning_logger.addHandler(warning_file_handler)
        self.error_logger.addHandler(error_file_handler)
        self.fatal_logger.addHandler(fatal_file_handler)

    """ Checking if log directiory exists and if not - creating one"""
    def create_log_directory(self):
        log_directory = os.path.join(os.getcwd(),'log', 'data')
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)

    """ Create info to publish """
    def create_published_topic(self):
        date = datetime.datetime.now()
        self.log.date = date.strftime("%Y-%m-%d %H:%M:%S")
        self.log.level = self.create_log_level()
        self.log.message = str(self.received_log.msg)
        if self.debug_enabled:
            self.log.node_name = self.received_log.name
            self.log.file = self.received_log.file + ' / ' + self.received_log.function
            self.log.code_line = str(self.received_log.line)
    
    """ Create info level to publish """
    def create_log_level(self):
        if self.received_log.level == 1:
            return 1
        elif self.received_log.level == 2:
            return 2
        elif self.received_log.level == 4:
            return 3
        elif self.received_log.level == 8:
            return 4
        elif self.received_log.level == 16:
            return 5

    """ Check if the log is a new log """
    def is_log_new(self, received_log):
        if received_log.header.seq != self.last_received_log.header.seq:
            self.last_received_log = received_log
            return True
        return False

    """Creating an saving log file """
    def create_log_line(self):
        log_msg = "{timestamp} - {level} - {msg} - {file}-{function}-{line} - {caller}".format(
            timestamp=self.received_log.header.stamp.secs,
            level=self.received_log.level,
            msg=self.received_log.msg,  
            file=self.received_log.file,
            function=self.received_log.function,
            line=self.received_log.line,
            caller=self.received_log.name
        )
        if self.received_log.level == 1 and self.debug_logger.isEnabledFor(logging.DEBUG):
            self.debug_logger.debug(log_msg)
        elif self.received_log.level == 2 and self.info_logger.isEnabledFor(logging.INFO):
            self.info_logger.info(log_msg)
        elif self.received_log.level == 4 and self.warning_logger.isEnabledFor(logging.WARNING):
            self.warning_logger.warning(log_msg)
        elif self.received_log.level == 8 and self.error_logger.isEnabledFor(logging.ERROR):
            self.error_logger.error(log_msg)
        elif self.received_log.level == 16 and self.fatal_logger.isEnabledFor(logging.CRITICAL):
            self.fatal_logger.critical(log_msg)

    """ Checking the file size of log, and backup last one to .last file"""
    def check_file_size(self):
        if os.path.exists(self.debug_log_file):
            file_size = os.path.getsize(self.debug_log_file)
            if file_size > self.log_file_max_size * 1024 * 1024:  
                last_log_file = self.debug_log_file + '.last'
                os.rename(self.debug_log_file, last_log_file)
                self.set_file_handling()
        if os.path.exists(self.info_log_file):
            file_size = os.path.getsize(self.info_log_file)
            if file_size > self.log_file_max_size * 1024 * 1024:  
                last_log_file = self.info_log_file + '.last'
                os.rename(self.info_log_file, last_log_file)
                self.set_file_handling()
        if os.path.exists(self.warning_log_file):
            file_size = os.path.getsize(self.warning_log_file)
            if file_size > self.log_file_max_size * 1024 * 1024:  
                last_log_file = self.warning_log_file + '.last'
                os.rename(self.warning_log_file, last_log_file)
                self.set_file_handling()
        if os.path.exists(self.error_log_file):
            file_size = os.path.getsize(self.error_log_file)
            if file_size > self.log_file_max_size * 1024 * 1024:  
                last_log_file = self.error_log_file + '.last'
                os.rename(self.error_log_file, last_log_file)
                self.set_file_handling()
        if os.path.exists(self.fatal_log_file):
            file_size = os.path.getsize(self.fatal_log_file)
            if file_size > self.log_file_max_size * 1024 * 1024:  
                last_log_file = self.fatal_log_file + '.last'
                os.rename(self.fatal_log_file, last_log_file)
                self.set_file_handling()

if __name__ == '__main__':
    try:
        rospy.init_node('log_catcher', log_level=rospy.DEBUG)
        rospy.loginfo('Log catcher started')
        log_catcher = LogCatcher()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'Error detected in log catcher at main: {e}')
    finally:
        rospy.logwarn('Log catcher shuted down')
