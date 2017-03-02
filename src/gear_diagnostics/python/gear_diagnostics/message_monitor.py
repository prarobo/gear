#!/usr/bin/env python

"""
Message monitor to follow messages and generate diagnostic information 
"""

import rospy
from std_msgs.msg import Int64
from diagnostic_updater import DiagnosedPublisher, Updater, FrequencyStatusParam, TimeStampStatusParam
from diagnostic_updater._publisher import HeaderlessTopicDiagnostic

DEFAULT_FRAME_RATE = 15
DEFAULT_FREQUENCY_TOLERANCE = 0.1
DEFAULT_WINDOW_SIZE = 60
DEFAULT_MIN_ACCEPTABLE_DELAY = 0.0
DEFAULT_MAX_ACCEPTABLE_DELAY = 0.2
DEFAULT_UPDATE_INTERVAL = 1

SYNCHRONIZER_TOPIC = "/synchronizer/image_count"
SENSOR_LOGGER_TOPICS = ["/k1/color_image_count",
                        "/k2/color_image_count",
                        "/k3/color_image_count",
                        "/p1/color_image_count",
                        "/p2/color_image_count",
                        "/k1/depth_image_count",
                        "/k2/depth_image_count",
                        "/k3/depth_image_count"]                

class MessageMonitor(object):
    def __init__(self):
        rospy.init_node('message_monitor', anonymous=False)
        
        # Load parameters
        self._loadParameters()
        
        # Setup diagnostics updater
        self.logger_updater = Updater()
        self.synchronizer_updater = Updater()
        self.logger_updater.setHardwareID("logger_monitor")
        self.synchronizer_updater.setHardwareID("synchronizer_monitor")

        # Create diagnostics topics
        self._createTopicDiagnostics()
        
        # Timer to periodically update diagnostics
        rospy.Timer(rospy.Duration(self.update_interval), self._updateDiagnostics)

        rospy.spin()
        return
        
    
    def _createTopicDiagnostics(self):
        '''
        Create the diagnostics object for topics
        '''
        self.synchronizer_diagnostics = HeaderlessTopicDiagnostic \
                                        (self.synchronizer_topic, 
                                         self.synchronizer_updater,
                                         FrequencyStatusParam({"min": self.frame_rate, "max": self.frame_rate}, 
                                                              self.freq_tolerance,
                                                              self.window_size))
         
        self.sensor_logger_diagnostics = []
        for s in self.sensor_logger_topics:
            self.sensor_logger_diagnostics.append( HeaderlessTopicDiagnostic \
                                                   (s, self.logger_updater,
                                                    FrequencyStatusParam({"min": self.frame_rate, "max": self.frame_rate}, 
                                                                         self.freq_tolerance,
                                                                         self.window_size)))
        return
    
    def _loadParameters(self):
        '''
        Load ros parameters from parameter server
        '''
        # Get data topics
        self.synchronizer_topic = rospy.get_param("/synchronizer_topics", SYNCHRONIZER_TOPIC)
        self.sensor_logger_topics = rospy.get_param("/sensor_logger_topics", SENSOR_LOGGER_TOPICS)
            
        # Get diagnostic parameters
        self.frame_rate = rospy.get_param("/frame_rate", DEFAULT_FRAME_RATE)
        self.freq_tolerance = rospy.get_param("frequency_tolerance", DEFAULT_FREQUENCY_TOLERANCE)
        self.window_size = rospy.get_param("window_size", DEFAULT_WINDOW_SIZE)
        self.min_acceptable_delay = rospy.get_param("min_acceptable_delay", DEFAULT_MIN_ACCEPTABLE_DELAY)
        self.max_acceptable_delay = rospy.get_param("max_acceptable_delay", DEFAULT_MAX_ACCEPTABLE_DELAY)
        self.update_interval = rospy.get_param("update_interval", DEFAULT_UPDATE_INTERVAL)
        
        rospy.loginfo("[MessageMonitor] synchronizer_topic: "+self.synchronizer_topic)
        for s in self.sensor_logger_topics:
            rospy.loginfo("[MessageMonitor] sensor_topic: "+s)
        return
    
    def _updateDiagnostics(self, event):
        '''
        Update diagnostics function
        '''
        # Update tick
        self.synchronizer_diagnostics.tick()
        for s in self.sensor_logger_diagnostics:
            s.tick()
        
        # Send diagnostics out
        self.logger_updater.update()
        self.synchronizer_updater.update()
        return            

if __name__ == "__main__":
    MessageMonitor()
