#!/usr/bin/env python

"""
Synchronized logging module for the 
data streams for a multiple camera setup
"""

import message_filters
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int64
from std_srvs.srv._SetBool import SetBool, SetBoolResponse
import threading
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

DEFAULT_TOPICS = ["/k1/hd/image_color",
                  "/k2/hd/image_color",
                  "/k3/hd/image_color",
                  "/k1/sd/image_depth",
                  "/k2/sd/image_depth",
                  "/k3/sd/image_depth",
                  "/p1/hd/image_color",
                  "/p2/hd/image_color"]

DEFAULT_FRAME_RATE = 15

class SynchronizedImageLogger(object):

    def __init__(self):
        rospy.init_node('synchronized_logger', anonymous=False)
        
        # Initialize cv bridge
        self.bridge = CvBridge()
        
        # Create mutexes
        self.enable_lock = threading.Lock()
        
        # Parameter that controls logging
        self.enable_lock.acquire()
        self.enable = False
        self.enable_lock.release()
        
        # Setting image count
        self.image_count = 0
        
        # Loading ros parameters
        self._loadParameters()
        
        # Compute time synchronizer slop as half the frame rate delay
        ts_slop = 0.05
        
        # Create subscribers for all topics
        subs = [message_filters.Subscriber(t, Image) for t in self.data_topics]
        
        # Create synchronized logger message filter
        ts = message_filters.ApproximateTimeSynchronizer(subs, 1, ts_slop)
        ts.registerCallback(self.syncLoggerCB)
        
        # Service call to trigger logging
        rospy.Service("/sync_logger_enable", SetBool, self._toggleLogger)
        
        # Image subscribers
        self.pubs = []
        for t in self.data_topics:
            base_name, image_type = self.parseTopic(t)
            self.pubs.append(rospy.Publisher('/synchronizer/'+base_name+'_'+image_type, Image, queue_size=10))
        
        # Image count publisher
        self.image_count_pub = rospy.Publisher('image_count', Int64, queue_size=10)
        
        rospy.spin()
        return

    def _toggleLogger(self, req):
        '''
        Toggles the logger node on response to service calls
        '''
        resp = SetBoolResponse()
        if req.data:
            
            self.enable_lock.acquire()
            self.enable = True            
            self.enable_lock.release()        

            rospy.loginfo("[SyncLogger] Logger ON")
            resp.message = "Logger ON"
        else:
            self.enable_lock.acquire()
            self.enable = False            
            self.enable_lock.release()        

            rospy.loginfo("[SyncLogger] Logger OFF")
            resp.message = "Logger OFF"
        
        resp.success = True
        return resp
              
    def _loadParameters(self):
        '''
        Load ros parameters from parameter server
        '''
        # Get data topics
        self.data_topics = rospy.get_param("/data_topics", DEFAULT_TOPICS)
            
        # Get frame rate to use
        self.frame_rate = rospy.get_param("/frame_rate", DEFAULT_FRAME_RATE)
          
        # Log messages      
        for t in self.data_topics:
            rospy.loginfo("[SyncLogger] Logging topic: "+ t)
        rospy.loginfo("[SyncLogger] Frame rate input: "+ str(self.frame_rate))
        
        return
    
    def parseTopic(self, topic_name):
        '''
        Parse a topic name to get the directory name
        '''
        topic_parts = topic_name.split('/')
        base_name = topic_parts[1]
        image_type = topic_parts[-1].split('_')[1]
        return base_name, image_type
    
    def syncLoggerCB(self,*args):
        rospy.logdebug("[SyncLogger] Synchronized input available ...")
        self.enable_lock.acquire()
        if self.enable:            
            timestamp = args[0].header.stamp
            for arg, pub in zip(args, self.pubs):
                arg.header.stamp = timestamp
                pub.publish(arg)
                                      
            self.image_count += 1
            self.image_count_pub.publish(Int64(self.image_count))
        self.enable_lock.release()            
        return

if __name__=="__main__":
    syncLogger = SynchronizedImageLogger()
    
    
    