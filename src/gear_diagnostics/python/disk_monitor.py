#!/usr/bin/env python

"""
Message monitor to follow messages and generate diagnostic information 
"""

import rospy
import os
from diagnostic_updater import DiagnosedPublisher, Updater, FrequencyStatusParam, TimeStampStatusParam
from diagnostic_updater._publisher import HeaderlessTopicDiagnostic
from std_msgs.msg._Float32 import Float32
from diagnostic_msgs.msg import DiagnosticStatus

DEFAULT_MOUNT_POINT = "/mnt/md0"
DEFAULT_UPDATE_INTERVAL = 60    #seconds
DEFAULT_LOW_FREE_SPACE = 500   #GB
DEFAULT_CRITICAL_FREE_SPACE = 100   #GB

class MessageMonitor(object):
    def __init__(self):
        rospy.init_node('disk_monitor', anonymous=False)
        
        # Load parameters
        self._loadParameters()
        
        # Setup diagnostics updater
        self.disk_updater = Updater()
        self.disk_updater.setHardwareID("disk_monitor")
        
        # Add disk check diagnostic task
        self.disk_updater.add("Memory check", self._diskCheckTask)

        # Publisher for disk space
        self.disk_pub = rospy.Publisher("disk_space", Float32, queue_size=1)

        # Timer to periodically update diagnostics
        rospy.Timer(rospy.Duration(self.update_interval), self._updateDiagnostics)
        
        rospy.spin()
        return
        
    
    def _loadParameters(self):
        '''
        Load ros parameters from parameter server
        '''
        self.mount_point = rospy.get_param("~mount_point", DEFAULT_MOUNT_POINT)
        self.update_interval = rospy.get_param("~update_interval", DEFAULT_UPDATE_INTERVAL)
        self.low_free_space = rospy.get_param("~low_free_space", DEFAULT_LOW_FREE_SPACE)
        self.critical_free_space = rospy.get_param("~critical_free_space", DEFAULT_CRITICAL_FREE_SPACE)
        
        rospy.loginfo("[DiskMonitor] mount point : "+self.mount_point)
        rospy.loginfo("[DiskMonitor] update interval : "+str(self.update_interval))
        rospy.loginfo("[DiskMonitor] low free space : "+str(self.low_free_space))
        rospy.loginfo("[DiskMonitor] critical free space : "+str(self.critical_free_space))
        return
    
    def _getAvailableDiskSpace(self):
        '''
        Get the space available in disk (in GB)
        '''
        stat = os.statvfs(self.mount_point)
        return (stat.f_frsize * stat.f_bavail)/(1024**3)
    
    def _diskCheckTask(self, stat):
        '''
        Check if disk has sufficient space
        '''
        if self._getAvailableDiskSpace() < self.critical_free_space:
            stat.summary(DiagnosticStatus.ERROR, "Disk space CRITICAL")
        elif self._getAvailableDiskSpace() < self.low_free_space:
            stat.summary(DiagnosticStatus.WARN, "Disk space LOW")
        else:
            stat.summary(DiagnosticStatus.OK, "Disk space OK")
        return stat
    
    def _updateDiagnostics(self, event):
        '''
        Update diagnostics function
        '''
        # Send disk space message
        disk_space = self._getAvailableDiskSpace()
        self.disk_pub.publish(disk_space)
        
        # Send diagnostics out
        self.disk_updater.update()
        return            

if __name__ == "__main__":
    MessageMonitor()
