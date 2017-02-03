#!/usr/bin/env python
import rospy
import sys
import tf
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import yaml
import os
from rospkg import RosPack
from gear_data_handler.log_calibration_data import CalibrationLogger
from std_srvs.srv._Trigger import Trigger, TriggerResponse, TriggerRequest

rospack = RosPack()
DEFAULT_TF_DIR = os.path.join(rospack.get_path("gear_launch"), "calibration")
DEFAULT_TF_FILE = "camera_poses.yaml"
DEFAULT_VICON_TF_FILE = "vicon2k2.yaml"
VICON_FRAME = "vicon_world"
WORLD_FRAME = "world"

class PublishTF(object):
    def __init__(self):
        # Ros startup
        self.pub = rospy.Publisher("/tf_static", TFMessage , queue_size=1, latch=True)     
        
        # Creating the publish tf service trigger
        trigger_tf = rospy.Service("trigger_tf", Trigger, self.load_and_publish_tf)
        
        # Load parameters
        self.load_params()
        
        # Publish TF if requested
        if not self.playback_mode:
            req = TriggerRequest()
            self.load_and_publish_tf(req)
            
        return
    
    def load_params(self):
        '''
        Load parameters from parameter server
        '''
        # Check if playback mode
        self.playback_mode = rospy.get_param("~playback_mode", False)
        
        # Get tf_dir
        self.tf_dir = rospy.get_param("~tf_dir", DEFAULT_TF_DIR)
        
        # Get tf_file
        self.tf_file = rospy.get_param("~tf_file", DEFAULT_TF_FILE)

        # Get vicon tf_file
        self.vicon_tf_file = rospy.get_param("~vicon_tf_file", DEFAULT_VICON_TF_FILE)
        return
        
    def create_tf_transform(self, parent, frame, pose_obj):
        '''
        Publish static tf transform
        '''
        static_transformStamped = TransformStamped()
      
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent
        static_transformStamped.child_frame_id = frame
      
        static_transformStamped.transform.translation.x = float(pose_obj["translation"]["x"])
        static_transformStamped.transform.translation.y = float(pose_obj["translation"]["y"])
        static_transformStamped.transform.translation.z = float(pose_obj["translation"]["z"])
      
        static_transformStamped.transform.rotation.x = float(pose_obj["rotation"]["x"])
        static_transformStamped.transform.rotation.y = float(pose_obj["rotation"]["y"])
        static_transformStamped.transform.rotation.z = float(pose_obj["rotation"]["z"])
        static_transformStamped.transform.rotation.w = float(pose_obj["rotation"]["w"])
     
        return static_transformStamped
    
    def load_and_publish_tf(self, req):
        '''
        Load TF from disk and publish
        '''
        if self.playback_mode:
            self.tf_dir, _ =  CalibrationLogger.get_playback_calibration_directory()
        tf_transform_list = []
        
        # Check if the tf file exists
        self.tf_path = os.path.join(self.tf_dir, self.tf_file)
        rospy.loginfo("[TFBroadcaster] Loading tf from file: "+self.tf_path)
        if not os.path.exists(self.tf_path):
            rospy.logerr("[TFBroadcaster] Invalid tf file")

        # Check if the vicon tf file exists
        self.vicon_tf_path = os.path.join(self.tf_dir, self.vicon_tf_file)
        rospy.loginfo("[TFBroadcaster] Loading vicon tf from file: "+self.vicon_tf_path)
        if not os.path.exists(self.vicon_tf_path):
            rospy.logerr("[TFBroadcaster] Invalid vicon tf file")
    
        # Load tf information from file
        tf_info = yaml.load(open(self.tf_path))

        # Load vicon tf information from file
        vicon_tf_info = yaml.load(open(self.vicon_tf_path))
         
        # Publish transform berween vicon frame and world frame
        dummy_pose_obj = {"translation":{"x":0, "y":0, "z":0}, "rotation":{"x":0, "y":0, "z":0, "w":1}}
        tf_transform_list.append(self.create_tf_transform(WORLD_FRAME, VICON_FRAME, dummy_pose_obj))
        
        # Publish the vicon tf
        pose_obj = {"translation":vicon_tf_info["translation"], "rotation":vicon_tf_info["rotation"]}
        tf_transform_list.append(self.create_tf_transform(VICON_FRAME, vicon_tf_info["root_frame"], pose_obj))
        
        # Publish tf that was loaded from file
        for k,v in tf_info["poses"].iteritems():
            if k != vicon_tf_info["root_frame"]:
                tf_transform_list.append(self.create_tf_transform(vicon_tf_info["root_frame"], k, v))
            tf_transform_list.append(self.create_tf_transform(k, k+"_link", dummy_pose_obj))
    
        # Publish tfs    
        rospy.loginfo("[TFBroadcaster] Publishing TF")
        self.pub.publish(tf_transform_list)    
        
        # Send the response
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Published TF successfully"
        
        return resp
    
if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    publish_tf = PublishTF()
    rospy.spin()
    