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

rospack = RosPack()
DEFAULT_TF_DIR = os.path.join(rospack.get_path("gear_launch"), "calibration")
DEFAULT_TF_FILE = "camera_poses.yaml"
VICON_FRAME = "vicon_world"
WORLD_FRAME = "world"

def publish_tf_transform(parent, frame, pose_obj):
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
    
if __name__ == '__main__':
    # Ros startup
    rospy.init_node('tf_broadcaster')
    pub = rospy.Publisher("/tf_static", TFMessage , queue_size=1, latch=True)
    tf_transform_list = []
    
    # Check if playback mode
    playback_mode = rospy.get_param("~playback_mode", False)
    
    # Get tf_dir
    if playback_mode:
        tf_dir, _ =  CalibrationLogger.get_playback_calibration_directory()
    else:
        tf_dir = rospy.get_param("~tf_dir", DEFAULT_TF_DIR)
    
    # Get tf_file
    tf_file = rospy.get_param("~tf_file", DEFAULT_TF_FILE)
    
    tf_path = os.path.join(tf_dir, tf_file)
    rospy.loginfo("[TFBroadcaster] Loading tf from file: "+tf_path)
    if not os.path.exists(tf_path):
        rospy.logerror("[TFBroadcaster] Invalid tf file")
    
    # Load tf information from file
    tf_info = yaml.load(open(tf_path))
     
    # Publish transform berween vicon frame and world frame
    pose_obj = {"translation":{"x":0, "y":0, "z":0}, "rotation":{"x":0, "y":0, "z":0, "w":1}}
    tf_transform_list.append(publish_tf_transform(WORLD_FRAME, VICON_FRAME, pose_obj))
    
    # Publish tf that was loaded from file
    for k,v in tf_info["poses"].iteritems():
        tf_transform_list.append(publish_tf_transform(VICON_FRAME, k, v))

    # Publish tfs    
    pub.publish(tf_transform_list)    
    rospy.spin()
