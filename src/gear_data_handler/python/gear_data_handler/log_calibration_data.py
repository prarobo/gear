#!/usr/bin/python
'''
Node to log calibration data
'''
import rospy
import shutil
import os
import re
import rospkg
from std_srvs.srv._SetBool import SetBool, SetBoolResponse

CALIBRATION_DIR_NAME = "calibration"
CONFIGURATION_DIR_NAME = "config"
DEFAULT_DATA_DIR = "/mnt/md0/gear_data"

class CalibrationLogger(object):
    def __init__(self):
        rospy.init_node('calibration_logger', anonymous=False)
        
        # Create a service to trigger logging
        rospy.Service("/log_calibration_data_enable", SetBool, self._logger)
        
        rospy.spin()        
        return
    
    def _logger(self, req):
        '''
        Callback for logging calibration data
        '''
        # Load parameters
        self.load_params()
        
        # Move calibration information
        self._move_calib_info()

        # Move configuration information
        self._move_config_info()
        
        # Load sensor information
        kinect_info, pointgrey_info = self._load_sensor_info()
        
        # Organize calibration info for playback
        self._organize_calib_info(kinect_info, pointgrey_info)
        
        rospy.loginfo("[CalibrationLogger] Successfully saved calibration data")
        resp = SetBoolResponse()
        resp.success = True
        resp.message = "logger ON"
        return resp
    
    @classmethod
    def load_params(self):
        '''
        Load parameter
        '''
        subject = rospy.get_param("/subject_id")
        session = rospy.get_param("/session_id")
        activity = rospy.get_param("/activity_id")
        condition = rospy.get_param("/condition_id")
        trial = str(rospy.get_param("/trial_id"))
        data_dir = rospy.get_param("/data_dir", DEFAULT_DATA_DIR)
        return subject, session, activity, condition, trial, data_dir
    
    def _move_calib_info(self):
        '''
        Move raw calibration information
        '''
        # Get the calibration data directory
        src_calib_dir = os.path.join(rospkg.RosPack().get_path("gear_launch"), CALIBRATION_DIR_NAME)
        
        # Check if calibration data exists
        if not os.path.exists(src_calib_dir):
            rospy.logerr("[CalibrationLogger] Missing calibration data: "+src_calib_dir)
            
        # Get the destination calibration directories
        self.dest_calib_dir, self.dest_raw_calib_dir = self.get_playback_calibration_directory()

        # Check any existing old calibration data
        if os.path.exists(self.dest_calib_dir):
            rospy.logwarn("[CalibrationLogger] Old calibration data removed: "+self.dest_calib_dir)
            shutil.rmtree(self.dest_calib_dir)        
                
        # Copy raw calibration information
        shutil.copytree(src_calib_dir, self.dest_raw_calib_dir)
        return

    def _move_config_info(self):
        '''
        Move raw configuration information
        '''
        # Get the configuration data directory
        src_config_dir = os.path.join(rospkg.RosPack().get_path("gear_launch"), CONFIGURATION_DIR_NAME)
        
        # Check if configuration data exists
        if not os.path.exists(src_config_dir):
            rospy.logerr("[CalibrationLogger] Missing configuration data: "+src_config_dir)
            
        # Get the destination configuration directories
        self.dest_config_dir = self.get_playback_configuration_directory()

        # Check any existing old configuration data
        if os.path.exists(self.dest_config_dir):
            rospy.logwarn("[CalibrationLogger] Old configuration data removed: "+self.dest_config_dir)
            shutil.rmtree(self.dest_config_dir)        
                
        # Copy raw configuration information
        shutil.copytree(src_config_dir, self.dest_config_dir)
        return

    @classmethod
    def get_playback_calibration_directory(self):
        '''
        Get the calibration directory while playback
        '''
        
        # Get the calibration data directory
        src_calib_dir = os.path.join(rospkg.RosPack().get_path("gear_launch"), CALIBRATION_DIR_NAME)
        
        # Check if calibration data exists
        if not os.path.exists(src_calib_dir):
            rospy.logerr("[CalibrationLogger] Missing calibration data: "+src_calib_dir)
        
        # Get the trial directory
        trial_dir = self.get_trial_dir()

        # Get the destination calibration directory
        dest_calib_dir = os.path.join(trial_dir, CALIBRATION_DIR_NAME)
        
        # Create the destination directory
        dest_raw_calib_dir = os.path.join(dest_calib_dir, "raw_calib")
        
        return dest_calib_dir, dest_raw_calib_dir

    @classmethod
    def get_trial_dir(self):
        '''
        Get the trial directory
        '''
        
        # Get the parameters
        subject, session, activity, condition, trial, data_dir = self.load_params()

        # Get the trial directory
        trial_dir = os.path.join(data_dir, subject, session, '_'.join([activity, condition, trial]))

        return trial_dir

    @classmethod
    def get_playback_configuration_directory(self):
        '''
        Get the configuration directory while playback
        '''
        # Get the trial directory
        trial_dir = self.get_trial_dir()

        # Get the destination calibration directory
        dest_config_dir = os.path.join(trial_dir, CONFIGURATION_DIR_NAME)

        return dest_config_dir

    def _load_sensor_info(self):
        '''
        Find sensors in setup
        '''       
        kinect_names = self.get_matching_parameters(re.compile(r"/k(?P<num>\d+)_serial"), 'k')
        pointgrey_names = self.get_matching_parameters(re.compile(r"/p(?P<num>\d+)_serial"), 'p')
        
        # Load kinect serial numbers
        kinect_info = {k:rospy.get_param("/"+k+"_serial") for k in kinect_names}
            
        # Load pointgrey serial numbers
        pointgrey_info = {k:rospy.get_param("/"+k+"_serial") for k in pointgrey_names}
        
        return kinect_info, pointgrey_info
           
    @classmethod
    def get_matching_parameters(self, reg_exp, prefix):
        '''
        Find parameters that match some regular expression
        '''
        matching_names = []
        
        # Get all parameters from parameter server
        param_names = rospy.get_param_names()
        
        # Get all names that match the input regular expression
        for p in param_names:
            match_obj = re.match(reg_exp, p)
            
            if match_obj:
                matching_names.append(prefix+match_obj.group("num"))
        return matching_names
    
    def _organize_calib_info(self, kinect_info, pointgrey_info):
        '''
        Organize calibration information for playback to read
        '''
        
        # Organize pointgrey information
        for k,v in pointgrey_info.iteritems():
            src_file = os.path.join(self.dest_raw_calib_dir, v+".yaml")
            dest_file = os.path.join(self.dest_calib_dir, k+"_color.yaml")
            self._copy_file(src_file, dest_file)
            
        # Organize kinect information
        for k,v in kinect_info.iteritems():
            
            # Load color information
            src_file = os.path.join(self.dest_raw_calib_dir, v,"calib_color.yaml")
            dest_file = os.path.join(self.dest_calib_dir, k+"_color.yaml")
            self._copy_file(src_file, dest_file)
            
            # Load ir information
            src_file = os.path.join(self.dest_raw_calib_dir, v,"calib_ir.yaml")
            dest_file = os.path.join(self.dest_calib_dir, k+"_depth.yaml")
            self._copy_file(src_file, dest_file)
            
            # Load pose information
            src_file = os.path.join(self.dest_raw_calib_dir, v,"calib_pose.yaml")
            dest_file = os.path.join(self.dest_calib_dir, k+"_pose.yaml")
            self._copy_file(src_file, dest_file)

        # Load extrinsic calibration information
        src_file = os.path.join(self.dest_raw_calib_dir, "camera_poses.yaml")
        dest_file = os.path.join(self.dest_calib_dir, "camera_poses.yaml")
        self._copy_file(src_file, dest_file)

        # Load vicon calibration information
        src_file = os.path.join(self.dest_raw_calib_dir, "vicon2k2.yaml")
        dest_file = os.path.join(self.dest_calib_dir, "vicon2k2.yaml")
        self._copy_file(src_file, dest_file)
        return    
        
    def _copy_file(self, src_file, dest_file):
        '''
        Copy src_file to dst_file
        '''
        if os.path.exists(src_file):
            shutil.copyfile(src_file, dest_file)
        else:
            rospy.logerr("[CalibrationLogger] Missing file: "+src_file)
        return
        
    
if __name__ == "__main__":
    CalibrationLogger();
    