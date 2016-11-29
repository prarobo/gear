#!/usr/bin/env python

from __future__ import print_function
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import cv2
import shutil
from decimal import Decimal, getcontext

VIDEO_ENCODING = cv2.cv.CV_FOURCC('M','J','P','G')
IMAGE_DIR = "images"
VIDEO_DIR = "videos"

class PostProcessor(object):
    def __init__(self, root_dir="/mnt/md0/gear_data", 
                 video_extn=".avi",
                 fps="15"):
        
        # Record inputs
        self.root_dir = root_dir
        self.video_extn = video_extn
        self.fps = fps
        self.video_enc = VIDEO_ENCODING
                                    
        return
        
    def _get_sensor_image_directories(self, task):
        '''
        Get the directories where images for each sensor are present
        '''
        image_root_dir = self.get_image_root_dir(task)
        sub_dir = []
        for d in os.listdir(image_root_dir):
            temp_path = os.path.join(image_root_dir, d)
            
            if os.path.isdir(temp_path) and len(os.listdir(temp_path)) != 0:
                sub_dir.append(temp_path) 
                   
        return sub_dir

    def _time_sequence_dir(self, image_dir):
        '''
        Load images from a directory along with the time sequence
        '''
        time_seq = [self._parse_image_names(f) for f in os.listdir(image_dir)]
        time_seq.sort()
        return time_seq
    
    def _parse_image_names(self, image_name):
        '''
        Get timestamp information from image name
        '''
        image_name,_=os.path.splitext(image_name)
        getcontext().prec=19
        timestamp = Decimal(image_name.split('_')[1]+'.'
                           +image_name.split('_')[2])
        return timestamp
    
    def _build_image_name(self, timestamp, image_extn, image_prefix):
        '''
        Build image filename from timestamp
        '''
        image_name = image_prefix+'_'+str(int(timestamp))+'_'\
                     +str(timestamp).split('.')[1]+image_extn
        return image_name
    
    def _build_video_name(self, task):
        '''
        Build video name from task
        '''
        return '_'.join(task) 
    
    def compute_time_seq_statistics(self, time_seq):
        '''
        Compute statistics related to time sequence
        '''
        time_diff = []
        if time_seq:
            for i in xrange(len(time_seq)-1):
                time_diff.append(time_seq[i+1]-time_seq[i])
                
            mean_time_diff = np.mean(np.array(time_diff))
            std_time_diff = np.std(np.array(time_diff),ddof=1)
            mean_fps = 1/mean_time_diff
            std_fps = 1/std_time_diff
        else:
            mean_time_diff = 0
            std_time_diff = 0
            mean_fps = 0
            std_fps = 0
            
        return mean_time_diff, std_time_diff, mean_fps, std_fps
    
    def plot_framerate_statistics(self, time_seq_list):
        '''
        Plot the statistics corresponding to frame rates of different streams
        '''        
        x = np.array(xrange(len(time_seq_list)+1))
        
        # Compute fps statistics
        mean_fps_list = [0]
        std_fps_list = [0]
        for t in time_seq_list:
            # Compute the time sequence statistics
            mean_time_diff, std_time_diff, mean_fps, std_fps = self.compute_timing_statistics(t)
            
            mean_fps_list.append(mean_fps)
            std_fps_list.append(std_fps)
            
        plt.xticks(x, [""]+[str(i) for i in xrange(len(time_seq_list))])
        plt.xlim( (0, len(keys)+1) ) 
        plt.errorbar(x, mean_fps_list, std_fps_list, linestyle='None', marker='^')
        plt.show()
        
    def get_min_max_time_seq(self, time_seq_list):
        '''
        Get latest starting and earliest ending time
        '''
        min_list = [i[0] for i in time_seq_list]
        max_list = [i[-1] for i in time_seq_list]
        
        min_time = max(min_list)
        max_time = min(max_list)
        
        print("Min time = {min_time}".format(min_time=min_time))
        print("Max time = {max_time}".format(max_time=max_time))
        return min_time, max_time
    
    def get_image_root_dir(self, task):
        '''
        Get the image root directory from task
        '''
        return os.path.join(self.root_dir, task[0], task[1], '_'.join(task[2:5]), IMAGE_DIR)

    def get_video_root_dir(self, task):
        '''
        Get the image root directory from task
        '''
        return os.path.join(self.root_dir, task[0], task[1], '_'.join(task[2:5]), VIDEO_DIR)
        
    def find_video_limits(self, task):
        '''
        Find the time limits for a video from the recorded images
        '''
        image_root_dir = self.get_image_root_dir(task)
        
        # Find sensor directories
        sensor_dir = self._get_sensor_image_directories(task)
        
        # Get the image time sequences
        time_seq_list = []
        for k in sensor_dir:
            image_dir = os.path.join(image_root_dir, k)
            time_seq_list.append(self._time_sequence_dir(image_dir))
            
        # Get the minimum and maximum time sequence points
        min_time, max_time = self.get_min_max_time_seq(time_seq_list)
        return min_time, max_time
        
    
    def _get_video_information(self, time_seq, min_time, max_time, image_dir, image_type):
        '''
        Get information about the video to be generated
        '''
        
        # Frame rate computation
        getcontext().prec=19
        time_diff = max_time-min_time
        num_frames = sum([min_time<=t<=max_time for t in time_seq])
        current_fps = Decimal(num_frames)/time_diff
        if int(round(current_fps)) != int(self.fps):
            print("Framerate mismatch: expected {e} current {c}".format(e=self.fps_,c=current_fps))
        
        # Get image extension
        image_name = os.listdir(image_dir)[0]
        image_prefix = image_name.split("_")[0]
        _, image_extn = os.path.splitext(image_name)
        
        # Get frame size
        image_path = os.path.join(image_dir, image_name)
        image = cv2.imread(image_path)
        height, width, _ = image.shape
        frame_size = (width, height)
        
        # Get color channels
        if image_type == "color":
            is_color = cv2.CV_LOAD_IMAGE_COLOR
        elif image_type == "depth":
            is_color = cv2.CV_LOAD_IMAGE_GRAYSCALE
        else:
            is_color = cv2.CV_LOAD_IMAGE_UNCHANGED
            
        return frame_size, is_color, current_fps, image_extn, image_prefix
        
    def create_video(self, task):
        '''
        Generate videos for the given task
        '''

        # Initialize directories
        image_root_dir = self.get_image_root_dir(task)
        video_root_dir = self.get_video_root_dir(task)
        image_dir = os.path.join(image_root_dir, '_'.join(task[-2:]))
        if not os.path.exists(video_root_dir):
            os.makedirs(video_root_dir)
        
        # Find the time limits of the video
        min_time, max_time = self.find_video_limits(task)
        time_seq = self._time_sequence_dir(image_dir)
                    
        # Get video information
        frame_size, is_color, current_fps, image_extn, image_prefix = self._get_video_information(time_seq, min_time, max_time, image_dir, task[-1])
        
        # Create video writer object
        video_name = self._build_video_name(task)
        video_path = os.path.join(video_root_dir, video_name+self.video_extn)
        video_writer = cv2.VideoWriter(video_path, self.video_enc, float(current_fps), frame_size, is_color)
        if not video_writer.isOpened():
            raise Exception("Failed to load video")
        
        print("Processing video "+video_name+ " ...", end="")
        for t in time_seq:
            if min_time<=t<=max_time:
                image_name = self._build_image_name(t, image_extn, image_prefix)
                image_path = os.path.join(image_dir, image_name)
                image = cv2.imread(image_path, is_color)
                video_writer.write(image)

        print("done")
        return
    
    def create_composition(self, composition_task):
        '''
        Compose multiple videos into a single video
        '''
        
        '''
        if not set(COMPOSITION_KEYS)<=set(self.sensor_dir_.keys()):
            rospy.logfatal("Data for video composition unavailable")
            return
        rospy.loginfo("Processing composition video"+ " ...")

        _, _, current_fps = self._getVideoInformation(COMPOSITION_KEYS[0])
        video_frame_size = np.round([COMPOSITION_FRAME_SIZE[0]/COMPOSITION_TILES[0], 
                                     COMPOSITION_FRAME_SIZE[1]/COMPOSITION_TILES[1]])  
        
        video_path = os.path.join(self.video_root_path_, "multi_view_composition"+self.video_extn_)
        video_writer = cv2.VideoWriter(video_path, self.video_format_, current_fps, tuple(COMPOSITION_FRAME_SIZE))                                
        if not video_writer.isOpened():
            raise Exception("Failed to load video")

        for t in self.time_seq_[COMPOSITION_KEYS[0]]:
            if self.start_time<t<self.end_time:
                images = []
                for v in COMPOSITION_KEYS:
                    image_name = self._buildImageName(t, self.image_extn_[v])
                    image_path = os.path.join(self.image_root_path_,v,image_name)
                    image = cv2.imread(image_path, 1)
                    images.append(cv2.resize(image, tuple(video_frame_size)))
        
                zero_image = np.zeros(np.append(video_frame_size,[3]),dtype=np.uint8)
                top_row = np.concatenate((images[0], zero_image), axis=1)
                bottom_row = np.concatenate((images[1], images[2]), axis=1)
                composed_image = np.concatenate((top_row, bottom_row), axis=0)
                video_writer.write(composed_image)
        rospy.loginfo("done")   
        '''               
        return
    
if __name__=="__main__":
    pass    
    
    
    
    