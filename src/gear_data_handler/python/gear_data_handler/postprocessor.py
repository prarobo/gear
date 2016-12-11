#!/usr/bin/env python

from __future__ import print_function
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import cv2
import shutil
from decimal import Decimal, getcontext
import sys

VIDEO_ENCODING = cv2.cv.CV_FOURCC('M','J','P','G')
IMAGE_DIR = "images"
VIDEO_DIR = "videos"
SCALING_FACTOR = 0.25

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

    @classmethod    
    def build_video_name(self, task):
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
            mean_fps = np.mean(np.array(np.reciprocal(time_diff)))
            std_fps = np.std(np.array(np.reciprocal(time_diff)),ddof=1)
        else:
            mean_time_diff = 0
            std_time_diff = 0
            mean_fps = 0
            std_fps = 0
            
        return mean_time_diff, std_time_diff, mean_fps, std_fps
    
    def generate_framerate_statistics(self, task):
        '''
        Generate framerate related statistics
        '''
        # Generate the time sequence list for all sensor streams
        time_seq_list = self.generate_time_seq_list(task)
 
        # Plot the framerate statistics
        self.plot_framerate_statistics(time_seq_list)
        return
    
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
            mean_time_diff, std_time_diff, mean_fps, std_fps = self.compute_time_seq_statistics(t)
            
            mean_fps_list.append(mean_fps)
            std_fps_list.append(std_fps)
            
        plt.xticks(x, [""]+[str(i) for i in xrange(len(time_seq_list))])
        plt.xlim( (0, len(time_seq_list)+1) ) 
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
        
        #print("Min time = {min_time}".format(min_time=min_time))
        #print("Max time = {max_time}".format(max_time=max_time))
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
    
    def generate_time_seq_list(self, task):
        '''
        Get the time sequence for all sensor streams
        '''
        image_root_dir = self.get_image_root_dir(task)
        
        # Find sensor directories
        sensor_dir = self._get_sensor_image_directories(task)
        
        # Get the image time sequences
        time_seq_list = []
        for k in sensor_dir:
            image_dir = os.path.join(image_root_dir, k)
            time_seq_list.append(self._time_sequence_dir(image_dir))
        return time_seq_list
    
    def find_video_limits(self, task):
        '''
        Find the time limits for a video from the recorded images
        '''
        # Generate the time sequence list for all sensor streams
        time_seq_list = self.generate_time_seq_list(task)
        
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
        
    def _initialize_video_info(self, task):
        '''
        Get the initialize video information
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
        frame_size, is_color, current_fps, image_extn, image_prefix \
            = self._get_video_information(time_seq, min_time, max_time, image_dir, task[-1])
                
        return image_root_dir, video_root_dir, frame_size, is_color, current_fps, \
                image_extn, image_prefix, time_seq, min_time, max_time, image_dir
    
    def create_video(self, task):
        '''
        Generate videos for the given task
        '''
        # Initialize directories
        image_root_dir, video_root_dir, frame_size, is_color, current_fps, \
            image_extn, image_prefix, time_seq, min_time, max_time, image_dir \
            = self._initialize_video_info(task)
                
        # Create video writer object
        video_name = self.build_video_name(task)
        video_path = os.path.join(video_root_dir, video_name+self.video_extn)
        video_writer = cv2.VideoWriter(video_path, self.video_enc, float(current_fps), frame_size, is_color)
        if not video_writer.isOpened():
            sys.stderr.out("Failed to load video")
            sys.stdout.flush()
        
        sys.stdout.write("Processing video "+video_name+self.video_extn+" ...\n")
        sys.stdout.flush()
        for t in time_seq:
            if min_time<=t<=max_time:
                image_name = self._build_image_name(t, image_extn, image_prefix)
                image_path = os.path.join(image_dir, image_name)
                image = cv2.imread(image_path, is_color)
                video_writer.write(image)
        return
    
    def create_composition(self, composition_task):
        '''
        Compose multiple videos into a single video
        '''       
        # Initialization
        image_extn_list = []
        image_prefix_list = []
        time_seq_list = []
        image_dir_list = []
        image_root_dir, video_root_dir, frame_size, is_color, current_fps, \
            image_extn, image_prefix, time_seq, min_time, max_time, image_dir \
            = self._initialize_video_info(composition_task[0])
        frame_size_list = [np.round([f*SCALING_FACTOR for f in frame_size])]
        image_extn_list.append(image_extn)
        image_prefix_list.append(image_prefix)
        time_seq_list.append(time_seq)
        image_dir_list.append(image_dir)
        
        # Get the composition frame sizes and other video specific parameters
        for i in xrange(1,len(composition_task)):
            _, _, frame_size, _, _, image_extn, image_prefix, time_seq, _, _, image_dir \
                = self._initialize_video_info(composition_task[i])
            scaling_factor = frame_size_list[0][1]/frame_size[1]
            dim2 = round(frame_size[0]*scaling_factor)
            frame_size_list.append(np.array([dim2, frame_size_list[0][1]]))
            time_seq_list.append(time_seq)
            image_extn_list.append(image_extn)
            image_prefix_list.append(image_prefix)
            image_dir_list.append(image_dir)
        
        # Create video writer object
        video_frame_size = tuple([int(sum([f[0] for f in frame_size_list])), int(frame_size_list[0][1])])                                     
        video_name = self.build_composition_name(composition_task)
        video_path = os.path.join(video_root_dir, video_name+self.video_extn)
        video_writer = cv2.VideoWriter(video_path, self.video_enc, float(current_fps), video_frame_size, is_color)
        if not video_writer.isOpened():
            sys.stderr.out("Failed to load video")
            sys.stdout.flush()
            return
        
        # Cleanup unwanted values from time sequences
        for t in time_seq_list:
            while t[0] < min_time:
                t.pop(0)
            while t[-1] > max_time:
                t.pop()
        
        # Sanity check to match time sequence lengths
        for i in xrange(1, len(time_seq_list)):
            if len(time_seq_list[i-1]) != len(time_seq_list[i]):
                sys.stderr.write("Time sequence list does not match")
                sys.stdout.flush()
        
        sys.stdout.write("Processing composition video "+video_name+self.video_extn+" ...\n")
        sys.stdout.flush()
        
        for t in time_seq_list[0]:
            if min_time<=t<=max_time:
                images = []
                for i in xrange(len(time_seq_list)):
                    image_name = self._build_image_name(t, image_extn_list[i], image_prefix_list[i])
                    image_path = os.path.join(image_dir_list[i], image_name)
                    image = cv2.imread(image_path, is_color)
                    image_resized = cv2.resize(image, tuple([int(f) for f in frame_size_list[i]]))
                    images.append(image_resized)

                composed_image = np.concatenate(tuple(images), axis=1)
                video_writer.write(composed_image)
        return
    
    @classmethod
    def build_composition_name(self, composition_task):
        '''
        Get the name of composition from composition task
        '''
        c_name = []
        for i in zip(*composition_task):
            c_name.append(''.join(sorted(set(list(i)))))
        video_name = "composition_"+'_'.join(c_name)
        return video_name

    
if __name__=="__main__":
    pass    
    
    
    
    