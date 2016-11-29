#!/usr/bin/env python

import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import cv2
import shutil
import rospy
from decimal import Decimal, getcontext
import rospy

COMPOSITION_KEYS = ["p1_color","k2_color", "p2_color"]
COMPOSITION_FRAME_SIZE = [1000, 1000]
COMPOSITION_TILES = [2, 2]

class ImageReader(object):
    def __init__(self, data_dir="/mnt/md0/gear_data", 
                 image_root_name="images", 
                 video_root_name="videos",
                 image_prefix="im",
                 session_id="test", 
                 activity_id="act", 
                 trial_id="1", 
                 video_extn=".avi",
                 video_format=cv2.cv.CV_FOURCC('M','J','P','G'),
                 fps="15",
                 time_offset=2):
        
        # Record inputs
        self.image_root_path_ = os.path.join(data_dir, session_id, 
                                             activity_id+'_'+str(trial_id), image_root_name)
        self.video_root_path_ = os.path.join(data_dir, session_id, 
                                             activity_id+'_'+str(trial_id), video_root_name)
        self.image_prefix_ = image_prefix

        self.video_extn_ = video_extn
        self.video_format_ = video_format
        self.fps_ = fps
        self.time_offset_ = time_offset
                
        # Initialize directories
        if not os.path.exists(self.video_root_path_):
            os.makedirs(self.video_root_path_)
        
        # Find sensor directories
        self.sensor_dir_, self.image_extn_ = self._getSensorImageDirectories()
        
        # Get the image time sequence
        self.time_seq_ = {}
        for k in self.sensor_dir_.keys():
            self.time_seq_[k] = self._readTimeSequenceOfImagesFromDir(self.sensor_dir_[k], self.image_extn_[k])
        
        self.meanTimeDiff = {}
        self.stdTimeDiff = {}
        for k in self.time_seq_.keys():
            self.meanTimeDiff[k], self.stdTimeDiff[k] = self.computeTimingStatistics(self.time_seq_[k])
                    
        return
        
    def _getSensorImageDirectories(self):
        '''
        Get the directories where images for each sensor are present
        '''
        sub_dir = {}
        image_extn = {}
        for d in os.listdir(self.image_root_path_):
            temp_path = os.path.join(self.image_root_path_, d)
            
            if os.path.isdir(temp_path) and len(os.listdir(temp_path)) != 0:
                sub_dir[d] = temp_path 
                filename = os.path.join(temp_path, os.listdir(temp_path)[0])
                image_extn[d] = os.path.splitext(filename)[1]
                   
        return sub_dir, image_extn

    def _readTimeSequenceOfImagesFromDir(self, image_dir, image_extn):
        '''
        Load images from a directory along with the time sequence
        '''
        image_file_pattern = os.path.join(image_dir,"*"+image_extn)
        images = [os.path.basename(f) for f in glob.glob(image_file_pattern)]
        time_seq = [self._parseImageNames(f) for f in images]
        time_seq.sort()
        return time_seq
    
    def _parseImageNames(self, image_name):
        '''
        Get timestamp information from image name
        '''
        image_name,_=os.path.splitext(image_name)
        getcontext().prec=19
        timestamp = Decimal(image_name.split('_')[1]+'.'
                           +image_name.split('_')[2])
        return timestamp
    
    def _buildImageName(self, timestamp, image_extn):
        '''
        Build image filename from timestamp
        '''
        image_name = self.image_prefix_+'_'+str(int(timestamp))+'_'\
                     +str(timestamp).split('.')[1]+image_extn
        return image_name
    
    def computeTimingStatistics(self, time_seq):
        '''
        Compute statistics related to time sequence
        '''
        timeDiff = []
        if time_seq:
            for i in xrange(len(time_seq)-1):
                timeDiff.append(time_seq[i+1]-time_seq[i])
                
            meanTimeDiff = np.mean(np.array(timeDiff))
            stdTimeDiff = np.std(np.array(timeDiff),ddof=1)
        else:
            meanTimeDiff = 0
            stdTimeDiff = 0
            
        return meanTimeDiff, stdTimeDiff
    
    def plotFramerateStatistics(self):
        '''
        Plot the statistics corresponding to frame rates of different streams
        '''        
        x = np.array(xrange(len(self.meanTimeDiff.keys())+1))
        keys = self.meanTimeDiff.keys()
        
        mean_fps = [0]
        std_fps = [0]
        for k in keys:
            mean_fps.append(1/self.meanTimeDiff[k] if self.meanTimeDiff[k] != 0 else 0)
            std_fps.append(1/self.stdTimeDiff[k] if self.stdTimeDiff[k] != 0 else 0)

        plt.xticks(x, [""]+keys)
        plt.xlim( (0, len(keys)+1) ) 
        plt.errorbar(x, mean_fps, std_fps, linestyle='None', marker='^')
        plt.show()
        
    def getMinMaxTimeSeq(self):
        '''
        Get latest starting and earliest ending time
        '''
        min_list = [i[0] for i in self.time_seq_.values()]
        max_list = [i[-1] for i in self.time_seq_.values()]
        
        self.start_time = max(min_list)+self.time_offset_
        self.end_time = min(max_list)-self.time_offset_
        self.time_diff = self.end_time-self.start_time

        self.num_frames = {k:sum(self.start_time<=x<=self.end_time for x in v)
                           for k,v in self.time_seq_.iteritems()}
        
        rospy.loginfo("Min time = {min_time}".format(min_time=self.start_time))
        rospy.loginfo("Max time = {max_time}".format(max_time=self.end_time))
        rospy.loginfo("Time difference = {time_diff}".format(time_diff=self.time_diff))
        rospy.loginfo("Number of frames: "+str(self.num_frames))
        return
    
    def _getVideoInformation(self, sensor_stream):
        '''
        Get information about the video to be generated
        '''
        
        # Frame rate
        getcontext().prec=19
        current_fps = Decimal(self.num_frames[sensor_stream])/self.time_diff
        #assert  int(round(current_fps)) == int(self.fps_), \
        #"Framerate mismatch: expected {e} current {c}".format(e=self.fps_,c=current_fps)
        if int(round(current_fps)) != int(self.fps_):
            rospy.logerr("Framerate mismatch: expected {e} current {c}".format(e=self.fps_,c=current_fps))
        
        # Frame size
        image_name = self._buildImageName(self.time_seq_[sensor_stream][0], self.image_extn_[sensor_stream])
        image_path = os.path.join(self.image_root_path_, sensor_stream, image_name)
        image = cv2.imread(image_path)
        height, width, _ = image.shape
        frame_size = (width, height)
        
        # Color channels
        if "color" in sensor_stream:
            is_color = cv2.CV_LOAD_IMAGE_COLOR
        elif "depth" in sensor_stream:
            is_color = cv2.CV_LOAD_IMAGE_GRAYSCALE
        else:
            is_color = cv2.CV_LOAD_IMAGE_UNCHANGED
            
        return frame_size, is_color, current_fps
        
    def encodeVideo(self):
        '''
        Generate videos from images
        '''
        for v in self.sensor_dir_.keys():
            frame_size, is_color, current_fps = self._getVideoInformation(v)
            video_path = os.path.join(self.video_root_path_, v+self.video_extn_)
            video_writer = cv2.VideoWriter(video_path, self.video_format_, float(current_fps), frame_size, is_color)
            
            if not video_writer.isOpened():
                raise Exception("Failed to load video")
            
            rospy.loginfo("Processing video "+v+ " ...")
            for t in self.time_seq_[v]:
                if self.start_time<t<self.end_time:
                    image_name = self._buildImageName(t, self.image_extn_[v])
                    image_path = os.path.join(self.image_root_path_,v,image_name)
                    image = cv2.imread(image_path, is_color)
                    video_writer.write(image)
                    
                    # Play video
                    # cv2.imshow('image',image)
                    # cv2.waitKey(10)
            rospy.loginfo("done")
        return
    
    def composeVideo(self):
        '''
        Compose multiple videos into a single video
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
        return
    
if __name__=="__main__":
    image_reader = ImageReader()
    #image_reader.plotFramerateStatistics()
    image_reader.getMinMaxTimeSeq()
    image_reader.encodeVideo()
    image_reader.composeVideo()
    
    
    
    
    