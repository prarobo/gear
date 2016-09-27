#!/usr/bin/env python

import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import cv2
import shutil
from decimal import Decimal, getcontext

COMPOSITION_KEYS = ["k3_color", "p1_color", "p2_color"]
COMPOSITION_FRAME_SIZE = [640, 480]
COMPOSITION_TILES = [2, 2]

class ImageReader(object):
    def __init__(self, data_dir="/mnt/md0/gear_data", 
                 image_root_name="images", 
                 video_root_name="videos",
                 image_prefix="im",
                 session_id="test", 
                 activity_id="act", 
                 trial_id="1", 
                 image_extn=".jpg",
                 video_extn=".avi",
                 video_format=cv2.cv.CV_FOURCC('M','J','P','G'),
                 fps="15",
                 time_offset=2):
        
        # Record inputs
        self.image_root_path_ = os.path.join(data_dir, session_id, 
                                             activity_id+'_'+trial_id, image_root_name)
        self.video_root_path_ = os.path.join(data_dir, session_id, 
                                             activity_id+'_'+trial_id, video_root_name)
        self.image_prefix_ = image_prefix

        self.image_extn_ = image_extn
        self.video_extn_ = video_extn
        self.video_format_ = video_format
        self.fps_ = fps
        self.time_offset_ = time_offset
                
        # Initialize directories
        if not os.path.exists(self.video_root_path_):
            os.makedirs(self.video_root_path_)
        
        # Find sensor directories
        self.sensor_dir_ = self._getSensorImageDirectories()
        
        # Get the image time sequence
        self.time_seq_ = {k:self._readTimeSequenceOfImagesFromDir(v)
                          for k,v in self.sensor_dir_.iteritems()}
        
        self.meanTimeDiff = {}
        self.stdTimeDiff = {}
        for k in self.time_seq_.keys():
            self.meanTimeDiff[k], self.stdTimeDiff[k] = self.computeTimingStatistics(self.time_seq_[k])
                    
        return
        
    def _getSensorImageDirectories(self):
        '''
        Get the directories where images for each sensor are present
        '''
        sub_dir = {d:os.path.join(self.image_root_path_,d) 
                   for d in os.listdir(self.image_root_path_) 
                   if os.path.isdir(os.path.join(self.image_root_path_, d))}
        return sub_dir

    def _readTimeSequenceOfImagesFromDir(self, image_dir):
        '''
        Load images from a directory along with the time sequence
        '''
        image_file_pattern = os.path.join(image_dir,"*"+self.image_extn_)
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
    
    def _buildImageName(self, timestamp):
        '''
        Build image filename from timestamp
        '''
        image_name = self.image_prefix_+'_'+str(int(timestamp))+'_'\
                     +str(timestamp).split('.')[1]+self.image_extn_
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

        self.num_frames = {k:sum(self.start_time<x<self.end_time for x in v)
                           for k,v in self.time_seq_.iteritems()}
        
        rospy.loginfo("Min time = {min_time}".format(min_time=self.start_time))
        rospy.loginfo("Max time = {max_time}".format(max_time=self.end_time))
        rospy.loginfo("Time difference = {time_diff}".format(time_diff=self.time_diff))
        rospy.loginfo("Number of frames", self.num_frames)
        return
    
    def _getVideoInformation(self, sensor_stream):
        '''
        Get information about the video to be generated
        '''
        
        # Frame rate
        getcontext().prec=19
        current_fps = Decimal(self.num_frames[sensor_stream])/self.time_diff
        assert  int(round(current_fps)) == int(self.fps_), \
        "Framerate mismatch: expected {e} current {c}".format(e=self.fps_,c=current_fps)
        
        # Frame size
        image_name = self._buildImageName(self.time_seq_[sensor_stream][0])
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
            
        return frame_size, is_color
        
    def encodeVideo(self):
        '''
        Generate videos from images
        '''
        for v in self.sensor_dir_.keys():
            frame_size, is_color = self._getVideoInformation(v)
            video_path = os.path.join(self.video_root_path_, v+self.video_extn_)
            video_writer = cv2.VideoWriter(video_path, self.video_format_, float(self.fps_), frame_size, is_color)
            
            if not video_writer.isOpened():
                raise Exception("Failed to load video")
            
            rospy.loginfo("Processing video"+ v+ " ...")
            for t in self.time_seq_[v]:
                if self.start_time<t<self.end_time:
                    image_name = self._buildImageName(t)
                    image_path = os.path.join(self.image_root_path_,v,image_name)
                    image = cv2.imread(image_path, is_color)
                    video_writer.write(image)
        return
    
    def composeVideo(self):
        '''
        Compose multiple videos into a single video
        '''
        
        assert set(COMPOSITION_KEYS)<=set(self.sensor_dir_.keys()), \
            "Data for video composition unavailable"
            
        for v in COMPOSITION_KEYS:
            pass
        return
    
if __name__=="__main__":
    image_reader = ImageReader()
    #image_reader.plotFramerateStatistics()
    image_reader.getMinMaxTimeSeq()
    image_reader.encodeVideo()
    image_reader.composeVideo()
    
    
    
    
    