#!/usr/bin/env python

import rospy
import cv2
from image_reader import ImageReader

def main():
    # Load parameters
    session_id = rospy.get_param("/session_id", "test")
    activity_id = rospy.get_param("/activity_id", "act")
    trial_id = rospy.get_param("/trial_id", 1)
    
    data_dir = rospy.get_param("~data_directory", "/mnt/md0/gear_data")
    image_root_name = rospy.get_param("~image_directory", "images")
    video_root_name = rospy.get_param("~video_directory", "videos")
    image_prefix = rospy.get_param("~image_prefix", "im")

    image_extn = rospy.get_param("~image_extension", ".jpg")
    video_extn = rospy.get_param("~video_extension", ".avi")
    fps = int(rospy.get_param("~frame_rate", "15"))
    time_offset = rospy.get_param("~time_offset", 0)
    
    video_format = cv2.cv.CV_FOURCC('M','J','P','G')
    
    rospy.loginfo("[ImageProcessor] Parameter session_id: "+session_id)
    rospy.loginfo("[ImageProcessor] Parameter activity_id: "+activity_id)
    rospy.loginfo("[ImageProcessor] Parameter trial_id: "+str(trial_id))
    rospy.loginfo("[ImageProcessor] Parameter data_directory: "+data_dir)
    rospy.loginfo("[ImageProcessor] Parameter image_directory: "+image_root_name)
    rospy.loginfo("[ImageProcessor] Parameter video_directory: "+video_root_name)
    rospy.loginfo("[ImageProcessor] Parameter image_prefix: "+image_prefix)
    rospy.loginfo("[ImageProcessor] Parameter video_extn: "+video_extn)
    rospy.loginfo("[ImageProcessor] Parameter frame_rate: "+str(fps))
    rospy.loginfo("[ImageProcessor] Parameter time_offset: "+str(time_offset))
    rospy.loginfo("[ImageProcessor] Parameter video_format: "+str(video_format))
        
    # Create image reader object
    image_reader = ImageReader(data_dir, image_root_name, video_root_name,
                               image_prefix, session_id, activity_id, 
                               trial_id, video_extn,
                               video_format, fps, time_offset)
    
    #image_reader.plotFramerateStatistics()
    image_reader.getMinMaxTimeSeq()
    image_reader.encodeVideo()
    image_reader.composeVideo()
    
    return


if __name__=="__main__":
    try:
        rospy.init_node('image_processor')
        main()
        while not rospy.is_shutdown():
            pass        
    except rospy.ROSInterruptException:
        pass

