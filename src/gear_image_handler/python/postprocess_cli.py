#!/usr/bin/env python

import cv2
import argparse
from postprocessor import PostProcessor

COMPOSITION_PAIRS = [("k2","color"), ("p1","color"), ("p2","color")]
def main(info):
    
    # Create postprocessing object
    post_processor_obj = PostProcessor(info.root_dir, info.video_extn, info.fps)
    
    # Create a video task
    task = (info.subject, info.session, info.activity, info.condition, str(info.trial), info.sensor, info.video)
    
    # Generate video
    post_processor_obj.create_video(task)
    
    # Create composition
    composition_task = []
    for c in COMPOSITION_PAIRS:
        composition_task.append((info.subject, info.session, info.activity, info.condition, info.trial)+c)
    
    # Do composition video
    if info.do_composition:
        post_processor_obj.create_composition(composition_task)        
    
    #image_reader.plotFramerateStatistics()
    #image_reader.getMinMaxTimeSeq()
    #image_reader.encodeVideo()
    #image_reader.composeVideo()
    
    return

def parse_arguments():
    '''
    Parse commandline arguments
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument('--root-dir', required=False, nargs='?', type=str, default="/mnt/md0/gear_data/", help='Root directory where data is stored')
    parser.add_argument('--video-extn', required=False, nargs='?', type=str, default=".avi", help='Extension of video to be saved')
    parser.add_argument('--fps', required=False, nargs='?', type=int, default=15, help='Expected video framerate')
    parser.add_argument('--do-composition', required=False, action="store_true", help='Flag to do compostion video (Default: False)')
    parser.add_argument('--subject', required=True, nargs='?', type=str, help='Subject')
    parser.add_argument('--session', required=True, nargs='?', type=str, help='Session')
    parser.add_argument('--activity', required=True, nargs='?', type=str, help='Activity')
    parser.add_argument('--condition', required=True, nargs='?', type=str, help='Condition')
    parser.add_argument('--trial', required=True, nargs='?', type=int, help='Trial')
    parser.add_argument('--sensor', required=True, nargs='?', type=str, help='Sensor')
    parser.add_argument('--video', required=True, nargs='?', type=str, help='Video Type (color, depth, etc)')
    args = parser.parse_args()
    
    print("[ImageProcessor] Parameter subject: "+args.subject)
    print("[ImageProcessor] Parameter session: "+args.session)
    print("[ImageProcessor] Parameter activity: "+args.activity)
    print("[ImageProcessor] Parameter condition: "+args.condition)
    print("[ImageProcessor] Parameter trial: "+str(args.trial))
    print("[ImageProcessor] Parameter sensor: "+args.sensor)
    print("[ImageProcessor] Parameter video_type: "+args.video)
    print("[ImageProcessor] Parameter root_directory: "+args.root_dir)
    print("[ImageProcessor] Parameter video_extn: "+args.video_extn)
    print("[ImageProcessor] Parameter expected_frame_rate: "+str(args.fps))

    return args

if __name__=="__main__":
    info = parse_arguments()
    main(info)

