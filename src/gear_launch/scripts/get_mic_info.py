#!/usr/bin/python

import pyaudio 
import re
import sys

# Name of the mic we are looking for
MIC_NAME = "USB audio CODEC: Audio"

# Retrieve sound card information data
dev_data = pyaudio.PyAudio()
num_devs = dev_data.get_device_count()

mic_found = False

# Iterate over all devices and identify the required device
for i in xrange(num_devs):
    dev_info = dev_data.get_device_info_by_index(i)
    
    # Identify mic based on device name
    if MIC_NAME in dev_info["name"]:
        
        # Regex matching to get hardware id
        dev_match = re.match( r'(.*)\(hw:(?P<card_id>[0-9]+),(?P<dev_id>[0-9]+)\)$', dev_info["name"])
        sys.stdout.write("hw:"+dev_match.group("card_id")+","+dev_match.group("dev_id"))
        mic_found = True

# Use default value if microphone is not found
if not mic_found:
    sys.stdout.write("")