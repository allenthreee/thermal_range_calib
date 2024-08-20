#!/usr/bin/env python

import rosbag
import cv2
from cv_bridge import CvBridge
import os

# Initialize the CvBridge class
bridge = CvBridge()

# Path to your ROS bag file
bag_file = '/home/allen/calib_data/ign_simulation/ldiar80.5_thermal810.5/thermal_640x480.bag'

# Output directory for the images
output_dir = 'output_images'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Open the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/thermal_camera']):
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Create a filename based on the timestamp
        timestamp = t.to_nsec()
        filename = os.path.join(output_dir, 'image_{}.png'.format(timestamp))

        # Save the image as a PNG file
        cv2.imwrite(filename, cv_image)

        print('Saved {}'.format(filename))
