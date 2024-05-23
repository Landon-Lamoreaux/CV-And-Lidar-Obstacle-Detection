import pyrealsense2.pyrealsense2 as rs

# Import Numpy for easy array manipulation.
import numpy as np

# Import OpenCV for image viewing.
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
import std_msgs.msg as std_msgs
from geometry_msgs.msg import Point32
import sensor_msgs.msg as sensor_msgs
import sys

# Create a pipeline.
pipeline1 = rs.pipeline()

# Create a config and configure the pipeline to stream
# different resolutions of color and depth streams.
config1 = rs.config()
config1.enable_device(sys.argv[1])
profile1 = config1.resolve(pipeline1)

# Enable depth and color streams.
config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Create a pipeline.
pipeline2 = rs.pipeline()

config2 = rs.config()
config2.enable_device(sys.argv[2])
profile2 = config2.resolve(pipeline2)

print(sys.argv[1])
print(sys.argv[2])

# Enable depth and color streams.
config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming.
profile1 = pipeline1.start(config1)
profile2 = pipeline2.start(config2)

# Create an align object.
# rs.align allows us to perform alignment of depth frames to others frames.
# The "align_to" is the stream type to which we plan to align depth frames.
align = rs.align(rs.stream.color)

pub1 = rospy.Publisher('camera1Pre', numpy_msg(sensor_msgs.Image), queue_size=1)
pub2 = rospy.Publisher('camera2Pre', numpy_msg(sensor_msgs.Image), queue_size=1)
rospy.init_node('cameraAligning', anonymous=True)
r = rospy.Rate(20)

try:
    while True:
        # Getting depth and color frames.
        frames1 = pipeline1.wait_for_frames()
        frames2 = pipeline2.wait_for_frames()

        # Aligning the depth frame to the color frame.
        aligned_frames1 = align.process(frames1)
        aligned_frames2 = align.process(frames2)

        # Getting aligned frames.
        depth_frame1 = aligned_frames1.get_depth_frame()
        color_frame1 = aligned_frames1.get_color_frame()
        
        depth_frame2 = aligned_frames2.get_depth_frame()
        color_frame2 = aligned_frames2.get_color_frame()
        
        r.sleep()
        
finally:
    
    # Stopping both pipelines and destroying all the cv windows.
    pipeline1.stop()
    pipeline2.stop()
    cv2.destroyAllWindows()