#!/usr/bin/python3
# Importing the pyrealsense2 library functions.
import pyrealsense2.pyrealsense2 as rs
from pyrealsense2.pyrealsense2 import rs2_deproject_pixel_to_point
from pyrealsense2.pyrealsense2 import stream
from pyrealsense2.pyrealsense2 import pipeline
from pyrealsense2.pyrealsense2 import config
from pyrealsense2.pyrealsense2 import format
from pyrealsense2.pyrealsense2 import align

# Import Numpy for easy array manipulation.
import numpy as np
from numpy import asanyarray
from numpy import array
from numpy import ones
from numpy import argwhere

# Import OpenCV for image manipulation.
import rospy
from rospy.numpy_msg import numpy_msg
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
from time import time
import sys
from threading import Thread
from cv2 import cvtColor
from cv2 import COLOR_BGR2GRAY
from cv2 import COLOR_BGR2HLS
from cv2 import waitKey
from cv2 import destroyAllWindows
from cv2 import inRange
from cv2 import bitwise_and
from cv2 import bitwise_or
from cv2 import Canny
from cv2 import imshow
from cv2 import dilate
from cv2 import erode

# There is a comment at the bottom of the file to start the program with the serial numbers as arguments.

# Create a pipeline.
pipeline1 = pipeline()

# Create a config and configure the pipeline to stream
# different resolutions of color and depth streams.
config1 = config()
config1.enable_device(sys.argv[1])
profile1 = config1.resolve(pipeline1)

# Enable depth and color streams.
config1.enable_stream(stream.depth, 640, 480, format.z16, 30)
config1.enable_stream(stream.color, 640, 480, format.bgr8, 30)

# Create a pipeline.
pipeline2 = pipeline()

config2 = config()
config2.enable_device(sys.argv[2])
profile2 = config2.resolve(pipeline2)

# Enable depth and color streams.
config2.enable_stream(stream.depth, 640, 480, format.z16, 30)
config2.enable_stream(stream.color, 640, 480, format. bgr8, 30)

# Start streaming.
profile1 = pipeline1.start(config1)
profile2 = pipeline2.start(config2)

# Create an align object.
# rs.align allows us to perform alignment of depth frames to others frames.
# The "align_to" is the stream type to which we plan to align depth frames.
align = align(stream.color)

# Define upper and lower HSV Thresholds
lower1 = array([0,170,200])
upper1 = array([255,255,255])

lower2 = array([90,90,90])
upper2 = array([120,130,150])


pub1 = rospy.Publisher('camera1Pre', numpy_msg(PointCloud2), queue_size=1)
pub2 = rospy.Publisher('camera2Pre', numpy_msg(PointCloud2), queue_size=1)
rospy.init_node('cameraProcessing', anonymous=True)
r = rospy.Rate(30)

# Getting the intrinsics of the color stream to use in rs2.deproject_pixel_to_point.
color_intrin = profile1.get_stream(stream.color).as_video_stream_profile().get_intrinsics()

# Creating a kernel to erode and dilate with.
kernel = ones((2,2), np.uint8)


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )

def get_and_pub_obstacles(erod_masked, color_intrin, depth_frame, cameraPub, frameID):

    # Finding the locations of obstacles in the camera image.
    data_loc = argwhere(erod_masked != 0)[:,0:2]
    
    #  Creating an array of 1's to hold the 3D points of the obstacles.
    ob_array = ones((data_loc.shape[0], 7))
    
    
    # Finding the 3D points of the obstacles and filling the first 3 columns of ob_array with these points.
    i = 0
    size = data_loc.shape[0] / 2
    size -= 1
    while i < size:
        # Getting the 3D world point of obstacles from the 2D coordinates of the pixal in an image.
        point = (data_loc[i][1], data_loc[i][0])
        ob_array[i,0:3] = rs2_deproject_pixel_to_point(color_intrin, point, depth_frame.get_distance(point[0], point[1]))
        i += 2
        
    # Publishing a pointcloud of the obstacles to the ros topic "obstacles".
    cameraPub.publish(point_cloud(ob_array, frameID))
    
    return


def processCamera(frames, pub, sub_name):

    # Aligning the depth frame to the color frame.
    aligned_frames = align.process(frames)
    
    # Seperating the aligned depth and color frames.
    depth = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    # Converting the color frame to a numpy array.
    color = asanyarray(color_frame.get_data())
    
    # Converting the bgr image to a grey image and a HLS image.
    gray = cvtColor(color, COLOR_BGR2GRAY)
    hls = cvtColor(color, COLOR_BGR2HLS)
    
    # Finding the white lines out of shadow.
    inrange1 = inRange(hls, lower1, upper1)
    
    # Finding the white lines in shadow.
    inrange2 = inRange(hls, lower2, upper2)
    
    # Combining the 2 previous images to get all the white lines.
    inrange = bitwise_or(inrange1, inrange2)
    
    # Using OpenCV's Canny function to find all the edges in the image.
    canny = Canny(gray, 200, 400)
    
    # Dilating the lines to make them thicker.
    canny = dilate(canny, kernel, iterations=3)
    
    # Getting only the pixels in both images. So we just have the edges of the white lines.
    masked = bitwise_and(inrange, canny)
    
    # Chopping off the top half of the image.
    masked[0:int(masked.shape[0]/2),:] = 0

    masked = cv2.dilate(masked, kernel, iterations=1)
    
    # Uncomment to view what camera 1 sees.
    # if sub_name == "camera1":
    #     imshow(sub_name + 'InRange', inrange)
    #     imshow(sub_name + 'Canny', canny)
    #     imshow(sub_name + "Masked", masked)
    #     imshow(sub_name + ' Color', color)
    
    # Calling our function to create and publish a pointcloud of obstacles.
    get_and_pub_obstacles(masked, color_intrin, depth, pub, sub_name)
        
    return

now = time()

# Script loop.
try:
    while 1:
        # Getting depth and color frames.
        frames1 = pipeline1.wait_for_frames()
        frames2 = pipeline2.wait_for_frames()
        
        # Creating 2 threads, one for each of our cameras.
        t1 = Thread(target=processCamera, args=(frames1, pub1, "camera1"))
        t2 = Thread(target=processCamera, args=(frames2, pub2, "camera2"))
        
        # Starting the threads.
        t1.start()
        t2.start()
        
        # Waiting for both threads to finish before moving on.
        t1.join()
        t2.join()
        
        # Timing the time between frames.
        # print(time() - now)
        # now = time()
        
        # Uncomment to see OpenCV windows.
        # waitKey(1)
        
        # Setting the program to only run a certain number of times a second.
        r.sleep()
        
finally:
    
    # Stopping both pipelines and destroying all the cv windows.
    pipeline1.stop()
    pipeline2.stop()
    destroyAllWindows()
