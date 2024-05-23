#!/usr/bin/python3

# First import the library.
import pyrealsense2.pyrealsense2 as rs

# Import Numpy for easy array manipulation.
import numpy as np

# Import OpenCV for easy image rendering.
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
import std_msgs.msg as std_msgs
import scipy.ndimage
from skimage.draw import disk
from numpy_ros import to_numpy, to_message
from geometry_msgs.msg import Point32
import sensor_msgs.msg as sensor_msgs
import time

# Create a pipeline.
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
# different resolutions of color and depth streams.
config = rs.config()

# Enable depth and color streams.
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming.
profile = pipeline.start(config)

# Create an align object.
# rs.align allows us to perform alignment of depth frames to others frames.
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Define upper and lower HSV Thresholds
lower_white = np.array([0,0,168])
upper_white = np.array([255,111,255])


pub = rospy.Publisher('camera1Pre', numpy_msg(sensor_msgs.PointCloud2), queue_size=1)
rospy.init_node('processing', anonymous=True)
r = rospy.Rate(4)

# Getting the intrinsics of the color stream to use in rs2.deproject_pixel_to_point.
color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
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

def get_and_pub_obstacles(depth_map, color_intrin, depth_frame):
    
    # Setting anything that is nan to a 0.
    #depth_map[np.isnan(depth_map)] = 0

    data_loc = np.argwhere(depth_map != 0)[:,0:2]
    
    ob_array = np.ones((data_loc.shape[0], 7))
    
    
    # Finding the 3D points of the obstacles and filling the first 3 columns of ob_array with these points.
    i = 0
    size = data_loc.shape[0] / 2
    size -= 1
    while i < size:
        point = (data_loc[i][1], data_loc[i][0])
        ob_array[i,0:3] = rs.rs2_deproject_pixel_to_point(color_intrin, point, depth_frame.get_distance(point[0], point[1]))
        i += 2
    # Publishing a pointcloud of the obstacles to the ros topic "obstacles".
    pub.publish(point_cloud(ob_array, "camera1"))
    
    return

# Creating a cirle shape to erode our masked image with.
img = np.zeros((25,25), dtype=np.uint8)
rr, cc = disk((12,12), 5)
img[rr, cc] = 1

now = time.time()

# Script loop.
try:
    while True:
        # Getting depth and color frames.
        frames = pipeline.wait_for_frames()

        # Aligning the depth frame to the color frame.
        aligned_frames = align.process(frames)

        # Getting aligned frames.
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        # Converting frames to numpy arrays.
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert frame to HSV
        color_hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Create masks of the frame and depth data where there are white pixels.
        masked = cv2.inRange(color_hsv, lower_white, upper_white)
        erod_masked = np.asanyarray(scipy.ndimage.binary_erosion(masked, structure=img), dtype=np.uint8)

        erod_masked[0:int(erod_masked.shape[0]/2),:] = np.zeros((int(erod_masked.shape[0]/2), erod_masked.shape[1]))
        
        # Overlaying the depth image with the eroded masked image.
        depth_mask = cv2.bitwise_and(depth_image, depth_image, mask=erod_masked)

        # Creating a color map of the masked depth image for visual debugging.
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_mask, alpha=0.43), cv2.COLORMAP_JET)
        
        # Displaying the different images to the screen.
        #cv2.imshow('Color', color_image)
        #cv2.imshow('Eroded Mask', erod_masked * 255)
        #cv2.imshow('Mask', masked)
        #cv2.imshow('Depth', depth_colormap)
        #cv2.waitKey(1)
        
        # Create an obstacles list and publish it to the obstacles topic.
        obstacle_array = get_and_pub_obstacles(erod_masked, color_intrin, depth_frame)
        
        r.sleep()
        
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
