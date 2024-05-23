#!/usr/bin/env python3
import rclpy
 
# Because of transformations
import tf_conversions
 
import math
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg as sensor_msgs
from rclpy.numpy_msg import numpy_msg
 
#camera1 = rclpy.Publisher('leftCamera', numpy_msg(sensor_msgs.PointCloud2), queue_size=1)

# TODO MTK: I am increadibly unhappy about this
t = geometry_msgs.msg.TransformStamped()
rclpy.init()
node = rclpy.create_node('baseLinkUpdater')

initLat = 0
initLon = 0
initAlt = 0


def tf_update_translation(msg :sensor_msgs.NavSatFix):
    br = tf2_ros.TransformBroadcaster()
    
    #cloud = pcl.PointCloud()
    #pclCloud = pcl.pcl_conversion.toPCL(msg, cloud)
    
    # Give the transform being published a timestamp
    t.header.stamp = node.get_clock.now()
     
     
    if(initLat == 0):
        initLat = msg.latitude
        initLon = msg.longitude
        initAlt = msg.altitude

    # sets transform
    t.transform.translation.x = msg.latitude - initLat
    t.transform.translation.y = msg.longitude - initLon
    t.transform.translation.z = msg.altitude - initAlt
    br.sendTransform(t)


def tf_update_rotation(msg):
    br = tf2_ros.TransformBroadcaster()

    # Give the transform being published a timestamp
    t.header.stamp = node.get_clock.now()
     
    euler = tf_conversions.transformations.euler_from_quaternion([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
    euler = list(euler)
    euler[0] = euler[0] + math.pi
    q = tf_conversions.transformations.quaternion_from_euler(*euler)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    #trans_pcl = pcl_ros.transformPointCloud(pclCloud, t)
 
    #camera1.publish(trans_pcl)
    
    # Pass in the transform and send it.
    br.sendTransform(t)
 
if __name__ == '__main__':
    # Set the name of the parent link
    t.header.frame_id = "map"
     
    # Set the name of the child node
    t.child_frame_id = "base_link"
  
    node.create_subscription(geometry_msgs.msg.QuaternionStamped, 'quat', tf_update_rotation)
    node.create_subscription(sensor_msgs.NavSatFix, 'gps_fix', tf_update_translation)
    rclpy.spin()
