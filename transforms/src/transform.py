#!/usr/bin/env python3
import rospy
 
# Because of transformations
import tf_conversions
 
import math
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg as sensor_msgs
from rospy.numpy_msg import numpy_msg
 
#camera1 = rospy.Publisher('leftCamera', numpy_msg(sensor_msgs.PointCloud2), queue_size=1)

def tf_pointcloud(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    #cloud = pcl.PointCloud()
    #pclCloud = pcl.pcl_conversion.toPCL(msg, cloud)
    
    # Give the transform being published a timestamp
    t.header.stamp = rospy.Time.now()
     
    # Set the name of the parent link
    t.header.frame_id = "map"
     
    # Set the name of the child node
    t.child_frame_id = "base_link"
     
    # sets transform
    t.transform.translation.x = 0.5
    t.transform.translation.y = 0.3
    t.transform.translation.z = 0.1
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
    rospy.init_node('baseLinkUpdater')
  
    rospy.Subscriber('quat', geometry_msgs.msg.QuaternionStamped, tf_pointcloud)
    rospy.spin()
