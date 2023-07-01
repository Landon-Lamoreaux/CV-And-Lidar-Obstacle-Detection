#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Setting up global variables.
ros::Publisher *pub1;
ros::Publisher *pub2;
tf2_ros::Buffer *tfBuffer1;
tf2_ros::Buffer *tfBuffer2;


void camera1Transform(sensor_msgs::PointCloud2 cloudIn);

void camera2Transform(sensor_msgs::PointCloud2 cloudIn);

int main(int argc, char** argv)
{
    // Start up a ROS node and start advertising topics to publish on.
    ros::init(argc, argv, "camera1TransformListener");
    ros::NodeHandle node;
    ros::Publisher camera1Pub = node.advertise<sensor_msgs::PointCloud2>("camera1Post", 10);
    ros::Publisher camera2Pub = node.advertise<sensor_msgs::PointCloud2>("camera2Post", 10);

    // Setting the global publishers to be the publishers.
    pub1 = &camera1Pub;
    pub2 = &camera2Pub;

    // Creating and setting up the transform variables.
    tf2_ros::Buffer _tfBuffer1;
    tf2_ros::Buffer _tfBuffer2;
    tf2_ros::TransformListener _tfListener1(_tfBuffer1);
    tf2_ros::TransformListener _tfListener2(_tfBuffer2);
    tfBuffer1 = &_tfBuffer1;
    tfBuffer2 = &_tfBuffer2;

    // Subscribing to the 2 camera topics.
    ros::Subscriber camera1Sub = node.subscribe("camera1Pre", 10, camera1Transform);
    ros::Subscriber camera2Sub = node.subscribe("camera2Pre", 10, camera2Transform);
    ros::spin();

    return 0;
}


void camera1Transform(sensor_msgs::PointCloud2 cloudIn)
{
    sensor_msgs::PointCloud2 cloudOut;
    geometry_msgs::TransformStamped transform;

    // Looking up the transformation in the transform tree/table in ROS.
    try {
        transform = tfBuffer1->lookupTransform("map", "camera1", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    // Applying the looked up transformation to the point cloud.
    tf2::doTransform(cloudIn, cloudOut, transform);

    // Publishing the transformed pointcloud to ROS.
    pub1->publish(cloudOut);
}


void camera2Transform(sensor_msgs::PointCloud2 cloudIn)
{
    sensor_msgs::PointCloud2 cloudOut;
    geometry_msgs::TransformStamped transform;

    // Looking up the transformation in the transform tree/table in ROS.
    try {
        transform = tfBuffer2->lookupTransform("map", "camera2", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    // Applying the looked up transformation to the point cloud.
    tf2::doTransform(cloudIn, cloudOut, transform);

    // Publishing the transformed pointcloud to ROS.
    pub2->publish(cloudOut);
}