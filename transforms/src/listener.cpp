#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <message_filters/subscriber.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2/convert.h>
#include <tf2_ros/message_filter.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <tf2_ros/buffer.h>

using std::placeholders::_1;

// Setting up global variables.
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr *pub1{nullptr};
// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr *pub2{nullptr};
// std::unique_ptr<tf2_ros::Buffer> *tfBuffer1{nullptr};
// std::unique_ptr<tf2_ros::Buffer> *tfBuffer2{nullptr};

class Transforms : public rclcpp::Node
{
public:
    Transforms();
    void camera1Transform(sensor_msgs::msg::PointCloud2 cloudIn);
    void camera2Transform(sensor_msgs::msg::PointCloud2 cloudIn);

private:
    // Start up a ROS node and start advertising topics to publish on.
    // auto node = rclcpp::Node::make_shared("camera1TransformListener");
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera1Post", 10);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera2Post", 10);

    // Creating and setting up the transform variables.
    std::unique_ptr<tf2_ros::Buffer> tfBuffer1 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::unique_ptr<tf2_ros::Buffer> tfBuffer2 = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> _tfListener1 = std::make_shared<tf2_ros::TransformListener>(*tfBuffer1);
    std::shared_ptr<tf2_ros::TransformListener> _tfListener2 = std::make_shared<tf2_ros::TransformListener>(*tfBuffer2);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera1Sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera2Sub;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Transforms>());
    rclcpp::shutdown();
    return 0;
}


Transforms::Transforms() : Node("camera1TransformListener")
{
    std::cout << "1" << std::endl;
    camera1Sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera1Pre", 10, std::bind(&Transforms::camera1Transform, this, _1));
    camera2Sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera2Pre", 10, std::bind(&Transforms::camera2Transform, this, _1));
}


void Transforms::camera1Transform(sensor_msgs::msg::PointCloud2 cloudIn)
{
    sensor_msgs::msg::PointCloud2 cloudOut;
    geometry_msgs::msg::TransformStamped transform;

    // Looking up the transformation in the transform tree/table in ROS.
    try {
        //rclcpp::Time now = rclcpp::Time(0); // node->get_clock()->now();
        transform = tfBuffer1->lookupTransform("map", "camera1", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        // ROS_WARN("%s", ex.what());
        // rclcpp::Duration(1.0).sleep();
        return;
    }

    // Applying the looked up transformation to the point cloud.
    tf2::doTransform(cloudIn, cloudOut, transform);

    // Publishing the transformed pointcloud to ROS.
    pub1->publish(cloudOut);
}


void Transforms::camera2Transform(sensor_msgs::msg::PointCloud2 cloudIn)
{
    sensor_msgs::msg::PointCloud2 cloudOut;
    geometry_msgs::msg::TransformStamped transform;

    // Looking up the transformation in the transform tree/table in ROS.
    try {
        //rclcpp::Time now = rclcpp::Time(0); // rclcpp::Time()->get_clock()->now();
        transform = tfBuffer2->lookupTransform("map", "camera2", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        // ROS_WARN("%s", ex.what());
        // rclcpp::Duration(1.0).sleep();
        return;
    }

    // Applying the looked up transformation to the point cloud.
    tf2::doTransform(cloudIn, cloudOut, transform);

    // Publishing the transformed pointcloud to ROS.
    pub2->publish(cloudOut);
}