#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <message_filters/subscriber.h>
#include "std_msgs/msg/u_int8_multi_array.hpp"
// #include <tf/transform_listener.h>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include <tf2/convert.h>
#include <tf2_ros/message_filter.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <tf2/buffer_core.h>
// #include "pcl_ros/include/pcl_ros/point_cloud.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdio.h>
#include <iostream>

using std::placeholders::_1;

// Command to start LIDAR ros node:
// rosrun urg_node urg_node

// Theses are all in centimeters.
const int dividor = 3;
const int courseWidth = (int)(3048/dividor), courseHeight = (int)(3658/dividor);
const int xRightPadding = (int)(176/dividor), xLeftPadding = (int)(176/dividor), yRightPadding = (int)(171/dividor), yLeftPadding = (int)(171/dividor);
const int xOff = 300/dividor, yOff = 2000/dividor;
const int width = courseWidth + xRightPadding + xLeftPadding;
const int height = courseHeight + yRightPadding + yLeftPadding;
const int laserScanSize = 660;
const int laserMinVal = 200;
const int laserMaxVal = 860;

uint8_t course[height][width];

class MapCreation : public rclcpp::Node
{
    public:
    MapCreation();

    ~MapCreation();

    void camera1Callback(sensor_msgs::msg::PointCloud2 cloudIn);

    void camera2Callback(sensor_msgs::msg::PointCloud2 cloudIn);

    void laserCallback(sensor_msgs::msg::LaserScan laserIn);

    void createList();

    private:
    std::chrono::high_resolution_clock::time_point cStart;
    // ros::NodeHandle node;
    // auto node = rclcpp::Node::make_shared("obstacleImageGeneration");
    sensor_msgs::msg::PointCloud2 camera1Cloud;
    sensor_msgs::msg::PointCloud2 camera2Cloud;
    sensor_msgs::msg::PointCloud2 laserCloud;

    pcl::PointCloud<pcl::PointXYZ> pclLaserCloud;
    pcl::PointCloud<pcl::PointXYZ> pclCamera1Cloud;
    pcl::PointCloud<pcl::PointXYZ> pclCamera2Cloud;
    pcl::PointCloud<pcl::PointXYZ> obstacleCloud;
    sensor_msgs::msg::PointCloud2 rosCloud;

    std_msgs::msg::UInt8MultiArray imageArray;
    std::vector<uint8_t> imageVec;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    laser_geometry::LaserProjection projector;
    //ros::Publisher laserCloudPub = node.advertise<sensor_msgs::PointCloud2>("laserCloud", 10)

    // rclcpp::Publisher<pcl::PointCloud<pcl::PointXYZ>>::SharedPtr laserCloudPub = this->create_publisher<pcl::PointCloud<pcl::PointXYZ>>("laserCloud", 10);
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr obstacles;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ROSCloud;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera2;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser;

};


int main(int argc, char** argv)
{
    // Initilizing a ros node.
    rclcpp::init(argc, argv);

    // Creating our MapCreation object.
    // MapCreation points;

    rclcpp::spin(std::make_shared<MapCreation>());
    rclcpp::shutdown();
    return 0;
}


MapCreation::MapCreation() : Node("obstacleImageGeneration")
{
    int i = 0, j = 0;

    // Setting every point in the course array to be the max value 255.
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            course[i][j] = 255;
        }
    }
    

    // Setting our time variable to be right now.
    cStart = std::chrono::high_resolution_clock::now();

    // Configuring our custom data type to be a 2D array the size of our course.
    imageArray.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    imageArray.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    imageArray.layout.dim[0].label = "height";
    imageArray.layout.dim[1].label = "width";
    imageArray.layout.dim[0].size = height;
    imageArray.layout.dim[1].size = width;
    imageArray.layout.dim[0].stride = height * width;
    imageArray.layout.dim[1].stride = width;
    imageArray.layout.data_offset = 0;

    // Setting up the vector of the image to be the size of the image.
    imageVec.resize(width * height, 255);

    // Setting up Publishers
    ROSCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacleList", 10);
    obstacles = this->create_publisher<std_msgs::msg::UInt8MultiArray>("obstacleImage", 10);

    // Setting up the publishers and subscribers.
    camera1 = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera1Post", 10, std::bind(&MapCreation::camera1Callback, this, _1));
    camera2 = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera2Post", 10, std::bind(&MapCreation::camera2Callback, this, _1));
    laser = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&MapCreation::laserCallback, this, _1));
    // ros::spin();
    // rclcpp::spin(node);
}


MapCreation::~MapCreation()
{
    // Destroying all the windows used for debugging.
    cv::destroyAllWindows();
}


void MapCreation::camera1Callback(sensor_msgs::msg::PointCloud2 cloudIn)
{
    // Storing the point cloud in the class.
    camera1Cloud = cloudIn;

    // Converting the incoming point cloud into a pcl::PointCloud and storing it in the class.
    pcl::fromROSMsg(camera1Cloud, pclCamera1Cloud);
}


void MapCreation::camera2Callback(sensor_msgs::msg::PointCloud2 cloudIn)
{
    // Storing the point cloud in the class.
    camera2Cloud = cloudIn;
    
    // Converting the incoming point cloud into a pcl::PointCloud and storing it in the class.
    pcl::fromROSMsg(camera2Cloud, pclCamera2Cloud);
}


void MapCreation::laserCallback(sensor_msgs::msg::LaserScan laserIn)
{
    int i = 0;
    pcl::PointXYZ point;

    // std::cout << "AngleMin: " << laserIn.angle_min << " Angle Max: " << laserIn.angle_max << std::endl;
    // std::cout << "Angle Increment: " << laserIn.angle_increment << std::endl;
    // Obtaining the transformation from frame "laser" to frame "map".
    // if(!tfListener->waitForTransform("map", "laser", laserIn.header.stamp + rclcpp::Duration().seconds(laserIn.ranges.size()*laserIn.time_increment), rclcpp::Duration(1, 0)))
    //    return;

    for(i = 0; i < laserMinVal; i++)
        laserIn.ranges[i] = 200000;
         
    for(i = laserMaxVal; i < 1080; i++)
        laserIn.ranges[i] = 200000; 

    // Transforming the laser scan to a point cloud in map's coordinate frame.
    projector.transformLaserScanToPointCloud("map", laserIn, laserCloud, *tfBuffer);

    // Coneverting the sensor_msgs::Pointcloud2 into a pcl::PointCloud.
    pcl::fromROSMsg(laserCloud, pclLaserCloud);

    for(i = 0; i < int(pclLaserCloud.width * pclLaserCloud.height); i++)
    {
        point = pclLaserCloud.points[i];
        if(point.z < 0.2)
        {
            point.x = 10000;
            point.y = 10000;
            point.z = 10000;
        }
        //if(i == (pclLaserCloud.width * pclLaserCloud.height) / 2 )
        //    std::cout << "X: " << point.x << " Y: " << point.y << " Z: " << point.z << std::endl;
    }

    // Calling the function that creates a 2D image from the 3D point clouds.
    createList();

    // Publishing the laser point cloud for debugging.
    // laserCloudPub->publish(pclLaserCloud);
}


void MapCreation::createList() {
    int i = 0, j = 0;
    int x, y;
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> temp;
    int cloudSize = 0;
    int fadeSpeed = 17;

    // Fading out obstacles that we are no longer seeing.
    if(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - cStart) >= std::chrono::duration<double>(0.25))
    {
        for(i = 0; i < height; i++)
        {
            for( j = 0; j < width; j++)
            {
                if(course[i][j] != 255)
                {
                    course[i][j] += fadeSpeed;
                    imageVec[(i*width) + j] += fadeSpeed;
                }
            }
        }
        // Resetting the clock.
        cStart = std::chrono::high_resolution_clock::now();
    }


    // Combining all the point clouds into one point cloud.
    pcl::concatenate(pclCamera1Cloud, pclCamera2Cloud, temp);
    pcl::concatenate(temp, pclLaserCloud, obstacleCloud);
    //pcl::concatenate(pclLaserCloud, pclCamera1Cloud, obstacleCloud);

    pcl::toROSMsg(obstacleCloud, rosCloud);

    ROSCloud->publish(rosCloud);

    // Finding the size of the cloud.
    cloudSize = obstacleCloud.width * obstacleCloud.height;
    //cloudSize = pclLaserCloud.width * pclLaserCloud.height;

    for( i = 0; i < cloudSize; i++)
    {
        // Grabing a x, y, z point from the cloud.
        point = obstacleCloud.points[i];
        //point = pclLaserCloud.points[i];

        // Finding the image coordinates that correspond to the given point in the point cloud.
        x = (int)((point.x * (100/dividor)) + xOff);
        y = (int)((point.y * (100/dividor) * -1) + yOff);

        // Making sure any points that appear off the image go onto the edges of the image so we don't overstep the bounds of the array.
        if (y >= height)
            y = height - 1;
        else if (y < 0)
            y = 0;
        if (x >= width)
            x = width - 1;
        else if (x < 0)
            x = 0;

        // Setting that point to 0 to indicate there is an obstacle there.
        course[y][x] = 0;
        imageVec[(y*width) + x] = 0;
    }

    // Displaying the 2D array for debugging.
    cv::Mat image = cv::Mat(height, width, CV_8U, *course);
    cv::imshow("Image", image);
    cv::waitKey(1);
    
    // Publishing our image to the ros topic: obstacleImage.
    imageArray.data = imageVec;
    obstacles->publish(imageArray);

    return;
}
