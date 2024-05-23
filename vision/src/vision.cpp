#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include </home/jetson/librealsense-master/wrappers/opencv/cv-helpers.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <librealsense2/hpp/rs_types.hpp>
#include <iostream>
#include <chrono>
#include <thread>
//#include <cuda_runtime.h>

rs2::pipeline pipeline1;
rs2::config config1;

rs2::pipeline pipeline2;
rs2::config config2;

rs2::align align_to(RS2_STREAM_COLOR);

rs2_format formatDepth = RS2_FORMAT_Z16;
rs2_format formatColor = RS2_FORMAT_BGR8;

//float* gpu_data;
//size_t size = (640 * 480 * sizeof(float));
//cudaMalloc((void**)&gpu_data, size);

std::chrono::high_resolution_clock::time_point cStart = std::chrono::high_resolution_clock::now();

void processFrames(rs2::frameset frames, cv::Mat &masked, cv::Mat &erod_masked, cv::Mat &depthImage, cv::Mat &colorImage);

int main(int argc, char ** argv)
{
    cv::Mat erod_masked1;
    cv::Mat masked1;
    cv::Mat colorImage1;
    cv::Mat depthImage1;
    cv::Mat erod_masked2;
    cv::Mat masked2;
    cv::Mat colorImage2;
    cv::Mat depthImage2;
    rs2::frameset frames1;
    rs2::frameset frames2;
    int i, j;

    std::string serial1 = "838212072465";
    std::string serial2 = "944622074563";

    config1.enable_device(serial1);
    config1.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config1.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipeline1.start(config1);

    config2.enable_device(serial2);
    config2.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config2.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipeline2.start(config2);

    ros::init(argc, argv, "camera1");
    ros::NodeHandle node;
    ros::Publisher cameraPub = node.advertise<sensor_msgs::PointCloud2>("camera1Pre", 10);

    rs2_intrinsics colorIntrin = pipeline1.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    while(true)
    {
        frames1 = pipeline1.wait_for_frames();
        frames2 = pipeline2.wait_for_frames();

        processFrames(frames1, masked1, erod_masked1, depthImage1, colorImage1);
        processFrames(frames2, masked2, erod_masked2, depthImage2, colorImage2);

        std::cout << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - cStart).count() << std::endl;
        cStart = std::chrono::high_resolution_clock::now();

        //cv::imshow("Eroded Mask", erod_masked);
        //cv::imshow("Color", colorImage);
        //cv::imshow("Depth", depthColor);
        //cv::waitKey(1);
    }

    //cudaFree(gpu_data);
    pipeline1.stop();
    pipeline2.stop();
    cv::destroyAllWindows();
}


void processFrames(rs2::frameset frames, cv::Mat &masked, cv::Mat &erod_masked, cv::Mat &depthImage, cv::Mat &colorImage)
{
    int i, j;

    //cudaMemcpy(gpu_data, frames, size, cudaMemcpyHostToDevice);
    align_to.process(frames);
    //cudaMemcpy(frames, gpu_data, size, cudaMemcpyDeviceToHost);

    colorImage = frame_to_mat(frames.get_color_frame());
    depthImage = frame_to_mat(frames.get_depth_frame());

    cv::cvtColor(colorImage, colorImage, cv::COLOR_BGR2GRAY);

    cv::adaptiveThreshold(colorImage, masked, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 75, -22);
    cv::erode(masked, erod_masked, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4)));

    cv::convertScaleAbs(depthImage, depthImage, 0.03);
    cv::applyColorMap(depthImage, depthImage, cv::COLORMAP_JET);

    for(i = 0; i < erod_masked.rows/2; i++)
    {
        for(j = 0; j < erod_masked.cols; j++)
        {
            erod_masked.at<int>(i, j) = 0;
        }
    }

    return;
}