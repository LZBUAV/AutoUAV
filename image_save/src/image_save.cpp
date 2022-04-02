#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/TimeReference.h"
#include "std_msgs/Float64.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

cv::Mat rgbimage0, rgbimage1, undist_rgbimage0, undist_rgbimage1;
float height, height_thrd = 0.5;
mavros_msgs::State current_state;
unsigned long cam0_count = 0, cam1_count = 0;

//640*480
// cv::Mat camera_matrix0 = (cv::Mat_<float>(3, 3) << 477.8768, 0, 310.3520, 0, 477.8994, 245.2049, 0, 0, 1);
// cv::Mat distcoficient0 = (cv::Mat_<float>(5, 1) <<  -0.4344, 0.2158);

// cv::Mat camera_matrix1 = (cv::Mat_<float>(3, 3) << 478.1321, 0, 330.7059, 0, 478.0269, 251.7842, 0, 0, 1);
// cv::Mat distcoficient1 = (cv::Mat_<float>(5, 1) << -0.4182, 0.1704);

//1280*720
cv::Mat camera_matrix0 = (cv::Mat_<float>(3, 3) << 719.4612, 0, 617.6263, 0, 719.2882, 373.1899, 0, 0, 1);
cv::Mat distcoficient0 = (cv::Mat_<float>(5, 1) << -0.4226, 0.1868);

cv::Mat camera_matrix1 = (cv::Mat_<float>(3, 3) << 725.7144, 0, 654.3074, 0, 723.8206, 375.1982, 0, 0, 1);
cv::Mat distcoficient1 = (cv::Mat_<float>(5, 1) <<  -0.4479, 0.2425);

//1920*1080
// cv::Mat camera_matrix0 = (cv::Mat_<float>(3, 3) << 1072.0042, 0, 942.4123,  0, 1071.7591, 552.9838, 0, 0, 1);
// cv::Mat distcoficient0 = (cv::Mat_<float>(5, 1) << -0.3975, 0.1383);

// cv::Mat camera_matrix1 = (cv::Mat_<float>(3, 3) << 1078.9078, 0, 981.9544, 0, 1078.0105, 566.8858, 0, 0, 1);
// cv::Mat distcoficient1 = (cv::Mat_<float>(5, 1) << -0.4240, 0.1819);

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{ 
    current_state = *msg;
    // std::cout << "armd: " << std::to_string(current_state.armed) << " connected: " << std::to_string(current_state.connected) << " mode: " << current_state.mode << std::endl;
}

sensor_msgs::TimeReference gps_time;
void gps_time_cb(const sensor_msgs::TimeReference::ConstPtr& now_time)
{ 
    gps_time = *now_time;
    // std::cout << "gps_time: " << gps_time.header.stamp.sec << std::endl;
}

std_msgs::Float64 gps_rel_alt;
void gps_rel_alt_cb(const std_msgs::Float64::ConstPtr& rel_alt)
{ 
    gps_rel_alt = *rel_alt;
    // std::cout << "gps_rel_alt: " << gps_rel_alt.data << std::endl;
}

sensor_msgs::Range lidar_alt;
void lidar_alt_cb(const sensor_msgs::Range::ConstPtr& range)
{
    lidar_alt = *range;
    if(0.10 <= lidar_alt.range <= 7.00)
    {
        height = lidar_alt.range;
        std::cout << "height from lidar: " << height << std::endl;
    }
    else
    {
        height = gps_rel_alt.data;
        std::cout << "height from gps: " << height << std::endl;
    }
    
    // std::cout << "lidar_alt: " << lidar_alt.range << std::endl;
}

void image0cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(rgbimage0);

    // if(height >= height_thrd)
    {
        // cv::imshow("image0", rgbimage0);
        // cv::waitKey(1);
        // ros::Time time_begin = ros::Time::now();
        // cv::undistort(rgbimage0, undist_rgbimage0, camera_matrix0, distcoficient0);
        cv::imwrite("/home/lzb/catkin_ws/src/image_save/image/cam0/cam0_" + std::to_string(cam0_count) + "_.jpg", rgbimage0);
        // cv::imwrite("/home/nvidia/project/catkin_ws/src/image_save/image/undist/cam0/cam0_" + std::to_string(cam0_count) + "_undist_" + std::to_string(height) + "_" + std::to_string(gps_time.header.stamp.sec) + ".jpg", undist_rgbimage0);
        cam0_count++;
        std::cout << cam0_count << std::endl;
        // ros::Time time_end = ros::Time::now();
        // ros::Duration duration = time_end - time_begin;
        // std::cout << "time cost in save image: " << duration.toSec() << std::endl;
    } 
}

void image1cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(rgbimage1);

    if(height >= height_thrd)
    {
        // cv::imshow("image1", rgbimage1);
        // cv::waitKey(1);
        // cv::undistort(rgbimage1, undist_rgbimage1, camera_matrix1, distcoficient1);
        cv::imwrite("/home/nvidia/project/catkin_ws/src/image_save/image/cam1/cam1_" + std::to_string(cam1_count) + "_origin_" + std::to_string(height) + "_" +  std::to_string(gps_time.header.stamp.sec) + ".jpg", rgbimage1);
        // cv::imwrite("/home/nvidia/project/catkin_ws/src/image_save/image/undist/cam1/cam1_" + std::to_string(cam1_count) + "_undist_" + std::to_string(height) + "_" +  std::to_string(gps_time.header.stamp.sec) + ".jpg", undist_rgbimage1);
        // cv::waitKey(20);
        cam1_count++;
        std::cout << cam1_count << std::endl;
    } 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_save");
    ros::NodeHandle nh;
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
    // ros::Subscriber gps_rel_alt_sub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1, gps_rel_alt_cb);
    // ros::Subscriber lidar_alt_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/rangefinder_pub", 1, lidar_alt_cb);
    // ros::Subscriber gps_time_sub = nh.subscribe<sensor_msgs::TimeReference>("/mavros/time_reference", 1, gps_time_cb);
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image0_sub, image1_sub;
    image0_sub = it.subscribe("/iris_demo/camera/image_raw", 1, image0cb);
    // image1_sub = it.subscribe("/usb_cam1/image_raw", 1, image1cb);

    ros::spin();

    return 0;
}
