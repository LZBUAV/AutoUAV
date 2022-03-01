
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"
#include "nanodet.h"

cv::Mat rgbimage;
std::string config_file = "/home/nvidia/project/catkin_ws/src/nanodet/config.yaml";
nanodet Nanodet(config_file);
darknet_ros_msgs::BoundingBoxes results;
ros::Publisher pub;
int save_count = 0;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    cv_ptr->image.copyTo(rgbimage);
    results.bounding_boxes.clear();
    Nanodet.EngineInferenceOnce(rgbimage, results);
    if(!results.bounding_boxes.empty())
    {
        pub.publish(results);
    }
    // cv::imshow("nanodet", rgbimage);
    // cv::imwrite("/home/nvidia/project/catkin_ws/src/nanodet/nano_result/" + std::to_string(save_count) + ".jpg", rgbimage);
    // save_count ++;
    // cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nanodet");

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_;
    image_sub_ = it_.subscribe("/usb_cam0/image_raw", 1, imageCb);

    pub = nh_.advertise<darknet_ros_msgs::BoundingBoxes>("/nanodet/bounding_boxes", 1);

    Nanodet.LoadEngine();
    Nanodet.CreateContext();

    ros::spin();

    Nanodet.ReleaseTRT();
    cv::destroyAllWindows();

    return 0;
}
