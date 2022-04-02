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
#include <std_msgs/Float64.h>

#include <tracker_eco/tracker_result.h>
#include <image_match/match_result.h>

#include "eco.hpp"

cv::Mat origin_noresize, rgbimage;
cv::Rect selectRect;

bool bRenewROI = false;
bool bBeginECOHC = false;
bool is_init = false;

ros::Publisher pub;

std_msgs::Float64 quality;

eco::ECO tracker;
eco::EcoParameters parameters;
tracker_eco::tracker_result center;

int limit_x_y(int x, int low, int high)
{
    if(x < low)
    {
        x = low;
    }
    if(x > high)
    {
        x = high;
    }
    return x;
}

int limit_w_h(int x_y, int w_h, int high)
{
    if((x_y + w_h) > high)
    {
        w_h = high - x_y;
    }
    
    return w_h;
}

void get_bbox(const image_match::match_result bbox)
{
    if(!is_init)
    {
        selectRect.x = limit_x_y(int(bbox.x/1280.0*500.0), 0, 499);
        selectRect.y = limit_x_y(int(bbox.y/720.0*400.0), 0, 399);
        selectRect.width = limit_w_h(selectRect.x, int(bbox.width/1280.0*500.0), 499);
        selectRect.height = limit_w_h(selectRect.y, int(bbox.height/720.0*400.0), 399);
        bRenewROI = true;
        is_init = true;
    }
} 

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

    cv_ptr->image.copyTo(origin_noresize);
    if(!origin_noresize.empty())
    {
        cv::resize(origin_noresize, rgbimage, cv::Size(500, 400));

        if(bRenewROI)
        {
            std::cout << 1 << std::endl;
            tracker.init(rgbimage, selectRect, parameters);
            std::cout << 2 << std::endl;
            bBeginECOHC = true;
            bRenewROI = false;
        }
        if(bBeginECOHC)
        {
            cv::Rect2f bbox;
            
            tracker.update(rgbimage, bbox);
            
            center.center_x = limit_x_y(int((bbox.x + bbox.width/2)/500.0*1280.0), 0, 1279);
            center.center_y = limit_x_y(int((bbox.y + bbox.height/2)/400.0*720.0), 0, 719);
            center.x = limit_x_y(int(bbox.x/500.0*1280), 0, 1279);
            center.y = limit_x_y(int(bbox.y/400.0*720), 0, 719);
            center.width = limit_w_h(center.x, int(bbox.width/500.0*1280.0), 1279);
            center.height = limit_w_h(center.y, int(bbox.height/400.0*720.0), 719);

            cv::Rect result(center.x, center.y, center.width, center.height);            
            cv::rectangle(origin_noresize, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
            cv::putText(origin_noresize, std::to_string(quality.data), cv::Point(center.x, center.y-5), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            pub.publish(center);
            std::cout<< "center_x :" << center.center_x << ", center_y :" << center.center_y << " quality: " << quality.data << std::endl;
        }

        cv::imshow("tracker", origin_noresize);
        cv::waitKey(1);
    }
    
}

void tracker_qualityCb(const std_msgs::Float64::ConstPtr& confi)
{
    quality = *confi;
    if(quality.data < 0.3)
    {
        bRenewROI = false;
        bBeginECOHC = false;
        is_init = false;
        std::cout << "reinit: " << quality.data << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_eco");

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_;

    ros::Subscriber sub_bboxs;
    ros::Subscriber tracker_quality;

    image_sub_ = it_.subscribe("/iris_demo/camera/image_raw", 1, imageCb);
    sub_bboxs =  nh_.subscribe("/match_result", 1, get_bbox);
    tracker_quality = nh_.subscribe("/tracker_quality", 1, tracker_qualityCb);
    pub = nh_.advertise<tracker_eco::tracker_result>("tracker_result", 1);
    
    ros::spin();

    return 0;
};