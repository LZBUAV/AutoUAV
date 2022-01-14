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

#include <tracker_eco/tracker_result.h>
#include <image_match/match_result.h>

#include "eco.hpp"

static const std::string RGB_WINDOW = "RGB Image window";

cv::Mat rgbimage;
cv::Rect selectRect;
cv::Point origin;

bool select_flag = false;
bool bRenewROI = false;
bool bBeginECOHC = false;
bool is_person = false;
bool is_init = false;
int max_pra = 0;

eco::ECO tracker;
eco::EcoParameters parameters;
tracker_eco::tracker_result center;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginECOHC = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}

unsigned long GetTickCount()
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

}

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    ros::Publisher pub;
    ros::Subscriber sub_bboxs;

    ImageConverter() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/iris_demo/camera/image_raw", 1, &ImageConverter::imageCb, this);
        pub = nh_.advertise<tracker_eco::tracker_result>("tracker_result", 1);
        sub_bboxs =  nh_.subscribe("/match_result", 1, &ImageConverter::get_bbox, this);
        cv::namedWindow(RGB_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(RGB_WINDOW);
    }

    void get_bbox(const image_match::match_result bbox)
    {
        if(!is_init)
        {
            selectRect.x = bbox.x;
            selectRect.y = bbox.y;
            selectRect.width = bbox.width;
            selectRect.height = bbox.height;
            bRenewROI = true;
            is_person = false;
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

        cv_ptr->image.copyTo(rgbimage);

        cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

        if(bRenewROI)
        {
            tracker.init(rgbimage, selectRect, parameters);
            bBeginECOHC = true;
            bRenewROI = false;
        }
        if(bBeginECOHC)
        {
            cv::Rect2f bbox;
            tracker.update(rgbimage, bbox);
            cv::Rect result = bbox;
            cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
            center.center_x = bbox.x + bbox.width/2;
            center.center_y = bbox.y + bbox.height/2;
        }
        else
            cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

        cv::imshow(RGB_WINDOW, rgbimage);

        cv::waitKey(1);
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_eco");
    ImageConverter ic;
        
    while(ros::ok())
    {
        unsigned long timer_start = GetTickCount();
        ros::spinOnce();
        ic.pub.publish(center);
        if (cvWaitKey(33) == 'q')
            break;
        unsigned long timer_end = GetTickCount();
        std::cout << "FPS: " << 1000.0/(timer_end - timer_start) <<"  ";
        std::cout<< "center_x :" << center.center_x << ", center_y :" << center.center_y << std::endl;
    }

    return 0;
};