#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"
#include <tracker_kcf/tracker_result.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <image_match/match_result.h>
#include <std_msgs/Float64.h>

cv::Mat rgbimage;
cv::Rect selectRect, last_selectRect;
int init_count = 0;
int tracker_count = 0;

cv::Rect result;
tracker_kcf::tracker_result center;

bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

bool is_init = false;

ros::Publisher pub;
std_msgs::Float64 quality;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float cv_rect_distance(const cv::Rect& rect1, const cv::Rect& rect2)
{
  float center1_x = rect1.x + rect1.width/2.0;
  float center1_y = rect1.y + rect1.height/2.0;
  float center2_x = rect2.x + rect2.width/2.0;
  float center2_y = rect2.y + rect2.height/2.0;
  float d1 = std::pow((center1_x - center2_x), 2);
  float d2 = std::pow((center1_y - center2_y), 2);
  return std::pow((d1 + d2), 0.5);
}

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
  if(!is_init && bbox.width > 0 && bbox.height > 0)
  {
      // if(init_count == 0)
      // {
      //   last_selectRect = selectRect;
      // }

      selectRect.x = limit_x_y(bbox.x, 0, 1279);
      selectRect.y = limit_x_y(bbox.y, 0, 719);
      selectRect.width = limit_w_h(selectRect.x, bbox.width, 1279);
      selectRect.height = limit_w_h(selectRect.y, bbox.height, 719);

      // if(cv_rect_distance(last_selectRect, selectRect) < 200.0)
      // {
      //   init_count++;
      //   last_selectRect = selectRect;
      // }
      // else
      // {
      //   init_count = 0;
      // }
      
      if(1)
      {
        bRenewROI = true;
        is_init = true;
        // init_count = 0;
      }
      
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


  if(bRenewROI)
  {
      tracker.init(selectRect, rgbimage);
      bBeginKCF = true;
      bRenewROI = false;
  }

  if(bBeginKCF)
  {
      result = tracker.update(rgbimage);

      center.center_x = limit_x_y(int((result.x + result.width/2)), 0, 1279);
      center.center_y = limit_x_y(int((result.y + result.height/2)), 0, 719);
      center.x = limit_x_y(result.x, 0, 1279);
      center.y = limit_x_y(result.y, 0, 719);
      center.width = limit_w_h(center.x, result.width, 1279);
      center.height = limit_w_h(center.y, result.height, 719);

      cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
      cv::putText(rgbimage, std::to_string(quality.data), cv::Point(center.x, center.y-5), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

      pub.publish(center);
      std::cout<< "center_x :" << center.center_x << ", center_y :" << center.center_y << " quality: " << quality.data << std::endl;
  }
  
  cv::imshow("tracker", rgbimage);
  cv::waitKey(1);
}

void tracker_qualityCb(const std_msgs::Float64::ConstPtr& confi)
{
    quality = *confi;
    tracker_count++;
    if((quality.data < 0.4 || tracker_count > 40) && is_init)
    {
        tracker_count = 0;
        bRenewROI = false;
        bBeginKCF = false;
        is_init = false;
        std::cout << "reinit: " << quality.data << std::endl;
    }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker_kcf");
	
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  
  
  ros::Subscriber sub_bboxs;
  ros::Subscriber tracker_quality;

  image_sub_ = it_.subscribe("/iris_demo/camera/image_raw", 1, imageCb);
  sub_bboxs = nh_.subscribe("/match_result", 1, get_bbox); 
  tracker_quality = nh_.subscribe("/tracker_quality", 1, tracker_qualityCb);
  pub = nh_.advertise<tracker_kcf::tracker_result>("tracker_result", 1);
  
  ros::spin();

	return 0;
}

