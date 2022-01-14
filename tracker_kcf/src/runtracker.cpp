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

static const std::string RGB_WINDOW = "RGB Image window";

cv::Mat rgbimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;
tracker_kcf::tracker_result center;
int center_x = 0;
int center_y = 0;
int max_pra = 0;
bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;
bool is_person = false;
bool is_init = false;
// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

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
        bBeginKCF = false;  
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

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
public:
  ros::Publisher pub;
  ros::Subscriber sub_bboxs;

  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/iris_demo/camera/image_raw", 1, &ImageConverter::imageCb, this);
    pub = nh_.advertise<tracker_kcf::tracker_result>("tracker_result", 1);
    sub_bboxs = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::get_bbox, this); 

    cv::namedWindow(RGB_WINDOW);
  }
 
  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
  }

  void get_bbox(const darknet_ros_msgs::BoundingBoxes& bboxs)
  {
    if(!is_init)
    {
      size_t num_boxs = bboxs.bounding_boxes.size();
      int index = 0;
      for(int i = 0; i < num_boxs; i++)
      {
        if(bboxs.bounding_boxes[i].id == 0 && bboxs.bounding_boxes[i].probability >= max_pra)
        {
          index = i;
          is_person = true;
          max_pra = bboxs.bounding_boxes[i].probability;
        }
      }

      if(is_person)
      {
        selectRect.x = bboxs.bounding_boxes[index].xmin;
        selectRect.y = bboxs.bounding_boxes[index].ymin;
        selectRect.width = bboxs.bounding_boxes[index].xmax - bboxs.bounding_boxes[index].xmin;
        selectRect.height = bboxs.bounding_boxes[index].ymax - bboxs.bounding_boxes[index].ymin;
        bRenewROI = true;
        is_person = false;
        is_init = true;
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

    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    if(bRenewROI)
    {
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
        enable_get_depth = true;
        center_x = result.x + result.width/2;
        center_y = result.y + result.height/2;
        center.center_x = center_x;
        center.center_y = center_y;
        std::cout<< "center_x :" << center_x << ", center_y :" << center_y << std::endl;
    }
    else
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker_kcf");
	ImageConverter ic;
  
	while(ros::ok())
	{
		ros::spinOnce();
    ic.pub.publish(center);

		if (cvWaitKey(33) == 'q')
      break;
	}

	return 0;
}

