#include "image_match.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <tracker_eco/tracker_result.h>
#include <tracker_kcf/tracker_result.h>

match::IMAGE_MATCH image_match_("/home/lzb/catkin_ws/src/image_match/base_images");
cv::Mat rgbimage;
image_match::match_result result, last_result;
int count = 0, result_count = 0;
std::string match_window = "match";
ros::Publisher pub;
ros::Publisher tracker_quality;


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return;
    }
    
    cv_ptr->image.copyTo(rgbimage);
}

void matchCb(const darknet_ros_msgs::BoundingBoxes bboxs)
{
    if(!rgbimage.empty())
    {
        image_match_.match(bboxs, rgbimage, result);
        if(count == 0)
        {
            last_result = result;
            count = 1;
        }
        else
        {
            float d1 = std::pow((result.x+result.width/2.0) - (last_result.x+last_result.width/2.0), 2);
            float d2 = std::pow((result.y+result.height/2.0) - (last_result.y+last_result.height/2.0), 2);
            float d3 = std::pow((d1+d2), 0.5);
            if(d3 > 600)
            {
                result.conf = 0;
            }
            else
            {
                last_result = result;
            }
        }
        if(result.conf > image_match_.max_conficient_thrd)
        {
            cv::line(rgbimage, cv::Point(result.x + result.width/2, result.y), cv::Point(result.x + result.width/2, result.y + result.height), cv::Scalar(0,0,255), 2, 0);
            cv::line(rgbimage, cv::Point(result.x, result.y + result.height/2), cv::Point(result.x + result.width, result.y + result.height/2), cv::Scalar(0,0,255), 2, 0);

        }
        else
        {
            count = 0;
            cv::putText(rgbimage, "one", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            std::cout << "No Match" << std::endl;
        }
        
        // cv::imshow(match_window, rgbimage);
        // cv::imwrite("/home/nvidia/project/catkin_ws/src/image_match/match_result/" + std::to_string(result_count) + ".jpg", rgbimage);
        // result_count++;
        // cv::waitKey(1);
    }
}

void track_bboxCb(const tracker_kcf::tracker_result bbox)
{
    if(!rgbimage.empty())
    {
        if(bbox.width > 0 && bbox.height > 0)
        {
            std_msgs::Float64 confi;
            cv::Rect roi(bbox.x, bbox.y, bbox.width, bbox.height);
            std::cout << bbox.x << " " << bbox.y << " " << bbox.width << " " << bbox.height << std::endl;
            confi.data = image_match_.compare_opencv(rgbimage, roi);
            tracker_quality.publish(confi);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "match_result");

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_;
    ros::Subscriber bbox_sub_;
    ros::Subscriber tracker_sub;
    
    image_sub_ = it_.subscribe("/iris_demo/camera/image_raw", 1, imageCb);
    bbox_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, matchCb);
    tracker_sub = nh_.subscribe("/tracker_result", 1, track_bboxCb);
    pub = nh_.advertise<image_match::match_result>("match_result", 1);
    tracker_quality = nh_.advertise<std_msgs::Float64>("tracker_quality", 1);

    ros::Rate rate(10);

    while (ros::ok())
    {
        result.conf = 0.0;
        result.is_person = 0;
        ros::spinOnce();
        pub.publish(result);
        if(!rgbimage.empty())
        {
            cv::imshow(match_window, rgbimage);
            cv::waitKey(1);
        }
        
        rate.sleep();
        
    }

    return 0;
    
}