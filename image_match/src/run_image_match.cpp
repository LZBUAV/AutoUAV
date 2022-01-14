#include "image_match.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

match::IMAGE_MATCH image_match_("/home/lzb/personal/project/cpptest/images/lzb/");
cv::Mat rgbimage;
image_match::match_result result;
std::string match_window = "match";

class RUN_MATCH
{
public:
    RUN_MATCH() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/iris_demo/camera/image_raw", 1, &RUN_MATCH::imageCb, this);
        bbox_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &RUN_MATCH::matchCb, this);
        pub = nh_.advertise<image_match::match_result>("match_result", 1);
        cv::namedWindow(match_window);
    }

    ~RUN_MATCH()
    {
        cv::destroyWindow(match_window);
    }

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
        if(image_match_.match(bboxs, rgbimage, result))
        {
            cv::rectangle(rgbimage, cv::Rect(result.x, result.y, result.width, result.height), cv::Scalar(255, 255, 255), 1, 0);
            pub.publish(result);
        }
        else
        {
            cv::putText(rgbimage, "No Match", cv::Point(50, 50), CV_FONT_NORMAL, 1, cv::Scalar(0, 255, 255), 3, 8);
            std::cout << "No Match" << std::endl;
        }
    }

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    ros::Subscriber bbox_sub_;
    ros::Publisher pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "match_result");
    RUN_MATCH rm;

    while (ros::ok())
    {
        ros::spinOnce();
        cv::imshow(match_window, rgbimage);
        cv::waitKey(1);
    }

    return 0;
    
}