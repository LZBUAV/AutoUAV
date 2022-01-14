#include <vector>
#include <iostream>
#include <experimental/filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <image_match/match_result.h>

namespace match{
class IMAGE_MATCH
{
public:
    IMAGE_MATCH(std::string base_feature_path);

    ~IMAGE_MATCH(){;}

    void get_color(const cv::Mat& rgbimage, const cv::Rect& roi, std::vector<cv::Mat>& cn_feature);

    double compare_opencv(const cv::Mat& rgbimage, const cv::Rect& roi);

    bool match(const darknet_ros_msgs::BoundingBoxes& bbox, cv::Mat& rgbimage, image_match::match_result& retult);

    std::vector<std::vector<cv::Mat>> base_features;
};
}