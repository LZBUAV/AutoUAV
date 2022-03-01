#ifndef IMAGE_MATCH_HPP
#define IMAGE_MATCH_HPP

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
#include <image_match_tool.hpp>
#include <algorithm>

namespace match{
class IMAGE_MATCH
{
public:
    IMAGE_MATCH(std::string base_feature_path);

    ~IMAGE_MATCH(){;}

    void update_base_features(int index);

    void get_color(const cv::Mat& rgbimage, const cv::Rect& roi, std::vector<cv::Mat>& cn_feature);

    double compare_opencv(const cv::Mat& rgbimage, const cv::Rect& roi);

    bool match(darknet_ros_msgs::BoundingBoxes bbox, cv::Mat& rgbimage, image_match::match_result& retult);

    std::vector<std::vector<cv::Mat>> base_features;
    std::vector<std::string> base_images_list;
    int update_index = 0;
    int update_count = 0;
    int max_images = 0;
    int save_index = 0;
    float max_conficient_thrd = 0.8;
    float update_conficient_thrd = 0.9;
    float det_probability_thrd = 0.5;
    // float weights[30] = {0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27,
    //                      0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27,
    //                      0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.4/27, 0.2, 0.2, 0.2};
    float weights[15] = {0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.3/12.0, 0.7/3.0, 0.7/3.0, 0.7/3.0};
};
}

#endif