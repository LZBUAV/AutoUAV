#include "image_match.hpp"

namespace match{

IMAGE_MATCH::IMAGE_MATCH(std::string base_feature_path)
{
    std::string image_path;
    for(auto file_path : std::experimental::filesystem::directory_iterator(base_feature_path))
    {
        image_path = file_path.path().string();
        // std::cout << image_path << std::endl;
        cv::Mat frame = cv::imread(image_path);
        cv::Rect roi(0, 0, frame.cols, frame.rows);
        std::vector<cv::Mat> base_feature;
        // std::cout << frame.cols << " " << frame.rows << " " << roi.size() << std::endl;
        get_color(frame, roi, base_feature);
        base_features.push_back(base_feature);
    }

}

void IMAGE_MATCH::get_color(const cv::Mat& rgbimage, const cv::Rect& roi , std::vector<cv::Mat>& cn_feature)
{
    cv::Mat frame = rgbimage(roi);
    cv::Mat frame_gray;
    cv::Mat frame_gray_resized;

    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2BGRA);

    int w = frame_gray.cols, h = frame_gray.rows;
    int size_col = w%3;
    int size_row = h%3;

    if(size_col)
    {
        w -= size_col;
    }

    if(size_row)
    {
        h -= size_row;
    }

    cv::resize(frame_gray, frame_gray_resized, cv::Size(w, h));

    int step_col = w/3;
    int step_row = h/3;

    int histsize[1] = {256};
    int channels[1] = {0};
    float hr1[2] = {0, 256};
    const float* ranges[] = {hr1};

    for(int x = 0; x < 3; x++)
    {
        for(int y = 0; y < 3; y++)
        {
            cv::Mat hist;
            cv::Rect sub_roi(x*step_col, y*step_row, step_col, step_row);
            cv::Mat sub_image = frame_gray_resized(sub_roi);
            cv::calcHist(&sub_image, 1, channels, cv::Mat(), hist, 1, histsize, ranges);

            hist.convertTo(hist, CV_32FC1);
            float sum = 0.0;
            for(int j = 0; j < hist.rows; j++)
            {
                sum +=hist.at<float>(cv::Point(j, 0));
            }
            
            for(int j = 0; j < hist.rows; j++)
            {
                hist.at<float>(cv::Point(j, 0)) /= sum + 0.000001;
            }

            cn_feature.push_back(hist);

        }
    }

}

double IMAGE_MATCH::compare_opencv(const cv::Mat& rgbimage, const cv::Rect& roi)
{
    std::vector<cv::Mat> feature;
    get_color(rgbimage, roi, feature);
    
    double conficient = 0.0;

    for(auto base_feature : base_features)
    {
        double temp_conficient = 0.0;
        for(int i = 0; i < feature.size(); i++)
        {
            temp_conficient += cv::compareHist(feature[i], base_feature[i], cv::HISTCMP_CORREL);
        }
        temp_conficient /= 9.0;
        conficient = conficient > temp_conficient ? conficient : temp_conficient;
    }

    std::cout << " 最大相似度 " << conficient << std::endl;

    return conficient;

}


bool IMAGE_MATCH::match(const darknet_ros_msgs::BoundingBoxes& bbox, cv::Mat& rgbimage, image_match::match_result& result)
{
    cv::Rect roi;

    size_t num_boxs = bbox.bounding_boxes.size();

    double max_conficient = 0.0;
    int max_conficient_index = 0;

    for(int i = 0; i < num_boxs; i++)
    {
        double conficient = 0.0;
        if(bbox.bounding_boxes[i].id == 0 && bbox.bounding_boxes[i].probability >= 0.5)
        {
            roi = cv::Rect(bbox.bounding_boxes[i].xmin, bbox.bounding_boxes[i].ymin, bbox.bounding_boxes[i].xmax-bbox.bounding_boxes[i].xmin, bbox.bounding_boxes[i].ymax-bbox.bounding_boxes[i].ymin);
            conficient = compare_opencv(rgbimage, roi);
            if(conficient > max_conficient)
            {
                max_conficient_index = i;
                max_conficient = conficient;
            }
        }
    }

    result.x = bbox.bounding_boxes[max_conficient_index].xmin;
    result.y = bbox.bounding_boxes[max_conficient_index].ymin;
    result.width = bbox.bounding_boxes[max_conficient_index].xmax-bbox.bounding_boxes[max_conficient_index].xmin;
    result.height = bbox.bounding_boxes[max_conficient_index].ymax-bbox.bounding_boxes[max_conficient_index].ymin;

    if(bbox.bounding_boxes[max_conficient_index].id == 0)
    {   
        cv::rectangle(rgbimage, cv::Rect(result.x, result.y, result.width, result.height), cv::Scalar(0, 255, 255), 1, 0);
        cv::putText(rgbimage, "max_conficient: " + std::to_string(max_conficient), cv::Point(50, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 3, 8);
        cv::Rect max_roi(result.x, result.y, result.width, result.height);
        // cv::imwrite("/home/lzb/personal/project/cpptest/images/lzb/" + std::to_string(std::rand()%1000) + ".jpg", rgbimage(max_roi));
    }

    if(max_conficient >= 0.5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

}