#include "image_match.hpp"

namespace match{

IMAGE_MATCH::IMAGE_MATCH(std::string base_feature_path)
{
    // std::string image_path;
    // int i = 0;
    // for(auto file_path : std::experimental::filesystem::directory_iterator(base_feature_path))
    // {
    //     image_path = file_path.path().string();
    //     // std::cout << image_path << std::endl;
    //     cv::Mat frame = cv::imread(image_path);
    //     cv::Rect roi(0, 0, frame.cols, frame.rows);
    //     std::vector<cv::Mat> base_feature;
    //     // std::cout << frame.cols << " " << frame.rows << " " << roi.size() << std::endl;
    //     get_color(frame, roi, base_feature);
    //     base_features.push_back(base_feature);
    //     // std::cout << i++ << std::endl;
    // }
    base_images_list = readFolder(base_feature_path);
    // std::sort(base_images_list.begin(), base_images_list.end(), cmp);
    for(auto image_name : base_images_list)
    {
        cv::Mat frame = cv::imread(image_name);
        // std::cout << frame.size() << " " << frame.empty() << " " << image_name << std::endl;
        // assert(!frame.empty());
        cv::Rect roi(0, 0, frame.cols, frame.rows);
        std::vector<cv::Mat> base_feature;
        get_color(frame, roi, base_feature);
        base_features.push_back(base_feature);
    }
    max_images = base_images_list.size();
}

void IMAGE_MATCH::update_base_features(int index)
{
    cv::Mat frame = cv::imread(base_images_list[index]);
    std::vector<cv::Mat> base_feature;
    cv::Rect roi(0, 0, frame.cols, frame.rows);
    get_color(frame, roi, base_feature);
    base_features[index] = base_feature;
    std::cout << "update the base image: " << base_images_list[index] << " index = " << index << std::endl;
}

void IMAGE_MATCH::get_color(const cv::Mat& rgbimage, const cv::Rect& roi , std::vector<cv::Mat>& cn_feature)
{
    cv::Mat frame = rgbimage(roi);
    cv::Mat frame_resized;

    int w = frame.cols, h = frame.rows;
    int size_col = w%2;
    int size_row = h%2;

    if(size_col != 0)
    {
        w -= size_col;
    }

    if(size_row != 0)
    {
        h -= size_row;
    }

    cv::resize(frame, frame_resized, cv::Size(w, h));

    int step_col = w/2;
    int step_row = h/2;

    int histsize[1] = {256};
    int channel_r[1] = {0};
    int channel_g[1] = {1};
    int channel_b[1] = {2};
    float hr1[2] = {0, 256};
    const float* ranges[] = {hr1};

    for(int x = 0; x < 2; x++)
    {
        for(int y = 0; y < 2; y++)
        {
            cv::Mat hist_r, hist_g, hist_b;
            cv::Rect sub_roi(x*step_col, y*step_row, step_col, step_row);
            cv::Mat sub_image = frame_resized(sub_roi);
            cv::calcHist(&sub_image, 1, channel_r, cv::Mat(), hist_r, 1, histsize, ranges);
            cv::calcHist(&sub_image, 1, channel_g, cv::Mat(), hist_g, 1, histsize, ranges);
            cv::calcHist(&sub_image, 1, channel_b, cv::Mat(), hist_b, 1, histsize, ranges);

            cn_feature.push_back(hist_r);
            cn_feature.push_back(hist_g);
            cn_feature.push_back(hist_b);

        }
    }

    cv::Mat hist_r_global, hist_g_global, hist_b_global;
    cv::calcHist(&frame, 1, channel_r, cv::Mat(), hist_r_global, 1, histsize, ranges);
    cv::calcHist(&frame, 1, channel_g, cv::Mat(), hist_g_global, 1, histsize, ranges);
    cv::calcHist(&frame, 1, channel_b, cv::Mat(), hist_b_global, 1, histsize, ranges);

    cn_feature.push_back(hist_r_global);
    cn_feature.push_back(hist_g_global);
    cn_feature.push_back(hist_b_global);

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
            temp_conficient += (cv::compareHist(feature[i], base_feature[i], cv::HISTCMP_CORREL)*weights[i]);
        }
        conficient = conficient > temp_conficient ? conficient : temp_conficient;
    }

    // std::cout << " 最大相似度 " << conficient << std::endl;

    return conficient;

}


bool IMAGE_MATCH::match(darknet_ros_msgs::BoundingBoxes bbox, cv::Mat& rgbimage, image_match::match_result& result)
{
    cv::Rect roi;

    size_t num_boxs = bbox.bounding_boxes.size();

    double max_conficient = 0.0;
    int max_conficient_index = 0;

    for(int i = 0; i < num_boxs; i++)
    {
        int x = bbox.bounding_boxes[i].xmin;
        int y = bbox.bounding_boxes[i].ymin;
        int w = bbox.bounding_boxes[i].xmax-bbox.bounding_boxes[i].xmin;
        int h = bbox.bounding_boxes[i].ymax-bbox.bounding_boxes[i].ymin;

        double conficient = 0.0;
        if(bbox.bounding_boxes[i].id == 0 && bbox.bounding_boxes[i].probability >= det_probability_thrd)
        {
            roi = cv::Rect(x, y, w, h); 
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
    result.conf = max_conficient;
    result.is_person = 0;

    if(bbox.bounding_boxes[max_conficient_index].id == 0)
    {   
        result.is_person = 1;
        cv::Rect max_roi(result.x, result.y, result.width, result.height);

        //comment the if code when sampling images.
        // if(max_conficient >= update_conficient_thrd)
        // {
        //     update_count++;
        //     if(update_count == 20)
        //     {
        //         cv::imwrite(base_images_list[update_index], rgbimage(max_roi));
            
        //         update_base_features(update_index);
                
        //         update_index++;
        //         if(update_index == max_images)
        //         {
        //             update_index = 0;
        //         }
        //         update_count = 0;
        //     }
            
        // }

        //commend the imwrite code when sample has complted.
        // cv::imwrite("/home/nvidia/project/catkin_ws/src/image_match/images/qianmuhu/" + std::to_string(save_index) + ".jpg", rgbimage(max_roi));
        // save_index++;

        // cv::rectangle(rgbimage, cv::Rect(result.x, result.y, result.width, result.height), cv::Scalar(0, 255, 255), 1, 0);
        // cv::putText(rgbimage, "max_conficient: " + std::to_string(max_conficient), cv::Point(result.x, result.y-5), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        std::cout << "最大相似度: " << max_conficient << std::endl;
        
    }
    
    return true;
    
}

}
