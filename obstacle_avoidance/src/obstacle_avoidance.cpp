#include <ros/ros.h>
#include <obstacle_avoidance/obstacle_avoidance_result.h>
#include <obstacle_avoidance/distance.h>
#include <laser_radar/distance.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <stack>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>

cv::Mat frame(60, 160, CV_8UC1), frame_resized;
// laser_radar::distance depth;
obstacle_avoidance::obstacle_avoidance_result os_result, os_result_last;
ros::Publisher res_pub;
int count_save = 0;
int result_count = 0;
std::ofstream outfile;

void filter(const std::vector<std::vector<int>>& src, std::vector<std::vector<int>>& dst)
{
    int row = src.size();
    int col = src[0].size();
    for(int i = 0; i < row; i++)
    {
        for(int j = 0; j < col; j++)
        {
            if((i-1)>=0 && (j-1)>=0)
            {
                if(src[i-1][j-1] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((i-1)>=0)
            {
                if(src[i-1][j] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((i-1)>=0 && (j+1)< col)
            {
                if(src[i-1][j+1] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((j-1)>=0)
            {
                if(src[i][j-1] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((j+1)<col)
            {
                if(src[i][j+1] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((i+1)<row && (j-1)>=0)
            {
                if(src[i+1][j-1] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((i+1)<row)
            {
                if(src[i+1][j] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            if((i+1)<row && (j+1)<col)
            {
                if(src[i+1][j+1] == 255)
                {
                    dst[i][j] = 255;
                    continue;
                }
            }

            dst[i][j] = src[i][j];
        }
    }
}

int maximalRectangle(std::vector<std::vector<int>>& matrix, cv::Rect& roi) 
{
    int m = matrix.size();
    if (m == 0) 
    {
        return 0;
    }
    int n = matrix[0].size();
    std::vector<std::vector<int>> left(m, std::vector<int>(n, 0));

    for (int i = 0; i < m; i++) 
    {
        for (int j = 0; j < n; j++) 
        {
            if (matrix[i][j] == 255) 
            {
                left[i][j] = (j == 0 ? 0: left[i][j - 1]) + 1;
            }
        }
    }

    roi.x = 80;
    roi.y = 30;
    roi.width = 0;
    roi.height = 0;

    for (int j = 0; j < n; j++) 
    { // 对于每一列，使用基于柱状图的方法
        std::vector<int> up(m, 0), down(m, 0);

        std::stack<int> stk;
        for (int i = 0; i < m; i++) 
        {
            while (!stk.empty() && left[stk.top()][j] >= left[i][j]) 
            {
                stk.pop();
            }
            up[i] = stk.empty() ? -1 : stk.top();
            stk.push(i);
        }
        stk = std::stack<int>();
        for (int i = m - 1; i >= 0; i--) 
        {
            while (!stk.empty() && left[stk.top()][j] >= left[i][j]) 
            {
                stk.pop();
            }
            down[i] = stk.empty() ? m : stk.top();
            stk.push(i);
        }

        for (int i = 0; i < m; i++) 
        {
            int height = down[i] - up[i] - 1;
            // int area = height * left[i][j];
            // if(height > 50 && left[i][j] > 120)
            // {
            //     roi.x = j - left[i][j] + 1;
            //     roi.y = up[i] + 1;
            //     roi.width = left[i][j];
            //     roi.height = height;
            //     return 1;
            // }
            if((height > 50 && left[i][j] >= 20 && (j == 159 || (j - left[i][j] + 1) == 0)) || (height > 50 && left[i][j] >= 120))
            {
                if(left[i][j] > roi.width)
                {
                    roi.x = j - left[i][j] + 1;
                    roi.y = up[i] + 1;
                    roi.width = left[i][j];
                    roi.height = height;
                }
            }
        }
    }
    if(roi.width >= 80)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void lidar_sub_cb(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> depth;
    pcl::fromROSMsg(*input, depth);
    
    std::vector<std::vector<int>> matrix, matrix_filter, matrix_filter_twice;

    std::vector<float> data(9600, 0.0);
    int num = 0;
    for(int i = 0; i < 160; i++)
    {
        for(int j = 0; j < 60; j++)
        {
            data[(59-j)*160+(159-i)] = depth.points[num].x;
            num++;
        }
    }

    for(int i = 0; i < 60; i++)
    {
        std::vector<int> row, row_filter, row_filter_twice;
        for(int j = 0; j < 160; j++)
        {
            int temp = 0;
            float dep = data[i*160 + j];
            // std::cout << dep << std::endl;
            if(dep > 1.5)
                {
                    temp = 255;
                }
            else
                {
                    temp = 0;
                }
            row.push_back(temp);
            row_filter.push_back(temp);
            row_filter_twice.push_back(temp);
        }
        matrix.push_back(row);
        matrix_filter.push_back(row_filter);
        matrix_filter_twice.push_back(row_filter_twice);  
    }

    // filter(matrix, matrix_filter);
    // filter(matrix_filter, matrix_filter_twice);

    for(int i = 0; i < 60; i++)
    {
        for(int j = 0; j < 160; j++)
        {
            frame.at<uchar>(i,j) = matrix[i][j];
            // frame.data[i*160+j] = matrix_filter[i][j];
            // frame.at<uchar>(i,j) = matrix_filter_twice[i][j];
        }
    }

    cv::Rect roi(0, 0, 0, 0);
    int ret = maximalRectangle(matrix_filter_twice, roi);
    if(ret == 1)
    {
        os_result.center_x = roi.x + int(roi.width/2);
        os_result.center_y = roi.y + int(roi.height/2);
    }
    else
    {
        if(roi.x == 0)
        {
            os_result.center_x = roi.x;
        }
        else
        {
            os_result.center_x = roi.x + roi.width - 1;
        }
        os_result.center_y = roi.y + int(roi.height/2);
    }

    if(result_count == 0)
    {
        os_result_last = os_result;
        result_count = 1;
    }

    if(std::abs(os_result.center_x - os_result_last.center_x) > 156)
    {
        os_result = os_result_last;
    }
    else
    {
        os_result_last = os_result;
    }

    if(roi.width > 0)
    {
        os_result.is_ok = 1;
    }
    else
    {
        os_result.is_ok = 0;
    }

    res_pub.publish(os_result);
    // outfile << os_result.center_x << "," << os_result.center_y << "," << os_result.is_ok << std::endl;

    std::cout << os_result.is_ok << " " << os_result.center_x << " " << os_result.center_y << std::endl;

    if(os_result.is_ok == 1)
    {
        cv::rectangle(frame, roi, cv::Scalar(0, 0, 255), 1, 0);
        cv::circle(frame, cv::Point(os_result.center_x, os_result.center_y), 2, cv::Scalar(0, 0, 255), 2);
    }

    cv::resize(frame, frame_resized, cv::Size(160*3, 60*3));

    cv::imshow("depth", frame_resized);
    // cv::imwrite("/home/lzb/catkin_ws/src/obstacle_avoidance/os_result/" + std::to_string(count_save) + ".jpg", frame);
    // count_save++;
    cv::waitKey(1);
        
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, lidar_sub_cb);
    res_pub = nh.advertise<obstacle_avoidance::obstacle_avoidance_result>("obstacle_avoidance_result", 1);
    // outfile.open("/home/lzb/catkin_ws/src/obstacle_avoidance/os_result/result.txt");

    ros::spin();

    return 0;

}