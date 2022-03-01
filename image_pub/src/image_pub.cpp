#include "image_pub.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

//640*480
// cv::Mat camera_matrix0 = (cv::Mat_<float>(3, 3) << 477.8768, 0, 310.3520, 0, 477.8994, 245.2049, 0, 0, 1);
// cv::Mat distcoficient0 = (cv::Mat_<float>(5, 1) <<  -0.4344, 0.2158);

// cv::Mat camera_matrix1 = (cv::Mat_<float>(3, 3) << 478.1321, 0, 330.7059, 0, 478.0269, 251.7842, 0, 0, 1);
// cv::Mat distcoficient1 = (cv::Mat_<float>(5, 1) << -0.4182, 0.1704);

//1280*720
cv::Mat camera_matrix0 = (cv::Mat_<float>(3, 3) << 719.4612, 0, 617.6263, 0, 719.2882, 373.1899, 0, 0, 1);
cv::Mat distcoficient0 = (cv::Mat_<float>(5, 1) << -0.4226, 0.1868);

cv::Mat camera_matrix1 = (cv::Mat_<float>(3, 3) << 725.7144, 0, 654.3074, 0, 723.8206, 375.1982, 0, 0, 1);
cv::Mat distcoficient1 = (cv::Mat_<float>(5, 1) <<  -0.4479, 0.2425);

//1920*1080
// cv::Mat camera_matrix0 = (cv::Mat_<float>(3, 3) << 1072.0042, 0, 942.4123,  0, 1071.7591, 552.9838, 0, 0, 1);
// cv::Mat distcoficient0 = (cv::Mat_<float>(5, 1) << -0.3975, 0.1383);

// cv::Mat camera_matrix1 = (cv::Mat_<float>(3, 3) << 1078.9078, 0, 981.9544, 0, 1078.0105, 566.8858, 0, 0, 1);
// cv::Mat distcoficient1 = (cv::Mat_<float>(5, 1) << -0.4240, 0.1819);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image0_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("usb_cam0/image_raw", 1);

    cv::Mat frame, undist;

    double fps = 0.0;

    std::vector<std::string> images_list = readFolder(argv[1]);
    std::sort(images_list.begin(), images_list.end(), cmp);

    ros::Rate loop_rate(10);
    int i = 0;
    while(ros::ok())
    {
        unsigned long timer_start = GetTickCount();

        frame = cv::imread(images_list[i]);
        std::cout << images_list[i] << std::endl;
        
        i++;
        if(i == images_list.size())
        {
            i = 0;
        }

        // cv::undistort(frame, undist, camera_matrix0, distcoficient0);
        // cv::imwrite("/home/nvidia/project/catkin_ws/src/image_pub/images/" + std::to_string(i) + ".jpg", undist);
        
        // cv::imshow("image_forward", undist);
        // int key = cv::waitKey(1);
        // if(key == 'q')
        // {
        //     break;
        // }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);

        unsigned long timer_end = GetTickCount();
        fps = 1000.0/(timer_end - timer_start);
        // std::cout << frame.cols << " " << frame.rows << " " << fps <<  std::endl;

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
