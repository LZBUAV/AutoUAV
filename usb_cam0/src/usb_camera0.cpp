#include "usb_cam0.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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

int save_count = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image0_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("usb_cam0/image_raw", 1);

    cv::Mat frame, undist;
    cv::VideoCapture capture(0);

    double width = 0.0;
    double height = 0.0;
    double fps = 0.0;

    //320*240(33fps),640*480(30fps),800*600(20fps),1280*720(11fps),1920*1080(5fps)
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    capture.set(cv::CAP_PROP_BRIGHTNESS, 60);
    capture.set(cv::CAP_PROP_CONTRAST, 50);
    capture.set(cv::CAP_PROP_EXPOSURE, 156);

    while(nh.ok())
    {
        unsigned long timer_start = GetTickCount();

        capture >> frame;
        width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
        height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);


        cv::undistort(frame, undist, camera_matrix0, distcoficient0);
        
        // cv::imshow("image_forward", undist);
        cv::imwrite("/home/nvidia/project/catkin_ws/src/usb_cam0/cam_result/" + std::to_string(save_count) + ".jpg", undist);
        save_count++;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undist).toImageMsg();
        pub.publish(msg);

        unsigned long timer_end = GetTickCount();
        fps = 1000.0/(timer_end - timer_start);
        std::cout << width << " " << height << " " << fps <<  std::endl;

        ros::spinOnce();

        int key = cv::waitKey(1);
        if(key == 'q')
        {
            break;
        }
    }

    return 0;
}
