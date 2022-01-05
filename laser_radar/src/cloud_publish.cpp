#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_radar/distance.h>

HPS3D_HandleTypeDef handle;
AsyncIObserver_t My_Observer;
ObstacleConfigTypedef ObstacleConf;
ros::Publisher cloud_pub;//Global variable, because the observer callback function needs to be used


//The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	sensor_msgs::PointCloud2 output;
	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			case SIMPLE_ROI_PACKET:				
				break;
			case FULL_ROI_PACKET:		
				break;
			case FULL_DEPTH_PACKET: /*point cloud data and depth data*/	
				if(ros::ok())
				{
					cloud.width = event->MeasureData.point_cloud_data->width;
					cloud.height = event->MeasureData.point_cloud_data->height;
					cloud.points.resize(cloud.width * cloud.height);
					for (size_t i = 0; i < cloud.points.size (); i++)
					{
						if(event->MeasureData.point_cloud_data[0].point_data[i].z < LOW_AMPLITUDE)
						{
							cloud.points[i].x  = event->MeasureData.point_cloud_data[0].point_data[i].x;
							cloud.points[i].y = event->MeasureData.point_cloud_data[0].point_data[i].y;
							cloud.points[i].z = event->MeasureData.point_cloud_data[0].point_data[i].z;	
						}
						else
						{
							cloud.points[i].x = 0;
							cloud.points[i].y = 0;
							cloud.points[i].z = 0;	
						}
					}					
					//Convert the cloud to ROS message
					pcl::toROSMsg(cloud, output);
					output.header.frame_id = "hps";
					cloud_pub.publish(output);
				
				}
				break;
			case SIMPLE_DEPTH_PACKET:			
				break;
			case OBSTACLE_PACKET:			
				break;
			case NULL_PACKET:
				printf("null packet\n");
				break;
			default:
				printf("system error!\n");
				break;
		}
	}
}

//check ctrl+c signal
void signal_handler(int signo)
{
    if(HPS3D_RemoveDevice(&handle) != RET_OK)
    {
		printf("HPS3D_RemoveDevice faild\n");
    }
    else
    {	
        printf("HPS3D_RemoveDevice succeed\n");
    }
    exit(0);
}


//printf log callback function
void my_printf(char *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "cloud_publish");//ros init
	ros::NodeHandle n;
	RET_StatusTypeDef ret = RET_OK;
	AsyncIObserver_t My_Observer;

	//Install the signal
	if(signal(SIGINT,signal_handler) == SIG_ERR || signal(SIGTSTP,signal_handler) == SIG_ERR)
	{
		printf("sigint error");
	}
	
	//Create a topic
	cloud_pub = n.advertise<sensor_msgs::PointCloud2> ("cloudpoint", 1);
	
	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(false);
	HPS3D_SetDebugFunc(&my_printf);

	int number = 0;
	ret = HPS3D_SetMeasurePacketType(DEPTH_DATA_PACKET);
	if(ret != RET_OK)
	{
		printf("SetMeasurePacketType failed\n");
		return 0;
	}
	else
	{
		printf("SetMeasurePacketType success\n");
	}
		
	number = HPS3D_AutoConnectAndInitConfigDevice(&handle);
	if(number == 0)
	{
		printf("AutoConnectAndInitConfigDevice failed\n");
		return 0;
	}
	else
	{
		printf("connected number:%d\n",number);
		printf("lidar Connect success\n");
	}
	HPS3D_SetEdgeDetectionEnable(true); 
	/*set convert point cloud data enable*/		
	HPS3D_SetOpticalEnable(&handle, true);
	HPS3D_SetPointCloudEn(true);
	
	//Add observer one
	My_Observer.AsyncEvent = ISubject_Event_DataRecvd;
	My_Observer.NotifyEnable = true;
	HPS3D_AddObserver(&User_Func, &handle, &My_Observer);		

	//Set running mode
	handle.RunMode = RUN_CONTINUOUS;
	HPS3D_SetRunMode(&handle);
	
	while(1)
	{
		sleep(10);		
	}
		
	return 0;
}

