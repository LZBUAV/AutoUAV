#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{ 
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 
    local_pos = *msg;
}

int main(int argc, char** argv)
{ 
    ros::init(argc,argv,"guided"); 
    ros::NodeHandle nh; 
    ros::Rate rate(50); 

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb); 
    ros::Subscriber pos_cur_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,pos_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); 
    ros::ServiceClient arming_clinet = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"); 
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff",10); 
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    

    geometry_msgs::PoseStamped pos_des; 
    pos_des.pose.position.x = 0; 
    pos_des.pose.position.y = 0; 
    pos_des.pose.position.z = 20; 

    mavros_msgs::SetMode guide_mode; 
    guide_mode.request.custom_mode = "GUIDED"; 

    mavros_msgs::CommandBool arming; 
    arming.request.value = true; 
    
    mavros_msgs::CommandTOL takeoff; 
    takeoff.request.altitude = 2; 

    while(ros::ok()) 
    { 
        if(!current_state.armed ) 
        { 
            arming_clinet.call(arming); 
        } 
        if(current_state.mode != "GUIDED") 
        { 
            set_mode_client.call(guide_mode); 
        } 
        if(current_state.armed && current_state.mode =="GUIDED" && local_pos.pose.position.z<=1.5 ) 
        { 
            takeoff_client.call(takeoff); 
        } 
        else
        { 
            pos_pub.publish(pos_des); 
        } 
        ros::spinOnce(); 
        rate.sleep(); 
    } 
    return 1;
}