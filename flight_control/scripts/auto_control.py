#! /usr/bin/env python
# encoding: utf-8

# By LiZhanBo 2021/5/21 此代码价值一亿人民币，版权所有 翻版必究
# 程序应该将Ardupilot的坐标系模式设置为FRAME_BODY_NED = 8，然后向MavROS的/mavros/setpoint_velocity/cmd_vel节点发送RFU坐标系，
# MAVROS会自动将RFU坐标系下的线速度和角速度转换为Ardupilot的FRAME_BODY_NED下的速度分量
# 启动SITL、GAZEBO、apm.launch后，启动该程序，该程序使用步骤：1.按g设置微GUIDED模式，2.按y解锁，3.按t起飞，4,起飞完成后按b开始控制，5.按f设置为FRAME_BODY_NED模式,然后按9个控制按键修改速度值
# 按键操作： w：增加前飞速度，x：增加后飞速度，a：增加左飞速度，d：增加右飞速度，q：增加左转速度，e：增加右转速度，z：增加高度，c：降低高度
#  q w e
#  a s d
#  z x c
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from track_kcf.msg import tracker_result
from obstacle_avoidance.msg import obstacle_avoidance_result
from simple_pid import PID

class auto_control():
    def __init__(self):
        self.forward  = 0.0
        self.right = 0.0
        self.up = 0.0
        self.angle  = 0.0
        self.MAX_LIN_VEL = 2.5
        self.MAX_ANGLE = 0.4
        self.LIN_VEL_STEP_SIZE = 0.1
        self.ANGLE_STEP_SIZE = 0.1
        self.flag = 0
        self.twist = TwistStamped()
        #航向PID控制
        self.yaw_pid = PID(1.0/320.0, 1.0/3420.0, 1.0/8200.0, 320.0)
        #前进速度PID控制
        self.forward_pid = PID(1.0/120.0, 1.0/1600.0, 1.0/9000.0, 340.0)
        #左右飞行速度PID控制
        self.right_pid = PID(1.0/12.0, 0.0, 0.0, 0)
        #上下飞行速度PID控制 
        self.up_pid = PID(1.0/6.0, 0.0, 0.0, 0)

    def track_call_back(self, center):

        #yaw控制
        d_v_angle = self.yaw_pid(center.center_x)
        self.angle = d_v_angle 

        #forward控制
        d_v_fowward = self.forward_pid(center.center_y)
        self.forward = d_v_fowward

        if self.forward > self.MAX_LIN_VEL:
            self.forward = self.MAX_LIN_VEL
        elif self.forward < -self.MAX_LIN_VEL:
            self.forward = -self.MAX_LIN_VEL

        if self.angle > self.MAX_ANGLE:
            self.angle = self.MAX_ANGLE
        elif self.angle < -self.MAX_ANGLE:
            self.angle = -self.MAX_ANGLE

        # print('angle :', self.angle)
        # print('foward', self.forward)

        # self.twist.twist.linear.x = self.forward
        # if self.flag == 1:
        #     self.twist.twist.linear.x = 0
        # self.twist.twist.angular.z = self.angle

        # pub.publish(self.twist)


    def avoidance_call_back(self, center):

        if center.center_x == 999:
            self.twist.twist.linear.y = 0 
            self.twist.twist.linear.x = 0
            self.right = 0
            self.up = 0
            self.flag = 1
        else:
            self.flag = 0
            #right控制
            d_v_right = self.right_pid(center.center_x)
            self.right = d_v_right

            #up控制cd 
            d_v_up = self.up_pid(center.center_y)
            self.up = -d_v_up

            if self.right > self.MAX_LIN_VEL:
                self.right = self.MAX_LIN_VEL
            elif self.right < -self.MAX_LIN_VEL:
                self.right = -self.MAX_LIN_VEL

            if self.up > self.MAX_LIN_VEL:
                self.up = self.MAX_LIN_VEL
            elif self.up < -self.MAX_LIN_VEL:
                self.up = -self.MAX_LIN_VEL

        print('right :', self.right)
        print('up', self.up)
        

        self.twist.twist.linear.x = 1.5
        self.twist.twist.linear.y = self.right 
        self.twist.twist.linear.z = self.up

        pub.publish(self.twist)



        


if __name__=="__main__":

    auto_control_ = auto_control()
    rospy.init_node('auto_control')
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    rospy.Subscriber('tracker_result', tracker_result, auto_control_.track_call_back)
    rospy.Subscriber('obstacle_avoidance_result', obstacle_avoidance_result, auto_control_.avoidance_call_back)
    rospy.spin()