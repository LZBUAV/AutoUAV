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
from sensor_msgs.msg import Range
from tracker_eco.msg import tracker_result
from image_match.msg import match_result
from obstacle_avoidance.msg import obstacle_avoidance_result
from simple_pid import PID
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMavFrame
import sys, select, os
import tty, termios

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class auto_control():
    def __init__(self):
        self.forward  = 0.0
        self.right = 0.0
        self.up = 0.0
        self.angle  = 0.0
        self.MAX_LIN_VEL_F = 0.8
        self.MIN_LIN_VEL_F = 0.0
        self.MAX_LIN_VEL_R = 0.6
        self.MAX_LIN_VEL_U = 1.0
        self.MAX_ANGLE_VEL = 1.0
        self.twist = TwistStamped()
        self.loss_count = 0
        self.mode = 'none'
        self.can_pass = 0
        self.last_mode = 'none'
        self.distance = 0.0
        #航向PID控制
        self.yaw_pid = PID(Kp=0.0012, Ki=0.0, Kd=0, setpoint=640.0, output_limits=(-self.MAX_ANGLE_VEL, self.MAX_ANGLE_VEL))
        #前进速度PID控制
        self.forward_pid = PID(1.0/120.0, 1.0/1600.0, 1.0/9000.0, 150.0, output_limits=(self.MIN_LIN_VEL_F, self.MAX_LIN_VEL_F))
        #左右飞行速度PID控制
        self.right_pid = PID(Kp=0.8, Ki=0.0003, Kd=0.0, setpoint=80, output_limits=(-self.MAX_LIN_VEL_R, self.MAX_LIN_VEL_R))
        #上下飞行速度PID控制 
        self.up_pid = PID(Kp=0.8, Ki=0.0004, Kd=0.0, setpoint=1.2, output_limits=(-self.MAX_LIN_VEL_U, self.MAX_LIN_VEL_U))

    def track_call_back(self, center):
        if self.can_pass == 1:
            #yaw控制
            d_v_angle = self.yaw_pid(center.center_x)
            self.angle = d_v_angle 

            #forward控制
            d_v_fowward = self.forward_pid(center.height)
            self.forward = d_v_fowward

            # if self.forward > self.MAX_LIN_VEL:
            #     self.forward = self.MAX_LIN_VEL
            # elif self.forward < -self.MAX_LIN_VEL:
            #     self.forward = -self.MAX_LIN_VEL

            # if self.angle > self.MAX_ANGLE:
            #     self.angle = self.MAX_ANGLE
            # elif self.angle < -self.MAX_ANGLE:
            #     self.angle = -self.MAX_ANGLE

            self.twist.twist.linear.x = self.forward
            self.twist.twist.angular.z = self.angle

    def track_call_back_match(self, center):
        if self.can_pass == 1:
            if center.is_person == 1 and center.conf > 0.2:
                self.loss_count = 0
                #yaw控制
                d_v_angle = self.yaw_pid(center.x+center.width//2)
                self.angle = d_v_angle 

                #forward控制
                d_v_fowward = self.forward_pid(center.height)
                self.forward = d_v_fowward

                self.twist.twist.linear.x = self.forward
                self.twist.twist.angular.z = self.angle
            else:
                self.loss_count += 1
                if self.loss_count >= 49:
                    self.forward = 0.0
                    self.angle = 0.2

                    self.twist.twist.linear.x = self.forward
                    self.twist.twist.angular.z = self.angle

    def avoidance_call_back(self, center):
        if center.is_ok:
            self.can_pass = 1
            #right控制
            d_v_right = self.right_pid(center.center_x)
            self.right = d_v_right

            self.twist.twist.linear.y = self.right 
        else:
            self.can_pass = 0
            self.forward = 0.0
            self.right = 0.0
            self.angle = 0.0

            self.twist.twist.linear.x = self.forward
            self.twist.twist.angular.z = self.angle
            self.twist.twist.linear.y = self.right 
            

    def state_call(self, state):
        self.mode = state.mode

    def height_call(self, height):
        self.distance = height.range
        #up控制 
        d_v_up = self.up_pid(height.range)
        self.up = d_v_up

        # self.twist.twist.linear.z = self.up

        
if __name__=="__main__":

    auto_control_ = auto_control()
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('auto_control')
    rospy.Subscriber('/mavros/state', State, auto_control_.state_call, queue_size=1)

    while not rospy.is_shutdown():
        if auto_control_.mode == 'GUIDED':
            break
        if auto_control_.last_mode != auto_control_.mode:
            print(auto_control_.mode)
            auto_control_.last_mode = auto_control_.mode 
        
    print(auto_control_.mode)

    rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
    frame = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMavFrame)
    frame_set_res = frame(8)
    while not frame_set_res.success:
        frame_set_res = frame(8)
    print('SET_FRAME: BOADY: ', frame_set_res.success)

    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
    rospy.Subscriber('tracker_result', tracker_result, auto_control_.track_call_back)
    # rospy.Subscriber('match_result', match_result, auto_control_.track_call_back_match)
    rospy.Subscriber('obstacle_avoidance_result', obstacle_avoidance_result, auto_control_.avoidance_call_back)
    # rospy.Subscriber('/mavros/distance_sensor/rangefinder_pub', Range, auto_control_.height_call)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        pub.publish(auto_control_.twist)
        # print(auto_control_.distance)

        key = getKey()
    
        if key == 'c':
            break

        if auto_control_.mode != 'GUIDED':
            break

        rate.sleep()


        # #航向PID控制
        # self.yaw_pid = PID(1.0/320.0, 1.0/3420.0, 1.0/8200.0, 640.0)
        # #前进速度PID控制
        # self.forward_pid = PID(1.0/120.0, 1.0/1600.0, 1.0/9000.0, 250.0)
        # #左右飞行速度PID控制
        # self.right_pid = PID(1.0/12.0, 0.0, 0.0, 80.0)
        # #上下飞行速度PID控制 
        # self.up_pid = PID(1.0/6.0, 0.0, 0.0, 30.0)