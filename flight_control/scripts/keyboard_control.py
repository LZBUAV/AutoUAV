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
import sys, select, os
import tty, termios
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, SetMavFrame

MAX_LIN_VEL = 20
MAX_ANGLE = 1
LIN_VEL_STEP_SIZE = 0.1
ANGLE_STEP_SIZE = 0.1


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_control')

    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    cmd= String()
    twist = TwistStamped()    
    forward  = 0.0
    right = 0.0
    up = 0.0
    angle  = 0.0
    takeoff_complete = 0
    

    while(1):
        key = getKey()

        if key == 'y':
            rospy.wait_for_service('/mavros/cmd/arming')
            arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_res = arming(True)
            if arm_res.success:
                print('Arming')

        elif key == 'n':
            rospy.wait_for_service('/mavros/cmd/arming')
            arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_res = arming(False)
            if arm_res.success:
                print('Disarming')

        elif key == 'r':
            rospy.wait_for_service('/mavros/set_mode')
            set_mod = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode_res = set_mod(custom_mode = 'RTL')
            if set_mode_res.mode_sent:
                takeoff_complete = 0
                print('RTL')

        elif key == 'l':
            rospy.wait_for_service('/mavros/cmd/land')
            land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            land_res = land(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
            if land_res.success:
                takeoff_complete = 0
                print('LAND')

        elif key == 'g':
            rospy.wait_for_service('/mavros/set_mode')
            set_mod = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode_res = set_mod(custom_mode = 'GUIDED')
            if set_mode_res.mode_sent:
                print('GUIDED')

        elif key == 't':
            alt = 1.5
            rospy.wait_for_service('/mavros/cmd/takeoff')
            takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            takeoff_res = takeoff(altitude = alt, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
            if takeoff_res.success:
                print('TAKEOFF :', alt, 'm')

        elif key == 'b':
            takeoff_complete = 1
            print('takeoff completed, begin fly')

        elif key == 'f':
            rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
            frame = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMavFrame)
            frame_set_res = frame(8)
            if frame_set_res.success:
                print('SET_FRAME: BOADY')
        
        elif key == 'w' :
            forward = forward + LIN_VEL_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 'x' :
            forward = forward - LIN_VEL_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 'a' :
            right = right + LIN_VEL_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 'd' :
            right = right - LIN_VEL_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 'z' :
            up = up + LIN_VEL_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 'c' :
            up = up - LIN_VEL_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))
        
        elif key == 'q' :
            angle = angle + ANGLE_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 'e' :
            angle = angle - ANGLE_STEP_SIZE
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))

        elif key == 's' :
            forward   = 0.0
            angle   = 0.0
            right = 0.0
            up = 0.0
            print("currently:\t forward vel %.2f\t right vel %.2f\t up vel %.2f\t steering angle %.2f " % (forward, right, up, angle))
            
        elif (key == '\x03'):
                break

        if forward > MAX_LIN_VEL:
            forward = MAX_LIN_VEL
        elif forward < -MAX_LIN_VEL:
            forward = -MAX_LIN_VEL

        if right > MAX_LIN_VEL:
            right = MAX_LIN_VEL
        elif right < -MAX_LIN_VEL:
            right = -MAX_LIN_VEL

        if up > MAX_LIN_VEL:
            up = MAX_LIN_VEL
        elif up < -MAX_LIN_VEL:
            up = -MAX_LIN_VEL

        if angle > MAX_ANGLE:
            angle = MAX_ANGLE
        elif angle < -MAX_ANGLE:
            angle = -MAX_ANGLE

        twist.twist.linear.y = right 
        twist.twist.linear.x = forward
        twist.twist.linear.z = up
        twist.twist.angular.z = angle

        if takeoff_complete:
            pub.publish(twist)   


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)