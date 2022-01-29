#!/usr/bin/python 
# encoding: utf-8
import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ctypes import *

from numpy.lib.function_base import select
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from obstacle_avoidance.msg import distance, obstacle_avoidance_result


class avoidance():

    def __init__(self):
        self.obstacleresult = obstacle_avoidance_result()

    def call_back(self, cloud):
        assert isinstance(cloud, PointCloud2)
        gen = point_cloud2.read_points(cloud)
        #实物
        data = []
        for point in gen:
            data.append(point[0])

        #仿真
        # cou = 0
        # swap_data = []
        # data_1 = []
        # for point in gen:
        #     data_1.append(0)
        #     swap_data.append(point[0])
        # for col in range(160):
        #     for row in range(60):
        #         data_1[160*row + col] = swap_data[cou]
        #         cou += 1
        # data = []
        # data = data_1[::-1]
        #仿真结束
        
        m10 = []
        '''
        二值化
        '''
        #安全距离，单位毫米mm
        safety_distance = 3
        i = 0
        for dist_value in range(9600):
            each_line=data[dist_value]
            #根据深度值是否在安全距离外，将其二值化
            if (each_line>safety_distance):#65500是饱和点，距离小于200mm；距离值0是无穷远点，绝对安全。
                each_line = 1
            else:
                each_line = 0
            m10.append(each_line)	

        m11=np.array(m10)

        #将9600个深度信息转换成60*160的矩阵
        Z = m11
        Z = np.array(Z[0:9600])
        Z = Z.reshape(60,160)



        '''
        滑窗算法
        '''

        '''
        滑窗算法Step1 : 初始设置与控制参数
        '''
        #初始滑窗大小
        window_size = 20
        #初始检测区域边界
        row_start = 0
        row_end = row_start + window_size
        column_start = 0
        column_end = column_start + window_size
        #生成20*20的滑窗
        window = np.ones((window_size,window_size))*1

        #控制参数
        #是否显示图像
        display_flag = 0
        #待检测块列步进值
        column_step = 10
        #待检测块行步进值
        row_step = 10
        #滑窗大小步进值
        window_size_step = 10

        #用于存储所有的安全滑窗边长
        safety_window_size = []
        #图像中心点
        centrl_point = []
        #最大安全滑窗下，所有能找到的此大小滑窗的中心点，safty_poit_window存储该中心点对应的滑窗边界
        safty_poit = []
        safty_poit_window = []

        '''
        滑窗算法Step2 : 在当前帧找出最优滑窗大小
        '''
        #计数用
        i = 0
        #当前检测区域
        while True:

            if display_flag ==1:
                Z_temp = m11
                Z_temp = np.array(Z[0:9600])
                Z_temp = Z_temp.reshape(60,160)
            
            #循环次数
            # print("迭代次数：",i)
            i = i+1

            #当前检测块安全时为1
            find_window_success= 1 
            #当前帧检测完成是为1
            search_compeleted = 0  

            #更新待检测块
            now_window = Z[row_start:row_end,column_start:column_end]

            if display_flag==1:
                #动态展示
                Z_temp[row_start:row_end,column_start:column_end]=0
                plt.ion()
                plt.imshow(Z_temp)
                ax = plt.gca()                                 #获取到当前坐标轴信息
                ax.xaxis.set_ticks_position('top')             #将X坐标轴移到上面
                plt.show()
                plt.pause(0.005)
                plt.clf()

            #检测当前的待检测块，按元素进行与运算，结果权威True则该块为安全区域，检测通过
            result = np.logical_and(window,now_window)

            #判断当前检测区域是否安全
            for wise in result.flat:#遍历二维数组的所有元素
                #当前块有一个与结果不是True，就不是安全区域
                if wise == False:
                    # print("now window is not safety")
                    #因为当前区域不是安全区域，所以待检测块右移column_step个像素
                    column_start = column_start+column_step
                    column_end = column_start + window_size
                    #如果待检测块已经移动到最右端，那么待检测块下移row_step个单位，在新的待检测行中，再次从最左端开始右移
                    if column_end > 160:
                        column_start = 0
                        column_end = column_start + window_size
                        row_start = row_start+row_step
                        row_end = row_start + window_size
                        #如果检测块已经移动到了最右下角，则本帧图像搜索完毕，search_compeleted置1
                        if row_end >60:
                            # print("Seach Compeleted")
                            search_compeleted = 1
                            break#本帧图像检测完，跳出该for循环
                    #当前检测块不是安全区域，所以将find_window_success清零
                    find_window_success = 0
                    break#一旦确定该待检测块不是安全区域时，就跳出该for循环

            #本帧图像检测完成，跳出while循环
            if search_compeleted==1:
                break

            #如果当前检测块是安全区域，则在上述for循环中find_window_success不会被清零，就会执行下边if代码
            if find_window_success==1:
                # print("now window is safety")
                #记录本次安全区域对应的滑窗值
                safety_window_size.append(window_size)
                #原地向右下角方向将滑窗值增加window_size_step
                window_size = window_size+window_size_step
                #更新检测滑窗
                window = np.ones((window_size,window_size))*1
                #更新待检测块边界，与检测滑窗匹配
                row_end = row_start + window_size
                column_end = column_start + window_size
                #待检测块任意一边碰到边界，则认为上一个检测块大小就是最优滑窗大小，本帧图像搜索结束，跳出while循环
                if column_end > 160 or row_end > 60:
                    break

        #并输出历史安全滑窗
        # print("所有可行的安全滑窗值：")
        # print(safety_window_size)

        #如果当前帧存在大于20的安全滑窗则继续
        if len(safety_window_size) and max(safety_window_size)>=20:

            '''
            滑窗算法Step3 : 找出当前帧中所有符合最优滑窗大小的安全区域
            '''
            #已经发现了全部存在的可行滑窗大小，并存储在safety_window_size中，其中最后一个是最大的滑窗值，就是最优滑窗值，下面开始寻找整幅图中该滑窗值下的最优安全区域
            optimal_window_size = safety_window_size.pop()

            #初始检测区域边界，从左上角开始，检测区域边长固定为最优滑窗大小optimal_window_size
            row_start = 0
            row_end = row_start + optimal_window_size
            column_start = 0
            column_end = column_start + optimal_window_size

            #生成最优大小的滑窗
            window = np.ones((optimal_window_size,optimal_window_size))*1

            #计数
            i = 0
            while True:
                if display_flag == 1:
                    Z_temp = m11
                    Z_temp = np.array(Z[0:9600])
                    Z_temp = Z_temp.reshape(60,160)

                # print("迭代次数：",i)
                i = i+1

                #当前检测块安全时为1
                find_window_success= 1 
                #当前帧检测完成是为1
                search_compeleted = 0  

                #更新待检测块
                now_window = Z[row_start:row_end,column_start:column_end]

                if display_flag==1:
                    #动态展示
                    Z_temp[row_start:row_end,column_start:column_end]=0
                    plt.ion()
                    plt.imshow(Z_temp)
                    ax = plt.gca()                                 #获取到当前坐标轴信息
                    ax.xaxis.set_ticks_position('top')             #将X坐标轴移到上面
                    plt.show()
                    plt.pause(0.005)
                    plt.clf()

                #检测当前的待检测块，按元素进行与运算，结果权威True则该块为安全区域，检测通过
                result = np.logical_and(window,now_window)

                #判断当前检测区域是否安全
                for wise in result.flat:#遍历二维数组的所有元素
                    #当前块有一个与结果不是True，就不是安全区域
                    if wise == False:
                        # print("now window is not safety")
                        #因为当前区域不是安全区域，所以待检测块右移column_step个像素
                        column_start = column_start+column_step
                        column_end = column_start + optimal_window_size
                        #如果待检测块已经移动到最右端，那么待检测块下移row_step个单位，在新的待检测行中，再次从最左端开始右移
                        if column_end > 160:
                            column_start = 0
                            column_end = column_start + optimal_window_size
                            row_start = row_start+row_step
                            row_end = row_start + optimal_window_size
                            #如果检测块已经移动到了最右下角，则本帧图像搜索完毕，search_compeleted置1
                            if row_end >60:
                                # print("Seach Compeleted")
                                search_compeleted = 1
                                break#本帧图像检测完，跳出该for循环
                        #当前检测块不是安全区域，所以将find_window_success清零
                        find_window_success = 0
                        break#一旦确定该待检测块不是安全区域时，就跳出该for循环
                
                #本帧图像检测完成，跳出while循环
                if search_compeleted==1:
                    break
                
                #如果当前检测块是安全区域，则在上述for循环中find_window_success不会被清零，就会执行下边if代码
                if find_window_success==1:
                    # print("now window is safety")

                    #记录待安全块的信息，包括中心点坐标X_point、Y_point，当前的检测起始边界
                    X_point = (int)((column_start+column_end-1)/2)
                    Y_point = (int)((row_start+row_end-1)/2)
                    safty_poit.append([X_point,Y_point])
                    safty_poit_window.append([row_start,column_start])

                    #同时记录图像中心点，方便后期计算使用
                    centrl_point.append([79,29])

                    #当前区域是安全区域，待检测块右移column_step个像素
                    column_start = column_start+column_step
                    column_end = column_start + optimal_window_size

                    #如果待检测块已经移动到最右端，那么待检测块下移row_step个单位，在新的待检测行中，再次从最左端开始右移
                    if column_end > 160:
                        column_start = 0
                        column_end = column_start + optimal_window_size
                        row_start = row_start+row_step
                        row_end = row_start + optimal_window_size
                        #如果检测块已经移动到了最右下角，则本帧图像搜索完毕，跳出while循环
                        if row_end >60:
                            # print("Seach Compeleted")
                            break
                
            #输出本帧图像在最优滑窗大小下的所有安全点，以及与之对应的滑窗起始边界
            # print("safty_poit: ")
            # print(safty_poit)
            # print("safty_poit_window: ")
            # print(safty_poit_window)

            '''
            滑窗算法Step4 : 找出当前帧中安全区域
            '''
            #计算每个安全点到图像中心点的距离，距离为两点x、y坐标差的绝对值的和
            safty_point_err = np.subtract(safty_poit,centrl_point)
            safty_point_dist = abs(safty_point_err[:,0]) + abs(safty_point_err[:,1])

            #输出距离
            # print("safty_point_dist: ")
            # print(safty_point_dist)

            #找出最小距离点对应的索引值optimal_index，并求出其对应的最优点optimal_point和对应的滑窗起始边界optimal_window_corresponding_to_optimal_point
            safty_point_dist = list(safty_point_dist)
            optimal_index = safty_point_dist.index(min(safty_point_dist))
            optimal_point = safty_poit[optimal_index]
            optimal_window_corresponding_to_optimal_point = safty_poit_window[optimal_index]

            #输出上述信息
            # print("optimal_point: ")
            # print(optimal_point)
            # print("optimal_window_corresponding_to_optimal_point: ")
            # print(optimal_window_corresponding_to_optimal_point)
            # print("optimal_window_size:")
            # print(optimal_window_size)

            '''
            滑窗算法Step4 : 结果显示
            '''
            #画出最优区域
            row_start = optimal_window_corresponding_to_optimal_point[0]
            row_end = row_start + optimal_window_size
            column_start = optimal_window_corresponding_to_optimal_point[1]
            column_end = column_start + optimal_window_size

            plt.ion()
            plt.imshow(Z)
            ax = plt.gca()                                 #获取到当前坐标轴信息
            ax.xaxis.set_ticks_position('top')             #将X坐标轴移到上面
            ax.add_patch(plt.Rectangle((column_start-1,row_start-1),optimal_window_size,optimal_window_size,fill=False))
            plt.scatter(optimal_point[0],optimal_point[1],c='r',alpha=1,marker='o')
            plt.show()
            plt.pause(0.05)
            plt.cla()
            print((optimal_point[0]-79,29-optimal_point[1],1))
            self.obstacleresult.center_x = optimal_point[0]-79
            self.obstacleresult.center_y = 29-optimal_point[1]
        else:
            print("当前帧没有可通行区域")
            print((0,0,0))
            self.obstacleresult.center_x = 999
            self.obstacleresult.center_y = 999
            plt.ion()
            plt.imshow(Z)
            ax = plt.gca()                                 #获取到当前坐标轴信息
            ax.xaxis.set_ticks_position('top')             #将X坐标轴移到上面
            plt.show()
            plt.pause(0.05)
            plt.cla()
        pub.publish(self.obstacleresult)

    def call(self, cloud):
        assert isinstance(cloud, PointCloud2)
        gen = point_cloud2.read_points(cloud)
        data = []
        count = 0
        for point in gen:
            count += 1
            data.append(point[0])
        print(count)
        print(sum(data)/count)
 
if __name__=='__main__':
    avoidance = avoidance()
    pub = rospy.Publisher('obstacle_avoidance_result', obstacle_avoidance_result, queue_size=1)
    rospy.init_node('obstacle_avoidance', anonymous=True)
    rospy.Subscriber("/cloudpoint", PointCloud2, avoidance.call_back, queue_size = 1, buff_size = 52428800)
    rospy.spin()