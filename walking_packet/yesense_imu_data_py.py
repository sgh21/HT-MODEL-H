#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# created by qianruiluo(luoqr21@mails.tsinghua.edu.cn) on    2023/11/1
# last modified 2023/12/19


# THIS IS FOR PYBULLET SIMULATION

import threading
import time
import math
import numpy as np
import rospy
from sensor_msgs.msg import Imu

class yesense_imu_Subscriber(object):
    def __init__(self,  filter_factor=0):
        '''
        :param filter_factor: indicates the ratio of data contributed by previous measurements. =0 means no filter but should not be 1. 0.3~0.7 is recommended.
        '''
        
        
        self.imu_ready = False

        # robot orientation
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        # acceleration
        self.ax = 0
        self.ay = 0
        self.az = 0
        # body velocity
        self.vx = 0
        self.vy = 0
        self.vz = 0
        # angular velocity
        self.wx = 0
        self.wy = 0
        self.wz = 0
        # angular acceleration
        self.bx = 0
        self.by = 0
        self.bz = 0
        # filter
        self.filter = filter_factor

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        



    # imu topic contains data of orientation, acceleration and angular velocity.
    
    def imu_callback(self,data):
        w = data.angular_velocity
        orn = data.orientation
        self.quaternion_to_rpy(orn)
        wx_new = (1.0 - self.filter) * w.x + self.filter * self.wx
        wy_new = -(1.0 - self.filter) * -w.y + self.filter * self.wy
        wz_new = -(1.0 - self.filter) * -w.z + self.filter * self.wz
        self.bx = (wx_new - self.wx) * 100
        self.by = (wy_new - self.wy) * 100
        self.bz = (wz_new - self.wz) * 100
        self.wx = wx_new
        self.wy = wy_new
        self.wz = wz_new
        self.imu_ready = True
        rospy.loginfo("angular_velocity:%f %f %f rpy:%f %f %f  ", self.wx, self.wy,self.wz, self.roll,self.pitch,self.yaw )


    def quaternion_to_rpy(self, quaternion):
        # 提取四元数的x、y、z、w分量
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z

        # 计算旋转矩阵
        rotation_matrix = np.array([[1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                                    [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                                    [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]])

        # 提取RPY角
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        # 前x右y下z,如果参考系改的话改这里
        self.roll = roll
        self.pitch = -pitch
        self.yaw = -yaw

if __name__ == '__main__':
    rospy.init_node("test_imu")
    imu = yesense_imu_Subscriber()
    rospy.spin()
    while True:
        if imu.imu_ready:
            print("a:")
            print(imu.ax, imu.ay, imu.az)
            print("ypr:")
            print(imu.yaw, imu.pitch, imu.roll)
            print("w:")
            print(imu.wx, imu.wy, imu.wz)
            print("b:")
            print(imu.bx, imu.by, imu.bz)
            imu.imu_ready = False
        # if imu.velo_ready:
        #     print("v:")
        #     print(imu.vx, imu.vy, imu.vz)
        #     imu.velo_ready = False
