#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        model_h_kinematics.py
# Creator:          JinYin Zhou
# Version:          0.1
# Created:          2023/2/20
# Description:      leg ik packet
# Function List:    ModelHLegIK:  LegIKMove
# History:
#   <author>      <version>       <time>          <description>
#   Jinyin Zhou     0.1           2024/3/2       create
# -----------------------------------------------------
import math
import numpy as np
import time
import ctypes
#lib = ctypes.CDLL('./MODEL_H_IK_BUILDER/build/libmodel_h_leg_ik.so')
lib = ctypes.CDLL('./walking_packet/MODEL_H_IK_BUILDER/build/libmodel_h_leg_ik.so')
# Define the function argument and return types
lib.leg_ik.argtypes = [
    ctypes.c_double * 7, #length
    ctypes.c_double * 6, # pose
    ctypes.c_double * 6, # way
    ctypes.c_double * 6, # ang
    ctypes.c_bool 
]
lib.leg_ik.restype = None

class ModelHLegIK:
    """inverse kinematics for two leg"""
    def __init__(self, way_left = [1.0,-1.0,1.0,-1.0,-1.0,-1.0], way_right = [-1.0,1.0,-1.0,1.0,-1.0,-1.0],leg_rod_length = [0.15,0.16,0.0274,0.0354,0.022,0.112,0.065]):
        """
        initialize class
        """
        self.leg_rod_length = leg_rod_length
        self.way_left = way_left
        self.way_right = way_right
        # get C list pointer
        self.leg_ang = [0] * 6
        self.end_pos = [0] * 6
        self.c_leg_ang = (ctypes.c_double * 6)(*self.leg_ang)
        self.c_end_pos = (ctypes.c_double * 6)(*self.end_pos)
        self.c_leg_len = (ctypes.c_double * 7)(*self.leg_rod_length)
        self.c_motor_way_l = (ctypes.c_double * 6)(*self.way_left)
        self.c_motor_way_r = (ctypes.c_double * 6)(*self.way_right)
        self.c_left_or_right = (ctypes.c_bool)(True)
        # set pointer
        self.c_leg_ang_ptr = ctypes.pointer(self.c_leg_ang)
        self.c_end_pos_ptr = ctypes.pointer(self.c_end_pos)
        self.c_leg_len_ptr = ctypes.pointer(self.c_leg_len)
        self.c_motor_way_l_ptr = ctypes.pointer(self.c_motor_way_l)
        self.c_motor_way_r_ptr = ctypes.pointer(self.c_motor_way_r) 
        self.c_left_or_right_ptr = ctypes.pointer(self.c_left_or_right)   
            
 
    def LegIKMove(self, LeftorRight, end_pos):
        """
        move left and right leg
        """       
        for index in range(6):
            self.c_end_pos[index] = end_pos[index]
        # caculate 
        if (LeftorRight == 'Left' or LeftorRight == 'left'):
            self.c_left_or_right = False
            lib.leg_ik(self.c_leg_len, self.c_end_pos, self.c_motor_way_l, self.c_leg_ang, self.c_left_or_right)
        else:
            self.c_left_or_right = True
            lib.leg_ik(self.c_leg_len, self.c_end_pos, self.c_motor_way_r, self.c_leg_ang, self.c_left_or_right)
        theta = ctypes.cast(self.c_leg_ang_ptr, ctypes.POINTER(ctypes.c_double * 6)).contents
        for index in range(6):
            if index == 2: #and (LeftorRight == 'right' or LeftorRight == 'Right' ):
                self.leg_ang[3] = theta[index]
            elif index == 3: #and (LeftorRight == 'right' or LeftorRight == 'Right' ):
                self.leg_ang[2] = theta[index]  
            else:              
                self.leg_ang[index] = theta[index]
        return self.leg_ang
        
               
if __name__ == '__main__':
    leg_1 = ModelHLegIK()
    a = leg_1.LegIKMove("left",[-0.3355, 0, 0, 0 , -np.pi/2 , 0])
    print(np.array(a))
    a = leg_1.LegIKMove("left",[0, -0.3355, 0, -np.pi/2, 0, 0])
    print(np.array(a))
    a = leg_1.LegIKMove("left",[0, 0, -0.3355, 0, 0, -np.pi/2])
    print(np.array(a))
    a = leg_1.LegIKMove("left",[0, 0.08, -0.28, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, -0.08, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, 0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, -0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, 0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("left",[0, -0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    print("right ----")
    a = leg_1.LegIKMove("right",[-0.3355, 0, 0, 0 , -np.pi/2 , 0])
    print(np.array(a))
    a = leg_1.LegIKMove("right",[0, -0.3355, 0, -np.pi/2, 0, 0])
    print(np.array(a))
    a = leg_1.LegIKMove("right",[0, 0, -0.3355, 0, 0, -np.pi/2])
    print(np.array(a))
    a = leg_1.LegIKMove("right",[0, 0.08, -0.28, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, -0.08, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, 0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, -0.0001, -0.31, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, 0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
    a = leg_1.LegIKMove("right",[0, -0, -0.30, 0, 0, 0])
    print(np.array(a)/np.pi*180)
