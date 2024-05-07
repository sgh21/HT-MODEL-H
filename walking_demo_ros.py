#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import numpy as np
import sys
import rospy
import time
import matplotlib.pyplot as plt

from random import random 
from time import sleep
from sensor_msgs.msg import JointState
sys.path.append('./walking_packet')
from thmos_walk_engine import *
#from model_h_motors_control import *
from joy_control import Teleop



if __name__ == '__main__':
  # with open("walking_data.txt",'w') as f:
  #   f.write("\n")
  TIME_STEP = 0.001
  # initial ---
  # mctl = ModelHCtl()
  arm_r_ang = [0, -0.1, 0]
  arm_l_ang = [0, 0.1, 0]
  # -- stand leg zero position --    
  zero_list = [0] * 12
  
  # left leg
  zero_list[0] = 0 / 180 * np.pi  
  zero_list[1] = 0 / 180 * np.pi
  zero_list[2] = 0 / 180 * np.pi
  zero_list[3] = -115 / 180 * np.pi
  zero_list[4] = 0 / 180 * np.pi
  zero_list[5] = 0 / 180 * np.pi

      
  # right leg
  zero_list[6] = 10 / 180 * np.pi
  zero_list[7] = 15 / 180 * np.pi
  zero_list[8] = 0 / 180 * np.pi
  zero_list[9] = 20 / 180 * np.pi
  zero_list[10] = 0 / 180 * np.pi
  zero_list[11] = 0 / 180 * np.pi

  # control box ----


  sys.path.append(sys.path[0] + '/param.txt')
  param_path=sys.path[-1]
  param=np.genfromtxt(fname=param_path,dtype=float,delimiter=",",comments="#",max_rows=38,invalid_raise=False)
  Params = {              
            'foot_width' : param[0],
            'ex_foot_width' : param[1],
            'foot_height' :param[2],
            'com_height' : param[3],
            'com_x_offset' : param[4],
            'com_y_offset' :param[5],
            'trunk_height' : param[6],
            'walking_period' : param[7],
            'both_foot_support_time' : param[8],
            'dt' : param[9],
            'max_vx' : param[10],
            'max_vy': param[11],
            'max_vth' : param[12],
            'k_x_offset':param[13],#ex_com_x_offset k
            'k_y_offset':param[14],#ex_com_y_offset k
            'trunk_pitch':param[15],
            'way_left' : [1.0,-1.0,1.0,-1.0,-1.0,1.0],
            'way_right' : [-1.0,1.0,-1.0,1.0,-1.0,1.0],
            'leg_rod_length' : [0.15,0.16,0.0255,0.036,0.025,0.112,0.065]
            }

  walk = walking(**Params)
  j = 0
  n = 0
  k = 0 
  nk = 0

  rospy.init_node('thmos_zmp_walk', anonymous=True) 
  joint_goal_publisher = rospy.Publisher('/walking_motor_goals', JointState, queue_size=1)
  joint_goal_msg = JointState()
  rate = rospy.Rate(100)

  teleop = Teleop()


  record_com_x = []
  record_com_y = []
  record_L_foot_z = []
  record_R_foot_z = []

  
  for step in range(10000):
    j +=1
    if j>=2:
      if n == 0:
        walk.setGoalVel([0.05, 0.0, 0.0])
      j = 0
      leg_r_ang,leg_l_ang,n = walk.getNextPos()
      record_com_x.append(walk.body_x)
      command_list = leg_l_ang +leg_r_ang  
      ang_move = np.array(command_list) + np.array(zero_list)
      record_com_y.append(walk.body_y)
      record_L_foot_z.append(walk.left_up - walk.trunk_height)
      record_R_foot_z.append(walk.right_up - walk.trunk_height)
      # simulation / motor control ---
      command_list = leg_l_ang +leg_r_ang  
      ang_move = np.array(command_list) + np.array(zero_list)
      joint_goal_msg.position = ang_move
      joint_goal_publisher.publish(joint_goal_msg)  
      rate.sleep()



  plt.figure("com state")
  plt.subplot(2,1,1)
  plt.plot(record_com_x,label="com x")
  plt.legend()
  plt.subplot(2,1,2)
  plt.plot(record_com_y,label="com y")
  plt.legend()
  plt.figure("foot state")
  plt.subplot(2,1,1)
  plt.plot(record_L_foot_z,label="L foot Z")
  plt.legend()
  plt.subplot(2,1,2)
  plt.plot(record_R_foot_z,label="R foot Z")
  plt.legend()
  plt.show()
  

        # sleep(0.03)

