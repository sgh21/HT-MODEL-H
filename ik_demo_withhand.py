#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sys

print("Python version:", sys.version)

import numpy as np
import rospy
from sensor_msgs.msg import JointState

sys.path.append("./walking_packet")
from model_h_kinematics import *
from time import sleep
import time

if __name__ == "__main__":
    rospy.init_node('thmos_zmp_walk', anonymous=True) 
    joint_goal_publisher = rospy.Publisher('/walking_motor_goals', JointState, queue_size=1)
    joint_goal_msg = JointState()
    rate = rospy.Rate(50)
    leg_c = ModelHLegIK(way_left = [1.0,-1.0,1.0,-1.0,-1.0,1.0], way_right = [-1.0,1.0,-1.0,1.0,-1.0,1.0])
    leg_r_ang = leg_c.LegIKMove('right', [0.0, 0.0, -0.30, 0, 0, 0.2]).copy()
    arm_r_ang = [0, -0.1, 0]
    leg_l_ang = leg_c.LegIKMove('left', [0.0, 0.0, -0.30, 0, 0, 0.2]).copy()
    arm_l_ang = [0, 0.1, 0]
    command_list = leg_l_ang + leg_r_ang 
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
    


    ang_move = np.array(command_list)+ np.array(zero_list)
    for step in range(1000):
        joint_goal_msg.position = ang_move
        joint_goal_publisher.publish(joint_goal_msg)  
        rate.sleep()
    # mctl = ModelHCtl()
    # for i in range(100):
    #   mctl.MotorSafeMove(ang_move.tolist())
    #   sleep(0.05)
