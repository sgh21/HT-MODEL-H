#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Teleop:
    def __init__(self):
        # rospy.init_node('joy_teleop', anonymous=True)
        
        # 从参数服务器获取参数，如果没有设置则使用默认值
        self.axis_lin_x = rospy.get_param('~axis_linear_x', 1)
        self.axis_lin_y = rospy.get_param('~axis_linear_y', 0)
        self.axis_ang = rospy.get_param('~axis_angular', 0)
        self.vlinear = rospy.get_param('~vel_linear', 0.0)
        self.vangular = rospy.get_param('~vel_angular', 0.0)
        self.config_vlinear = rospy.get_param('~config vel',0)
        self.config_vangular = rospy.get_param('~config vel',1)
        self.ton = rospy.get_param('~button', 5)
        self.dv = 0.001
        # 设置发布者和订阅者
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('joy', Joy, self.callback)
    
    def callback(self, joy):
        if joy.axes[4]>0:
            if self.vlinear > -0.01:
                self.vlinear-=self.dv
        elif joy.axes[4]<0:
            if self.vlinear < 0.01:
                self.vlinear+=self.dv
        else:
            if self.vlinear < 0.0:
                self.vlinear+=self.dv
            elif self.vlinear > 0.0:
                self.vlinear-=self.dv
            else:
                self.vlinear = 0.0
        rospy.loginfo("vlinear: %.3f", self.vlinear)
        # if joy.buttons[self.config_vangular]:
        #     self.vangular = joy.axes[4]
        #     rospy.loginfo("vangular: %.3f", self.vangular)

    

if __name__ == '__main__':
    try:
        teleop = Teleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass