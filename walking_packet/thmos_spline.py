#!/usr/bin/python
# -*- coding: UTF-8 -*-

# Copyright (c). All Rights Reserved.
# -----------------------------------------------------
# File Name:        thmos_spline
# Creator:          Fu Hao
# Version:          0.1
# Created:          2023/11/30
# Description:      Polynomial of degree seven interpolation
# History:
#   <author>      <version>       <time>          <description>
#    Fu Hao         0.1         2023/11/30          create
# -----------------------------------------------------

import numpy as np
class thmos_spline():
  def __init__(self,x0,x1,x2,N):
    """
    Polynomial coefficient calculations
    Parameters;
     x0 - start value
     x1 - mid value
     x2 - end value
     N - frames num
    """
    self.a0 = x0
    self.a3 = ( 64  * (x1-x0) - 22 * (x2-x0) )/ N ** 3
    self.a4 = (-192 * (x1-x0) + 81 * (x2-x0) )/ N ** 4
    self.a5 = ( 192 * (x1-x0) - 90 * (x2-x0) )/ N ** 5
    self.a6 = (-64  * (x1-x0) + 32 * (x2-x0) )/ N ** 6
    self.x_list = []
    self.N = N
    for t in range(N):
      x = self.a0 + self.a3*(t**3) + self.a4*(t**4) + self.a5*(t**5) + self.a6*(t**6)
      self.x_list.append(x)
    return  


  def spline_tracker(self,frame_in):
    """
    Polynomial motion output
    Parameters;
     frame_in - now frame
    Returns:
     x - now motion    
    """
    frame = int(frame_in)
    up_index = 1
    down_index = self.N/2
    fast_period = 0.6
    if(frame < 0):
      x = self.x_list[0]
    elif(frame >= self.N):
      x = self.x_list[self.N - 1]
    else:
      x = self.x_list[frame]
    # elif (frame<self.N*fast_period):
    #   up_index = round(self.N/2*np.sin(frame/(self.N*fast_period)*3.14/2))
    #   x = self.x_list[up_index]
    # elif (frame>=self.N*fast_period and frame<self.N):
    #   down_index = round(self.N/2-1 + self.N/2*np.sin((frame-self.N*fast_period)/(self.N*(1-fast_period))*3.14/2))
    #   x = self.x_list[down_index]


    return x





