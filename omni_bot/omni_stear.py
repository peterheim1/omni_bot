#!/usr/bin/env python
'''
import omni_stear
st = omni_stear.Control
st(twist)
twist=[0.5, 0, 0.3]


'''
import math
from math import sin, cos, pi

def Control(twist):
    R = 48.4148737476408
    RevD = 1 # used to set forwards or backwards
    v = twist[0]       # m/s
    y = twist[1]        # m/s
    omega = twist[2]      # rad/s
    if v < 0:
        RevD =-1
    A = y + omega*(38/R)
    B = y - omega*(38/R)
    C = abs(v) + omega*(30/R)
    D = abs(v) - omega*(30/R)
                
    j1 = ((math.atan2(A, D))*180) / math.pi #front left in degrees
    j2 = ((math.atan2(A, C))*180) / math.pi #front right
    j3 = math.atan2(B, D)*180 / math.pi # rear left
    j4 = math.atan2(B, C)*180 / math.pi # rear right
    #convert angles to encoder ticks
    Front_left = j1 +90#int(self.translate(j1, -150, 150, 0, 1024))
    Front_right = j2 +90#int(self.translate(j2, -150, 150, 0, 1024))
    Rear_left = j3 +90#int(self.translate(j3, -150, 150, 0, 1024))
    Rear_right = j4 +90#int(self.translate(j4, -150, 150, 0, 1024))



    #velocity commands to be turned into ticks
    V_F_L = ((math.sqrt((B*B)+(D*D))) / 0.29845130209103)*60
    V_F_R = ((math.sqrt((B*B)+(C*C))) / 0.29845130209103)*60
    V_R_L = ((math.sqrt((A*A)+(D*D))) / 0.29845130209103)*60
    V_R_R = ((math.sqrt((A*A)+(C*C))) / 0.29845130209103)*60

    #print(Front_left)
    #print(Front_right)
    #print(Rear_left)
    #print(Rear_right)
    a =[Front_left,Front_right,Rear_left,Rear_right]
    return a
