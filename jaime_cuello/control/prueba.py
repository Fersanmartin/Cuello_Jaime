#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray


x,y=[40,40]


#Callback para arduino serial
def callback_IMU(data):
    global x,y

    x, y =data.data[0], data.data[1]


def goal_vel(ID , vel):
    #rospy.wait_for_service('/dynamixel_command')
    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Moving_Speed',vel)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)




rospy.init_node('Dynamixel', anonymous=True)

rospy.Subscriber("gyro", Float64MultiArray, callback_IMU)

y_a=y

goal_vel(2,400)
n=0

if __name__ == "__main__":

    while n==0:

        rospy.Subscriber("gyro", Float64MultiArray, callback_IMU)

        time.sleep(0.5)
        if y_a<y and y<0:
            vel=400+1023
        elif y_a<y and y>0:
            vel=400
        elif y_a>y and y<0:
            vel=400
        elif y_a>y and y>0:
            vel=400+1023
            
        y_a=y
        goal_vel(2,vel)
        #rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback)
        if y<10 and y>-10:
            goal_vel(2,0)
            n=1

        print(vel)
           
        
    






        