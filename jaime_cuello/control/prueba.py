#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray

last_received_time = None
class IMU():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0
        
    def callback(self, data):
        self.x = input("pos : ")
        goal_position(10,self.x)

    def callback_vel(self,data):

        pid_goal = 0
        current_pos = ((1*self.x)*(1023/90)*0.5)
        d = data.dynamixel_state[1].present_velocity
        kp = 3
        kd = 0.1
        
        new_speed = kp*(pid_goal-current_pos) + kd*d
        if self.x>0:
               new_speed=(abs(new_speed/2))+ 1024 
        else:
              new_speed=(abs(new_speed/2))
        print(new_speed)
        

        goal_vel(4, new_speed,self.x,min=200, max=800)



        

    def listener(self):

        rospy.init_node('Dynamixel', anonymous=True)
        
        rospy.Subscriber("gyro", Float64MultiArray, self.callback)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback)
        
        
        
        rospy.spin() 
        


def goal_position(ID , pos ):
    #rospy.wait_for_service('/dynamixel_command')

    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Goal_Position',pos)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def goal_vel(ID , vel, Pos, min=0,max=1023):
    #rospy.wait_for_service('/dynamixel_command')


    


    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Moving_Speed',vel)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    # Inicializa la clase IMU
    imu = IMU()
    
    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.listener()

        
        