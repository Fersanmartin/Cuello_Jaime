#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import numpy as np
import pandas as pd
import pickle
from sensor_msgs.msg import Joy

last_received_time = None
class Master():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0

        self.home=[0,0,0,0,0]

        self.pos=[0,0,0,0,0]
        self.inicio=0
        
        self.joy=[None]

        self.angulo=0
    #Callback para arduino serial
    def callback_IMU(self, data):
        self.x, self.y =data.data[0], data.data[1]


    #Callback para tren superior
    def callback_vel(self,data):

        pid_goal = 0
        current_pos = ((1*self.angulo)*(1023/90)*0.5)
        d = data.dynamixel_state[1].present_velocity
        kp = 3
        kd = 0.1
        
        new_speed = kp*(pid_goal-current_pos) + kd*d
        if self.angulo<0:
               new_speed=(abs(new_speed/2))+ 1024 
        else:
              new_speed=(abs(new_speed/2))
        #print(new_speed)
        

        goal_vel(4, new_speed)


    def callback_multivuelta(self,data):
        offset1= self.home[0]
        offset2= self.home[1]
        
        self.pos[0]=(data.dynamixel_state[0].present_position)
        self.pos[1]=(data.dynamixel_state[1].present_position)
        self.pos[2]=(data.dynamixel_state[4].present_position)

        velocidad=np.abs(300*self.joy[1])
        
        
        


        if velocidad !=0:
            if self.joy[1]>0:
                print(velocidad)
                goal_position(2,28000,modo=1,vel=velocidad)
                rospy.sleep(0.001)
                
            else:
                print(velocidad)
                goal_position(2,-28000,modo=1,vel=velocidad)
                rospy.sleep(0.001)
        

        rospy.sleep(0.001)
        
        
        
    def callback_joy(self,data):
        self.joy=[data.axes[1],data.axes[4]]
        
    #Función calback lectura posicion inidical
    def callback_read(self,data):
        
        self.pos[0]=(data.dynamixel_state[0].present_position)
        self.pos[1]=(data.dynamixel_state[1].present_position)
        self.pos[2]=(data.dynamixel_state[4].present_position)
        # for i in range(3):
        #     self.pos[i]=(data.dynamixel_state[i].present_position)

    def homie(self,data):
        self.home=data.data
        print(self.home)
        print("offset is : ")
        print(self.home[0])
        print(self.home[1])
        

    def Main(self):

        #Se inicia nodo 
        rospy.init_node('Dynamixel', anonymous=True)

        rospy.Subscriber("homie_node", Float64MultiArray, self.homie)
        
        rospy.Subscriber("gyro", Float64MultiArray, self.callback_IMU)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_vel)
        
        rospy.Subscriber("joy", Joy, self.callback_joy)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_multivuelta)
            
        rospy.spin() 
        



###################################################################3

def goal_position(ID , pos, modo,vel):
    
    if modo==0:
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    if modo==1:
        goal_vel(ID,vel)
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    if modo==2:
        if vel>=0:
            goal_vel(ID,vel)
            pos_p=[28000]
            #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos_p)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        if vel<0:
            goal_vel(ID,vel)
            pos_n=[-28000]
        #rospy.wait_for_service('/dynamixel_command')
        try:
            #Se manda el servicio para mover el motor
            dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
            print ("ServiceProxy success ...")
                
            resp= dynamixel_command( '',ID,'Goal_Position',pos_n)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


def goal_vel(ID , vel):
    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        #print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Moving_Speed',vel)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#####################################################################################






############################################################################################





if __name__ == "__main__":
    # Inicializa la clase IMU
    imu = Master()

    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.Main()

        
          