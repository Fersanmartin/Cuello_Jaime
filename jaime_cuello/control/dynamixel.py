#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
from std_msgs.msg import Float64MultiArray



matriz=[[[0000,0,0],[1000,0,0],[2000,0,0],[3000,0,0],[4000,0,0],[5000,0,0],[6000,0,0]],[[0000,0,0],[1000,0,0],[2000,0,0],[3000,0,0],[4000,0,0],[5000,0,0],[6000,0,0]]]

last_received_time = None
class IMU():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0
        self.homie=[0,0,0,0,0]

        self.pos=[0,0,0,0,0]
        self.inicio=0

    #Callback para arduino serial
    def callback_IMU(self, data):
        self.x, self.y =data.data[0], data.data[1]


    #Callback para tren superior
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
        #print(new_speed)
        

        goal_vel(4, new_speed)


    def callback_multivuelta(self,data):

        m=matriz[int(data.data[0])][int(data.data[1])]
        
        goal_position(1,m[0],min=-12000, max=1500)
        goal_position(2,m[1],min=100, max=4000)

        
    #Función calback lectura posicion inidical
    def callback_read(self,data):
        
        for i in range(5):
            self.pos[i]=(data.dynamixel_state[i].present_position)

        if self.inicio==0:
            self.homie=self.pos
            self.inicio=1
            print(self.homie)
        



    def Main(self):

        #Se inicia nodo 
        rospy.init_node('Dynamixel', anonymous=True)

        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_read)

        rospy.Subscriber("gyro", Float64MultiArray, self.callback_IMU)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_vel)
        
        rospy.Subscriber("IK_node", Float64MultiArray, self.callback_multivuelta)
        
        rospy.spin() 
        



###################################################################3

def goal_position(ID , pos , min=0,max=1023 ):
    #rospy.wait_for_service('/dynamixel_command')

    #Se delimita el movimiento
    if pos<min:
        pos=min

    if pos>max:
        pos=max


    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Goal_Position',pos)
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
    imu = IMU()
    
    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.Main()

        
        