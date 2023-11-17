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
class Master():
    def __init__(self):
        self.IMU=list()
        
        self.x=0
        self.y=0

        self.home=[0,0,0,0,0]

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
        offset1= 9000-self.home[0]
        offset2= -1000-self.home[1]

        print(self.homie)
        conf1= [-3000, 700, 1000, 300, 0]
        conf2= [3000, -700, 5000, -700, 0]
        conf3= [0, 700, 1000, 300, 0]
        conf4= [-3000, 700, 1000, 300, 0]
        conf5= [-3000, 700, 1000, 300, 0]
        #m=matriz[conf1, conf2, conf3, conf4, conf5]
        
        goal_position(1,data.data[0] - offset1)
        goal_position(2,data.data[1] - offset2)
        # goal_position(3,data.data[2])
        # goal_position(4,data.data[3])
        # goal_position(5,data.data[4])
        print("move motor con id 1 a : ")
        print(data.data[0]- offset1)
        print("move motor con id 2 a : ")
        print(data.data[1] - offset2)
        print("move motor con id 3 a : ")
        print(data.data[2])
        print("move motor con id 4 a : ")
        print(data.data[3])
        print("move motor con id 5 a : ")
        # print(data.data[4])
        # Conf = data.data[0]
        # pos= look[Conf]
        # for i in range(4):
        #     goal_position(i+1, pos[i])
        # goal_position(1,pos[0])
        # goal_position(2,pos[1])
        # goal_position(3,pos[2])
        # goal_position(4,pos[3])
        # goal_position(5,pos[4])
        
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
        print(9000-self.home[0])
        print(-1000-self.home[1])
        


    def Main(self):

        #Se inicia nodo 
        rospy.init_node('Dynamixel', anonymous=True)

        rospy.Subscriber("homie_node", Float64MultiArray, self.homie)
            

        #rospy.Subscriber("gyro", Float64MultiArray, self.callback_IMU)
        
        #rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_vel)
        
        rospy.Subscriber("IK_node", Float64MultiArray, self.callback_multivuelta)
        
        rospy.spin() 
        



###################################################################3

def goal_position(ID , pos ):
    #rospy.wait_for_service('/dynamixel_command')

    #Se delimita el movimiento
   # if pos<min:
   #     pos=min

    #if pos>max:
    #    pos=max


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
    imu = Master()

    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.Main()

        
          