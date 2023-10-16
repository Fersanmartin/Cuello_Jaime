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
        self.x, self.y =data.data[0], data.data[1]
        #self.x = (self.x+90)*(800-260)/180 + 260
        #self.y = (self.y+90)*(800-260)/180 + 260
        
       
        # Llama a la función goal_position para mover el servo

        #tercer motor
        #goal_position(3, int(x), min=50, max=310)
        #Cuarto motor
        #goal_position(4, int(y), min=260, max=800)
        #Quinto motor
        #goal_position(5, int(z), min=260, max=800)

    
    def callback_vel(self,data):
        global last_received_time
        current_time = rospy.Time.now()
    
        if last_received_time is None or (current_time - last_received_time).to_sec() >= 0.05:
            Position=data.dynamixel_state[0].present_position
            last_received_time = current_time

            print(Position)
            if self.x<0:
                v=(-1*self.x)*(1023/90) + 1024
            else:
                v=(1*self.x)*(1023/90)
            goal_vel(5, v,Position,min=200, max=800)
            
        

        

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('Dynamixel', anonymous=True)
        
        rospy.Subscriber("gyro", Float64MultiArray, self.callback)
        
        rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, self.callback_vel)
        
        
        
        rospy.spin() #simply keeps python from exiting until this node is stopped
        


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


def goal_vel(ID , vel, Pos, min=0,max=1023):
    #rospy.wait_for_service('/dynamixel_command')

    if Pos<min and vel>1024:
        vel=0

    elif Pos>max and vel<1024:
        vel=0
    


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

        
        