#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
import time 
from sensor_msgs.msg import Joy
pos=[0,]
def callback(data):
    global pos
    primer_joint=data.axes[1]
    segundo_joint=data.axes[4]
    pos=[primer_joint,segundo_joint]

array=[[0,7],[0,6],[1,6],[1,5],[1,4],[1,3],[1,2],[1,1],[1,0],[2,1],[2,2],[2,3],[2,4],[2,5]]
# Inicializa el nodo ROS
rospy.init_node('joy_publisher', anonymous=True)
#rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('Jaime_joy_node', Float64MultiArray, queue_size=10)


while not rospy.is_shutdown():
    rospy.Subscriber("joy", Joy, callback)
    #for i in array:

        # Crea un mensaje Float64MultiArray y publícalo
    data = Float64MultiArray()
        #data.data = [input("motor 1 :"),input("motor 2 :"), input("motor 3 :"), input("motor 4:"),input("motor 5 :")]
    data.data = pos
    pub.publish(data)
    rospy.sleep(0.1)