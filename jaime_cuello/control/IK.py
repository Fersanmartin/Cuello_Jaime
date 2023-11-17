#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
import time 


# Inicializa el nodo ROS
rospy.init_node('IK_publisher', anonymous=True)
#rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('IK_node', Float64MultiArray, queue_size=10)
 
while not rospy.is_shutdown():
        
        
    # Crea un mensaje Float64MultiArray y publícalo
    

    data = Float64MultiArray()
    #data.data = [input("motor 1 :"),input("motor 2 :"), input("motor 3 :"), input("motor 4:"),input("motor 5 :")]
    data.data = [input("config:")]
    pub.publish(data)
        
        
