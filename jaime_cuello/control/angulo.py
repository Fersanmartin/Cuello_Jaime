#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from dynamixel_workbench_msgs.srv import *
from dynamixel_workbench_msgs.msg import *
import time 

# Inicializa el nodo ROS
rospy.init_node('angulo_publisher', anonymous=True)
#rospy.Subscriber("/dynamixel_workbench/dynamixel_state", DynamixelStateList, callback_read)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('angulo_node', Float64MultiArray, queue_size=10)


while not rospy.is_shutdown():


    data = Float64MultiArray()

    data.data = [float(input("angulo: "))]
    pub.publish(data)
    rospy.sleep(0.1)