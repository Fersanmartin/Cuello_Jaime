#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
import time 



# Inicializa el nodo ROS
rospy.init_node('IK_publisher', anonymous=True)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('IK_node', Float64MultiArray, queue_size=10)


while not rospy.is_shutdown():
        
        
    # Crea un mensaje Float64MultiArray y publícalo
    data = Float64MultiArray()
    data.data = [input("x :"),input("y :")]
    pub.publish(data)
        
        
