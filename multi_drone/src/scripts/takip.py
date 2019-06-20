#!/usr/bin/env python

import rospy 
import math
import time
#from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

msg = PositionTarget()




def call_back(data):

    lastErrorX = 0
    lastErrorY = 0
    lastErrorR = 0
    
    XY = data.data
    XY = XY.split(" ")

    x= float(XY[0])
    y= float(XY[1])
    r = float(XY[2])
    
    
    

    errorR = 55 - r

    kpR = 1.0/80.0
    kdR = 1.0/100.0
    print("omer")
    turevR = errorR-lastErrorR
    yaklasma =  errorR*kpR + turevR*kdR
    msg.velocity.y = yaklasma

    lastErrorR = errorR
    # -----------------------

    errorY = 165-y

    kp = 1.0/100.0
    kd = 1.0/150.0

    turevY = errorY - lastErrorY
    yukselmeY =  errorY*kp + turevY*kd
    msg.velocity.z = yukselmeY

    lastErrorY = errorY
    
    #---------------------------
    errorX = x - 300 # sebebi artis ters yonlu

    kp = 0.00523
    kd = 0.009
    print("omer")
    turevX = errorX-lastErrorX

    P_cont = errorX*kp 
    D_cont = turevX*kd

    yawRate = -P_cont
   
	
    if(yawRate <= 0.1 and yawRate > 0):

        yawRate = 0
        
    elif(yawRate > -0.1 and yawRate < 0):
        
        yawRate = 0
    
    
    msg.yaw_rate = yawRate
	
	
    print(yawRate)

	
	
    lastErrorX = errorX

if  __name__ == '__main__':

	try:
		rospy.init_node('eksen',anonymous=True)
		
		

		
		pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	
		rospy.Subscriber('konum', String, call_back) 

   
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id= "world"
		msg.coordinate_frame = 8
		msg.type_mask= 1479

		msg.velocity.x = 0.0
		msg.velocity.y = 0.0
		msg.velocity.z = 0.0
		msg.yaw_rate = 0.0


		while not rospy.is_shutdown():

			pub.publish(msg)

	except rospy.ROSInterruptException:
		pass

