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

def alcalma():
    print "hahah"
    
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id= "world"
    msg.coordinate_frame = 8
    msg.type_mask=1991

    msg.velocity.x = 0.0
    msg.velocity.y = 0.0
    msg.velocity.z = -0.75
    msg.yaw = 0.0
    yaw_rate=1
	
	
    while not rospy.is_shutdown():

	    pub.publish(msg)


    
    
    
if  __name__ == '__main__':

    try:
	    rospy.init_node('eksen',anonymous=True)
	
	    
	    pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	    alcalma()
        

    except rospy.ROSInterruptException:
	    pass

