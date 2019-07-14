#!/usr/bin/env python

import rospy
import random
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *

def create_waypoint():

	x = 47.3977417
	y = 8.5455943
	a = x + 0.00004
	b = y + 0.00007 # buradaki degeri 400/700 ile carpip ayni skalaya cek
	# sebebi 400 5m ise 700 long da 5m 
	x_eksen = a - x 
	y_eksen = (b - y) * 400/700
	tan = math.atan(y_eksen/x_eksen)
	print("a :", a, "b :" , "bolme :" ,y_eksen/x_eksen )
	print("tanjant degeri : " ,tan )
	waypoint_clear_client()
	wl = []
	"""
	wp = Waypoint()

	wp.frame = 3
	wp.command = 22  # takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # takeoff altitude
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = 47.3971396
	wp.y_long = 8.5452028
	wp.z_alt = 4.0
	wl.append(wp)
    """
    	
	wp = Waypoint() 

	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # delay 
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = x
	wp.y_long = y
	wp.z_alt = 4.0
	wl.append(wp)

    
	wp = Waypoint()

	wp.frame = 3
	wp.command = 19  # Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # delay
	wp.param2 = 0
	wp.param3 = 1
	wp.param4 = 0
	wp.x_lat = a
	wp.y_long = b
	wp.z_alt = 4.0
	wl.append(wp)
	
	print(wl)

	try:
	    service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	    
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e


def waypoint_clear_client():
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            return response.call().success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False


if __name__ == '__main__':
	rospy.init_node('waypoint_node', anonymous=True)
	pub = rospy.Publisher('global',String,queue_size=10)
	konum = "suan buradasin" # to be used later
	pub.publish(konum)
	create_waypoint()
































"""
start_time = 0

def waypoint_clear_client():
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            return response.call().success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False


def create_waypoints():
	global start_time
	wl = []

	waypoint_clear_client()
		
	
	r1 = random.randint(-9,9)
	r2 = random.randint(-9,9)
	
	
	print("wooooeyyy")
	
	
	wp = Waypoint()
	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # delay 
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = 47.3977417 + r1* 0.00001
	wp.y_long = 8.5455943 + r2* 0.00001
	wp.z_alt = 4.0
	wl.append(wp)


	print(wl)
	start_time = time.time()
	
	try:
	    service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	  
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e
	
        '''
	    if service.call(wl).success: #burasi belki list istiyordur. birde oyle dene
	        print 'write mission success'
	    else:
	        print 'write mission error'
	    '''


if __name__ == '__main__':
	
	
	try:
		rospy.init_node('waypoint_node', anonymous=True)
		
		pub = rospy.Publisher('global',String,queue_size=10)
		konum = "suan buradasin"

		
		start_time = time.time()


		while not rospy.is_shutdown():

			current_time = time.time()
			print(int(current_time) - int(start_time))
			if (int(current_time) - int(start_time))>= 5:

				create_waypoints()
				print(start_time)

			pub.publish(konum)

	except rospy.ROSInterruptException:
		pass

"""


