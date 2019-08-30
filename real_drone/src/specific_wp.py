#!/usr/bin/env python

import rospy
import random
import time
import mavros
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *


old_lat = 47.3977417
old_long = 8.5455943
old_alt = 2  # sonrada n / mavros/setpoint_possition/globala sucscriber oluancak
start_time = 0
current_state = State()
msg = PositionTarget()
current_lat = 0
current_long = 0
current_alt = 10


waypoint_list1 = [[old_lat,old_long,old_alt],[old_lat+0.00004,old_long+0.00007,old_alt],[old_lat-0.00004,old_long-0.00007,old_alt],[old_lat-0.00004,old_long+0.00007,old_alt],[old_lat+0.00004,old_long-0.00007,old_alt],[old_lat,old_long,old_alt]] # [[lat1,long1,alt1],[lat2,long2,alt2]....]


def chooseWaypoits():
	rate = rospy.Rate(0.2)
	pub = rospy.Publisher('spesific_waypoint', String, queue_size=10)

	start_time = time.time()
	for waypoint in waypoint_list1:

		
		
		while True:
			current_time = time.time()
			if round(current_time,1) - round(start_time,1)>=7:
				print round(current_time,1) - round(start_time,1)

				print "breakledim" 
				start_time = time.time()
				break 

		coordinates = str(waypoint[0]) + "," + str(waypoint[1]) + "," + str(waypoint[2])
		pub.publish(coordinates)
				


	
       
if __name__ == '__main__':



	rospy.init_node('spesific_waypoints', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)


	try:
		
		
		
	
		chooseWaypoits()
		
	except rospy.ROSInterruptException:
		pass


