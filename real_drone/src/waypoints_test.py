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

alt = 0
lat = 0
longi = 0 

current_alt = 10
current_lat = 47.3977417
current_long = 8.5455943  # sonrada n / mavros/setpoint_possition/globala sucscriber oluancak
start_time = 0

current_state = State()
msg = PositionTarget()

def state_cb(state):
    global current_state
    current_state = state

def waypoint_clear_client():
        try:
            response = rospy.ServiceProxy('/uav1/mavros/mission/clear', WaypointClear)
            return response.call().success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False



def call_back_current_position(data):

	global current_alt,current_lat,current_long


	current_lat =  data.latitude
	current_long = data.longitude
def call_back_coordinates(data):

	global lat, longi, alt

	#print data.data
	
	
 	(t_lat, t_longi, t_alt) = data.data.split(",")
 	
 	(lat, longi, alt) = (float(t_lat), float(t_longi), float(t_alt))
	print("call back ", lat, longi,alt)
	create_waypoints()	

def create_waypoints():
	global start_time
	global current_long
	global current_lat

	rate = rospy.Rate(20)

	wl = []

	start_time3 = time.time()
	"""
	while True:

		current_time3 = time.time()	
		if int(current_time3) - int(start_time3) >= 2:
			print
			break	
		#print "zaman ", int(current_time3) - int(start_time)
	print "ciktim"
	"""
	waypoint_clear_client()

	start_time2 = time.time()
	current_time2 = time.time()


	y_eksen = lat - current_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	x_eksen = (longi - current_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	
	tan = math.atan2(x_eksen,y_eksen)
	
	yaw =  (tan / math.pi) * 180
	
	print("lat: " ,y_eksen," long : ",x_eksen)
	print("tanjant: ", tan)

	

	print
	
	
	while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			#print "buradasin auto"
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			 
			set_mode(0,'AUTO.MISSION')
			#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()		

	
	
	wp2 = Waypoint()
	wp2.frame = 3
	wp2.command = 115  #Navigate to waypoint.
	#wp2.is_current = True
 	#wp2.autocontinue = False
	wp2.param1 = 90  # delay 
	#wp.param2 = 0
	wp2.param3 = -1
	wp2.param4 = 1
	
	wl.append(wp2)
	"""
	wp = Waypoint()
	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = True
 	wp.autocontinue = False
	wp.param1 = 0  # delay 
	#wp.param2 = 0
	#wp.param3 = 1
	#wp.param4 = yaw
	wp.x_lat = lat 
	wp.y_long = longi
	wp.z_alt = alt
	wl.append(wp)

	"""	


	#print(wl)
	start_time = time.time()
	"""
	current_lat = lat 
	current_long = longi
	current_alt = alt 
	"""
	try:
	    service = rospy.ServiceProxy('/uav1/mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	  
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e
	


if __name__ == '__main__':

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('/uav1/mavros')
	rate = rospy.Rate(20)
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	state_sub = rospy.Subscriber('/uav1/mavros/state', State, state_cb)
	waypoint_clear_client()
	rospy.Subscriber('spesific_waypoint', String, call_back_coordinates)
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position)  

	try:
		
		
		
		
		
		while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			print "buradasin"
			 
			set_mode(0,'AUTO.MISSION')
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()
		
		start_time = time.time()
		start_time2 = time.time()

		
		while not rospy.is_shutdown():

			current_time = time.time()
			#print(int(current_time) - int(start_time))

			if (round(current_time,1) - round(start_time2,1) == 3):
					print "arm"
					try:
						armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

						armService(True)
					except rospy.ServiceException, e: # metin abi hold ona al dedi
	 					
	 					pass
	
	
			#if (int(current_time) - int(start_time))>= 6:

			#	create_waypoints()
				#print(start_time)

			#pub.publish(konum)
		waypoint_clear_client()
		
	except rospy.ROSInterruptException:
		pass
