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
old_long = 8.5455943  # sonrada n / mavros/setpoint_possition/globala sucscriber oluancak
start_time = 0
current_state = State()
msg = PositionTarget()

def state_cb(state):
    global current_state
    current_state = state

def waypoint_clear_client():
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
            return response.call().success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False



def create_waypoints():
	global start_time
	global old_long
	global old_lat

	rate = rospy.Rate(20)

	wl = []
	pub = rospy.Publisher('/mavros/dronemap/random_waypoint', String,queue_size=10)


	
	waypoint_clear_client()
		
	
	r1 = random.randint(-9,9)  # gecici cozum
	r2 = random.randint(-9,9)
	altitude = 10
	start_time2 = time.time()
	current_time2 = time.time()

	if r1 == 0:
		r1 = 1
	if r2 == 0:
		r2 = 1
	

	y_eksen = old_lat + (r1* 0.00001) - old_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	x_eksen = (old_long + (r2* 0.00001) - old_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	
	tan = math.atan2(x_eksen,y_eksen)
	
	yaw =  (tan / math.pi) * 180
	print("random degerler" , r1, r2)
	print("lat: " ,y_eksen," long : ",x_eksen)
	print("tanjant: ", tan)

	coordinates = str(old_lat + (r1* 0.00001)) + " " + str(old_long + (r2* 0.00001)) + " " + str(altitude)
	pub.publish(coordinates)

	print
	
	rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
	
	while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			#print "buradasin auto"
			 
			set_mode(0,'AUTO.MISSION')
			#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()		

	
	
	wp = Waypoint()
	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = True
 	wp.autocontinue = False
	wp.param1 = 0  # delay 
	#wp.param2 = 0
	wp.param3 = 1
	wp.param4 = yaw
	wp.x_lat = old_lat + r1* 0.00001
	wp.y_long = old_long + r2* 0.00001
	wp.z_alt = altitude
	wl.append(wp)


	#print(wl)
	start_time = time.time()

	old_lat = old_lat + r1* 0.00001 
	old_long = old_long + r2* 0.00001
	
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

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)
	rospy.wait_for_service('/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
	waypoint_clear_client()


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

			if (current_time - start_time2 == 3):

					try:
						armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

						armService(True)
					except rospy.ServiceException, e: # metin abi hold ona al dedi
	 					
	 					pass
	
	
			if (int(current_time) - int(start_time))>= 6:

				create_waypoints()
				#print(start_time)

			#pub.publish(konum)
		waypoint_clear_client()
		
	except rospy.ROSInterruptException:
		pass

