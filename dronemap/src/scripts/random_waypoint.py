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
	pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=10)

	x = 47.3977417
	y = 8.5455943
	a = x + 0.00004
	b = y + 0.00007 # buradaki degeri 400/700 ile carpip ayni skalaya cek
	# sebebi 400 5m ise 700 long da 5m 
	
	waypoint_clear_client()
		
	
	r1 = random.randint(-9,9)  # gecici cozum
	r2 = random.randint(-9,9)
	start_time2 = time.time()
	current_time2 = time.time()

	if r1 == 0:
		r1 = 1
	if r2 == 0:
		r2 = 1
	

	x_eksen = old_lat + (r1* 0.00001) - old_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	y_eksen = (old_long + (r2* 0.00001) - old_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	tan = math.atan(y_eksen/x_eksen) # radyan cinsinden deger donderir.
	#print("a :", x_eksen, "b :" y_eksen, "bolme :" ,y_eksen/x_eksen )
	#print("tanjant: ", tan)

	msg.yaw = tan
	#pub.publish(msg)
	print("random degerler" , r1, r2)
	print("eksenler : " ,x_eksen,y_eksen)
	print("tanjant: ", tan)
	print("wooooeyyy")
	print
	rospy.loginfo("OFFBOARD mod istegi gonderildi")

	while current_state.mode != "OFFBOARD":
			
			waypoint_clear_client()
			pub.publish(msg)
			#print "buradasin OFFBOARD"
			 
			set_mode(0,'OFFBOARD')
			#rospy.loginfo("OFFBOARD mod istegi gonderildi")
			
			rate.sleep()
	#print ("time : ", current_time2, start_time2)
	while int(current_time2) - int(start_time2)<= 2:
		#print("don haci",int(start_time2) - int(current_time2) )
		current_time2 = time.time() 

	rospy.loginfo("AUTO.MISSION mod istegi gonderildi")

	while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			#print "buradasin auto"
			 
			set_mode(0,'AUTO.MISSION')
			#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()		

	
	
	wp = Waypoint()
	wp.frame = 3
	wp.command = 19  #Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # delay 
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = old_lat + r1* 0.00001
	wp.y_long = old_long + r2* 0.00001
	wp.z_alt = 4.0
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
		armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

		armService(True)
	except rospy.ServiceException, e: # metin abi hold ona al dedi
	 	pass
	
	
	try:
		
		
		pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "world"
		msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
		msg.type_mask = 3015
		msg.velocity.x = 0.0
		msg.velocity.y = 0.0
		msg.velocity.z = 0.0
		msg.yaw = 0.0
		konum = "suan buradasin"
		print konum
		while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			print "buradasin"
			 
			set_mode(0,'AUTO.MISSION')
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()
		
		start_time = time.time()


		while not rospy.is_shutdown():

			current_time = time.time()
			#print(int(current_time) - int(start_time))
			if (int(current_time) - int(start_time))>= 4:

				create_waypoints()
				#print(start_time)

			#pub.publish(konum)
		waypoint_clear_client()
		print "omer servis cagrildi"

	except rospy.ROSInterruptException:
		pass

