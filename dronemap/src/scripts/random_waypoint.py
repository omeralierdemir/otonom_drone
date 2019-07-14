#!/usr/bin/env python

import rospy
import random
import time
import mavros
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *


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

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)
	rospy.wait_for_service('/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)
	state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException, e:
	 	pass
	
	
	try:
		
		
		pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
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
			if (int(current_time) - int(start_time))>= 5:

				create_waypoints()
				print(start_time)

			#pub.publish(konum)

	except rospy.ROSInterruptException:
		pass

