#!/usr/bin/env python

import rospy 
#from threading import Thread, Timer
#import time
import threading
import time
import math
#from geometry_msgs.msg import Twist
import mavros
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import *
from std_msgs.msg import String


def _state_callback(topic):

	print "Mode: %s" % topic.mode
	

if  __name__ == '__main__':

	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(20)
	mavros.set_namespace('mavros')

	state_sub = rospy.Subscriber(mavros.get_topic('state'), mavros_msgs.msg.State, _state_callback)

	rospy.wait_for_service('/mavros/cmd/arming')
	 	
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException, e:
	 	pass
	
	time.sleep(3)

	


	msg = PositionTarget()

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

	msg.position.x = 0.0
	msg.position.y = 0.0
	msg.position.z = 2.0

	pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

	for i in range(0,50):
		pub.publish(msg)
		rate.sleep()


	set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)
	set_mode(0,'OFFBOARD')

	

	counter = 0

	try:

		while not rospy.is_shutdown():
			pub.publish(msg)

			counter= counter + 1

			if( counter>=50):

				msg.type_mask ^= PositionTarget.IGNORE_YAW
				msg.yaw = math.pi
				print counter

			if( counter>=100):

				set_mode(0,'AUTO.LAND')

			rate.sleep()

	except rospy.ROSInterruptException:
		pass


