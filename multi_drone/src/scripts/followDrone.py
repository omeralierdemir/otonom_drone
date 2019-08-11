#!/usr/bin/env python

import rospy 
import mavros
#from threading import Thread, Timer
#import time
import threading
import time
import math
#from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

msg = PositionTarget()

lastErrorY = 0
lastErrorR = 0
logPID = open("log_pid.txt", "w")
logPID2 = open("log_pid2.txt", "w")
def call_back(data):
	global lastErrorY
	global lastErrorR
	global logPID



	XY = data.data
	if XY != "null":
		XY = XY.split(" ")

		x = float(XY[0])
		y = float(XY[1])
		r = float(XY[2])

		# -----------------------
		errorY = 300-y

		kp = 1.0/80.0#0.02 #1.0/100.0 -----> 1/ 80.0  max hiz 3~2 m/s
		kd = 1.0/75.0#0.012#1.0/150.0 -----> 1.0/75.0

		turevY = errorY-lastErrorY
		yukselmeY =  errorY*kp + turevY*kd
		msg.velocity.z = yukselmeY
		
		lastErrorY = errorY

		# -----------------------

		errorR = 55 - r

		kpR = 1.0/20.0
		kdR = 1.0/50.0

		turevR = errorR-lastErrorR
		yaklasma =  errorR*kpR + turevR*kdR
		msg.velocity.y = yaklasma
		
		lastErrorR = errorR

		# -----------------------


		errorX = 300-x
		
		if errorX < 30 and errorX > -30:
			errorX =0


		yaw_target = yaw_degrees + (errorX*35)/300

		if yaw_target >= 360:
			yaw_target = yaw_target - 360

		if yaw_target < 0:
			yaw_target = yaw_target + 360	
		#print "r = ", r, " errorX = ", errorX , " yaw_degrees " , yaw_degrees, " yaw_target", yaw_target
		print "y = ", y, " errorY = ", errorY , " kP*errorY :" , errorY*kp, " turevY*kd :", turevY*kd ,"yukselme hizi: ", yukselmeY
		#print "r = ", r, " errorR = ", errorR , " errorR*kpR :" , errorR*kpR, " turevR*kdR :", turevR*kdR ,"yaklasma hizi: ", yaklasma , "turevR : ", turevR
			
		temp = " " + str(r) + " " +  str(errorR) +  " " +  str(errorR*kpR) + " " + str(turevR*kdR) + " " +  str(yaklasma) + " " + str(turevR)+ "\n" 
		temp2 = " " + str(y) + " " +  str(errorY) +  " " +  str(errorY*kp) + " " + str(turevY*kd) + " " +  str(yukselmeY) + " " + str(x)+ " " + str(turevY)+  "\n"
		msg.yaw = float(yaw_target*math.pi/180.0)
		logPID.write(temp)
		logPID2.write(temp2)
yaw_degrees = roll = pitch = yaw = 0.0

def get_rotation (msg):
    global yaw_degrees,roll, pitch, yaw
#    print msg.pose.pose.orientation
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw_degrees = yaw * 180.0 / math.pi
    if( yaw_degrees < 0 ):
        yaw_degrees += 360.0
    #print "yaw_degrees HAS " , yaw_degrees
	



if  __name__ == '__main__':

	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(20)
	mavros.set_namespace('/uav1/mavros')
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	 	
	try:
		armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException, e:
	 	pass
	
	set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)
	

	print "harun"

	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW| PositionTarget.IGNORE_YAW_RATE

	msg.position.x = 0.0
	msg.position.y = 0.0
	msg.position.z = 10.0
	#msg.yaw = 0.0

	counter = 0


	while 1:
			
		pub.publish(msg)

		counter= counter + 1

		if (counter >5 and counter<10):
			set_mode(0,'OFFBOARD')
		if( counter>100 ):
			break

			print counter

		rate.sleep()


	msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE

	msg.velocity.x = 0.0
	msg.velocity.y = 0.0
	#msg.velocity.z = 0.0
	msg.yaw = 0.0

	try:
		
		sub = rospy.Subscriber ('/uav1/mavros/local_position/odom', Odometry, get_rotation)	
	
		rospy.Subscriber('konum', String, call_back) # kamera goruntusunden x,y kordinatlari
		
		
		print "mdmmd"

		
		print "girdimv"
		
		

		while not rospy.is_shutdown():
			
			pub.publish(msg)

			rate.sleep()

			
		
		
		logPID.close()
		logPID2.close()

	except rospy.ROSInterruptException:
		pass


