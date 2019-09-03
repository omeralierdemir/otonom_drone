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
from mavros_msgs.msg import State

msg = PositionTarget()
msg2 = PositionTarget()

lastErrorY = 0
lastErrorR = 0
yaw_target = 0
current_state=State()

count_yaw = 0
hedef_yoksa_aramak_icin_bekle = 0
hedef_yok_aramaya_basla = False
def call_back(data):
	#rospy.loginfo("cameradan veri geldi")
	global lastErrorY
	global lastErrorR
	global yaw_target
	global msg
	global count_yaw
	global hedef_yoksa_aramak_icin_bekle
	global hedef_yok_aramaya_basla
	XY = data.data
	XY = XY.split(" ")

	if XY[0] != "null": 
		hedef_yoksa_aramak_icin_bekle = 0
		hedef_yok_aramaya_basla = False
		#print "hedef takipte"
		x = float(XY[0])
		y = float(XY[1])
		r = float(XY[2])
		

		# -----------------------
		errorY = 213-y

		kp = 1.0/750.0
		kd = 1.0/1000.0

		turevY = errorY-lastErrorY
		yukselmeY =  errorY*kp + turevY*kd
		if yukselmeY > 0.8:
			yukselmeY = 0.8
		if yukselmeY < -0.8:
			yukselmeY = -0.8
		msg.velocity.z = yukselmeY
		
		lastErrorY = errorY

		# -----------------------

		errorR = 45 - r

		kpR = 1.0/220.0
		kdR = 1.0/300.0

		turevR = errorR-lastErrorR
		yaklasma =  errorR*kpR + turevR*kdR
		if yaklasma > 0.7:
			yaklasma = 0.7
		if yaklasma < -0.7:
			yaklasma = -0.7
		msg.velocity.y = yaklasma
		
		lastErrorR = errorR

		# -----------------------


		errorX = 213-x # eskiden 300 duler
		
		if errorX < 20 and errorX > -20:
			errorX =0

		yaw_target = yaw_degrees + (errorX*35)/213
		if yaw_target >= 360:
			yaw_target = yaw_target - 360

		if yaw_target < 0:
			yaw_target = yaw_target + 360	
		#print "r = ", r, " errorX = ", errorX , " yaw_degrees " , yaw_degrees, " yaw_target", yaw_target
		#print " yaw_target " ,yaw_degrees + (errorX*35)/213, float(yaw_target*math.pi/180.0), " yukselme ", yukselmeY, " yaklasma  ", yaklasma		
		#print " yaw_degrees " , float(yaw_target*math.pi/180.0), " yukselme ", yukselmeY, " yaklasma  ", yaklasma
		#print "r = ", r, " errorX = ", errorX , " yaw_degrees " , float(yaw_target*math.pi/180.0), " yukselme ", yukselmeY, " yaklasma  ", yaklasma 
		file.write(str(errorX) + "," + str(errorY) + "," + str(errorR) + "," +str(yukselmeY) + "," + str(yaw_target)  + "," + str(yaklasma) + "," + str(x) + ","+ str(y) + "\n")
		#file.write(str(yukselmeY) + "," + str(yaw_target)  + "," + str(yaklasma)+ "\n")		
		msg.yaw = float(yaw_target*math.pi/180.0)
		print(str(x) + " *** "+ str(y) + " *** " + str(yaw_target*math.pi/180.0) + " --- "+ str(yaw_degrees)+ " *** "+ str(msg.velocity.y) + " *** "+ str(msg.velocity.z) + "\n")
	else:	# bu kisimda hedef yoksa


		print "hedef yok" 
		
		hedef_yoksa_aramak_icin_bekle = hedef_yoksa_aramak_icin_bekle +1
		if hedef_yoksa_aramak_icin_bekle == 2*900: #2*30 da  5 saniye
			hedef_yok_aramaya_basla = True
		if hedef_yok_aramaya_basla == True:
			count_yaw = count_yaw +1
			print "hedef araniyor"
			msg.velocity.y = 0
			msg.velocity.z = 0
		
			msg.yaw = float(yaw_target*math.pi/180.0)
			if count_yaw == 2*2: # 200 ms

				yaw_target = yaw_target + 10
				rate.sleep()
				count_yaw = 0
			if yaw_target >= 360:
				yaw_target = yaw_target - 360
	
		



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
	

def state_cb(state):
    global current_state
    current_state = state

if  __name__ == '__main__':
	file = open("work.txt","w")

	global current_state
	global msg
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
	state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)

	rospy.loginfo("arming succesfull")

	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ| PositionTarget.IGNORE_YAW_RATE

	
	msg2.header.stamp = rospy.Time.now()
	msg2.header.frame_id = "world"
	msg2.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg2.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW| PositionTarget.IGNORE_YAW_RATE

	msg2.position.x = 0.0
	msg2.position.y = 0.0
	msg2.position.z = 2.0
	#msg.yaw = 0.0

	counter = 0

	rospy.loginfo("ilk veri gonderilcek")


	while current_state.mode != "OFFBOARD":
			
		pub.publish(msg2)
		 
		set_mode(0,'OFFBOARD')
		rospy.loginfo("OFFBOARD mod istegi gonderildi")
		
		rate.sleep()
	rospy.loginfo("2 saniye havada asisli kal")
	ilk_yukselme = 0
	while(1):
		pub.publish(msg2)
		ilk_yukselme = ilk_yukselme+1
		rate.sleep()
		if ilk_yukselme == 2*100: # 10 saniye bekle
			break

	#pub.publish(msg2)
	msg.velocity.x = 0.0
	#pub.publish(msg2)
	msg.velocity.y = 0.0
	#pub.publish(msg2)	
	msg.velocity.z = 0.0
	msg.yaw = 0.0
	#pub.publish(msg2)	
	rospy.loginfo("velocity asamasina gecildi")
	try:
		#pub.publish(msg2)		
		sub = rospy.Subscriber ('/uav1/mavros/local_position/odom', Odometry, get_rotation)	
		#pub.publish(msg2)	
		rospy.Subscriber('konum', String, call_back) # kamera goruntusunden x,y kordinatlari
		#pub.publish(msg2)		
		rospy.loginfo("Subscriber islemleri tamam")
		
		#pub.publish(msg2)
		while not rospy.is_shutdown():
			
			#print msg.velocity.z, msg.yaw
			pub.publish(msg)

			rate.sleep()


	except rospy.ROSInterruptException:
		pass

