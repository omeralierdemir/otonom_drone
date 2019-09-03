#!/usr/bin/env python

import rospy
import random
import time
import mavros
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import *
from mavros_msgs.srv import *



current_state = State()
msg = PositionTarget()



current_alt = 0
current_lat = 0
current_long = 0

target_alt = 0
target_lat = 0
target_long = 0 

positionX = 0
positionY = 0
positionZ = 0

lastErrorY = 0



def state_cb(state):
    global current_state
    current_state = state





def get_rotation(msg):
    global positionX, positionY, positionZ
    #    print msg.pose.pose.orientation
    position_q = msg.pose.pose.position
    
    (positionX, positionY, positionZ) = (position_q.x, position_q.y, position_q.z)
   
    
    
  
    # print "yaw_degrees HAS " , yaw_degrees

def set_single_position(desired_x=0, desired_y=0, desired_z=10):

	global msg

	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	rate = rospy.Rate(20)


	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW| PositionTarget.IGNORE_YAW_RATE

	msg.position.x = desired_x
	msg.position.y = desired_y
	msg.position.z = desired_z
	#msg.yaw = 0.0

	pub.publish(msg)

	while current_state.mode != "OFFBOARD":
			
			pub.publish(msg)
			rate.sleep()
			
			rospy.loginfo("OFFBOARD mod istegi gonderildi")
			 
			set_mode(0,'OFFBOARD')
		

	




def call_back_current_position(data):

	global current_alt,current_lat,current_long
	global home_longi, home_lat, home_alt
	global ilk_ucus_temp
	global gps_yenilenme_zamani
	global yki_sola_git


	"""
	if ilk_ucus_temp:
		#buraya seriportan istenen veri geldiyse kosulu konacak
		home_lat =  data.latitude
		home_longi = data.longitude
		home_alt = 10.0 #baslangic yuksekligi 10m set edildi
		
		print ("home paremetreleri set edildi", home_lat,home_longi,home_alt)
	"""  # buna gerek kalmayabilir
	

	current_lat =  data.latitude
	current_long = data.longitude

	gps_yenilenme_zamani = time.time()
		
	#print current_lat,current_long



def call_back_coordinates(data):

	global target_lat, target_long, target_alt
	global baglanti_yenilenme_zamani


 	(t_lat, t_longi, t_alt) = data.data.split(",")
 	
 	(target_lat, target_long, target_alt) = (float(t_lat), float(t_longi), float(t_alt))

 	baglanti_yenilenme_zamani = time.time()

 	
 	
	#print(target_lat, target_long, target_alt) 	

	


def flight_controller(follow_speed= 3.0):

	global current_lat, current_long, current_alt
	global ilk_ucus_temp
	global target_lat, target_long, target_alt
	global positionZ 
	global msg
	global lastErrorY


	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	rate = rospy.Rate(20)


	errorY = target_alt - positionZ

	kp = 1.0/80.0#0.02 #1.0/100.0 -----> 1/ 80.0  max hiz 3~2 m/s
	kd = 1.0/75.0#0.012#1.0/150.0 -----> 1.0/75.0

	turevY = errorY-lastErrorY
	yukselmeY =  errorY*kp + turevY*kd
	msg.velocity.z = yukselmeY
	
	lastErrorY = errorY

	print target_alt, positionZ

	y_eksen = target_lat - current_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	x_eksen = (target_long - current_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	
	yaw_radyan = math.atan2(y_eksen,x_eksen)
	yaw_radyan2 = math.atan(x_eksen/y_eksen)
	target_yaw =  (yaw_radyan / math.pi) * 180
	
	print("lat: " ,y_eksen," long : ",x_eksen)
	print("tanjant: ", yaw_radyan,yaw_radyan2)
	print("lat, long create daki" ,target_lat, target_long, target_yaw )
	

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE

	msg.velocity.x = 0.0
	msg.velocity.y = 5.0#follow_speed # suankil 4m/s sonradan arguman olarak yer istasyonundan alinacak (float)
	msg.velocity.z = 0.0#yukselmeY # suanlik 0 sonradan konrolcu eklenecek
	msg.yaw = yaw_radyan
	pid_wait = time.time()
	rate.sleep()

	
	
 
	while current_state.mode != "OFFBOARD":
		
		pub.publish(msg)
		rate.sleep()
	
		rospy.loginfo("OFFBOARD mod istegi gonderildiiiiiiiiiiiiiiiii")
		 
		set_mode(0,'OFFBOARD')
		

    



if __name__ == '__main__':

	rospy.init_node('gps_velocity', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)
	rate2 = rospy.Rate(10)
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	state_sub = rospy.Subscriber('/uav1/mavros/state', State, state_cb)
	
	#rospy.Subscriber('test_waypoint', String, call_back_coordinates)
	rospy.Subscriber('waypoint_random', String, call_back_coordinates)
	#rospy.Subscriber('yer_istasyonu', String, call_back_yki)
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position)  
	#rospy.Subscriber('get_pid', String, call_back_pid)
	rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, get_rotation)


	try:
		armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException, e:
	 
		print "arm hatasi"


	
	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)

	set_single_position()

	rate.sleep()

	try:
		
		
		
		
		
		while not rospy.is_shutdown():
			#pub.publish(msg)
			#flight_controller()
			rate.sleep()
			flight_controller()		
			

		
		
	except rospy.ROSInterruptException:
		pass


	