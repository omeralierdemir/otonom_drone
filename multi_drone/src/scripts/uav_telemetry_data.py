#!/usr/bin/env python

import rospy
import random
import time
import mavros
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import datetime
#data = NavSatFix()

current_alt = 0
current_lat = 0
current_long = 0

voltage_rate = 0

roll_degrees = 0
pitch_degress = 0
yaw_degrees = 0

sistem_h = 0
sistem_m = 0
sistem_s = 0
sistem_ns = 0

kilitlenme_h = 0
kilitlenme_m = 0
kilitlenme_s = 0
kilitlenme_ns = 0

kilitlenme_zamani_temp = 1

x_speed = 0

def call_back_yolo(data):

	global sistem_h, sistem_m, sistem_s,sistem_ns
	global kilitlenme_h, kilitlenme_m, kilitlenme_s,kilitlenme_ns

	(x_axis, y_axis, weight, height, kilitlenme_durumu) = data.data.split()# null durumlarda var # kilitlenme durumu 0 olunca
																					# kilitlenme bitisi 1 olunca kilitlenme baslangici gonder


	if kilitlenme_durumu == 1 and kilitlenme_zamani_temp == 1:

		(kilitlenme_h, kilitlenme_m, kilitlenme_s,kilitlenme_ns) = (sistem_h, sistem_m, sistem_s,sistem_ns) 
		kilitlenme_zamani_temp = 0


	if kilitlenme_durumu == 0 and kilitlenme_zamani_temp == 0:

		(kilitlenme_h, kilitlenme_m, kilitlenme_s,kilitlenme_ns) = (sistem_h, sistem_m, sistem_s,sistem_ns) 
		kilitlenme_zamani_temp = 1

	# bu kod calisir diye dusunuyorum # kilitlenme zamaninin 3 sn lik toleransı yoloda verilecek kontrol et tekrar




def call_back_current_position(data):

	global current_alt,current_lat,current_long
	

	current_lat =  data.latitude
	current_long = data.longitude
	current_alt = data.altitude  # elde edilen yukseklikte ikinti var 500 lu donuyo
	
	#	print current_alt,current_lat,current_long
	

def call_back_gps_time(data):

	global sistem_h, sistem_m, sistem_s,sistem_ns
	
	a = data.header.stamp.secs
	zaman = str(datetime.datetime.now().time())

	(sistem_h, sistem_m, sistem_s) = zaman.split(':')

	(sistem_s,sistem_ns) = sistem_s.split('.')

	(sistem_h, sistem_m, sistem_s, sistem_ns) = (float(sistem_h), float(sistem_m), float(sistem_s), float(sistem_ns))  # float(sistem_ns) bunun int olması lazım olabilir

	print (sistem_h, sistem_m, sistem_s, sistem_ns)




def call_back_battery_state(data):

	global voltage_rate
	voltage_rate = data.voltage * 100 / 16
	#	print voltage_rate


def get_rotation(msg):
    global yaw_degrees, roll_degrees, pitch_degress,x_speed
    #    print msg.pose.pose.orientation
    orientation_q = msg.pose.pose.orientation
    orientation_twist = msg.twist.twist.linear
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (x_speed,y_speed,z_speed) = [orientation_twist.x, orientation_twist.y, orientation_twist.z]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw_degrees = yaw * 180.0 / math.pi
    roll_degrees = roll * 180.0 / math.pi
    pitch_degress = pitch * 180.0 / math.pi
    if (yaw_degrees < 0):
        yaw_degrees += 360.0

    if (roll_degrees < 0):
        roll_degrees += 360.0

    if (pitch_degress < 0):
        pitch_degress += 360.0
    
    #print (roll_degrees, pitch_degress, yaw_degrees,x) 
    # print "yaw_degrees HAS " , yaw_degrees

if __name__ == '__main__':

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	
	rospy.Subscriber('/uav1/mavros/global_position/raw/gps_vel', TwistStamped, call_back_gps_time) 
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position) 
	rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, get_rotation)
	rospy.Subscriber('/uav1/mavros/battery', BatteryState, call_back_battery_state) 
	rospy.Subscriber('/no_name', String, call_back_yolo) 


	rospy.spin()
	