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

#data = NavSatFix()

current_alt = 0
current_lat = 0
current_long = 0
voltage_rate = 0
roll_degrees = 0
pitch_degress = 0
yaw_degrees = 0
x = 0

def call_back_current_position(data):

	global current_alt,current_lat,current_long
	

	current_lat =  data.latitude
	current_long = data.longitude
	current_alt = data.altitude  # elde edilen yukseklikte ikinti var 500 lu donuyo
	
	#	print current_alt,current_lat,current_long
	

def call_back_gps_time(data):
	
	data.header.stamp.secs


def call_back_battery_state(data):

	global voltage_rate
	voltage_rate = data.voltage * 100 / 16
	print voltage_rate


def get_rotation(msg):
    global yaw_degrees, roll_degrees, pitch_degress,x
    #    print msg.pose.pose.orientation
    orientation_q = msg.pose.pose.orientation
    orientation_twist = msg.twist.twist.linear
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (x,y,z) = [orientation_twist.x, orientation_twist.y, orientation_twist.z]
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
    
    print (roll_degrees, pitch_degress, yaw_degrees,x) 
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


	rospy.spin()
	