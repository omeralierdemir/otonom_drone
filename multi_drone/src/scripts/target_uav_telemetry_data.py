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
pitch_degrees = 0
yaw_degrees = 0

sistem_h = 0
sistem_m = 0
sistem_s = 0
sistem_ns = 0

kilitlenme_baslangic_h = 0
kilitlenme_baslangic_m = 0
kilitlenme_baslangic_s = 0
kilitlenme_baslangic_ns = 0


kilitlenme_bitis_h = 0
kilitlenme_bitis_m = 0
kilitlenme_bitis_s = 0
kilitlenme_bitis_ns = 0


kilitlenme_zamani_temp = 1

kilitlenme_paketini_yaz = 0 

x_target_axis = 0
y_target_axis = 0
target_weight = 0
target_height = 0
kilitlenme_durumu = 0




iha_otonom = 1

x_speed = 0
z_axis = 0 # kilitlenme 5 sn az olunca null gonder

uav_state = 0


if __name__ == '__main__':

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(10)
	


	#ucus_modu_file = open("/home/efl4tun/Desktop/ucus_modu.txt",'r') # nvidia kullanicisi olarak degistirmek zorundasin
	hedef_telemetry_file = open("/home/efl4tun/Desktop/iha_telemetry.txt",'r')
	#hiz_verisi_file = open("/home/efl4tun/Desktop/hiz_verisi.txt",'r')
	time.sleep(4.0) # herveri ilk degerlerini almasi icin beklendi
	liste = []
	try:		
		
		
		while not rospy.is_shutdown():

			#------------------------ hedef telemetry
			hedef_telemetry_file = open("/home/efl4tun/Desktop/iha_telemetry.txt",'r')
			hedef_telemetry_data = hedef_telemetry_file.readline()
			if hedef_telemetry_data  != "":
				print "parse yap try catch icinde"
				hedef_telemetry_data = hedef_telemetry_data.replace("\n", "")
				try:
					(enlem, boylam, irtifa) =  hedef_telemetry_data.split(",")
				except Exception as e:
					(enlem, boylam, irtifa) =  ("null","null","null")
				
				
			
			hedef_telemetry_file.close()
			rate.sleep()

		
			# ----------------------- ucus modu 




			ucus_modu_file = open("/home/efl4tun/Desktop/ucus_modu.txt",'r')
			ucus_modu_data = ucus_modu_file.readline()
			if ucus_modu_data != "":
				print "parse yap try catch icinde"
				ucus_modu_data  = ucus_modu_data.replace("\n", "")
				try:
					(ilk_ucus, savasa_basla, hedef_takip, saga_git, sola_git, ileri_git, geri_git, eve_don) =  ucus_modu_data.split(",")
				except Exception as e:
					(ilk_ucus, savasa_basla, hedef_takip, saga_git, sola_git, ileri_git, geri_git, eve_don) = ("null","null","null","null","null","null","null","null")
				
				
			
			ucus_modu_file.close()

			# ----------------------------------------------------------  hiz


			hiz_verisi_file = open("/home/efl4tun/Desktop/hiz_verisi.txt",'r')
			hiz_verisi_data = hiz_verisi_file.readline()
			if hiz_verisi_data != "":
				print "parse yap try catch icinde"
				hiz_verisi_data  = hiz_verisi_data.replace("\n", "")
				try:
					x_speed =  hedef_telemetry_data # splite gerek yok
					if x_speed >= 7:

						x_speed = 5

					if x_speed < 1:

						x_speed = 1
				except Exception as e:
					
					x_speed = 2
				
				
			
			ucus_modu_file.close()




			rate.sleep()
	except rospy.ROSInterruptException:
		pass



	