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
	
	pub = rospy.Publisher('target_telemetry_data', String, queue_size=10)
	pub2 = rospy.Publisher('ucus_gorevi', String, queue_size=10)
	pub3 = rospy.Publisher('hiz_data', String, queue_size=10)

	#ucus_modu_file = open("/home/efl4tun/Deskt5#9#4#6#4#4### ['5op/ucus_modu.txt",'r') # nvidia kullanicisi olarak degistirmek zorundasin
	#hedef_telemetry_file = open("/home/efl4tun/Desktop/iha_telemetry.txt",'r')
	#hiz_verisi_file = open("/home/efl4tun/Desktop/hiz_verisi.txt",'r')
	time.sleep(4.0) # herveri ilk degerlerini almasi icin beklendi
	liste = []

	try:		
		
		
		while not rospy.is_shutdown():

			#------------------------ hedef telemetry
			hedef_telemetry_file = open("/home/efl4tun/Desktop/11_09/io/target_iha_telemetry.txt",'r')
			hedef_telemetry_data = hedef_telemetry_file.readline()
			if hedef_telemetry_data  != "":
				
				hedef_telemetry_data = hedef_telemetry_data.replace("\n", "")
				try:
					(enlem, boylam, irtifa) =  hedef_telemetry_data.split("#")
					#(enlem_sahte, boylam_sahte, sahte_irtifa) = (float(enlem), float(boylam), float(irtifa))
				except Exception as e:
					(enlem, boylam, irtifa) =  ("null","null","null")
					print type(enlem)
					print e
				
				print "enlem boylam irtifa", (enlem, boylam, irtifa) 
				hedef_telemetry_temp = str(enlem) + "," + str(boylam) + "," + str(irtifa)
			#	burada publish et zaten surekkli yenilecek publishleyecek surekli veri akisi olacak
				pub.publish(hedef_telemetry_temp)
			open("/home/efl4tun/Desktop/11_09/io/target_iha_telemetry.txt",'w').close()
			

		
			# ----------------------- ucus modu 




			ucus_gorevi_file = open("/home/efl4tun/Desktop/11_09/io/ucus_gorevi.txt",'r')
			ucus_gorevi_data = ucus_gorevi_file.readline()
			if ucus_gorevi_data != "":  
				
				

				
				print "ucus_gorevi"
				ucus_gorevi_data  = ucus_gorevi_data.replace("\n", "")
				print ucus_gorevi_data, ucus_gorevi_data.split("#")
				try:
					(ilk_ucus, savasa_basla, hedef_takip, saga_git, sola_git, ileri_git, pid_disable , eve_don) =  ucus_gorevi_data.split("#")
					(ilk_ucus_t, savasa_basla_t, hedef_takip_t, saga_git_t, sola_git_t, ileri_git_t, pid_disable_t , eve_don_t) = (int(ilk_ucus), int(savasa_basla), int(hedef_takip), int(saga_git), int(sola_git), int(ileri_git), int(pid_disable) , int(eve_don))
				except Exception as e:
					(ilk_ucus, savasa_basla, hedef_takip, saga_git, sola_git, ileri_git, pid_disable , eve_don) = ("null","null","null","null","null","null","null","null")
					print e
				# burada publish et zaten 1 kere publishleyecek surekli veri akisi yok zaten.
				temp_deg = str(ilk_ucus) + "," + str(savasa_basla) + "," + str(hedef_takip) + "," + str(saga_git) + "," + str(sola_git) + "," + str(ileri_git) + "," + str(pid_disable) + "," + str(eve_don)
				pub2.publish(temp_deg)
				
				open("/home/efl4tun/Desktop/11_09/io/ucus_gorevi.txt",'w').close()
			


			# ----------------------------------------------------------  hiz


			hiz_verisi_file = open("/home/efl4tun/Desktop/11_09/io/hiz.txt",'r')
			hiz_verisi_data = hiz_verisi_file.readline()
			if hiz_verisi_data != "":
				
				print "hiz veririsi " 
				hiz_verisi_data  = hiz_verisi_data.replace("\n", "")
				print "[" + hiz_verisi_data + "]"
				print(len(hiz_verisi_data) )
				if len(hiz_verisi_data) == 1:
					print("Len == 1")
					try:
						x_speed =  float(hiz_verisi_data) # splite gerek yok
						if x_speed >= 7.0:

							x_speed = 7.0

						if x_speed < 1.0:

							x_speed = 1.0
					except Exception as e:
						
						print e
						x_speed = 5.0

					pub3.publish(str(x_speed))

				open("/home/efl4tun/Desktop/11_09/io/hiz.txt",'w').close()
			
			
			ucus_gorevi_file.close()




			rate.sleep()
	except rospy.ROSInterruptException:
		pass



	