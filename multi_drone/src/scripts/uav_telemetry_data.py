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
def call_back_ucus_durumu(data):

	global uav_state

	uav_state = data.data
	#print uav_state


def call_back_yolo(data):# sucscreber olmadin

	global sistem_h, sistem_m, sistem_s, sistem_ns
	global kilitlenme_bitis_h, kilitlenme_bitis_m, kilitlenme_bitis_s, kilitlenme_bitis_ns
	global kilitlenme_baslangic_h, kilitlenme_baslangic_m, kilitlenme_baslangic_s, kilitlenme_baslangic_ns
	global kilitlenme_paketini_yaz, kilitlenme_zamani_temp
	global x_target_axis, y_target_axis, target_weight, target_height, kilitlenme_durumu
	
	(x_target_axis, y_target_axis, target_weight, target_height, kilitlenme_durumu) = data.data.split(" ")# null durumlarda var # kilitlenme durumu 0 olunca
																					# kilitlenme bitisi 1 olunca kilitlenme baslangici gonder

	(x_target_axis, y_target_axis, target_weight, target_height, kilitlenme_durumu) = (int(x_target_axis), int(y_target_axis), int(target_weight), int(target_height), int(kilitlenme_durumu))							

	if kilitlenme_durumu == 1 and kilitlenme_zamani_temp == 1:

		(kilitlenme_baslangic_h, kilitlenme_baslangic_m, kilitlenme_baslangic_s, kilitlenme_baslangic_ns) = (sistem_h, sistem_m, sistem_s,sistem_ns) 
		kilitlenme_zamani_temp = 0
		kilitlenme_geri_sayim_baslangic = time.time()


	if kilitlenme_durumu == 0 and kilitlenme_zamani_temp == 0:

		(kilitlenme_bitis_h, kilitlenme_bitis_m, kilitlenme_bitis_s, kilitlenme_bitis_ns) = (sistem_h, sistem_m, sistem_s,sistem_ns) 
		kilitlenme_zamani_temp = 1
		kilitlenme_geri_sayim_bitis = time.time()

	# bu kod calisir diye dusunuyorum # kilitlenme zamaninin 3 sn lik toleransi yoloda verilecek kontrol et tekrar
	# int degerler olacak 
	# burada kilitlenme zamanini mi servera gondercem yoksa kilitlenme boyunca mi zamani gondercem?

	#(x_axis, y_axis, weight, height, kilitlenme_durumu) = (int(x_axis), int(y_axis), int(weight), int(height), int(kilitlenme_durumu))

	if kilitlenme_geri_sayim_bitis - kilitlenme_geri_sayim_baslangic >= 7 :
		
		 print "dosyaya yaz"
		 kilitlenme_paketini_yaz = 1

	else:

		print "dosyaya null yaz"
		kilitlenme_paketini_yaz = 0
	#print(type(x_axis), y_axis, weight, height, kilitlenme_durumu) 

def call_back_current_position(data):

	global current_alt,current_lat,current_long
	

	current_lat =  data.latitude
	current_long = data.longitude
	current_alt = data.altitude  # elde edilen yukseklikte ikinti var 500 lu donuyo
	
	#print current_alt,current_lat,current_long
	#print type(current_alt),current_lat,current_long
	

def call_back_gps_time(data):

	global sistem_h, sistem_m, sistem_s,sistem_ns
	
	a = data.header.stamp.secs
	zaman = str(datetime.datetime.now().time())

	(sistem_h, sistem_m, sistem_s) = zaman.split(':')
	sistem_s = round(float(sistem_s),3)
	sistem_s = str(sistem_s)
	(sistem_s,sistem_ns) = sistem_s.split('.')


	(sistem_h, sistem_m, sistem_s, sistem_ns) = (int(sistem_h), int(sistem_m), int(sistem_s), int(sistem_ns))  # float(sistem_ns) bunun int olmasi lazim olabilir

	#print (sistem_h, sistem_m, sistem_s, sistem_ns)




def call_back_battery_state(data):

	global voltage_rate
	voltage_rate = data.voltage * 100 / 16
	
	voltage_rate = int(round(voltage_rate))
	#print voltage_rate

def get_rotation(msg):
    global yaw_degrees, roll_degrees, pitch_degrees,x_speed, z_axis
    #    print msg.pose.pose.orientation

    orientation_q = msg.pose.pose.orientation
    orientation_p = msg.pose.pose.position
    orientation_twist = msg.twist.twist.linear

    z_axis = int(orientation_p.z)
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (x_speed,y_speed,z_speed) = [orientation_twist.x, orientation_twist.y, orientation_twist.z]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw_degrees = yaw * 180.0 / math.pi
    roll_degrees = roll * 180.0 / math.pi
    pitch_degrees = pitch * 180.0 / math.pi
    if (yaw_degrees < 0):
        yaw_degrees += 360.0

    if (roll_degrees < 0):
        roll_degrees += 360.0

    if (pitch_degrees < 0):
        pitch_degrees += 360.0


    (yaw_degrees, roll_degrees, pitch_degrees) = (round(yaw_degrees,4), round(roll_degrees,4), round(pitch_degrees,4))
    x_speed = round(x_speed,4)
    #print z_axis,x_speed
    #print (yaw_degrees, roll_degrees, pitch_degrees,x_speed)
    #print (type(roll_degrees), pitch_degrees, yaw_degrees,x_speed) 
    # print "yaw_degrees HAS " , yaw_degrees
    # picth roll  yaw fatmanur siralayacak , 

if __name__ == '__main__':

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(5)
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	
	rospy.Subscriber('/uav1/mavros/global_position/raw/gps_vel', TwistStamped, call_back_gps_time) 
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position) 
	rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, get_rotation)
	rospy.Subscriber('/uav1/mavros/battery', BatteryState, call_back_battery_state) 
	rospy.Subscriber('/hedef_durumu', String, call_back_yolo) 
	rospy.Subscriber('/iha_ucus_modu', String, call_back_ucus_durumu)


	iha_telemetry_file = open("/home/efl4tun/Desktop/11_09/io/iha_telemetry.txt",'w') # nvidia kullanicisi olarak degistirmek zorundasin
	#request_file = open("/home/efl4tun/Desktop/request.txt",'r')
	time.sleep(4.0) # herveri ilk degerlerini almasi icin beklendi
	
	try:		
		
		
		while not rospy.is_shutdown():


			print "\n---------"

			#---------------------iha_telemetry------------
			iha_telemetry_file = open("/home/efl4tun/Desktop/11_09/io/iha_telemetry.txt",'w')
			#sirasini sor fatmanura
			telemetry_data = str(current_lat) + "#" + str(current_long) + "#" + str(z_axis) + "#" + str(roll_degrees) + "#" + str(yaw_degrees) + "#" + \
			str(pitch_degrees) + "#" + str(x_speed) + "#" + str(voltage_rate) + "#" + str(iha_otonom) + "#" + str(kilitlenme_durumu) + "#" + \
			str(x_target_axis) + "#" + str(y_target_axis) + "#" + str(target_weight) + "#" + str(target_height) + "#" + str(sistem_h) + "#" + str(sistem_m) + "#" + \
			str(sistem_s) + "#" + str(sistem_ns)

			
			iha_telemetry_file.write(telemetry_data)
			time.sleep(0.1)
			
			print "IHA_TELEMETRY \t" + str(telemetry_data)	
			open("/home/efl4tun/Desktop/11_09/io/iha_telemetry.txt",'w').close()

			#------------------------------------------------------------------------------ request

			sistem_saati_file = open("/home/efl4tun/Desktop/11_09/io/sistem_saati.txt",'w')

			sistem_saati_data = str(sistem_h) + "#" + str(sistem_m) + "#" + str(sistem_s) + "#" + str(sistem_ns)

			sistem_saati_file.write(sistem_saati_data)

			time.sleep(0.1)
			print "SISTEM_SAATI \t" + str(sistem_saati_data)
			open("/home/efl4tun/Desktop/11_09/io/sistem_saati.txt",'w').close()



			#--------------------------------------kilitlenme paketi

			if kilitlenme_paketini_yaz == 1:
				
				kilitlenme_file = open("/home/efl4tun/Desktop/11_09/io/kilitlenme.txt",'w')

				kilitlenme_data = str(kilitlenme_baslangic_h) + "#" + str(kilitlenme_baslangic_m) + "#" + str(kilitlenme_baslangic_s) + "#" + \
				str(kilitlenme_baslangic_ns) + "#" + str(kilitlenme_bitis_h) + "#" + str(kilitlenme_bitis_m) + "#" + str(kilitlenme_bitis_s) + "#" + \
				str(kilitlenme_bitis_ns) + "#" + str(iha_otonom)

				kilitlenme_file.write(kilitlenme_data)

				time.sleep(0.1)
				print "KITLENME \t" + str(kilitlenme_data)
				open("/home/efl4tun/Desktop/11_09/io/kilitlenme.txt",'w').close()
			#---------------------------------------------------------------- iha durumu

			uav_state_file = open("/home/efl4tun/Desktop/11_09/io/iha_durumu.txt",'w')
			#sirasini sor fatmanura
			uav_data = str(uav_state) 

			
			iha_telemetry_file.write(uav_data)
			
			open("/home/efl4tun/Desktop/11_09/io/iha_durumu.txt",'w').close()



	
	except rospy.ROSInterruptException:
		pass



	