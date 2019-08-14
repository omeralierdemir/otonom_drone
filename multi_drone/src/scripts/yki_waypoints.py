#!/usr/bin/env python

import rospy
import random
import time
import mavros
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *

temp = 0 # gecici get_home cagrilmasi icin olusturulmus count degiskeni
temp2 = 1
baslangic_ucusu = 0


(yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don) = (0,0,0,0,0,0,0,0)

home_alt = 0
home_lat = 0
home_longi = 0

temp_time = 0

alt = home_alt
lat = home_lat
longi = home_longi 

current_alt = 0#10
current_lat = 0#47.3977417
current_long = 0#8.5455943  # sonrada n / mavros/setpoint_possition/globala sucscriber oluancak
start_time = 0
ilk_ucus_temp = 1 # bu ilk kez 1 edildiginde baslangic noktasi set edilmis olunur bu sayede get_home ve ilk ucus saglanir
eve_don_temp = 0	
current_state = State()
msg = PositionTarget()


def setArm():
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

		armService(True)
	except rospy.ServiceException, e: # metin abi hold ona al dedi
	 					
	 	pass

def state_cb(state):
    global current_state
    current_state = state

def waypoint_clear_client():
        try:
            response = rospy.ServiceProxy('/uav1/mavros/mission/clear', WaypointClear)
            return response.call().success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False



def setLandMode(altitude = 0, latitude = home_lat, longitude = home_longi, min_pitch = 0, yaw = 0):
   rospy.wait_for_service('/uav1/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/uav1/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
       #http://wiki.ros.org/mavros/CustomModes for custom modes
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e






def get_home():

	global home_alt,home_longi,home_lat
	global current_alt, current_lat, current_long,eve_don_temp


	if eve_don_temp :
		singleWaypoint(home_lat,home_longi,home_alt)# home_alt bu standart ilk ciktigi yukseklik
		while True:
			
			if round(current_lat,5) == round(home_lat,5) and round(current_long,5) == round(home_longi,5):
				print "break"
				break

		setLandMode()
	eve_don_temp = 0

def get_takeoff():

	
	global home_alt,home_longi,home_lat,baslangic_ucusu,ilk_ucus_temp
	
	if ilk_ucus_temp:
		setArm()
		singleWaypoint(home_lat,home_longi,home_alt)

		while True:

			if True: # yer istasyonundan veri gelene kadar bekle
				print "beni birkere cagircak"
				break
		#baslangic_ucusu = 1		# null veri gonderince yer istasyonu ne yaptigini test etmen lazim
	ilk_ucus_temp = 0 # bu ilk kez 1 edildiginde baslangic noktasi set edilmis olunur bu sayede get_home ve ilk ucus saglanir ve birden fazle bu metodun
					#cagrilmasini onler











def call_back_yki(data):
	
	global yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip
	global yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don
	
	print data.data.split(",")

	(yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don) = data.data.split(',')

	(yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don) = (int(yki_ilk_ucus), int(yki_savasa_basla), int(yki_hedef_takip), int(yki_saga_git), int(yki_sola_git), int(yki_ileri_git), int(yki_geri_git), int(yki_eve_don))


	# bak burada neleri set ettigine cok dikkat et yoksa sacma harektler yapar iha

	if yki_eve_don:
		print "hadi eve saat gec oldu"
		(yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don) = (0,0,0,0,0,0,0,1)
		eve_don_temp = 1
	if yki_ilk_ucus:
		ilk_ucus_temp = 1 # ilerisi icin tehlikeli bir kod mutlaka tarik hocaya danis. Bu islem ilk komutta drone kalkmazsa diye konuldu ykiden
		             #gonderilen paketleri buna gore ayarla
	print (yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don)




def call_back_current_position(data):

	global current_alt,current_lat,current_long, home_longi, home_lat, home_alt,ilk_ucus_temp

	if ilk_ucus_temp:
		#buraya seriportan istenen veri geldiyse kosulu konacak
		home_lat =  data.latitude
		home_longi = data.longitude
		home_alt = 10.0 #baslangic yuksekligi 10m set edildi
		
		print ("home paremetreleri set edildi", home_lat,home_longi,home_alt)



	current_lat =  data.latitude
	current_long = data.longitude


def call_back_coordinates(data):

	global lat, longi, alt,temp,temp2,temp_time
	#print data.data
	#temp = temp + 1 #bu degisken eve donus testi icin olusturuldu. 7 call_back cagrisindan sonra eve donmesi planlandi test icin
	
	


 	(t_lat, t_longi, t_alt) = data.data.split(",")
 	
 	(lat, longi, alt) = (float(t_lat), float(t_longi), float(t_alt))

	#print("call back ", lat, longi,alt)
	#print temp
	#if temp == 7:
	#	get_home()
	#	temp2 = 0
	#	print "eve don"
	#if temp2:

	



def call_createWaypoints():

	global lat, longi, alt,yki_ilk_ucus,yki_savasa_basla,temp,temp2,temp_time,basla
	

	current_time = time.time()
	print "yki_savasa_basla,yki_ilk_ucus",yki_savasa_basla ,yki_ilk_ucus
	if yki_savasa_basla: # eger baslangic ucusu yapildi ise ve yki den veri geldiyse hedef konumlari olustur.
		
		if current_time % 2 >= 1.97:
			
			print "create_waypoints cagrildi"
			print("calbackdeki ",lat, longi, alt)
			create_waypoints()

	if yki_ilk_ucus: # baslangic_ucusu gerceklesir ise true olacak. Bu baslangic ucusunun gerceklesmesi icin get_takeoff calismali 
		print "yki_ilk_ucus"
		get_takeoff()    # onun calismasi icin yki_ilk_ucus_onay true yani onay verilmesi lazim


	if yki_eve_don:

		get_home()













def create_waypoints():
	global start_time
	global current_long
	global current_lat
	global ilk_ucus_temp,temp,temp_time


	temp_time = 0
	rate = rospy.Rate(20)

	wl = []

	start_time3 = time.time()
	while True:

		current_time3 = time.time()	
		if int(current_time3) - int(start_time3) >= 0:
			print
			break	
		#print "zaman ", int(current_time3) - int(start_time)
	print "ciktim"
	waypoint_clear_client()


	y_eksen = lat - current_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	x_eksen = (longi - current_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	
	tan = math.atan2(x_eksen,y_eksen)
	
	yaw =  (tan / math.pi) * 180
	
	print("lat: " ,y_eksen," long : ",x_eksen)
	print("tanjant: ", tan)
	print("lat, long create daki" ,lat, longi)
	
	while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			#print "buradasin auto"
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			 
			set_mode(0,'AUTO.MISSION')
			#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()		

	
	
	wp = Waypoint()
	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = True
 	wp.autocontinue = False
	wp.param1 = 0  # delay 
	#wp.param2 = 0
	wp.param3 = 0
	wp.param4 = yaw
	wp.x_lat = lat 
	wp.y_long = longi
	wp.z_alt = alt
	wl.append(wp)


	#print(wl)
	start_time = time.time()
	"""
	current_lat = lat 
	current_long = longi
	current_alt = alt 
	"""
	try:
	    service = rospy.ServiceProxy('/uav1/mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	    #ilk_ucus_temp = 0 # bu ilk kez 1 edildiginde baslangic noktasi set edilmis olunur bu sayede get_home ve ilk ucus saglanir
	  
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e
	



def singleWaypoint(except_lat,except_longi, except_alt):
	

	rate = rospy.Rate(20)

	wl = []

	
	waypoint_clear_client()



	y_eksen = except_lat - current_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	x_eksen = (except_longi - current_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	
	tan = math.atan2(x_eksen,y_eksen)
	
	yaw =  (tan / math.pi) * 180
	
	print("lat: " ,y_eksen," long : ",x_eksen)
	print("tanjant: ", tan)

	
	
	while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			#print "buradasin auto"
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			 
			set_mode(0,'AUTO.MISSION')
			#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()		

	
	
	wp = Waypoint()
	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = True
 	wp.autocontinue = False
	wp.param1 = 0  # delay 
	#wp.param2 = 0
	wp.param3 = 1
	wp.param4 = yaw
	wp.x_lat = except_lat 
	wp.y_long =except_longi
	wp.z_alt = except_alt 
	wl.append(wp)


	try:
	    service = rospy.ServiceProxy('/uav1/mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	    
	  
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e
	











if __name__ == '__main__':

	rospy.init_node('waypoint_node', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)
	rate2 = rospy.Rate(10)
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	state_sub = rospy.Subscriber('/uav1/mavros/state', State, state_cb)
	waypoint_clear_client()
	rospy.Subscriber('waypoint_random', String, call_back_coordinates)
	rospy.Subscriber('yer_istasyonu', String, call_back_yki)
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position)  

	try:
		
		
		
		
		
		while current_state.mode != "AUTO.MISSION":
			
			#pub.publish(msg)
			print "buradasin"
			 
			set_mode(0,'AUTO.MISSION')
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()
		
		start_time = time.time()
		start_time2 = time.time()

		
		while not rospy.is_shutdown():

			current_time = time.time()
			#print(int(current_time) - int(start_time))
			"""
			if (int(current_time) - int(start_time2) == 3):
					print "arm"
					try:
						armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)

						armService(True)
					except rospy.ServiceException, e: # metin abi hold ona al dedi
	 					
	 					pass
			#print current_time % 2 >= 1.95
			
			
			"""
			call_createWaypoints()
			rate2.sleep()
			#if (int(current_time) - int(start_time))>= 6:

			#	create_waypoints()
				#print(start_time)

			#pub.publish(konum)


		waypoint_clear_client()
		
	except rospy.ROSInterruptException:
		pass


