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



baslangic_ucusu = 0

pid_wait = time.time()
condition_waypoint = 1

(yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don) = (0,0,0,0,0,0,0,0)



baglanti_koptu_temp = 1

alt = 0
lat = 0
longi = 0 

baglanti_yenilenme_zamani = 0
gps_yenilenme_zamani = 0

current_alt = 0#10
current_lat = 0#47.3977417
current_long = 0#8.5455943  # sonrada n / mavros/setpoint_possition/globala sucscriber oluancak

home_alt = current_alt
home_lat = current_lat
home_longi = current_long  # sebebi bazen yeniden baslatmak gerekir. baslangic noktasi havada iken ilk olarak atansin

ilk_ucus_temp = 0 # bu ilk kez 1 edildiginde baslangic noktasi set edilmis olunur bu sayede get_home ve ilk ucus saglanir
eve_don_temp = 0	
current_state = State()
msg = PositionTarget()

frameAxisX = 416
frameAxisY = 416

def compareDetectionArea(arg_y, arg_x):

	global frameAxisX, frameAxisY

	yatay_eksen = frameAxisX * 0.25
	dikey_eksen = frameAxisY * 0.1
	kilitlenme_dortgeni_icinde = 0
	if (dikey_eksen <= arg_y and (frameAxisY - dikey_eksen) >= arg_y) and (yatay_eksen <= arg_x and (frameAxisX - yatay_eksen)>=arg_x):

		kilitlenme_dortgeni_icinde = 1

	return kilitlenme_dortgeni_icinde

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
		print "hadi eve saat gec oldu"
		singleWaypoint(home_lat,home_longi,home_alt)# home_alt bu standart ilk ciktigi yukseklik
		while True:
			
			if round(current_lat,5) == round(home_lat,5) and round(current_long,5) == round(home_longi,5):
				print "break"
				break

		setLandMode()
	eve_don_temp = 0

def get_takeoff():

	
	global home_alt, home_longi, home_lat
	global baslangic_ucusu, ilk_ucus_temp, baglanti_koptu_temp 
	
	baglanti_koptu_temp = 1

	print "yki_ilk_ucus_temp", ilk_ucus_temp
	if ilk_ucus_temp:
		print " take offfffffffffffffff"
		setArm()
		singleWaypoint(home_lat,home_longi,home_alt)

		while True:

			if True: # yer istasyonundan veri gelene kadar bekle
				print "beni birkere cagircak"
				break
		#baslangic_ucusu = 1		# null veri gonderince yer istasyonu ne yaptigini test etmen lazim
	ilk_ucus_temp = 0 # bu ilk kez 1 edildiginde baslangic noktasi set edilmis olunur bu sayede get_home ve ilk ucus saglanir ve birden fazle bu metodun
					#cagrilmasini onler




def sudden_right():

	global current_alt, current_lat, current_long

	singleWaypoint(current_lat,current_long,current_alt)




def call_back_current_position(data):

	global current_alt,current_lat,current_long
	global home_longi, home_lat, home_alt
	global ilk_ucus_temp
	global gps_yenilenme_zamani
	global yki_sola_git

	if ilk_ucus_temp:
		#buraya seriportan istenen veri geldiyse kosulu konacak
		home_lat =  data.latitude
		home_longi = data.longitude
		home_alt = 10.0 #baslangic yuksekligi 10m set edildi
		
		print ("home paremetreleri set edildi", home_lat,home_longi,home_alt)

	if yki_sola_git == 0:

		current_lat =  data.latitude
		current_long = data.longitude
		gps_yenilenme_zamani = time.time()
		

def call_back_coordinates(data):

	global lat, longi, alt
	global baglanti_yenilenme_zamani

 	(t_lat, t_longi, t_alt) = data.data.split(",")
 	
 	(lat, longi, alt) = (float(t_lat), float(t_longi), float(t_alt))

 	baglanti_yenilenme_zamani = time.time()
 	
	



def call_back_yki(data):
	
	global yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip
	global yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don
	global eve_don_temp, ilk_ucus_temp
	
	print data.data.split(",")

	
	(ilk_ucus, savasa_basla, hedef_takip, saga_git, sola_git, ileri_git, geri_git, eve_don) = data.data.split(',')

	yki_ilk_ucus = int(ilk_ucus)
	yki_savasa_basla = int(savasa_basla)
	yki_hedef_takip = int(hedef_takip)
	yki_saga_git = int(saga_git)
	yki_sola_git = int(sola_git)
	yki_ileri_git = int(ileri_git)
	yki_geri_git = int(geri_git)
	yki_eve_don = int(eve_don)
	# bak burada neleri set ettigine cok dikkat et yoksa sacma harektler yapar iha

	if yki_eve_don:
		
		yki_ilk_ucus = 0
		yki_savasa_basla = 0
		yki_hedef_takip = 0
		yki_saga_git = 0
		yki_sola_git = 0
		yki_ileri_git = 0
		yki_geri_git = 0
		yki_eve_don = 1

		eve_don_temp = 1

	if yki_savasa_basla:
		yki_ilk_ucus = 0
	if yki_ilk_ucus:
		ilk_ucus_temp = 1 # ilerisi icin tehlikeli bir kod mutlaka tarik hocaya danis. Bu islem ilk komutta drone kalkmazsa diye konuldu ykiden
		             #gonderilen paketleri buna gore ayarla
		print "3.ifdeyim"
	print (yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip, yki_saga_git, yki_sola_git, yki_ileri_git, yki_geri_git, yki_eve_don)





def call_back_pid(pid_data):
	global axis_z, axis_r, axis_yaw, msg
	global pid_wait, condition_waypoint
	global yki_savasa_basla

	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	rate = rospy.Rate(20)

	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE

	
	if pid_data.data != "null" and yki_savasa_basla == 1: # buraya birde ekranda ne kadar yer kapladigi kosunu koy

		print "pid geciyom haci"
		condition_waypoint = 0
		
		(axis_z, axis_r, axis_yaw) = pid_data.data.split(",") 
		(axis_z, axis_r, axis_yaw) = (float(axis_z), float(axis_r), float(axis_yaw))
		print(axis_z, axis_r, axis_yaw)
		msg.velocity.x = 0.0
		msg.velocity.y = axis_r
		msg.velocity.z = axis_z
		msg.yaw = axis_yaw
		pid_wait = time.time()

		pub.publish(msg)
		rate.sleep()#buna cok gerek olmayabilir. hesapla ne kadar geciktigini


		while current_state.mode != "OFFBOARD":
			waypoint_clear_client()
			pub.publish(msg)
			rate.sleep()#buna cok gerek olmayabilir. hesapla ne kadar geciktigini
			#pub.publish(msg)
			#print "buradasin auto"
			rospy.loginfo("OFFBOARD mod istegi gonderildi")
			 
			set_mode(0,'OFFBOARD')
			#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()
		
	else:
		current_time4 = time.time()
		
		if current_time4 - pid_wait >= 3 and yki_savasa_basla == 1: # eger 3 sn boyunca hedef iha yok ise ekranda waypoint ile arama yap bu zamanlada sikinti olabilir

			
			while current_state.mode != "AUTO.MISSION":
					
				rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
				 
				set_mode(0,'AUTO.MISSION')
				#rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
				
				rate.sleep()		

	
			condition_waypoint = 1
			#print "way pointe geciyom haci"




	



def call_createWaypoints():

	global lat, longi, alt
	global yki_ilk_ucus, yki_savasa_basla, yki_eve_don
	global baglanti_yenilenme_zamani, baglanti_koptu_temp, gps_yenilenme_zamani # baglanti_koptu_temp get_home()  metodunun birkere cagrilamsi icin
	global eve_don_temp																		# olusturuldu.
	


	current_time = time.time()
	print "yki_savasa_basla,yki_ilk_ucus,yki_eve_don",yki_savasa_basla ,yki_ilk_ucus,yki_eve_don

	if yki_ilk_ucus: # baslangic_ucusu gerceklesir ise true olacak. Bu baslangic ucusunun gerceklesmesi icin get_takeoff calismali 
		
		

		get_takeoff()    # onun calismasi icin yki_ilk_ucus_onay true yani onay verilmesi lazim



	if yki_savasa_basla: # eger baslangic ucusu yapildi ise ve yki den veri geldiyse hedef konumlari olustur.
		
		if current_time % 2 >= 1.9 and condition_waypoint == 1:
			
			print "create_waypoints cagrildi"
			print("calbackdeki ",lat, longi, alt)
			create_waypoints()

	
	if yki_eve_don:
		print "zamaninda cok konustum faydasini gormedim"
		get_home()
		yki_eve_don = 0
	
	print "cuurent time : ",current_time, baglanti_yenilenme_zamani, "----", gps_yenilenme_zamani , " -oooooo- ", current_time- baglanti_yenilenme_zamani,baglanti_koptu_temp == 1
	#print current_time - baglanti_yenilenme_zamani, " -----" , current_time - gps_yenilenme_zamani
	

	if current_time - baglanti_yenilenme_zamani >= 10.0 and baglanti_koptu_temp == 1 and yki_savasa_basla == 1: # eger baglanti kopar ise 20 saniye sonra eve don bunu yer istasyonuna sucscreber
															# oldugun her metotda kullanabilirsin
		print "ifdeyim baglanti koptu"															
		#get_home()
		#yki_eve_don = 0
		yki_ilk_ucus = 0
		yki_savasa_basla = 0
		yki_eve_don = 1
		baglanti_koptu_temp = 0
		eve_don_temp = 1 # eve don cagrilabilmesi icin lazim


	if current_time - gps_yenilenme_zamani >= 10.0 and yki_savasa_basla == 1: # eger gps baglantisi kopar ise 10 saniye sonra inise gec

		print "gps baglantisi koptuuuu"
		setLandMode()
		yki_ilk_ucus = 0
		yki_savasa_basla = 0
		yki_eve_don = 0
    







def create_waypoints():

	global current_lat, current_long, current_alt
	global ilk_ucus_temp
	global lat, longi, alt


	rate = rospy.Rate(20)

	wl = []
	
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
	wp.param2 = 0
	wp.param3 = 1
	wp.param4 = yaw
	wp.x_lat = lat 
	wp.y_long = longi
	wp.z_alt = alt
	wl.append(wp)


	
	try:
	    service = rospy.ServiceProxy('/uav1/mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	    #ilk_ucus_temp = 0 # bu ilk kez 1 edildiginde baslangic noktasi set edilmis olunur bu sayede get_home ve ilk ucus saglanir
	  
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e
	



def singleWaypoint(except_lat,except_longi, except_alt):
	
	global current_lat, current_long, current_alt
	global lat, longi, alt
	rate = rospy.Rate(20)

	wl = []

	
	waypoint_clear_client()



	y_eksen = except_lat - current_lat #burada direk r1* 0.00001 i esitlenebilir ama hatirlanman icin boyle yaptin 
	x_eksen = (except_longi - current_long) * 400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
	
	tan = math.atan2(x_eksen,y_eksen)
	
	yaw =  (tan / math.pi) * 180
	
	print("except_lat : " ,y_eksen," except_lat : ",x_eksen)
	print("tanjant: ", tan)

	
	
	while current_state.mode != "AUTO.MISSION":
			
			
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
	wp.param2 = 0
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
	state_sub = rospy.Subscriber('/uav1/mavros/state', State, state_cb)
	waypoint_clear_client()
	rospy.Subscriber('waypoint_random', String, call_back_coordinates)
	rospy.Subscriber('yer_istasyonu', String, call_back_yki)
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position)  
	rospy.Subscriber('get_pid', String, call_back_pid)

	try:
		
		
		
		"""
		
		while current_state.mode != "AUTO.MISSION":
			
			
			print "buradasin"
			 
			set_mode(0,'AUTO.MISSION')
			rospy.loginfo("AUTO.MISSION mod istegi gonderildi")
			
			rate.sleep()
		

		"""
		
		while not rospy.is_shutdown():

			
			call_createWaypoints()
			rate2.sleep()
			


		waypoint_clear_client()
		
	except rospy.ROSInterruptException:
		pass


