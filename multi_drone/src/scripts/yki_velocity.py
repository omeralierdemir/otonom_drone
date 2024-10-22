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

ucus_hizi = 5.0
iha_ucus_mod = 0

yki_ilk_ucus = 0
yki_savasa_basla = 0
yki_hedef_takip = 0
yki_saga_git = 0
yki_sola_git = 0
yki_ileri_git = 0
yki_pid_disable = 0
yki_eve_don = 0



eve_don_temp = 0
ilk_ucus_temp = 0
baglanti_koptu_temp = 1
condition_velocity = 1


baglanti_durumu_koptu = 0
savas_basladi = 0

current_state = State()
msg = PositionTarget()



current_alt = 0
current_lat = 0
current_long = 0


home_alt = current_alt
home_lat = current_lat
home_longi = current_long  

target_alt = 0
target_lat = 0
target_long = 0 

positionX = 0
positionY = 0
positionZ = 0

lastErrorY = 0

gps_yenilenme_zamani = 0
baglanti_yenilenme_zamani = 0
yki_savasa_basla_start_time = time.time()
pid_wait = time.time() # eger 3 saniye boyunca pid aktif degilse velocity takibine gec

def state_cb(state):
    global current_state
    current_state = state

"""
def baglanti_durum_kontrol():

	global baglanti_yenilenme_zamani, baglanti_durumu_koptu

	current_time = time.time()
	baglanti_durumu = 0

	if current_time - baglanti_yenilenme_zamani <= 2.0 and baglanti_durumu_koptu == 1: # baglanti yeniden saglandi ise ve en az 2 sn bir
																					   # veri aliyorsa tekrar savasa basla
		print "kurtardin yine iyisin haaa"
		yki_ilk_ucus = 0
		yki_savasa_basla = 1
		yki_eve_don = 0
		baglanti_koptu_temp = 1
		eve_don_temp = 0 # eve don cagrilabilmesi icin lazim
		baglanti_durumu_koptu = 0
		baglanti_durumu = 1

	else:
		baglanti_durumu = 0

	return baglanti_durumu
"""
def get_rotation(msg):
    global positionX, positionY, positionZ
    #    print msg.pose.pose.orientation
    position_q = msg.pose.pose.position
    
    (positionX, positionY, positionZ) = (position_q.x, position_q.y, position_q.z)
   



def setLandMode(altitude = 0, latitude = home_lat, longitude = home_longi, min_pitch = 0, yaw = 0):
   rospy.wait_for_service('/uav1/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy('/uav1/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
       #http://wiki.ros.org/mavros/CustomModes for custom modes
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e


def check_collisyon():


	global current_alt, current_lat, current_long
	global target_lat, target_long, target_alt

	collisyon = 0

	if round(current_lat,5) == round(target_lat,5) and round(current_long,5) == round(target_long,5) and round(current_alt) == round(target_alt):

		collisyon = 1

	return collisyon


def get_home():

	global home_alt,home_longi,home_lat
	global current_alt, current_lat, current_long
	global target_lat, target_long, target_alt
	global eve_don_temp
	global msg
	global baglanti_durumu_koptu

	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	rate = rospy.Rate(5)
	land_temp = 1
	(target_lat, target_long, target_alt) = (home_lat, home_longi, home_alt)
	if eve_don_temp :
		print "hadi eve saat gec oldu"
		
		while True:
			
			flight_controller()
			pub.publish(msg)
			#print baglanti_durumu_koptu == 1
			#baglanti_durumu = baglanti_durum_kontrol()
			print "get_home"
			if round(current_lat,5) == round(home_lat,5) and round(current_long,5) == round(home_longi,5):
				print "break"
				break
			"""
			if baglanti_durumu == 0: #baglanti tekrar saglandiysa inme
				print "buradaki ife girdimmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm"
				land_temp = 0
				break
			"""
			rate.sleep()
		
		if land_temp == 1:
			setLandMode()
	eve_don_temp = 0
    
  
    # print "yaw_degrees HAS " , yaw_degrees


def call_back_current_position(data):

	global current_alt,current_lat,current_long
	global home_longi, home_lat, home_alt
	global ilk_ucus_temp
	global gps_yenilenme_zamani
	global yki_sola_git


	if ilk_ucus_temp == 1:
		#buraya seriportan istenen veri geldiyse kosulu konacak
		home_lat =  data.latitude
		home_longi = data.longitude
		home_alt = 30.0 #baslangic yuksekligi 10m set edildi
		
		print ("home paremetreleri set edildi", home_lat,home_longi,home_alt)
	  # buna gerek kalmayabilir
	

	current_lat =  data.latitude
	current_long = data.longitude

	gps_yenilenme_zamani = time.time()
		
	#print current_lat,current_long


def call_change_speed(data):

	global ucus_hizi, baglanti_yenilenme_zamani

	ucus_hizi = float(data.data)
	
	baglanti_yenilenme_zamani = time.time()



def call_back_coordinates(data):

	global target_lat, target_long, target_alt
	global baglanti_yenilenme_zamani
	global eve_don_temp
	global positionZ 

	print data.data.split(",")

	telemetry_temp = data.data.split(",")
	if eve_don_temp == 0: # eve_don komutu aktif olunca hedefin konumunu artik almayacak konum olarak home_telemeteleri set edilecek


		
		if telemetry_temp[0] != "null" :
			
		 	(t_lat, t_longi, t_alt) = data.data.split(",")# null gelme durumunu dusun
		 	
		 	(target_lat, target_long, target_alt) = (float(t_lat), float(t_longi), float(t_alt))

		 	if target_alt < 25.0:

		 		print "sd"

		 	if target_alt == 0.0:

		 		target_alt = 35.0
		else:

			print "nullllllllllll atadifsngfnmfgnmfgnm"
	baglanti_yenilenme_zamani = time.time() # yki baglanti kontrolu icin olusturulmus zaman degiskeni bunu her yer istasyonu komtu gelince guncelle

 	
 	

	#print (target_lat, target_long, target_alt)








def flight_controller(follow_speed= 3.0):

	global current_lat, current_long, current_alt
	global ilk_ucus_temp
	global target_lat, target_long, target_alt
	global positionZ 
	global msg
	global lastErrorY
	global iha_ucus_mod

 	"""
	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	rate = rospy.Rate(20)
	"""

	# ------ yukselme kontrolcusu ---------
	if condition_velocity == 1:

		iha_ucus_mod = 2

		errorY = target_alt - positionZ

		hizYukselme = 0.75
        kp =  hizYukselme / 213
        kd =  hizYukselme / (213 + hizYukselme * (35)) 
		

        turevY = errorY-lastErrorY
        yukselmeY =  errorY*kp + turevY*kd
		#msg.velocity.z = yukselmeY
		
        lastErrorY = errorY

        if yukselmeY >= 1.3:
            yukselmeY =1.3

        if yukselmeY <= -1.3:

            yukselmeY = -1.3

	
        #print target_alt, positionZ, yukselmeY

	    # ------------------------------------------



		

        y_eksen = target_lat - current_lat 
        x_eksen = (target_long - current_long) *   0.7627 #yeni deger #400/700 # ayni scalayacektik enlem ile boylami (yaklasik olarak)
		
        if y_eksen == 0:
            y_eksen = 0.0000001
        if x_eksen == 0:
            x_eksen = 0.0000001


        yaw_radyan = math.atan2(y_eksen,x_eksen)
        yaw_radyan2 = math.atan(x_eksen/y_eksen)
        target_yaw =  (yaw_radyan / math.pi) * 180
		

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE

        msg.velocity.x = 0.0
        msg.velocity.y = follow_speed#follow_speed # suankil 4m/s sonradan arguman olarak yer istasyonundan alinacak (float)
        msg.velocity.z = yukselmeY # suanlik 0 sonradan konrolcu eklenecek
        msg.yaw = yaw_radyan
		#pid_wait = time.time()
        rate.sleep()

		#_________________________________________________




def call_back_pid(pid_data):
	global axis_z, axis_r, axis_yaw, msg
	global pid_wait, condition_waypoint
	global yki_savasa_basla, msg
	global iha_ucus_mod

	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	#set_mode(0,'MANUAL')
	rate = rospy.Rate(20)
	"""
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = "world"
	msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
	msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
	"""
	
	if pid_data.data != "null" and yki_savasa_basla == 1 and yki_pid_disable == 0: # yki_pid_disable == 0 ise pid aktifdir yer istasyonundan kontrol edilecek

		iha_ucus_mod = 3
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
		condition_velocity = 0		#pub.publish(msg)
		rate.sleep()                #buna cok gerek olmayabilir. hesapla ne kadar geciktigini


	else:
		current_time4 = time.time()
		
		if current_time4 - pid_wait >= 3 and yki_savasa_basla == 1: # eger 3 sn boyunca hedef iha yok ise ekranda waypoint ile arama yap bu zamanlada sikinti olabilir

			
			print "bekl,torum"
			condition_velocity = 1
			#print "way pointe geciyom haci"




	


def set_single_position(desired_x=0, desired_y=0, desired_z=32):

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
	#print msg
	#msg.yaw = 0.0



def call_back_yki(data):
	
	global yki_ilk_ucus, yki_savasa_basla, yki_hedef_takip
	global yki_saga_git, yki_sola_git, yki_ileri_git, yki_pid_disable, yki_eve_don
	global eve_don_temp, ilk_ucus_temp
	global baglanti_yenilenme_zamani
	
	mod_data =  data.data.split(",")
	print "buradayim", mod_data

	if mod_data[0] != "null":

		(ilk_ucus, savasa_basla, pid_disable, eve_don) = data.data.split(',')
		
		print "ifdeyim", (ilk_ucus, savasa_basla, pid_disable, eve_don)

		yki_ilk_ucus = int(ilk_ucus)
		yki_savasa_basla = int(savasa_basla)
		
		yki_pid_disable = int(pid_disable)
		yki_eve_don = int(eve_don)
		
		# bak burada neleri set ettigine cok dikkat et yoksa sacma harektler yapar iha

		if yki_eve_don == 1:

			print "yki_eve_don"
			
			yki_ilk_ucus = 0
			yki_savasa_basla = 0
			yki_eve_don = 1

			eve_don_temp = 1
			baglanti_yenilenme_zamani = time.time()

		if yki_savasa_basla == 1:

			print "yki_savasa_basla"
			yki_ilk_ucus = 0
			baglanti_yenilenme_zamani = time.time()

		if yki_ilk_ucus == 1:

			print "yki_ilk_ucus"
			ilk_ucus_temp = 1 # ilerisi icin tehlikeli bir kod mutlaka tarik hocaya danis. Bu islem ilk komutta drone kalkmazsa diye konuldu ykiden
			             #gonderilen paketleri buna gore ayarla
			baglanti_yenilenme_zamani = time.time()

		print (yki_ilk_ucus, yki_savasa_basla, yki_pid_disable, yki_eve_don)




def calling_methods():

	global lat, longi, alt
	global yki_ilk_ucus, yki_savasa_basla, yki_eve_don
	global baglanti_yenilenme_zamani, gps_yenilenme_zamani # baglanti_koptu_temp get_home()  metodunun birkere cagrilamsi icin
	global eve_don_temp, ilk_ucus_temp, baglanti_koptu_temp
	global yki_savasa_basla_start_time																# olusturuldu.
	global baglanti_durumu_koptu, savas_basladi 
	global iha_ucus_mod
	global ucus_hizi

	current_time = time.time()
	if yki_ilk_ucus == 1:
	# baslangic_ucusu gerceklesir ise true olacak. Bu baslangic ucusunun gerceklesmesi icin get_takeoff calismali 
		
		#ilk_ucus_temp = 1
		
		   # onun calismasi icin yki_ilk_ucus_onay true yani onay verilmesi lazim
		iha_ucus_mod = 1


	if yki_savasa_basla == 1: # eger baslangic ucusu yapildi ise ve yki den veri geldiyse hedef konumlari olustur.
		
		
		iha_ucus_mod = 2
		if current_time - yki_savasa_basla_start_time >=0.2:

			yki_savasa_basla_start_time = time.time()
			flight_controller(ucus_hizi)
			savas_basladi = 1 # bak bu cok dogru olmayabilir sebebi oyun sirasinda gps koparsa in diyebilmek her zaman aktif
								# bir kere savasa basla demek yeterli
	if yki_eve_don == 1:
		print "zamaninda cok konustum faydasini gormedim"
		get_home()
		yki_eve_don = 0
		iha_ucus_mod = 4


	#print "cuurent time : ",current_time, baglanti_yenilenme_zamani, "----", gps_yenilenme_zamani , " -oooooo- ", current_time- baglanti_yenilenme_zamani,baglanti_koptu_temp == 1 ,yki_savasa_basla == 1

	if current_time - baglanti_yenilenme_zamani >= 15.0 and baglanti_koptu_temp == 1 and yki_savasa_basla == 1: # eger baglanti kopar ise 20 saniye sonra eve don bunu yer istasyonuna sucscreber
															# oldugun her metotda kullanabilirsin
		print "ifdeyim baglanti koptu"															
		#get_home()
		#yki_eve_don = 0
		yki_ilk_ucus = 0
		yki_savasa_basla = 0
		yki_eve_don = 1
		baglanti_koptu_temp = 0
		eve_don_temp = 1 # eve don cagrilabilmesi icin lazim
		baglanti_durumu_koptu = 1
		iha_ucus_mod = 6

	"""
	print current_time - baglanti_yenilenme_zamani <= 2.0 , baglanti_durumu_koptu == 1

	if current_time - baglanti_yenilenme_zamani <= 2.0 and baglanti_durumu_koptu == 1: # baglanti yeniden saglandi ise ve en az 2 sn bir
																					   # veri aliyorsa tekrar savasa basla
		print "kurtardin yine iyisin haaa"
		yki_ilk_ucus = 0
		yki_savasa_basla = 1
		yki_eve_don = 0
		baglanti_koptu_temp = 1
		eve_don_temp = 0 # eve don cagrilabilmesi icin lazim
		baglanti_durumu_koptu = 0

	"""

	if current_time - gps_yenilenme_zamani >= 10.0 and yki_savasa_basla == 1: # eger gps baglantisi kopar ise 10 saniye sonra inise gec

		print "gps baglantisi koptuuuu"
		setLandMode()
		yki_ilk_ucus = 0
		yki_savasa_basla = 0
		yki_eve_don = 0
		iha_ucus_mod = 5

if __name__ == '__main__':

	print "Rahman ve Rahim olan ALLAH'IN adiyla"
	print "Onlar, ustlerinde dizi dizi kanat acip kapayarak ucan kuslari gormuyorlar mi? Onlari Rahman (olan ALLAH')tan baskasi (boslukta) tutmuyor. Suphesiz O, herseyi hakkiyla gorendir."
	print "La havle ve la kuvvete illa billahil aliyyil azim ----- Guc ve kuvvet, sadece Yuce ve Buyuk olan ALLAH'in yardimiyla elde edilir. "
	rospy.init_node('gps_velocity', anonymous=True)
	mavros.set_namespace('mavros')
	rate = rospy.Rate(20)
	rate2 = rospy.Rate(5)
	rospy.wait_for_service('/uav1/mavros/cmd/arming')
	set_mode = rospy.ServiceProxy('/uav1/mavros/set_mode', mavros_msgs.srv.SetMode)
	state_sub = rospy.Subscriber('/uav1/mavros/state', State, state_cb)
	
	#rospy.Subscriber('spesific_waypoint', String, call_back_coordinates)
	rospy.Subscriber('target_telemetry_data', String, call_back_coordinates)
	rospy.Subscriber('ucus_gorevi', String, call_back_yki)
	rospy.Subscriber('hiz_data', String, call_change_speed)
	#rospy.Subscriber('waypoint_random', String, call_back_coordinates)
	#rospy.Subscriber('yer_istasyonu', String, call_back_yki)
	rospy.Subscriber('/uav1/mavros/global_position/global', NavSatFix, call_back_current_position)  
	rospy.Subscriber('get_pid', String, call_back_pid)
	rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, get_rotation)
	pub3 = rospy.Publisher('iha_ucus_modu', String, queue_size=10)






	
	pub = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	#print msg 
	set_mode(0,'OFFBOARD')

	#rate.sleep()
	time.sleep(4.0)
	
	try:
		
		
		
		start_time3 = time.time()
		
		while not rospy.is_shutdown():
			#pub.publish(msg)
			#flight_controller()
			current_time = time.time()
			 

			if yki_ilk_ucus == 1 and ilk_ucus_temp == 1:	

				baglanti_koptu_temp = 1 # bunun sebebi tekrar kalkis gerekirse acil durum inisi icin gerekli!

				try:
					armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
					armService(True)
				except rospy.ServiceException, e:
	 
					print "arm hatasi"


				set_single_position()
				pub.publish(msg)
				while current_state.mode != "OFFBOARD":
		
					pub.publish(msg)
					rate.sleep()
		
					#rospy.loginfo("OFFBOARD mod istegi gonderildi")
					#print msg 
					set_mode(0,'OFFBOARD')

					#rate.sleep()
				time.sleep(4.0)

				ilk_ucus_temp = 0
				


			if (current_time - start_time3)>= 0.2 :# and yki_savasa_basla == 1:
				



				#print "girdim"
				start_time3 = time.time()
				calling_methods()
				pub3.publish(str(iha_ucus_mod))
				pub.publish(msg)	
				#	 print ucus_hizi	
		
				

	

		
		
	except rospy.ROSInterruptException:
		pass


	