#!/usr/bin/env python

import time
import serial
import rospy
import mavros
from std_msgs.msg import String
'''
isim ornek_veri yer/drone herzaman/tetiklenince

telemetry_data, <lat,long,alt,...volatge> ,drone , her zaman
sistem_saati, <baslangic_saat,baslangic_dk,baslangic_sn,bitis_saat> ,drone , her zaman
kilitlenme , <kilitlenme zaman>, drone, kilitlendiginde
iha durumu , <rakam (1-10 arasinda)>,drone , her zaman

hedef_telemetry_data, <(enlem, boylam, irtifa)> , yki, her zaman
ucus_gorevi_data ,< 1,0,0,0,0,0,0,0> yki, tetiklenme
hiz_verisi_data  , < 1-7 arasinda rakam> , yki , gerektiginde
yukselme ,< yukselme >

'''

# ------ Models ----------
def creteTelemetri():
    telemetri = dict(
        takimNumarasi = 0,
        IHA_enlem = 0,
        IHA_boylam = 0,
        IHA_irtifa = 0,
        IHA_dikilme = 0,
        IHA_yonelme = 0,
        IHA_yatis = 0,
        IHA_hiz = 0,
        IHA_batarya = 0,
        IHA_otonom = 0,
        IHA_kilitlenme = 0,
        Hedef_merkez_x = 0,
        Hedef_merkez_y = 0,
        Hedef_genislik = 0,
        Hedef_yukseklik = 0,
        GPSSAAT = 0,
    )
    return telemetri

def createSystemTime():
    time = dict(
        Hour = 0,
        Minitue = 0,
        Second = 0,
        MiliSecond = 0
    )

def createDetectionInfo():
    detectionInfo = dict(
        state = 0
    )



# ---- serial

#ser = serial.Serial('/dev/ttyTELEMETRY', 57600, timeout=500,
#parity=serial.PARITY_EVEN, rtscts=1)
ser = serial.Serial('/dev/ttyUSB2', 57600, timeout=500,
parity=serial.PARITY_NONE, rtscts=1)

# ---- Package

curPackageId = 0
diveder = '_'

# Sizes
idSize = 5
lenghtSize = 4
typeSize = 3
dataIndex = 5+4+3

# Types
PackageTypes = dict(
    Telemetri=333,
    Config = 999,
    SystemTime = 111,
    Location = 222,
    Command = 555,
    Speed=666,
    Config2=777
)

# ---- Boxses
InBox = dict()
WaitBox = dict()

# -------------- Package
iha_telemetry = 0
sistem_saati_q = 0

PID = 0
HomeSweetHome = 0
Start = 0
TakeOff = 0


def call_back_iha_telemetry(data):
    global iha_telemetry
    iha_telemetry = data.data
    #iha_telemetry = data

    data = iha_telemetry.replace(".",",").replace("#","_")
    arr = data.split("_")

    arr[len(arr)-4] =  str(int(arr[len(arr)-4]) -3)
    string = "_"
    string = string.join(arr)
    package = createPackage(True)
    package["data"] = string
    package["lenght"] = len(package["data"])
    package["type"] = PackageTypes["Telemetri"]
    writeStringData((packageToString(package, True)))


def call_back_sistem_saati(data):
    global sistem_saati_q
    sistem_saati_q = data.data
    #print sistem_saati_q.replace(".",",").replace("#","_")
    #WriteData(sistem_saati_q.replace(".",",").replace("#","_"))

def call_back_kilitlenme_verisi(data):

    global kilitlenme_q
    kilitlenme_q = data.data
    print "3",kilitlenme_q.replace(".",",").replace("#","_")

def createPackage(newPackage):
    global curPackageId
    package = dict(
        id = 0,
        lenght = 0,
        type = PackageTypes["Telemetri"],
        data = "",
        lastSended = 0
    )
    if newPackage is True:
        package["id"] = curPackageId
        package["lastSended"] = time.time()
        curPackageId = curPackageId + 1


    return package

def packageToConfig(package):
    package["type"] = PackageTypes["Config"]
    return package

def propertySizeAdjust(value,size):

    value = str(value)
    size = size
    string = ""
    limit = size - len(value)

    for index in range(size):
        string = string + "0"

    string = string + str(value)
    return string

def packageToString(package,add):
    if add is True:
        return propertySizeAdjust(package["id"],idSize) + propertySizeAdjust(package["lenght"],lenghtSize-2) + str(package["type"]) + package["data"]
    else:
        return str(package["id"]) + str(package["lenght"]) + str(package["type"]) + str(package["data"])


def packageToObj(package):

    if package["type"] == str(PackageTypes["Command"]):
        stringToCommand(package["data"])
    elif  package["type"] == str(PackageTypes["Location"]):
        print(package["data"])


# -------------- Package
def stringToCommand(string):
    telemetri = dict()

    if string == "TakeOff":
        telemetri["Command"] = "TakeOff"
    elif string == "Start":
        telemetri["Command"] = "Start"
    elif string == "HomeSweetHome":
        telemetri["Command"] = "HomeSweetHome"
    elif string == "PID":
        telemetri["Command"] = "PID"

    return telemetri

def stringToPackage(str):

    package = createPackage(False)
    index = 0
    package["id"] = str[index:idSize]
    index = idSize
    package["lenght"] = str[idSize:index+lenghtSize]
    index = index+lenghtSize
    package["type"] = str[index:index+typeSize]
    index = index+typeSize
    package["data"] = str[index:str.find('\n')]
    return package

    return package

def configTimeOut():
    print("")

def WriteData(string):

    print("----- Senden DATA ")
    print(packageToString(string,False) )
    ser.write(packageToString(string,False) + "\n" );
    #print("Triggered")


def writeStringData(string):
    print("----- Senden DATA ")
    print(string)
    ser.write(string + "\n")

def run():
    listen()

def listen():

    pub = rospy.Publisher('target_telemetry_data', String, queue_size=10)
    pub2 = rospy.Publisher('ucus_gorevi', String, queue_size=10)
    pub3 = rospy.Publisher('hiz_data', String, queue_size=10)

    while True:
        line = ser.readline()
        package = stringToPackage(line)
        
        print("\n--------------------" + package["id"] + "-------------------")
        print(line[:len(line)-1 ])

        if str(package["type"]) == str(PackageTypes["Config2"]):
            print "----- Package is Succesfull !!!!!"

            package = InBox[package["id"]]

            if InBox[package["id"]] is None:
                continue

            # ------- Location
            if package["type"] == str(PackageTypes["Location"]): # Target IHA
                target_iha = str(package["data"]).replace(",",".").replace("_",",")
                pub.publish(target_iha)
                 
                print "----- Drone Received Data"
                print target_iha

            # ------- Command
            elif package["type"] == str(PackageTypes["Command"]): # Komutlar

                TakeOff =0
                Start=0
                PID = 0
                HomeSweetHome = 0

                if str(package["data"]) == "TakeOff":
                    TakeOff = 1
                if str(package["data"]) == "Start":
                    Start = 1
                if str(package["data"]) == "PID":
                    PID = 1
                if str(package["data"]) == "HomeSweetHome":
                    HomeSweetHome = 1

                flight_command = str(TakeOff) + "," + str(Start) + "," +str(PID) + "," +str(HomeSweetHome)

                 #flight_command = str(package["data"]).replace(",",".").replace("_",",")
                pub2.publish(flight_command)
                
                print "----- Drone Received Data"
                print flight_command

            # ------- Speed
            elif package["type"]== str(PackageTypes["Speed"]): # Hiz komutlari
                speed_data = str(package["data"]).replace(",",".").replace("_",",")
                pub3.publish(speed_data)
                
                print "----- Drone Received Data"
                print speed_data

        elif str(package["type"]) == str(PackageTypes["Config"]):

            clone = package.copy()
            clone["type"] = PackageTypes["Config2"]
            config = packageToString(clone,False)

            WriteData(config)

        else:
            if package["id"] not in InBox:
                InBox[package["id"]] = package.copy()

            config = packageToConfig(package)
            WriteData(config)


if __name__ == '__main__':

    rospy.init_node('seri_haberlesme', anonymous=True)
    mavros.set_namespace('mavros')

    # string = call_back_iha_telemetry("47.3977413#8.5456072#0#358.8596#356.6365#1.1037#0.0222#76#1#0#0#0#0#0#9#45#50#247")
    rospy.Subscriber('/iha_telemetry', String, call_back_iha_telemetry)
    rospy.Subscriber('/sistem_saati', String, call_back_sistem_saati)
    rospy.Subscriber('/kilitlenme_verisi', String, call_back_kilitlenme_verisi)

    run()
    ##ros.spin()
