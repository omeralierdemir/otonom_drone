#!/usr/bin/env python

import rospy
import mavros
# from threading import Thread, Timer
# import time
import threading
import time
import math
# from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import *
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# msg = PositionTarget()

lastErrorY = 0
lastErrorR = 0
pid_value = None

yaw_degrees = roll = pitch = yaw = 0.0  # bu hata sebebi olabilir


def call_back(data):
    global lastErrorY
    global lastErrorR
    global pid_value

    XY = data.data.split(" ")
    print data.data , XY[0]
    if XY[0] != "null":

        XY = data.data.split(" ")

        x = float(XY[0])
        y = float(XY[1])
        r = float(XY[2])
        if r >= 10:
            # -----------------------
            errorY = 213 - y

            hizYukselme = 0.75
            kp =  hizYukselme / 213
            kd =  hizYukselme / (213 + hizYukselme * (35)) 

            turevY = errorY - lastErrorY
            yukselmeY = errorY * kp + turevY * kd
            # msg.velocity.z = yukselmeY
            axis_z = str(yukselmeY)

            lastErrorY = errorY

            # -----------------------

            errorR = 55 - r

            kpR = 1.0 / 80.0
            kdR = 1.0 / 100.0

            turevR = errorR - lastErrorR
            yaklasma = errorR * kpR + turevR * kdR
            # msg.velocity.y = yaklasma
            axis_r = str(yaklasma)
            lastErrorR = errorR

            # -----------------------

            errorX = 300 - x

            if errorX < 30 and errorX > -30:
                errorX = 0

            yaw_target = yaw_degrees + (errorX * 35) / 300

            if yaw_target >= 360:
                yaw_target = yaw_target - 360

            if yaw_target < 0:
                yaw_target = yaw_target + 360
            # print "r = ", r, " errorX = ", errorX , " yaw_degrees " , yaw_degrees, " yaw_target", yaw_target
            print"y = ", y, " errorY = ", errorY, " kP*errorY :", errorY * kp, " turevY*kd :", turevY * kd, "yukselme hizi: ", yukselmeY

            # msg.yaw = float(yaw_target*math.pi/180.0)
            yaw = float(yaw_target * math.pi / 180.0)
            yaw = str(yaw)

            pid_value = axis_z + "," + axis_r + "," + yaw
        # print pid_value
        else:
            pid_value = "null"


    else:
        pid_value = "null"


def get_rotation(msg):
    global yaw_degrees, roll, pitch, yaw
    #    print msg.pose.pose.orientation
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw_degrees = yaw * 180.0 / math.pi
    if (yaw_degrees < 0):
        yaw_degrees += 360.0
    # print "yaw_degrees HAS " , yaw_degrees


if __name__ == '__main__':

    rospy.init_node('pid', anonymous=True)
    rate = rospy.Rate(20)
    mavros.set_namespace('/uav1/mavros')
    rospy.wait_for_service('/uav1/mavros/cmd/arming')

    set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)

    pub = rospy.Publisher('get_pid', String, queue_size=10)

    try:

        sub = rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, get_rotation)

        rospy.Subscriber('konum', String, call_back)  # kamera goruntusunden x,y kordinatlari

        while not rospy.is_shutdown():
            pub.publish(pid_value)

            rate.sleep()






    except rospy.ROSInterruptException:
        pass

