#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def menu():
    print "Press"
    print "1: to set mode to GUIDED"
    print "2: to set mode to STABILIZE"
    print "3: to set mode to ARM the drone"
    print "4: to set mode to DISARM the drone"
    print "5: to set mode to TAKEOFF"
    print "6: to set mode to LAND"
    print "7: print GPS coordinates"
    

        
        
    

if __name__ == '__main__':
    rospy.init_node('yki', anonymous=True)
    
    rate = rospy.Rate(1)
    mavros.set_namespace('mavros')
    menu()
    pub = rospy.Publisher('yer_istasyonu', String, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped

    while (not rospy.is_shutdown():
    
        x = raw_input("Enter your input: ");
        pub.Publisher(x)
        rate.sleep()