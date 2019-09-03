#!/usr/bin/env python

import rospy
import mavros
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *


def create_waypoint():
    
	waypoint_clear_client()
	wl = []
	"""
	wp = Waypoint()

	wp.frame = 3
	wp.command = 22  # takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # takeoff altitude
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = 47.3971396
	wp.y_long = 8.5452028
	wp.z_alt = 4.0
	wl.append(wp)
    """
    
	
	wp = Waypoint() 

	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = True
	wp.autocontinue = True
	wp.param1 = 0  # delay 
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = 47.3977417
	wp.y_long = 8.5455943
	wp.z_alt = 4.0
	wl.append(wp)

    
	wp = Waypoint()

	wp.frame = 3
	wp.command = 19  # Navigate to waypoint.
	wp.is_current = True
	wp.autocontinue = False
	wp.param1 = 0  # delay
	wp.param2 = 0
	wp.param3 = 1
	wp.param4 = 0
	wp.x_lat = 47.3977817
	wp.y_long = 8.5455943
	wp.z_alt = 4.0
	wl.append(wp)
	
	
	'''
	wp = Waypoint()

	wp.frame = 3
	wp.command = 19  # takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # takeoff altitude
	wp.param2 = 0
	wp.param3 = 1
	wp.param4 = 0
	wp.x_lat = 47.3975396
	wp.y_long = 8.5456028
	wp.z_alt = 6.0
	wl.append(wp)

	wp = Waypoint()

	wp.frame = 3
	wp.command = 19  # takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # takeoff altitude
	wp.param2 = 0
	wp.param3 = 1
	wp.param4 = 0
	wp.x_lat = 47.3976396
	wp.y_long = 8.5457028
	wp.z_alt = 4.0
	wl.append(wp)

	wp = Waypoint()

	wp.frame = 3
	wp.command = 19  # takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0 # takeoff altitude
	wp.param2 = 0
	wp.param3 = 1
	wp.param4 = 0
	wp.x_lat = 47.3977396
	wp.y_long = 8.5458028
	wp.z_alt = 6.0
	wl.append(wp)
	'''

	print(wl)
	
	try:
	    service = rospy.ServiceProxy('/uav1/mavros/mission/push', WaypointPush, persistent=True)
	    service(start_index=0, waypoints=wl)
	    

	    '''
	    if service.call(wl).success: #burasi belki list istiyordur. birde oyle dene
	        print 'write mission success'
	    else:
	        print 'write mission error'
	    '''
	except rospy.ServiceException, e:
	    print "Service call failed: %s" % e

	



def waypoint_clear_client():
        try:
            response = rospy.ServiceProxy('/uav1/mavros/mission/clear', WaypointClear)
            return response.call().success
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False


if __name__ == '__main__':
	rospy.init_node('waypoint_node', anonymous=True)
	pub = rospy.Publisher('global',String,queue_size=10)
	konum = "suan buradasin" # to be used later


	rate = rospy.Rate(20)
	mavros.set_namespace('/uav1/mavros')
	rospy.wait_for_service('/uav1/mavros/cmd/arming')



	try:
		armService = rospy.ServiceProxy('/uav1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException, e:
	 	pass
	pub.publish(konum)
	create_waypoint()