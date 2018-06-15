#!/usr/bin/env python
from plutodrone.srv import *
import rospy
import roslib
from std_msgs.msg import Char,Int16

alti = 0
class altitude():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('alt_hold')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		rospy.spin()

	def access_data(self, req):
		 global alti
		 print "altitude = " +str(req.alt)
		 alti.publish(req.alt)
		 rospy.sleep(.1)
		 return PlutoPilotResponse(rcAUX2 =1500)

if __name__=="__main__":
	alti = rospy.Publisher('/altitude',Int16, queue_size=5)
	test = altitude()
		
