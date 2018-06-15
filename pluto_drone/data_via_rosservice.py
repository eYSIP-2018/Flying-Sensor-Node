#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
import rospy
import roslib
import queue
from std_msgs.msg import Float32,Float32MultiArray
from geometry_msgs.msg import Point

max_size=20
coordinate=[0.0,0.0,0.0]
data_x=queue.Queue(max_size)
data_y=queue.Queue(max_size)
data_z=queue.Queue(max_size)
count=0
sum_x=0.0
sum_y=0.0
sum_z=0.0
x1=[0.0,0.0,0.0]
class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		self.co = rospy.Publisher('/coord', Point, queue_size=5)
		#self.yaw_pub = rospy.Publisher('/yaw_publisher', Float64, queue_size=1)
		#self.pub=rospy.Publisher('/drone_command',PlutoMsg,queue_size=10)
		#self.arm()
		self.cmd=Point()
		#self.pub1=PlutoMsg()
		#self.arm()
		rospy.spin()
		
	def arm(self):
		self.pub1.rcRoll = 1500
		self.pub1.rcYaw = 1500
		self.pub1.rcPitch = 1500
		self.pub1.rcThrottle = 1000
		self.pub1.rcAUX4 = 1500
		self.pub.publish(self.pub1)
		rospy.sleep(0.1)

	def access_data(self, req):
		global data_x
		global data_y
		global data_z
		global count
		global sum_x
		global sum_y
		global sum_z
		global x1
		print "accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ)
		print "gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ)
		print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		print "altitude = " +str(req.alt)
		 
		#print "battery = " +str(req.battery)
		print "uwb_x = " +str(req.uwb_x)
		print "uwb_y = " +str(req.uwb_y)
		print "uwb_z = " +str(req.uwb_z)
		#print "temp ="+str(req.temp)
		x=req.uwb_x
		y=req.uwb_y
		z=req.uwb_z
		data_x.put(x)
		data_y.put(y)
		data_z.put(z)
		sum_x=sum_x+x*0.05
		sum_y=sum_y+y*0.05
		sum_z=sum_z+z*0.05
		count=count+1
		if(count>=max_size):
			x1[0]=sum_x
			x1[1]=sum_y
			x1[2]=sum_z
			self.cmd.x=x1[0]
			self.cmd.y=x1[1]
			self.cmd.z=x1[2]
			x=data_x.get()
			y=data_y.get()
			z=data_z.get()
			sum_x=sum_x-x*0.05
			sum_y=sum_y-y*0.05
			sum_z=sum_z-z*0.05
			#print count
			self.co.publish(self.cmd)
		#x.publish(req.uwb_x)
		#y.publish(req.uwb_y)
		#z.publish(req.uwb_z)
		
		
		
		#print "x_coordinate = "+str(req.uwb_x) #, "y_coordinate = "+str(req.uwb_y) , "z_coordinate = "+str(req.uwb_z)
		 
		rospy.sleep(.1)
		return PlutoPilotResponse(rcAUX2 =1500)
if __name__=="__main__":
	
	#y = rospy.Publisher('/y_coord',Float32, queue_size=5)
	#z = rospy.Publisher('/z_coord',Float32, queue_size=5)
	test = request_data()
		
