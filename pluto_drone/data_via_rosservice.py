#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
import rospy
import roslib
from queue import *
import time
from std_msgs.msg import Float32,Float32MultiArray
from geometry_msgs.msg import Point

max_size=20
coordinate=[0.0,0.0,0.0]
count=0
sum_x=0.0
sum_y=0.0
sum_z=0.0
x1=[0.0,0.0,0.0]
data_x=Queue(maxsize=20)
data_y=Queue(maxsize=20)
data_z=Queue(maxsize=20)

class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		self.co = rospy.Publisher('/coord', Point, queue_size=5)
		#self.roll=rospy.Publisher('/roll_sensor',Float32,queue_size=5)
		#self.pitch=rospy.Publisher('/pitch_sensor',Float32,queue_size=5)
		self.compl=rospy.Publisher('/Sensor_data',Point,queue_size=5)
		#self.accel=rospy.Publisher('/acc_sensor',Point,queue_size=5)
		#self.gyrosc=rospy.Publisher('/gyro_sensor',Point,queue_size=5)		
		#self.yaw_pub = rospy.Publisher('/yaw_publisher', Float64, queue_size=1)
		#self.pub=rospy.Publisher('/drone_command',PlutoMsg,queue_size=10)
		#self.arm()
		self.temperature=rospy.Publisher('/temperature',Float32,queue_size=5)
		self.cmd=Point()
		self.last=Point()
		#self.acc=Point()
		#self.gyro=Point()
		
		#self.pub1=PlutoMsg()
		#self.arm()
		self.list_x=[0]
		self.list_y=[0]
		self.list_z=[0]
		
		#complement
		self.K=0.98
		self.gyro_scaled_x=0.0
		self.gyro_scaled_y=0.0
		#self.gyro_scaled_z=0
		self.acc_scale_x=0.0
		self.acc_scale_y=0.0
		#self.acc_scale_z=0
		self.now_time=time.time()
	


		self.roll=0
		self.pitch=0
		self.prev_posx=0.0
		self.prev_posy=0.0
		self.prev_posz=0.0
		self.posx=0
		self.posy=0
		self.posz=0



		rospy.spin()

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
		#print "magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ)
		print "roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw)
		#print "altitude = " +str(req.alt)
		"""print "Sensor_data"
		print "x="+str(req.posx),"y="+str(req.posy),"z="+str(req.posz)"""
		print "temp=" + str(req.temp)
		self.roll=req.roll
		self.pitch=req.pitch
		self.temperature.publish(req.temp)
		x=int(req.posx/30)
		#y=int(req.posy/50)
		y=int(req.posy/20)
		#z=int(req.posz/20)
		z=int(req.posz/10)
		#print "x="+str(x),"y="+str(y),"z="+str(z)
		#self.cmd.x=int(x/40)
		#self.cmd.y=int(y/40)
		#self.cmd.z=int(z/10)
		data_x.put(x)
		data_y.put(y)
		data_z.put(z)
		"""self.list_x.append(x)
		self.list_z.append(z)
		self.list_y.append(y)"""
		sum_x=sum_x+x*0.05
		sum_y=sum_y+y*0.05
		sum_z=sum_z+z*0.05
		#print "unpopped"+str(self.list_x)
		count=count+1#len(self.list_y)
		if(count>=max_size):
			#print count
			self.posx=sum_x
			self.posy=sum_y
			self.posz=sum_z
			x=data_x.get()
			y=data_y.get()
			z=data_z.get()
			sum_x=sum_x-x*0.05
			sum_y=sum_y-y*0.05
			sum_z=sum_z-z*0.05
			#print self.cmd.x
			#print self.cmd.y
			#print self.cmd.z
			#self.co.publish(self.cmd)
			self.complement()
		self.prev_posz=self.posz
		self.prev_posy=self.posy
		self.prev_posx=self.posx		 
		rospy.sleep(.1)
		return PlutoPilotResponse(rcAUX2 =1500)
		
	def complement(self):

		change_posx=self.posx-self.prev_posx
		change_posy=self.posy-self.prev_posy
		change_posz=self.posz-self.prev_posz
		change_posy=change_posy*0.1+self.pitch*0.9
		change_posx=change_posx*0.1-self.roll*0.9

		self.cmd.x=self.prev_posx+change_posx
		self.cmd.y=self.prev_posy+change_posy
		self.cmd.z=self.posz
		print self.cmd

		self.co.publish(self.cmd)


		
if __name__=="__main__":
	test = request_data()
		
