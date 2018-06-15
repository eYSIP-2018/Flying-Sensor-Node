#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *

from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray, Point
from pid_tune.msg import PidTune


import rospy
import time

class pluto_pid():
	def __init__(self):
		rospy.init_node('drone_new')

		self.way_point = [120.0,78.0,31.0]
		# self.Kp_x = 0;self.Ki_x = 0;self.Kd_x = 0
		# self.Kp_y = 0;self.Ki_y = 0;self.Kd_y = 0
		# self.Kp_z = 0;self.Ki_z = 0;self.Kd_z = 0
		
		self.Kp_x = 18;self.Ki_x = 0;self.Kd_x = 175
		self.Kp_y = 20;self.Ki_y = 0;self.Kd_y = 200
		self.Kp_z = 40;self.Ki_z = 0;self.Kd_z = 0
		self.Kp_yaw = 0;self.Ki_yaw = 0;self.Kd_yaw = 0
		
		self.x_cord_prev_err = 0
		self.y_cord_prev_err = 0
		self.z_cord_prev_err = 0
		self.yaw_prev_err = 0
		self.err_i_x_list = [0]
		self.err_i_x = 0 
		self.err_i_y_list = [0]
		self.err_i_y = 0 
		self.err_i_z_list = [0]
		self.err_i_z = 0 


		self.cord = [120.0,78.0,31.0]
		self.prev_cord = [0,0,0]
		self.yaw = 0
		self.land_alt_dec = 0 
		self.land_list_flag = 0
		

		self.key_value =1
		self.rate = rospy.Rate(10)
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500 # -10 drone intrensic parameter
		self.cmd.rcPitch = 1500 # 20 drone intrensic parameter
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000

		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#self.sub_pose = rospy.Subscriber('/whycon/poses',PoseArray,self.get_data)
		#self.yaw_pose = rospy.Subscriber('/yaw_publisher',Float64,self.yaw_data)
		rospy.Subscriber('/input_key', Int16, self.indentify_key )
		#just like whycon getting data
		rospy.Subscriber('/coord',Point,self.get_data)

		#self.pathStatus = rospy.Subscriber('/pathStatus',Int16,self.get_stat)
		
		#self.waypoint = rospy.Subscriber('/way_point',Point,self.set_way)
		
		self.roll_err_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.pitch_err_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.alt_err_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)

		#self.pid_roll = rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_data)
		#self.pid_pitch = rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_data)
		#self.pid_alt = rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_data)
		#self.pid_yaw = rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_data_pid)
		
		#self.arm()
		#self.disarm()

	def arm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.1)
		
	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def indentify_key(self, msg):
		self.key_value = msg.data
		

	def get_data(self,msg):
		self.cord[0] = msg.x
		self.cord[1] = msg.y
		self.cord[2] = msg.z
		

	def yaw_data(self,yaw):
		self.yaw = yaw.data

	
	"""def pitch_data(self,pitch):
		self.Kp_x = pitch.Kp
		self.Kd_x = pitch.Kd
		self.Ki_x = pitch.Ki
		
		

	def roll_data(self,roll):
		self.Kp_y = roll.Kp
		self.Kd_y = roll.Kd
		self.Ki_y = roll.Ki

	def yaw_data_pid(self,yaw_pid):
		self.Kp_yaw = yaw_pid.Kp
		self.Kd_yaw = yaw_pid.Kd
		self.Ki_yaw = yaw_pid.Ki

	def altitude_data(self,altitude):
		self.Kp_z = altitude.Kp
		self.Kd_z = altitude.Kd
		self.Ki_z = altitude.Ki"""

	"""def set_way(self,point):
		if self.stat != -1:
			self.way_point = [point.x,point.y,point.z]
		else
			if self.land_list_flag == 0:
				self.land_z = range(point.z,.01,30)
				self.land_list_flag = 1
			
			self.way_point[3] = self.land_z[self.land_alt_dec]
			
			if self.land_alt_dec < len(self.land_z):
				self.land_alt_dec = self.land_alt_dec + 1

	def get_stat(self,data):
		self.stat = data.data"""
	
	def cap(self,value):
		
		if (value > 2000):
			value = 2000
			
		elif (value < 1000):
			value = 1000
		
		return value

	def pid_calc(self):
		if self.key_value == 70:
				self.arm()
		if self.key_value == 0:         
				self.disarm()

		
		y_cord_err = self.way_point[0]-self.cord[0]
		x_cord_err = self.way_point[1]-self.cord[1]
		z_cord_err = self.way_point[2]-self.cord[2] 
		print y_cord_err,x_cord_err,z_cord_err 
		yaw_err = self.yaw + 0.4 # value as found according to positive x direction of world frame
		 
	
		self.err_i_x_list.append(x_cord_err)
		if len(self.err_i_x_list) > 20:
			self.err_i_x_list.pop(0)
		self.err_i_x = sum(self.err_i_x_list)

		x_add = self.Kp_x * x_cord_err + self.Kd_x * (x_cord_err - self.x_cord_prev_err) + ( self.Ki_x * self.err_i_x ) 
		self.cmd.rcPitch = 1500 + x_add
		#print x_add,'\t'
		self.pitch_err_pub.publish(x_cord_err)
		self.cmd.rcPitch = self.cap(self.cmd.rcPitch)
	
			
		self.err_i_y_list.append(y_cord_err)
		if len(self.err_i_y_list) > 3:
			self.err_i_y_list.pop(0)
		self.err_i_y = sum(self.err_i_y_list)
		
		y_add = self.Kp_y * y_cord_err + self.Kd_y * (y_cord_err - self. y_cord_prev_err) + ( self.Ki_y * self.err_i_y ) 
		self.cmd.rcRoll = 1500 + y_add
		self.cmd.rcRoll = self.cap(self.cmd.rcRoll)
		self.roll_err_pub.publish(y_cord_err)
		#print y_add,'\t'
		
		
		self.err_i_z_list.append(z_cord_err)
		if len(self.err_i_z_list) > 3:
			self.err_i_z_list.pop(0)
		
		self.err_i_z = sum(self.err_i_z_list) 
		
		z_add = (self.Kp_z * z_cord_err) + ( self.Kd_z * ( z_cord_err - self.z_cord_prev_err)) + ( self.Ki_z * self.err_i_z ) 
		self.cmd.rcThrottle = 1500 + z_add
		self.cmd.rcThrottle = self.cap(self.cmd.rcThrottle)
		self.alt_err_pub.publish(z_cord_err)
		#print z_add,'\n'
	

		yaw_add = self.Kp_yaw * yaw_err + self.Kd_yaw * (yaw_err - self.yaw_prev_err)
		self.cmd.rcYaw = 1500 # - yaw_add
		self.cmd.rcYaw = self.cap(self.cmd.rcYaw)
		
		self.x_cord_prev_err = x_cord_err
		self.y_cord_prev_err = y_cord_err
		self.z_cord_prev_err = z_cord_err
		self.yaw_prev_err = yaw_err
		
		self.command_pub.publish(self.cmd)

		 
if __name__ == '__main__':
	do=pluto_pid()
	while not rospy.is_shutdown():
		do.pid_calc()
		do.rate.sleep()
	
# 	rospy.spin()
