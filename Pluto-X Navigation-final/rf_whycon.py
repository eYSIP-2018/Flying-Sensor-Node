#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *

from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray, Point
from pid_tune.msg import PidTune
from std_msgs.msg import Char,Int16

#import sys, select, termios, tty


import rospy
import time
import math

class pluto_pid():
	def __init__(self):
		rospy.init_node('drone_new')

		self.way_point_rf = [25.0,11.0,5.0]
		self.way_point_why=[[1.95,3.15,17.5],[0.7,-1.1,22.5]]
		# self.Kp_x = 0;self.Ki_x = 0;self.Kd_x = 0
		# self.Kp_y = 0;self.Ki_y = 0;self.Kd_y = 0
		# self.Kp_z = 0;self.Ki_z = 0;self.Kd_z = 0
		
		self.Kp_x = 18;self.Ki_x = 0;self.Kd_x = 175
		self.Kp_y = 20;self.Ki_y = 0;self.Kd_y = 200
		self.Kp_z = 40;self.Ki_z = 0;self.Kd_z = 0
		self.Kp_yaw = 0;self.Ki_yaw = 0;self.Kd_yaw = 0
		
		self.x_cord_prev_err = 0.0
		self.y_cord_prev_err = 0.0
		self.z_cord_prev_err = 0.0
		self.yaw_prev_err = 0.0
		self.err_i_x_list = [0]
		self.err_i_x = 0 
		self.err_i_y_list = [0]
		self.err_i_y = 0 
		self.err_i_z_list = [0]
		self.err_i_z = 0 
		self.flag=1


		self.cord = [0.0,0.0,0.0]
		self.prev_cord = [0,0,0]
		self.yaw = 0
		self.land_alt_dec = 0 
		self.land_list_flag = 0
		

		self.key_value = 0
		self.rate = rospy.Rate(10)
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500 # -10 drone intrensic parameter
		self.cmd.rcPitch = 1500 # 20 drone intrensic parameter
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1000
		self.cmd.rcAUX2 = 1000
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.index=0

		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		rospy.Subscriber('/whycon/poses',PoseArray,self.get_data_whycon)
		#just like whycon getting data
		rospy.Subscriber('/coord',Point,self.get_data)
		
		"""self.roll_err_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.pitch_err_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.alt_err_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		"""
		self.arm()
		
	
	def arm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.1)

	def land(self):
		print "land"
		start_time=time.time()
		current_time=0.0
		cond=True
		while(cond):
			self.cmd.rcRoll = 1500
			self.cmd.rcYaw = 1500
			self.cmd.rcPitch = 1500
			self.cmd.rcThrottle = 1490
			self.command_pub.publish(self.cmd)
			current_time=time.time()
			if(current_time-start_time>2):
				cond=False
		self.disarm()
		sys.exit(1)


	def disarm(self):
		print "charging on"
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def get_data(self,msg):
		if self.flag==1:
			self.cord[0] = msg.x
			self.cord[1] = msg.y
			self.cord[2] = msg.z
			print "rf"
		
	def get_data_whycon(self,msg):
		if self.flag==1:
			self.flag=0
			print "whycon"
			#Waypoint for whycon is different uncomment and replace x y z
			#self.way_point=[x,y,z]
			self.x_cord_prev_err = 0
			self.y_cord_prev_err = 0
			self.z_cord_prev_err = 0
		self.cord[0] = msg.poses[0].position.x
		self.cord[1] = msg.poses[0].position.y
		self.cord[2] = msg.poses[0].position.z
	
	def cap(self,value):
		
		if (value > 1510):
			value = 1510
			
		elif (value < 1480):
			value = 1480
		
		return value
	def cap_why(self,value):
		
		if (value > 2000):
			value = 2000
			
		elif (value < 1000):
			value = 1000
		
		return value
		
	def cap_z(self,value):
		
		if (value > 1800):
			value = 1800
			
		elif (value < 1400):
			value = 1400
		
		return value
		
	def cap_why_z(self,value):
		
		if (value > 2000):
			value = 2000
			
		elif (value < 1000):
			value = 1000
		
		return value

	def pid_calc(self):
		if self.flag == 1:
			self.pid_rf()
		else:
			self.pid_whycon()
		
	def pid_whycon(self):
		x_cord_err = self.cord[0]-self.way_point_why[self.index][0]
		y_cord_err = self.cord[1]-self.way_point_why[self.index][1]
		z_cord_err = self.cord[2]-self.way_point_why[self.index][2]
		if(math.sqrt(x_cord_err*x_cord_err+ y_cord_err*y_cord_err+z_cord_err*z_cord_err)<1.4):
			print x_cord_err,y_cord_err,z_cord_err
			print self.index
			if (self.index==1):
				self.disarm()
			self.index=self.index+1
			 
		yaw_err = self.yaw + 0.4 # value as found according to positive x direction of world frame
		 
	
		self.err_i_x_list.append(x_cord_err)
		if len(self.err_i_x_list) > 20:
			self.err_i_x_list.pop(0)
		self.err_i_x = sum(self.err_i_x_list)

		x_add = self.Kp_x * x_cord_err + self.Kd_x * (x_cord_err - self.x_cord_prev_err) + ( self.Ki_x * self.err_i_x ) 
		self.cmd.rcPitch = 1500 - x_add
		#print x_add,'\t'
		#self.pitch_err_pub.publish(x_cord_err)
		self.cmd.rcPitch = self.cap_why(self.cmd.rcPitch)
	
			
		self.err_i_y_list.append(y_cord_err)
		if len(self.err_i_y_list) > 3:
			self.err_i_y_list.pop(0)
		self.err_i_y = sum(self.err_i_y_list)
		
		y_add = self.Kp_y * y_cord_err + self.Kd_y * (y_cord_err - self. y_cord_prev_err) + ( self.Ki_y * self.err_i_y ) 
		self.cmd.rcRoll = 1500 - y_add
		self.cmd.rcRoll = self.cap_why(self.cmd.rcRoll)
		#self.roll_err_pub.publish(y_cord_err)
		#print y_add,'\t'
		
		
		self.err_i_z_list.append(z_cord_err)
		if len(self.err_i_z_list) > 3:
			self.err_i_z_list.pop(0)
		self.err_i_z = sum(self.err_i_z_list) 
		
		
		z_add = (self.Kp_z * z_cord_err) + ( self.Kd_z * ( z_cord_err - self.z_cord_prev_err)) + ( self.Ki_z * self.err_i_z ) 
		self.cmd.rcThrottle = 1500 + z_add
		self.cmd.rcThrottle = self.cap_why_z(self.cmd.rcThrottle)
		#self.alt_err_pub.publish(z_cord_err)
		#print z_add,'\n'
		
		yaw_add = self.Kp_yaw * yaw_err + self.Kd_yaw * (yaw_err - self.yaw_prev_err)
		self.cmd.rcYaw = 1500 # - yaw_add
		self.cmd.rcYaw = self.cap(self.cmd.rcYaw)
		
		self.x_cord_prev_err = x_cord_err
		self.y_cord_prev_err = y_cord_err
		self.z_cord_prev_err = z_cord_err
		self.yaw_prev_err = yaw_err
		
		self.command_pub.publish(self.cmd)
		
	def pid_rf(self):
		y_cord_err = self.way_point_rf[0]-self.cord[0]
		x_cord_err = self.way_point_rf[1]-self.cord[1]
		z_cord_err = self.way_point_rf[2]-self.cord[2] 
		#print y_cord_err,x_cord_err,z_cord_err 
		yaw_err = self.yaw + 0.4 # value as found according to positive x direction of world frame
		 
	
		self.err_i_x_list.append(x_cord_err)
		if len(self.err_i_x_list) > 20:
			self.err_i_x_list.pop(0)
		self.err_i_x = sum(self.err_i_x_list)

		x_add = self.Kp_x * x_cord_err + self.Kd_x * (x_cord_err - self.x_cord_prev_err) + ( self.Ki_x * self.err_i_x ) 
		self.cmd.rcPitch = 1500 - x_add
		#print x_add,'\t'
		#self.pitch_err_pub.publish(x_cord_err)
		self.cmd.rcPitch = self.cap(self.cmd.rcPitch)
	
			
		self.err_i_y_list.append(y_cord_err)
		if len(self.err_i_y_list) > 3:
			self.err_i_y_list.pop(0)
		self.err_i_y = sum(self.err_i_y_list)
		
		y_add = self.Kp_y * y_cord_err + self.Kd_y * (y_cord_err - self. y_cord_prev_err) + ( self.Ki_y * self.err_i_y ) 
		self.cmd.rcRoll = 1500 - y_add
		self.cmd.rcRoll = self.cap(self.cmd.rcRoll)
		#self.roll_err_pub.publish(y_cord_err)
		#print y_add,'\t'
		
		
		self.err_i_z_list.append(z_cord_err)
		if len(self.err_i_z_list) > 3:
			self.err_i_z_list.pop(0)
		
		self.err_i_z = sum(self.err_i_z_list) 
		
		z_add = (self.Kp_z * z_cord_err) + ( self.Kd_z * ( z_cord_err - self.z_cord_prev_err)) + ( self.Ki_z * self.err_i_z ) 
		self.cmd.rcThrottle = 1500 + z_add
		self.cmd.rcThrottle = self.cap_z(self.cmd.rcThrottle)
	
		#self.alt_err_pub.publish(z_cord_err)
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
	#settings = termios.tcgetattr(sys.stdin)
	do=pluto_pid()
	while not rospy.is_shutdown():
		do.pid_calc()
		do.rate.sleep()
	
# 	rospy.spin()
