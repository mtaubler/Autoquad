#!/usr/bin/env python
import sys
import math
import time
import rospy
import os
import cv2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from ardrone_autonomy.msg import Navdata
from ardrone_video.msg import Noisy

class Data_Handler:
	def __init__(self):
		self.nav_sub = rospy.Subscriber('ardrone/navdata', Navdata, self.nav_callback)
		self.noisy_pub = rospy.Publisher('ardrone_video/noisy', Noisy)

		self.coord = [0,0,0]
		self.vel = [0,0,0]
		self.cur_time = 0
		self.old_time = 0
		self.frame_rate = 50

	def nav_callback(self,data):
		self.cur_time = data.tm
		self.vel = [data.vx,data.vy,float(data.altd / 1000.0)]
		if (self.old_time != 0):
			self.calc_cur_pos()

		location = Vector3(self.coord[0],self.coord[1],self.coord[2])
		rotation = Vector3(data.rotX,data.rotY,data.rotZ)
		dLocation = Vector3(data.vx,data.vy,data.vz)
		dRotation = Vector3(0,0,0)
		pose = Twist(location,rotation)
		dPose = Twist(dLocation,dRotation)
		self.noisy_pub.publish(pose,dPose)

		self.old_time = self.cur_time

	# last in meters, vel in mm/s, delta in microseconds
	def calc_cur_pos(self):
		delta = self.cur_time - self.old_time
		for i in range(2):
			self.coord[i] = self.coord[i] + delta * self.vel[i] / 10**9
		self.coord[2] = self.vel[2]

def main():
	rospy.init_node('noisy_coord_handler',anonymous=True)
	Data_Handler()
	rospy.spin()
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

