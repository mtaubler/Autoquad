#!/usr/bin/env python
import sys
import math
import time
import rospy
import os
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from ardrone_video.msg import Noisy
from ardrone_video.msg import Objective

class Vector:
	def __init__(self,x,y):
		self.x = x
		self.y = y

class Mission_Driver:
	def __init__(self):
		self.mission_sub = rospy.Subscriber('ardrone_video/mission', String, self.command_decoder)
		self.coord_sub = rospy.Subscriber('ardrone_video/noisy', Noisy, self.noisy_callback)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist)
		self.takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty)
		self.land_pub = rospy.Publisher('ardrone/land', Empty)
		self.tum_pub = rospy.Publisher('tum_ardrone/com', String)

		self.position = Vector3(0,0,0)
		self.rotation = Vector3(0,0,0)
		self.dPosition = Vector3(0,0,0)
		self.dRotation = Vector3(0,0,0)

		self.tum_position = Vector3(0,0,0)
		self.tum_rotation = Vector3(0,0,0)
		self.tum_dPosition = Vector3(0,0,0)
		self.tum_dRotation = Vector3(0,0,0)

		self.commands = []
		self.executing = False

		cur_dir = str(os.path.realpath(__file__))
		dir_tree = cur_dir.split('/')
		dir_tree = dir_tree[0:len(dir_tree)-2]
		dir_tree.append('data')
		dir_tree = '/'.join(dir_tree)

		self.f = open(dir_tree+'/data.txt', 'w')

	def noisy_callback(self,data):
		self.position = data.pose.linear
		self.rotation = data.pose.angular
		self.dPosition = data.dPose.linear
		self.dRotation = data.dPose.angular

	def command_decoder(self,data):
		commands = data.data.split('\n')
		command = commands.pop(0)
		if command == 'noisy':
			self.run_noisy_driver(commands)
		elif command == 'manual':
			self.run_manual(commands)
		elif command == 'tum':
			self.run_tum_driver(commands)

	def run_noisy_driver(self, commands):
		self.takeoff_pub.publish(Empty())
		self.f.write("Autopilot mode enabled...\nPath:\n")
		for c in commands:
			if c != ' ' and c != '':
				coordinate = [float(e) for e in c.split(' ')]
				self.f.write(str(coordinate[0]) + ', ' + str(coordinate[1]) + '\n')
				vector = Vector(coordinate[0],coordinate[1])
				self.commands.append(vector)

		time.sleep(5)
		if not self.executing:
			self.executing = True
			for c in self.commands:
				self.f.write("Executing command (" + str(c.x) + ", " + str(c.y) + ")...\n")
				displacementX = c.x - self.position.x
				displacementY = c.y - self.position.y
				self.noisy_move_drone(c.x,c.y,displacementX,displacementY)
				self.f.write("Command complete.\n")
					
			self.executing = False
			self.land_pub.publish(Empty())
			
	def noisy_move_drone(self,x,y,sum_x,sum_y):
		balanced = False
		count = 0
		while (not balanced):
			error_x = x-self.position.x
			error_y = y-self.position.y

			self.f.write("Error from target is (" + str(error_x) + ", " + str(error_y) + ")\n")

			if error_x > 1:
				speed_x = .2*error_x / sum_x
			else:
				speed_x = .2*error_x

			if error_y > 1:
				speed_y = .2*error_y / sum_y
			else:
				speed_y = .2*error_y

			self.vel_pub.publish(Twist(Vector3(speed_x,speed_y,0),Vector3(0,0,0)))
			if ((error_x**2+error_y**2) < .0025):
				count+=1
				if (count >= 20):
					balanced = True
			else:
				count = 0
			time.sleep(.05)

	def run_manual(self,commands):
		c = commands.pop(0)
		if c == 'north':
			self.vel_pub.publish(Twist(Vector3(0.2,0,0),Vector3(0,0,0)))
		elif c == 'south':
			self.vel_pub.publish(Twist(Vector3(-0.2,0,0),Vector3(0,0,0)))
		elif c == 'east':
			self.vel_pub.publish(Twist(Vector3(0,-0.2,0),Vector3(0,0,0)))
		elif c == 'west':
			self.vel_pub.publish(Twist(Vector3(0,0.2,0),Vector3(0,0,0)))
		elif c == 'stop':
			self.vel_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	def run_tum_driver(self,commands):
		self.f.write('tum initialized...\n')
		tum_commands = []
		tum_commands.append('c autoInit 500 800 4000 0.5')
		tum_commands.append('c setReference $POSE$')
		tum_commands.append('c setInitialReachDist 0.2')
		tum_commands.append('c setStatWithinDist 0.2')
		tum_commands.append('c setStayTime 3')
		tum_commands.append('c lockScaleFP')
		for c in commands:
			if c != '':
				tum_commands.append('c goto ' + c + ' 0 0')
		tum_commands.append('c start')
		tum_commands.append('f reset')

		for t in tum_commands:
			self.f.write(t + '\n')
			self.tum_pub.publish(String(t))



def main():
	rospy.init_node('mission_driver',anonymous=True)
	Mission_Driver()
	rospy.spin()
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass