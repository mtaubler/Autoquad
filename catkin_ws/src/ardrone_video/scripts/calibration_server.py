#!/usr/bin/env python

import rospy
import time
import sys
import math
from std_msgs.msg import *
from ardrone_autonomy.msg import Navdata
from ardrone_video.msg import Noisy
from ardrone_video.srv import Calibrate
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

move_pub = rospy.Publisher('cmd_vel', Twist)
coord = [0,0,0]
last_coord = [0,0,0]
rot = 0

def handle_calibrate(req):
	noisy_s = Subscriber('slam/noisy', Noisy, noisy_callback)
	options = 	{'standard' : standard,
				 #'horizontal' : horizontal,
				 #'rotation' : rotation,
				 #'vertical' : vertical
				 }
	options[req.mode]()
	noisy_s.unsubscribe()
	return CalibrateResponse(1)


def noisy_callback(data):
	global noisy_data = data
	coord[0] = data.x - last_coord[0]
	coord[1] = data.y - last_coord[0]
	coord[2] = data.z - last_coord[0]
	rot = data.rot

frame_count = 0

def standard():
	for i in range(8):
		stop_drone()
		move_drone()
		stop_drone()
		#handle image data
		rotate_drone(45)
		stop_drone()
		#handle image data
	
	time.sleep(2)

def move_drone():
	time.sleep(5)
	balanced = False
	delta_y = 1
	delta_x = 0
	count = 0
	while (not balanced):		
		speed_y = .05*(1-coord[1])
		speed_x = .05*(0-coord[0])
		move_pub.publish(Twist(Vector3(speed_x,speed_y,0),Vector3(0,0,0)))
		if ((speed_x**2+speed_y**2)**(0.5) < .01):
			count+=1
			if (count >= 50):
				balanced = True
		else:
			count = 0
		time.sleep(.05)

def rotate_drone(theta):
	time.sleep(5)
	target = (rot+180+theta)%360-180
	balanced = False
	count = 0
	while (not balanced):		
		move_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,.25)))
		if (fabs(target-rot) < 1):
			balanced = True

def stop_drone():
	move_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	#take new images
	image_sub = rospy.Subscriber('/ardrone/front/image_raw',Image,image_callback)
	time.sleep(3)
	frame_count = 0
	last_coord[0] = noisy_data.x
	last_coord[1] = noisy_data.y
	last_coord[2] = noisy_data.z


def calibrate_server():
	rospy.init_node('calibration_server', anonymous=True)
	s = rospy.Service('calibrate', Calibrate, handle_calibrate)
	print "Ready to calibrate."
	rospy.spin()

	

if __name__ == '__main__':
	try:
		calibrate_server()
	except rospy.ROSInterruptException:
		pass