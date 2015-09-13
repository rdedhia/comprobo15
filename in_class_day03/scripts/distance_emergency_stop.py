#!/usr/bin/env python

"""This script is used to move the Neato forward until it senses that
one of the lasers is detecting something close to it. Also preserves
the bump sensor estop just in case shit gets fucked"""

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import Twist
import rospy

# Creating a robot class to represent the speed
class Robot(object):
	def __init__(self, speed):
		self.speed = speed
		self.ranges = []
		self.averagerange = 0
		self.bumpflag = False
	def change_speed(self, speed):
		self.speed = speed
	def change_bumpflag(self, bumpflag):
		self.bumpflag = bumpflag
	def change_ranges(self, ranges):
		self.ranges = ranges[0:30] + ranges[330:]
		for item in self.ranges:
			if item == 0:
				item = 5.0
		self.averagerange = sum(self.ranges)/len(self.ranges)
		if self.averagerange < 0.6 or self.bumpflag == True:
			self.change_speed(0)
		else:
			self.change_speed(1)

def estop(msg):
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bumpflag(True)
	else:
		neato.change_bumpflag(False)

def distance_estop(msg):
	neato.change_ranges(msg.ranges[0:360])
	

# initialize instance of Robot class
neato = Robot(speed=0)

rospy.init_node('emergency_stop')
rospy.Subscriber('/bump', Bump, estop)
rospy.Subscriber('/scan', LaserScan, distance_estop)

header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

while not rospy.is_shutdown():
	twist = Twist()
	twist.linear.x = neato.speed
	pub.publish(twist)

twist = Twist()
twist.linear.x = 0
pub.publish(twist)
