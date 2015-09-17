#!/usr/bin/env python

"""This script is used to move the Neato forward until it senses that
one of the lasers is detecting something close to it. Also preserves
the bump sensor estop just in case shit gets fucked"""

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import Twist
import rospy
import time

# Creating a robot class to represent the speed
class Robot(object):
	def __init__(self, speed):
		self.speed = speed
		self.angular_speed = 0
		self.ranges = []
		self.average_ranges = []
		self.bump_flag = False
	def change_speed(self, speed):
		self.speed = speed
	def change_angular_speed (self, angular_speed):
		self.angular_speed = angular_speed
	def change_bump_flag(self, bump_flag):
		self.bump_flag = bump_flag
	def change_ranges(self, ranges):
		self.ranges = []
		average_ranges = []

		for item in ranges:
			if item == 0.0:
				self.ranges.append(5.0)
			else:
				self.ranges.append(item)
		# print "\nRanges\n"
		# print self.ranges
		for i in range(0, 360, 10):
			total = self.ranges[i:i+10]
			average = sum(total) / float(len(total))
			average_ranges.append(average)
		self.average_ranges = average_ranges
		# print "\nAverage Ranges\n"
		# print self.average_ranges

def estop(msg):
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bump_flag(True)
	else:
		neato.change_bump_flag(False)

def distance_estop(msg):
	# print "Running distance_estop"
	neato.change_ranges(msg.ranges[0:360])

def turn_control(initialpart):
	average_ranges = neato.average_ranges
	try:
		min_index = average_ranges.index(min(average_ranges))
	except ValueError:
		print average_ranges
	if min_index > 9:
		speed = (min_index - 8) / 10.0
		neato.change_angular_speed(speed)
	elif min_index < 8:
		neato.change_angular_speed(1)
	else:
		initialpart = True
	return min_index, initialpart

def forward_control():
	average_ranges = neato.average_ranges
	min_index = average_ranges.index(min(average_ranges))
	if min_index > 9 and min_index < 27:
		# turn left
		neato.change_angular_speed((min_index - 8) / 5.0)
	elif min_index >= 27:
		# turn right quickly
		neato.change_angular_speed(-2)
	elif min_index < 8:
		# turn right slowly
		neato.change_angular_speed((min_index - 8) / 5.0)
	else:
		neato.change_angular_speed(0)
	return min_index

# initialize instance of Robot class
neato = Robot(speed=0)
initialpart = False

rospy.init_node('emergency_stop')
rospy.Subscriber('/bump', Bump, estop)
rospy.Subscriber('/scan', LaserScan, distance_estop)

header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

time.sleep(2)
cur_time = time.time()
prev_time = cur_time

while not initialpart:
	min_index, initialpart = turn_control(initialpart)
	cur_time = time.time()
	if cur_time - prev_time > 1:
		print neato.average_ranges
		print initialpart
		print min_index
		print neato.angular_speed
		print '\n'
		prev_time = cur_time
	twist = Twist()
	twist.angular.z = neato.angular_speed
	pub.publish(twist)

neato.change_speed(0.2)
while not rospy.is_shutdown():
	min_index = forward_control()
	cur_time = time.time()
	if cur_time - prev_time > 1:
		print neato.average_ranges
		print min_index
		print neato.angular_speed
		print neato.bump_flag
		print '\n'
		prev_time = cur_time
	twist = Twist()
	if neato.bump_flag:
		twist.linear.x = 0
		twist.angular.z = 0
	else:
		twist.linear.x = neato.speed
		twist.angular.z = neato.angular_speed
	pub.publish(twist)

twist = Twist()
twist.linear.x = 0
twist.angular.z = 0
pub.publish(twist)