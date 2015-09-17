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
		fix_ranges = []
		temp_ranges = []
		ranges = ranges[285:] + ranges[:75]

		for item in ranges:
			if item == 0.0:
				fix_ranges.append(5.0)
			else:
				fix_ranges.append(item)
		for i in range(len(fix_ranges) - 10):
			sub_range = fix_ranges[i:i+10]
			temp_ranges.append(sum(sub_range) / float(len(sub_range)))
		self.ranges = temp_ranges

def estop(msg):
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bump_flag(True)
	else:
		neato.change_bump_flag(False)

def distance_estop(msg):
	# print "Running distance_estop"
	neato.change_ranges(msg.ranges[0:360])

def forward_control():
	ranges = neato.ranges
	try:
		min_range = min(ranges)
		min_index = ranges.index(min_range)
	except:
		print ranges
	if min_index > 65:
		# turn left
		angular_speed = (min_index - 65) / 30.0
		if angular_speed > 1:
			angular_speed = 1
		neato.change_angular_speed(angular_speed)
	elif min_index < 65:
		# turn right
		angular_speed = (min_index - 65) / 30.0
		if angular_speed < -1:
			angular_speed = -1
		neato.change_angular_speed(angular_speed)
	else:
		neato.change_angular_speed(0)
	# Using min_range to determine speed
	if min_range > 1.5:
		min_range = 1.5
	neato.change_speed(min_range / 2.0 - 0.5)
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

neato.change_speed(0.1)
while not rospy.is_shutdown():
	min_index = forward_control()
	cur_time = time.time()
	if cur_time - prev_time > 1:
		print neato.ranges
		print min_index
		print neato.speed
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