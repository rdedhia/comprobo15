#!/usr/bin/env python

"""This script is used to have the neato follow a person and mimic 
movements while maintaining a specified distance"""

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import Twist
import rospy
import time

# Creating a robot class to represent the Neato
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
		"""Takes in the 360 degree range array and calculates a running
		average of the data points in the 150 degree cone facing forward
		from the neato. Used to find closest person (object) to follow
		in that forward direction"""
		fix_ranges = []
		temp_ranges = []
		ranges = ranges[285:] + ranges[:75]
		# set values of 0 to 5.0 for the purpose of range averages
		for item in ranges:
			if item == 0.0:
				fix_ranges.append(5.0)
			else:
				fix_ranges.append(item)
		# Calculating running averages of 10 data points each
		for i in range(len(fix_ranges) - 10):
			sub_range = fix_ranges[i:i+10]
			temp_ranges.append(sum(sub_range) / float(len(sub_range)))
		# Wait until end to set self.ranges so it is never empty
		self.ranges = temp_ranges

def estop(msg):
	"""Callback function is used for stopping the neato when it bumps 
	into an object by changing the bump flag of the neato object"""	
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bump_flag(True)
	else:
		neato.change_bump_flag(False)

def update_ranges(msg):
	"""Callback function used to update the ranges of the neato object"""
	neato.change_ranges(msg.ranges[0:360])

def forward_control():
	"""Function controls the forward speed and angular speed based on the 
	orientation of the laser to the person the neato is following"""
	ranges = neato.ranges
	# Try / except syntax in case neato.ranges is empty
	try:
		min_range = min(ranges)
		min_index = ranges.index(min_range)
	except:
		print ranges
	# Turn left if person is to the left based on error from center
	if min_index > 65:
		angular_speed = (min_index - 65) / 30.0
		# Limit angular speed to 1
		if angular_speed > 1:
			angular_speed = 1
		neato.change_angular_speed(angular_speed)
	# Turn right if person is to the right based on error from center
	elif min_index < 65:
		angular_speed = (min_index - 65) / 30.0
		# Limit angular speed to -1
		if angular_speed < -1:
			angular_speed = -1
		neato.change_angular_speed(angular_speed)
	else:
		neato.change_angular_speed(0)
	# Using min_range to determine speed. Designed to keep neato
	# 0.5 meters away from person
	if min_range > 1.5:
		min_range = 1.5
	neato.change_speed(min_range / 2.0 - 0.5)
	return min_index

# initialize instance of Robot class
neato = Robot(speed=0)
initialpart = False

# Initialize callbacks for bump and laser
rospy.init_node('person following')
rospy.Subscriber('/bump', Bump, estop)
rospy.Subscriber('/scan', LaserScan, update_ranges)

# Set up frame and Twist() for publishing to neato
header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Sleep briefly to give subscribers time
time.sleep(2)

# Using timing just to print stuff every second for debugging
cur_time = time.time()
prev_time = cur_time

# Move forward at speed of 0.1 until an object is identified
neato.change_speed(0.1)

# Main loop for updating robot state
while not rospy.is_shutdown():
	# Calls forward_control() function to move neato
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
		# Publish speeds to neato using twist() object
	twist = Twist()
	# If the neato bumps into anything stop otherwise move forward
	if neato.bump_flag:
		twist.linear.x = 0
		twist.angular.z = 0
	else:
		twist.linear.x = neato.speed
		twist.angular.z = neato.angular_speed
	pub.publish(twist)

# Reset speeds to zero at the end
twist = Twist()
twist.linear.x = 0
twist.angular.z = 0
pub.publish(twist)