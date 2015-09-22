#!/usr/bin/env python

"""This script is used to have the Neato move forward while 
aligning its direction of motion to be parallel to the nearest wall """

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
		"""Takes in the 360 degree range array and averages it into a 36
		length array"""
		self.ranges = []
		average_ranges = []
		# set values of 0 to 5.0 for the purpose of range averages
		for item in ranges:
			if item == 0.0:
				self.ranges.append(5.0)
			else:
				self.ranges.append(item)
		# Average taken for every 10 degrees and set to new average ranges array of length 36
		for i in range(0, 360, 10):
			total = self.ranges[i:i+10]
			average = sum(total) / float(len(total))
			average_ranges.append(average)
		self.average_ranges = average_ranges

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

def turn_control(initialpart):
	"""Function to orient the neato parallel to the wall (seeing the wall at
	90 degrees). Takes in initialpart flag that tells you whether or not the
	neato is oriented yet, and returns it. Also returns the index from the 36
	length array of where the neato detects the closest object (ideally the
	wall). Uses basic proportional control to get the neato parallel to the wall
	"""
	average_ranges = neato.average_ranges
	# Try / except syntax in case neato.average_ranges is empty
	try:
		min_index = average_ranges.index(min(average_ranges))
	except ValueError:
		print average_ranges	# Turns left if the angle to the wall is greater than 90 degrees. Turns 
	# slower when the angle is close to 90 degrees, to not overshoot
	if min_index > 9:
		speed = (min_index - 8) / 10.0
		neato.change_angular_speed(speed)
	# Turn quickly left if the angle if the angle is less than 90 degrees
	elif min_index < 8:
		neato.change_angular_speed(1)
	# Set initialpart to True is neato is parallel to wall to begin forward control
	else:
		initialpart = True
	return min_index, initialpart

def forward_control():
	"""Function to keep neato moving forward parallel to wall after it has been
	centered parallel to the wall in turn_control(). Has the capability to turn
	corners and fall another wall"""
	average_ranges = neato.average_ranges
	min_index = average_ranges.index(min(average_ranges))
	# Control loop keeps neato centered moving parallel to wall
	# Turn left based on error from 90 degrees
	if min_index > 9 and min_index < 27:
		neato.change_angular_speed((min_index - 8) / 5.0)
	# Turn right quickly
	elif min_index >= 27:
		neato.change_angular_speed(-2)
	# Turn right slowly based on error from 90 degrees
	elif min_index < 8:
		neato.change_angular_speed((min_index - 8) / 5.0)
	# Set angular speed to 0 when neato is parallel to wall
	else:
		neato.change_angular_speed(0)
	return min_index

# Initialize instance of Robot class
neato = Robot(speed=0)
initialpart = False

# Initialize callbacks for bump and laser
rospy.init_node('wall following')
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

# Main loop for updating robot state before robot is oriented parallel
# (wall is at 90 degrees from laser)
while not initialpart:
	# Turn until neato reaches a 90 degree orientation to the wall
	min_index, initialpart = turn_control(initialpart)
	cur_time = time.time()
	# Print data every second
	if cur_time - prev_time > 1:
		print neato.average_ranges
		print initialpart
		print min_index
		print neato.angular_speed
		print '\n'
		prev_time = cur_time
	# Publish speeds to neato using twist() object
	twist = Twist()
	twist.angular.z = neato.angular_speed
	pub.publish(twist)

# Main loop after neato has been oriented to move forward (at a speed of 0.2)
neato.change_speed(0.2)
while not rospy.is_shutdown():
	# Run forward_control() function to keep neato parallel to closest wall
	min_index = forward_control()
	cur_time = time.time()
	# Print data every second
	if cur_time - prev_time > 1:
		print neato.average_ranges
		print min_index
		print neato.angular_speed
		print neato.bump_flag
		print '\n'
		prev_time = cur_time
	# Publish speeds to neato using twist() object
	twist = Twist()
	#If the neato bumps into anything stop otherwise move forward
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