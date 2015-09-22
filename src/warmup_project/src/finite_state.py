#!/usr/bin/env python

"""This script combines wall following and person following in a finite
state machine. Uses person following when it detects a 'person' in front 
of it. Otherwise switches to wall following"""

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
		self.wall_ranges = []
		self.person_ranges = []
		self.orienting_flag = True
		self.bump_flag = False
		self.person_flag = False
		self.wall_min_index = 0
		self.person_min_index = 0
	def change_speed(self, speed):
		self.speed = speed
	def change_angular_speed (self, angular_speed):
		self.angular_speed = angular_speed
	def change_bump_flag(self, bump_flag):
		self.bump_flag = bump_flag
	def change_orienting_flag(self, orienting_flag):
		self.orienting_flag = orienting_flag
	def change_person_flag(self, person_flag):
		self.person_flag = person_flag
	def change_wall_ranges(self, ranges):
		"""Takes in 360 degrees of ranges and converts it to an array of
		length 36 by taking 10 degree averages. Using for orienting to
		the wall for wall following"""
		temp_ranges = []
		avg_ranges = []		
		# set values of 0 to 5.0 for the purpose of range averages
		for item in ranges:
			if item == 0.0:
				temp_ranges.append(5.0)
			else:
				temp_ranges.append(item)
		# Convert 360 degree range to 36 length array
		for i in range(0, 360, 10):
			total = temp_ranges[i:i+10]
			average = sum(total) / float(len(total))
			avg_ranges.append(average)
		self.wall_ranges = avg_ranges
	def change_person_ranges(self, ranges):
		"""Looks at 60 degree range in front of robot for people, and calculates
		running averages of 10 data points each"""
		temp_ranges = []
		avg_ranges = []
		ranges = ranges[330:] + ranges[:30]
		# set values of 0 to 5.0 for the purpose of range averages
		for item in ranges:
			if item == 0.0:
				temp_ranges.append(5.0)
			else:
				temp_ranges.append(item)
		# Calculating running averages of 10 data points each
		for i in range(len(temp_ranges) - 10):
			sub_range = temp_ranges[i:i+10]
			avg_ranges.append(sum(sub_range) / float(len(sub_range)))
		self.person_ranges = avg_ranges
	def print_parameters(self):
		print "Speed: ", self.speed
		print "Angular Speed: ", self.angular_speed
		print "Orienting Flag: ", self.orienting_flag
		print "Bump Flag: ", self.bump_flag
		print "Person Flag: ", self.person_flag
		print "Wall Min Index: ", self.wall_min_index
		try:
			print "Wall Min Range: ", min(self.wall_ranges)
		except ValueError:
			print "Wall min range cannot be calculated"
		print "Person Min Index: ", self.person_min_index
		try:
			print "Person Min Range: ", min(self.person_ranges)
		except ValueError:
			print "Person min range cannot be calculated"
		print "\n"

def estop(msg):
	"""Callback function is used for stopping the neato when it bumps 
	into an object by changing the bump flag of the neato object"""	
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bump_flag(True)
	else:
		neato.change_bump_flag(False)

def change_ranges(msg):
	"""Callback function used to update the wall and person ranges 
	of the neato object"""
	neato.change_wall_ranges(msg.ranges[0:360])
	neato.change_person_ranges(msg.ranges[0:360])

def turn_control():
	"""Function to orient the neato parallel to the wall (seeing the wall at
	90 degrees) when moving out of person following mode. After being
	oriented to 90 degrees, stops turning and moves forward at a speed
	of 0.2"""
	wall_ranges = neato.wall_ranges
	try:
		neato.wall_min_index = wall_ranges.index(min(wall_ranges))
	except ValueError:
		return
	if neato.wall_min_index > 9:
		speed = (neato.wall_min_index - 8) / 10.0
		neato.change_angular_speed(speed)
	elif neato.wall_min_index < 8:
		neato.change_angular_speed(1)
	else:
		neato.orienting_flag = False
		neato.change_speed(0.2)
		neato.change_angular_speed(0)

def forward_control():
	"""Function that switches from following the wall in the forward
	direction to person following when a "person" is sensed"""
	# Switch back to person following mode if something closeby
	if min(neato.person_ranges) < 1.5:
		neato.change_person_flag(True)

def person_control():
	"""Function that follows a "person" at a distance of 0.5m away.
	Changes speed and angular speed depending on where the person is
	detected at. Switches back to turn control if person is out of
	range"""
	person_ranges = neato.person_ranges		
	# setting arbitrary max distance for following person
	max_distance = 0.5
	# Try / except syntax in case neato.average_ranges is empty
	try:
		min_range = min(person_ranges)
		neato.person_min_index = person_ranges.index(min_range)
	except ValueError:
		print person_ranges
	# switch out of person following mode if range is too large
	if min_range > 2.5:
		neato.change_person_flag(False)
		neato.change_orienting_flag(True)
		neato.change_speed(0)
		neato.change_angular_speed(0)
		return
	# Turn left if person is to the left based on error from center	
	if neato.person_min_index > 30:
		angular_speed = (neato.person_min_index - 30) / 20.0
		if angular_speed > 1:
			angular_speed = 1
		neato.change_angular_speed(angular_speed)
	# Turn right if person is to the right based on error from center
	elif neato.person_min_index < 30:
		angular_speed = (neato.person_min_index - 30) / 20.0
		if angular_speed < -1:
			angular_speed = -1
		neato.change_angular_speed(angular_speed)
	else:
		neato.change_angular_speed(0)
	# Using min_range to determine speed. Designed to keep neato 0.5
	# meters away from person
	if min_range > 1.5:
		min_range = 1.5
	neato.change_speed(min_range / 2.0 - max_distance)

# Initialize instance of Robot class
neato = Robot(speed=0)
initialpart = False

# Initialize callbacks for bump and laser
rospy.init_node('finite state')
rospy.Subscriber('/bump', Bump, estop)
rospy.Subscriber('/scan', LaserScan, change_ranges)

# Set up frame and Twist() for publishing to neato
header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
twist = Twist()

# Sleep briefly to give subscribers time
time.sleep(2)

# Using timing just to print stuff every second for debugging
cur_time = time.time()
prev_time = cur_time

# Main loop to switch between states based on flags
while not rospy.is_shutdown():
	if neato.orienting_flag:
		turn_control()
	elif not neato.person_flag:
		forward_control()
	elif neato.person_flag:
		person_control()
	# Print parameters
	cur_time = time.time()
	if cur_time - prev_time > 1:
		neato.print_parameters()
		prev_time = cur_time
	# Configure twist object with speeds
	if neato.bump_flag:
		twist.linear.x = 0
		twist.angular.z = 0
	else:
		twist.linear.x = neato.speed
		twist.angular.z = neato.angular_speed
	pub.publish(twist)

# Reset everything to zero at the end to stop neato
twist = Twist()
twist.linear.x = 0
twist.angular.z = 0
pub.publish(twist)