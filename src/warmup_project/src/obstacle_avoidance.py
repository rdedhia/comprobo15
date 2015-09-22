# !/usr/bin/env python

"""This script is used to make the Neato avoid obstacles in its path.
The neato always goes forward unless it encounters an obstacle"""

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import rospy
import time

# Creating a robot class to represent the Neato
class Robot(object):
	def __init__(self, speed):
		self.speed = speed
		self.pos_x = 0
		self.pos_y = 0
		self.angle = 0
		self.angular_speed = 0
		self.forward_range = 5.0
		self.side_range = 5.0
		self.bump_flag = False
		self.direction = 'forward'
	def change_speed(self, speed):
		self.speed = speed
	def change_angular_speed (self, angular_speed):
		self.angular_speed = angular_speed
	def change_bump_flag(self, bump_flag):
		self.bump_flag = bump_flag
	def change_direction(self, direction):
		self.direction = direction
	def change_position(self, pos_x, pos_y, angle):
		self.pos_x = pos_x
		self.pos_y = pos_y
		# convert to degrees
		self.angle = abs(angle * 180 / math.pi)
	def change_forward_range(self, ranges):
		"""Looks at 6 degree range in front of Neato to locate obstacles"""
		temp_ranges = []
		ranges = ranges[357:] + ranges[:3]
		# set values of 0 to 5.0 for the purpose of range averages
		for item in ranges:
			if item == 0.0:
				temp_ranges.append(5.0)
			else:
				temp_ranges.append(item)
		# take an average of the values taken from the 6 degree range
		# in front of the neato
		self.forward_range = sum(temp_ranges) / len(temp_ranges)
	def change_side_range(self, ranges):
		"""Look at 10 degree change to left of Neato to see if Neato has
		cleared the obstacle before turning forward again"""
		temp_ranges = []
		ranges = ranges[90:100]
		# set values of 0.0 to 5.0
		for item in ranges:
			if item == 0.0:
				temp_ranges.append(5.0)
			else:
				temp_ranges.append(item)
		# take an average of the values taken from the 90 to 100 range
		self.side_range = sum(temp_ranges) / len(temp_ranges)

def convert_pose_to_xy_and_theta(pose):
    """Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple"""
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

def estop(msg):
	"""Callback function is used for stopping the neato when it bumps 
	into an object by changing the bump flag of the neato object"""
		if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bump_flag(True)
	else:
		neato.change_bump_flag(False)

def update_ranges(msg):
	"""Callback function used to update the forward and side ranges 
	of the neato object"""
	neato.change_forward_range(msg.ranges[0:360])
	neato.change_side_range(msg.ranges[0:360])

def calculate_angles(msg):
	"""Callback function to et x and y position and angle  to change_position"""
	pos_x, pos_y, angle = convert_pose_to_xy_and_theta(msg.pose.pose)
	neato.change_position(pos_x, pos_y, angle)

def turn_control():
	"""Function to turn 90 degrees away from obstacles and clear them
	before turning to move forward again"""
	# If there's an object and the neato is moving forward, stop and 
	# turn right
	if neato.forward_range < 1.5 and neato.direction == 'forward':
		neato.change_speed(0)
		neato.change_angular_speed(-0.2)
		neato.change_direction('turn right')
	# If the neato is turning right and it's oriented between 88 and 92 
	# degrees, stop turning. Move forward at a speed of .2
	elif neato.angle > 88 and neato.angle < 92 and neato.direction == 'turn right':
		neato.change_angular_speed(0)
		neato.change_speed(0.2)
		neato.change_direction('forward right')
	# If the neato is going forward after turning right, and there's no 
	# object in it's vision at 90 to 100 degrees turn left
	elif neato.side_range > 4 and neato.direction == 'forward right':
		neato.change_speed(0)
		neato.change_angular_speed(0.2)
		neato.change_direction('turn left')
	# If the neato is turning left and it's oriented between 358 and 2 
	# degrees, stop turning. Now facing forward, move forward at a speed
	# of 0.2
	elif neato.angle > 358 or neato.angle < 2 and neato.direction == 'turn left':
		neato.change_speed(0.2)
		neato.change_angular_speed(0)
		neato.change_direction('forward')

# initialize instance of Robot class
neato = Robot(speed=0)

# Initialize callbacks for bump, laser, and odometry
rospy.init_node('obstacle avoidance')
rospy.Subscriber('/bump', Bump, estop)
rospy.Subscriber('/scan', LaserScan, update_ranges)
rospy.Subscriber('/odom', Odometry, calculate_angles)

# Set up frame and Twist() for publishing to neato
header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Sleep briefly to give subscribers time
time.sleep(2)

# Using timing just to print stuff every second for debugging
cur_time = time.time()
prev_time = cur_time

# Initialize the neato speed at 0.2
neato.change_speed(0.2)

# Main loop for updating robot state
while not rospy.is_shutdown():
	cur_time = time.time()
	# Call function to update robot parameters
	turn_control()
	# Print data every second for debugging
	if cur_time - prev_time > 1:
		print neato.speed
		print neato.angular_speed
		print neato.angle
		print neato.direction
		print neato.forward_range
		print neato.side_range
		print '\n'
		prev_time = cur_time
	twist = Twist()
	# If the neato bumps into anything stop
	if neato.bump_flag:
		twist.linear.x = 0
		twist.angular.z = 0
	# Else set the twist parameters to the neato's speeds to move it
	else:
		twist.linear.x = neato.speed
		twist.angular.z = neato.angular_speed
	pub.publish(twist)

# Reset everything to zero at the end to stop neato
twist = Twist()
twist.linear.x = 0
twist.angular.z = 0
pub.publish(twist)