#!/usr/bin/env python

"""This script is used to move the Neato forward until it senses that
one of the lasers is detecting something close to it. Also preserves
the bump sensor estop just in case shit gets fucked"""

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import rospy
import time

# Creating a robot class to represent the speed
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
		temp_ranges = []
		ranges = ranges[357:] + ranges[:3]

		for item in ranges:
			if item == 0.0:
				temp_ranges.append(5.0)
			else:
				temp_ranges.append(item)
		self.forward_range = sum(temp_ranges) / len(temp_ranges)
	def change_side_range(self, ranges):
		temp_ranges = []
		ranges = ranges[90:100]

		for item in ranges:
			if item == 0.0:
				temp_ranges.append(5.0)
			else:
				temp_ranges.append(item)
		self.side_range = sum(temp_ranges) / len(temp_ranges)

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

def estop(msg):
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_bump_flag(True)
	else:
		neato.change_bump_flag(False)

def update_ranges(msg):
	# print "Running distance_estop"
	neato.change_forward_range(msg.ranges[0:360])
	neato.change_side_range(msg.ranges[0:360])

def calculate_angles(msg):
	pos_x, pos_y, angle = convert_pose_to_xy_and_theta(msg.pose.pose)
	neato.change_position(pos_x, pos_y, angle)

def turn_control():
	if neato.forward_range < 1.5 and neato.direction == 'forward':
		neato.change_speed(0)
		neato.change_angular_speed(-0.2)
		neato.change_direction('turn right')
	elif neato.angle > 88 and neato.angle < 92 and neato.direction == 'turn right':
		neato.change_angular_speed(0)
		neato.change_speed(0.2)
		neato.change_direction('forward right')
	elif neato.side_range > 4 and neato.direction == 'forward right':
		neato.change_speed(0)
		neato.change_angular_speed(0.2)
		neato.change_direction('turn left')
	elif neato.angle > 358 or neato.angle < 2 and neato.direction == 'turn left':
		neato.change_speed(0.2)
		neato.change_angular_speed(0)
		neato.change_direction('forward')

# initialize instance of Robot class
neato = Robot(speed=0)

rospy.init_node('emergency_stop')
rospy.Subscriber('/bump', Bump, estop)
rospy.Subscriber('/scan', LaserScan, update_ranges)
rospy.Subscriber('/odom', Odometry, calculate_angles)

header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

time.sleep(2)
cur_time = time.time()
prev_time = cur_time

neato.change_speed(0.2)
while not rospy.is_shutdown():
	cur_time = time.time()
	turn_control()
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