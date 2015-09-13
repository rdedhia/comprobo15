#!/usr/bin/env python

"""This script is used to move the Neato forward until it senses that
on of the bump sensors has been hit"""

from std_msgs.msg import Header
from neato_node.msg import Bump
from geometry_msgs.msg import Twist
import rospy

# Creating a robot class to represent the speed
class Robot(object):
	def __init__(self, speed):
		self.speed = 0
	def change_speed(self, speed):
		self.speed = speed

def estop(msg):
	if msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide != 0:
		neato.change_speed(0)
	else:
		neato.change_speed(1)

# initialize instance of Robot class
neato = Robot(speed=0)

rospy.init_node('emergency_stop')
rospy.Subscriber('/bump', Bump, estop)

header_msg = Header(stamp=rospy.Time.now(), frame_id="odom")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

while not rospy.is_shutdown():
	twist = Twist()
	twist.linear.x = neato.speed
	pub.publish(twist)

twist = Twist()
twist.linear.x = 0
pub.publish(twist)
