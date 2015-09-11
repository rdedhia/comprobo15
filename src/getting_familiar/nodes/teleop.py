#!/usr/bin/env python
import tty
import select
import sys
import termios
import rospy
import roslib

from geometry_msgs.msg import Twist

mapping = {
    'a': (0,1),
    's': (-1,0),
    'd': (0,-1),
    'w': (1,0)
}

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

settings = termios.tcgetattr(sys.stdin)
key = None

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('teleop')


while key != '\x03':
    key = getKey()
    if key in mapping:
        forward = mapping[key][0]
        rotate = mapping[key][1]
    else:
        forward = 0
        rotate = 0
    twist = Twist()
    twist.linear.x = forward
    twist.angular.z = rotate
    pub.publish(twist)

twist = Twist()
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


