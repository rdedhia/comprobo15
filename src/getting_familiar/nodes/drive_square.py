#!/usr/bin/env python
import tty
import select
import sys
import termios
import rospy
import roslib
import time

from geometry_msgs.msg import Twist

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('drive_square')

neato_time = time.time()
prev_time = neato_time
direction = 'forward'
count = 0

timing = {
    'forward': (3.5,1,0),
    'turn': (1.6,0,1)
}

while count < 8:
    neato_time = time.time()

    forward = timing[direction][1]
    turn = timing[direction][2]
    if neato_time - prev_time > timing[direction][0]:
        prev_time = neato_time
        count += 1
        if direction == 'forward':
            direction = 'turn'
        else:
            direction = 'forward'

    twist = Twist()
    twist.linear.x = forward
    twist.angular.z = turn
    pub.publish(twist)

twist = Twist()
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


