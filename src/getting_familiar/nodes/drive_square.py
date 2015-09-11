#!/usr/bin/env python

"""This script moves the Neato in roughly a 1x1 meter square using
timing. Uses the Twist library to publish messages"""

import tty
import select
import sys
import termios
import rospy
import roslib
import time

from geometry_msgs.msg import Twist

# We don't know what this does, but we left it in from teleop
# just in case
settings = termios.tcgetattr(sys.stdin)

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('drive_square')

# Setting timers to track when the Neato should turn
neato_time = time.time()
prev_time = neato_time

# Setting the direction to 'forward' or 'turn' for timing purposes
direction = 'forward'

# Setting a counter to stop the Neato after it has made a square
count = 0

"""This dictionary is used to hard code how long the Neato should go
forward (3.5s) or turn (1.6s) to make a 1x1m square. The next two
elements of the tuple indicate the forward speed (in the x axis) and
the 'angular' speed (in the z axis). This allows for mostly accurate
behavior, though there is some inconsistency due to factors like
lag and wifi"""
timing = {
    'forward': (3.5,1,0),
    'turn': (1.6,0,1)
}

# Stop neato after it has gone forward 4 times and turned 4 times
while count < 8:
    neato_time = time.time()

    # Find forward and 'angular' velocities
    forward = timing[direction][1]
    turn = timing[direction][2]

    # If enough time has passed going forward or turning, switch the
    # direction, and increment the counter
    if neato_time - prev_time > timing[direction][0]:
        prev_time = neato_time
        count += 1
        if direction == 'forward':
            direction = 'turn'
        else:
            direction = 'forward'

    # Create twist object and publish based on forward and angular velocities
    twist = Twist()
    twist.linear.x = forward
    twist.angular.z = turn
    pub.publish(twist)

# Publish Twist object at end to stop Neato from moving
twist = Twist()
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

# Still don't know what this does
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


