#!/usr/bin/env python

"""Allows a user to control a neato using the WASD keys. Neato 
will keep driving until a new key is pressed. Publishes messages 
using the Twist library.
"""

import tty
import select
import sys
import termios
import rospy
import roslib

from geometry_msgs.msg import Twist

"""Setting up our internal mapping for moving around the Neato
    a turns left
    s drives backward
    d turns right
    w drives forward
press any other key will cause the robot to stop moving"""
mapping = {
    'a': (0,1),     
    's': (-1,0),    
    'd': (0,-1),    
    'w': (1,0)      
}    

# Function from teleop twist keyboard to get the next key press
def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

settings = termios.tcgetattr(sys.stdin)
key = None

# Set up publisher and initialize nodes
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.init_node('teleop')

# Gets the key pressed and uses the mapping to figure out which 
# direction to go. Hit Ctrl-C to stop while loop
while key != '\x03':
    key = getKey()
    if key in mapping:
        # Get forward and 'angular' speeds based on key pressed
        forward = mapping[key][0]
        rotate = mapping[key][1]
    else:
        # Any key not in mapping turns velocities to 0 (stops Neato)
        forward = 0
        rotate = 0

    # Use twist library to publish which direction to go. Forward speed
    # modifies the linear x speed, and 'angular' speed modifies the 
    # angular z speed
    twist = Twist()
    twist.linear.x = forward
    twist.angular.z = rotate
    pub.publish(twist)

# If while loop is exited, the neato will stop moving
twist = Twist()
twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
