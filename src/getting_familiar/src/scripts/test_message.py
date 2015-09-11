#!/usr/bin/env python

""" The script is used to create and publish a Marker object. The frame_id
is set to the base_link and the Marker is located at (1,2) """

# Any messages inside package will be found inside msg subpackage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import rospy
import tf

rospy.init_node('test_message')
br = tf.TransformBroadcaster()

header_msg = Header(stamp=rospy.Time.now(), frame_id="base_link")

# Hard coding the marker to have the position 1,2, be red, and be
# scaled by 0.1 in all the axes
msg = Marker(header=header_msg, type=2)
msg.pose.position.x = 1.0
msg.pose.position.y = 2.0
msg.color.r = 255.0
msg.color.a = 1.0
msg.scale.x = 0.1
msg.scale.y = 0.1
msg.scale.z = 0.1

# Setting up the marker to be published w/ a queue size of 10
pub = rospy.Publisher('marker', Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	# updating the timestamp to the current rospy time
	header_msg.stamp = rospy.Time.now()
	pub.publish(msg)
	r.sleep()