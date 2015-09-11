#!/usr/bin/env python

""" The script will be used to explore basics of ROS messages in Python """

# Any messages inside package will be found inside msg subpackage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import rospy
import tf

rospy.init_node('test_message')
br = tf.TransformBroadcaster()

point_msg = Point(x=1.0, y=2.0, z=0.0)
header_msg = Header(stamp=rospy.Time.now(), frame_id="base_link")

# msg = PointStamped(header=header_msg, point=point_msg)
# pub = rospy.Publisher('my_point', PointStamped, queue_size=10)

msg = Marker(header=header_msg, type=2)
msg.pose.position.x = 1.0
msg.pose.position.y = 2.0
msg.color.r = 255.0
msg.color.a = 1.0
msg.scale.x = 0.1
msg.scale.y = 0.1
msg.scale.z = 0.1

pub = rospy.Publisher('marker', Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	header_msg.stamp = rospy.Time.now()
	pub.publish(msg)
	r.sleep()