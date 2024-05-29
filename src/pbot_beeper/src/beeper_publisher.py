#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('beeper_topic', String, queue_size=10)
rospy.init_node('beeper_topic_publisher')
rospy.loginfo("Hello from PUB node")
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    pub.publish(String("Hello World"))
    rospy.loginfo("Pub message")
    rospy.sleep(1.0)
