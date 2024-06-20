#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from pbot_beeper.msg import Beep
from pbot_beeper.msg import Note

pub = rospy.Publisher('beeper_topic', Beep, queue_size=10)
rospy.init_node('beeper_topic_publisher')
rospy.loginfo("Hello from PUB node")
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    melody = []
    melody.append(Note('NTDH1', 1.1))
    melody.append(Note('NTE6', 1.0))

    message = Beep(2, melody)

    pub.publish(message)
    rospy.loginfo("Pub message")
    rospy.sleep(10.0)
