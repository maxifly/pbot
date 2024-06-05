#!/usr/bin/env python

import rospy
from beeper import Beeper
from std_msgs.msg import String

class BeeperWrapper:
    def __init__(self):
        self.beeper = Beeper()
        rospy.init_node('beeper')
        rospy.Subscriber("beeper_topic", String, self.callback_beep)

    def callback_beep(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        self.beeper.beep()

    def cleanup(self):
        self.beeper.cleanup()


def start():
       rospy.loginfo("Beeper node started")
       b = BeeperWrapper()
       rospy.on_shutdown(b.cleanup)

       rospy.loginfo("Beeper node started")
       rospy.spin()

if __name__ == '__main__':
    start()
