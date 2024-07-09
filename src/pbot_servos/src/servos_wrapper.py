#!/usr/bin/env python

import servo
import rospy
from std_msgs.msg import Int8


class ServosWrapper:
    def __init__(self):
        self.cam_h_servo = servo.cam_h_servo()
        self._current_cam_h_pos = 90

        rospy.init_node('pbot_servos')
        rospy.Subscriber("/pbot/cam_h_servo/target_pos", Int8, self.callback_cam_h)

        self.pub_h_servo = rospy.Publisher("/pbot/cam_h_servo/current_pos", Int8, queue_size=10)

    def cleanup(self):
        del self.cam_h_servo

    def callback_cam_h(self, msg: Int8):
        rospy.loginfo("cam_h. Target position %s", msg.data)

        self._current_cam_h_pos = self.cam_h_servo.servo_appointed_detection(msg.data)
        self.pub_h_servo.publish(self._current_cam_h_pos)


def start():
    s = ServosWrapper()
    rospy.on_shutdown(s.cleanup)

    rospy.loginfo("Servo node started")
    rospy.spin()


if __name__ == '__main__':
    start()
