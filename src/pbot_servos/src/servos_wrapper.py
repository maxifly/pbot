#!/usr/bin/env python

import servo
import rospy
from std_msgs.msg import Int16


class ServosWrapper:
    def __init__(self):
        self.cam_h_servo = servo.cam_h_servo()
        self.cam_v_servo = servo.cam_v_servo()
        self._current_cam_h_pos = 90
        self._current_cam_v_pos = 90

        rospy.init_node('pbot_servos')
        rospy.Subscriber("/pbot/cam_h_servo/target_pos", Int16, self.callback_cam_h)
        rospy.Subscriber("/pbot/cam_v_servo/target_pos", Int16, self.callback_cam_v)
        rospy.Subscriber("/pbot/cam_h_servo/change_pos", Int16, self.callback_cam_h_change)
        rospy.Subscriber("/pbot/cam_v_servo/change_pos", Int16, self.callback_cam_v_change)
        rospy.Subscriber("/pbot/cam_v_servo/reset_pos", Int16, self.callback_cam_reset)

        self.pub_h_servo = rospy.Publisher("/pbot/cam_h_servo/current_pos", Int16, queue_size=10)
        self.pub_v_servo = rospy.Publisher("/pbot/cam_v_servo/current_pos", Int16, queue_size=10)

        self.callback_cam_reset(Int16(90))

    def cleanup(self):
        del self.cam_h_servo
        del self.cam_v_servo

    def callback_cam_h(self, msg: Int16):
        rospy.logdebug("cam_h. Target position %s", msg.data)

        self._current_cam_h_pos = self.cam_h_servo.servo_appointed_detection(msg.data)
        self.pub_h_servo.publish(self._current_cam_h_pos)

    def callback_cam_h_change(self, msg: Int16):
        rospy.logdebug("cam_h_change. Delta %s", msg.data)
        self.callback_cam_h(Int16(self._current_cam_h_pos + msg.data))

    def callback_cam_v_change(self, msg: Int16):
        rospy.logdebug("cam_v_change. Delta %s", msg.data)
        self.callback_cam_v(Int16(self._current_cam_v_pos + msg.data))

    def callback_cam_reset(self, msg: Int16):
        rospy.logdebug("cam_reset")
        self.callback_cam_h(Int16(90))
        self.callback_cam_v(Int16(90))

    def callback_cam_v(self, msg: Int16):
        rospy.logdebug("cam_v. Target position %s", msg.data)

        self._current_cam_v_pos = self.cam_v_servo.servo_appointed_detection(msg.data)
        self.pub_v_servo.publish(self._current_cam_v_pos)


def start():
    s = ServosWrapper()

    rospy.on_shutdown(s.cleanup)

    rospy.loginfo("Servo node started")
    rospy.spin()


if __name__ == '__main__':
    start()
