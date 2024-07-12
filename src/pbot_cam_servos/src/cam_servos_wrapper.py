#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


# class ServosWrapper:
#     def __init__(self):
#         self.cam_h_servo = servo.cam_h_servo()
#         self.cam_v_servo = servo.cam_v_servo()
#         self._current_cam_h_pos = 90
#         self._current_cam_v_pos = 90
#
#         rospy.init_node('pbot_servos')
#         rospy.Subscriber("/pbot/cam_h_servo/target_pos", Int16, self.callback_cam_h)
#         rospy.Subscriber("/pbot/cam_v_servo/target_pos", Int16, self.callback_cam_v)
#
#         self.pub_h_servo = rospy.Publisher("/pbot/cam_h_servo/current_pos", Int16, queue_size=10)
#         self.pub_v_servo = rospy.Publisher("/pbot/cam_v_servo/current_pos", Int16, queue_size=10)
#
#         self.callback_cam_h(Int16(90))
#         self.callback_cam_v(Int16(90))
#
#     def cleanup(self):
#         del self.cam_h_servo
#         del self.cam_v_servo
#
#     def callback_cam_h(self, msg: Int16):
#         rospy.loginfo("cam_h. Target position %s", msg.data)
#
#         self._current_cam_h_pos = self.cam_h_servo.servo_appointed_detection(msg.data)
#         self.pub_h_servo.publish(self._current_cam_h_pos)
#
#     def callback_cam_v(self, msg: Int16):
#         rospy.loginfo("cam_v. Target position %s", msg.data)
#
#         self._current_cam_v_pos = self.cam_v_servo.servo_appointed_detection(msg.data)
#         self.pub_v_servo.publish(self._current_cam_v_pos)

class CamServos:
    def __init__(self):
        rospy.init_node('pbot_cam_servos')
        rospy.Subscriber("/pbot/cam_servos", Twist, self.callback)

    def callback(self, msg: Twist):
        rospy.loginfo("cam_servos. Twist %s", msg)

def start():
    cs = CamServos()

    rospy.loginfo("Servo node started")
    rospy.spin()


if __name__ == '__main__':
    start()
