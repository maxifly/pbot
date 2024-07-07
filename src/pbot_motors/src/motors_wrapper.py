#!/usr/bin/env python

import motor
import rospy
from std_msgs.msg import Float64
import time


class MotorWrapper:
    def __init__(self):
        self.left_motor = motor.left_motor()
        self.right_motor = motor.right_motor()
        self._current_right_rotation_speed = 0.
        self._current_left_rotation_speed = 0.
        self._current_right_command_value = 0
        self._current_left_command_value = 0
        rospy.init_node('pbot_motors')
        rospy.Subscriber("/pbot/right_wheel/target_velocity", Float64, self.callback_right_wheel)
        rospy.Subscriber("/pbot/left_wheel/target_velocity", Float64, self.callback_left_wheel)

        self.pub_right_wheel = rospy.Publisher("/pbot/right_wheel/current_velocity", Float64, queue_size=10)
        self.pub_left_wheel = rospy.Publisher("/pbot/left_wheel/current_velocity", Float64, queue_size=10)

    def cleanup(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.cleanup()
        self.right_motor.cleanup()
        del self.left_motor
        del self.right_motor

    def callback_right_wheel(self, msg: Float64):
        rospy.loginfo("right")
        self._current_right_rotation_speed = self.normalize_rotation_speed(msg.data)
        self._current_right_command_value = self.convert_rotation_to_command(self._current_right_rotation_speed)
        self.pub_right_wheel.publish(self._current_right_rotation_speed)

        if self._current_right_command_value == 0:
            self.right_motor.stop()
        else:
            if self._current_right_command_value > 0:
                self.right_motor.forward(self._current_right_command_value)
            else:
                self.right_motor.backward(self._current_right_command_value)

    def callback_left_wheel(self, msg: Float64):
        rospy.loginfo("left")
        self._current_left_rotation_speed = self.normalize_rotation_speed(msg.data)
        self._current_left_command_value = self.convert_rotation_to_command(self._current_left_rotation_speed)
        self.pub_left_wheel.publish(self._current_left_rotation_speed)

        if self._current_left_command_value == 0:
            self.left_motor.stop()
        else:
            if self._current_left_command_value > 0:
                self.left_motor.forward(self._current_left_command_value)
            else:
                self.left_motor.backward(self._current_left_command_value)

    def normalize_rotation_speed(self, speed):
        # if -24.9 < speed < 24.9:
        #     return 0.0
        if speed < -100.0:
            return -100.0
        if speed > 100.0:
            return 100.0
        return speed

    def convert_rotation_to_command(self, speed):
        return int(speed)

def start():
    m = MotorWrapper()
    rospy.on_shutdown(m.cleanup)

    rospy.loginfo("Motors node started")
    rospy.spin()


if __name__ == '__main__':
    start()
