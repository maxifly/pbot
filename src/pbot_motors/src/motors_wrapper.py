#!/usr/bin/env python

import motor
import dif_wheels
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

rad2pwm_k = 0.044
max_radian_speed = 100 * rad2pwm_k
min_radian_speed = -100 * rad2pwm_k


class MotorWrapper:
    def __init__(self):
        self.left_motor = motor.left_motor()
        self.right_motor = motor.right_motor()
        self._current_right_rotation_speed = 0.
        self._current_left_rotation_speed = 0.
        self._current_right_command_value = 0
        self._current_left_command_value = 0
        self._current_cmd_x_speed = 0.
        self._current_cmd_z_angular_speed = 0.

        rospy.init_node('pbot_motors')
        rospy.Subscriber("/pbot/right_wheel/target_velocity", Float64, self.callback_right_wheel)
        rospy.Subscriber("/pbot/left_wheel/target_velocity", Float64, self.callback_left_wheel)
        rospy.Subscriber("/pbot/cmd_vel", Twist, self.callback_cmd_vel)

        self.pub_right_wheel = rospy.Publisher("/pbot/right_wheel/current_velocity", Float64, queue_size=10)
        self.pub_left_wheel = rospy.Publisher("/pbot/left_wheel/current_velocity", Float64, queue_size=10)
        self.pub_target_right_wheel = rospy.Publisher("/pbot/right_wheel/target_velocity", Float64, queue_size=10)
        self.pub_target_left_wheel = rospy.Publisher("/pbot/left_wheel/target_velocity", Float64, queue_size=10)

    def cleanup(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.cleanup()
        self.right_motor.cleanup()
        del self.left_motor
        del self.right_motor

    def callback_right_wheel(self, msg: Float64):
        rospy.logdebug("right")
        self._current_right_rotation_speed = self.normalize_rotation_speed(msg.data)
        self._current_right_command_value = self.convert_rotation_to_pwm(self._current_right_rotation_speed)
        self.pub_right_wheel.publish(self._current_right_rotation_speed)

        if self._current_right_command_value == 0:
            self.right_motor.stop()
        else:
            if self._current_right_command_value > 0:
                self.right_motor.forward(self._current_right_command_value)
            else:
                self.right_motor.backward(self._current_right_command_value)

    def callback_left_wheel(self, msg: Float64):
        rospy.logdebug("left")
        self._current_left_rotation_speed = self.normalize_rotation_speed(msg.data)
        self._current_left_command_value = self.convert_rotation_to_pwm(self._current_left_rotation_speed)
        self.pub_left_wheel.publish(self._current_left_rotation_speed)

        if self._current_left_command_value == 0:
            self.left_motor.stop()
        else:
            if self._current_left_command_value > 0:
                self.left_motor.forward(self._current_left_command_value)
            else:
                self.left_motor.backward(self._current_left_command_value)

    def callback_cmd_vel(self, msg: Twist):
        rospy.logdebug("Velocity command %s", msg)
        if (self._current_cmd_x_speed != msg.linear.x
                or self._current_cmd_z_angular_speed != msg.angular.z):
            self._current_cmd_x_speed = msg.linear.x
            self._current_cmd_z_angular_speed = msg.angular.z
            target_left_wheel_angular_speed, target_right_wheel_angular_speed = (
                dif_wheels.calculate_wheel_speeds(self._current_cmd_x_speed, self._current_cmd_z_angular_speed))
            self.pub_target_left_wheel.publish(target_left_wheel_angular_speed)
            self.pub_target_right_wheel.publish(target_right_wheel_angular_speed)

    def normalize_rotation_speed(self, speed):
        # Speed in radian
        # if -24.9 < speed < 24.9:
        #     return 0.0
        if speed < min_radian_speed:
            return min_radian_speed
        if speed > max_radian_speed:
            return max_radian_speed
        return speed

    def convert_rotation_to_pwm(self, speed):
        return int(speed / rad2pwm_k)


def start():
    m = MotorWrapper()
    rospy.on_shutdown(m.cleanup)

    rospy.loginfo("Motors node started")
    rospy.spin()


if __name__ == '__main__':
    start()
