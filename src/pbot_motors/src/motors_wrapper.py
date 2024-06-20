#!/usr/bin/env python

import YB_Pcb_Car
import rospy
from std_msgs.msg import Float64
import time


class MotorWrapper:
    def __init__(self):
        self.motors = YB_Pcb_Car.YB_Pcb_Car()
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
        self.motors.Car_Stop()
        del self.motors

    def callback_right_wheel(self, msg: Float64):
        rospy.loginfo("right")
        self._current_right_rotation_speed = self.normalize_rotation_speed(msg.data)
        self._current_right_command_value = self.convert_rotation_to_command(self._current_right_rotation_speed)
        self.pub_right_wheel.publish(self._current_right_rotation_speed)
        self.driver_command()

    def callback_left_wheel(self, msg: Float64):
        rospy.loginfo("left")
        self._current_left_rotation_speed = self.normalize_rotation_speed(msg.data)
        self._current_left_command_value = self.convert_rotation_to_command(self._current_left_rotation_speed)
        self.pub_left_wheel.publish(self._current_left_rotation_speed)
        self.driver_command()

    # TODO speed translate

    def driver_command(self):
        rospy.loginfo("current command left: {} right: {}".format(
            self._current_left_command_value, self._current_right_command_value))
        if self._current_left_command_value == 0 and self._current_right_command_value == 0:
            self.motors.Car_Stop()
        else:
            self.motors.Control_Car(self._current_left_command_value, self._current_right_command_value)

    def normalize_rotation_speed(self, speed):
        if -24.9 < speed < 24.9:
            return 0.0
        if speed < -180.0:
            return -180.0
        if speed > 180.0:
            return 180.0
        return speed

    def convert_rotation_to_command(self, speed):
        return int(speed)


# car = YB_Pcb_Car.YB_Pcb_Car()
#
# i = 0
# for i in range(2):
#     car.Car_Run(150, 150)
#     time.sleep(2)
#     car.Car_Spin_Left(48, 48)
#     time.sleep(2)
#     car.Car_Run(150, 150)
#     time.sleep(2)
#     car.Car_Spin_Left(48, 48)
#     time.sleep(2)
#     i += 1
# car.Car_Stop()


def start():
    m = MotorWrapper()
    rospy.on_shutdown(m.cleanup)

    rospy.loginfo("Motors node started")
    rospy.spin()


if __name__ == '__main__':
    start()
