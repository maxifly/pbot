#!/usr/bin/env python

import YB_Pcb_Car
import rospy
from std_msgs.msg import Float64
import time


class MotorWrapper:
    def __init__(self):
        self.motors = YB_Pcb_Car.YB_Pcb_Car()
        # self._current_right_speed = 0.
        # self._current_left_speed = 0.
        self._current_right_speed = 0
        self._current_left_speed = 0
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
        self._current_right_speed = self.normalize_speed(msg.data)
        rospy.loginfo("current command left: {} right: {}",
                      self._current_left_speed, self._current_right_speed)
        self.motors.Control_Car(self._current_left_speed, self._current_right_speed)
        self.pub_right_wheel.publish(self._current_right_speed)
        time.sleep(2)
        self.motors.Car_Stop()

    def callback_left_wheel(self, msg: Float64):
        rospy.loginfo("left")
        self._current_left_speed = self.normalize_speed(msg.data)
        self.pub_left_wheel.publish(self._current_left_speed)

    # TODO speed translate
    # TODO driver command

    def normalize_speed(self, speed):
        if speed < -180.0:
            return -180.0
        if speed > 180.0:
            return 180
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
