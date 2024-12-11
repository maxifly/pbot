#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

TrackSensorLeftPin1 = 5  # The first tracking infrared sensor pin on the left is connected to  BCM port 3 of Raspberry pi
TrackSensorLeftPin2 = 29  # The second tracking infrared sensor pin on the left is connected to  BCM port 5 of Raspberry pi
TrackSensorRightPin1 = 7  # The first tracking infrared sensor pin on the right is connected to  BCM port 4 of Raspberry pi
TrackSensorRightPin2 = 12  # The second tracking infrared sensor pin on the right is connected to  BCMport 18 of Raspberry pi


class Tracker:
    def __init__(self):
        rospy.init_node('special_move')

        self._tracker_on = False

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        GPIO.setup(TrackSensorLeftPin1, GPIO.IN)
        GPIO.setup(TrackSensorLeftPin2, GPIO.IN)
        GPIO.setup(TrackSensorRightPin1, GPIO.IN)
        GPIO.setup(TrackSensorRightPin2, GPIO.IN)

        # Настройка подписчика на сообщения Int
        self.mode_sub = rospy.Subscriber('/pbot/special_mode', Int8, self.mode_callback)
        self.track_sub = rospy.Subscriber('/pbot/track', Int8, self.track_callback)

        # Настройка издателя для сообщений Twist
        self.twist_pub = rospy.Publisher('/pbot/cmd_vel', Twist, queue_size=10)
        self.track_pub = rospy.Publisher('/pbot/track', Int8, queue_size=1)

        self._spin_right = Twist()
        self._spin_left = Twist()
        self._right = Twist()
        self._left = Twist()
        self._forward = Twist()
        self._stop = Twist()
        self._stop.linear.x = 0.
        self._stop.angular.z = 0.

    def init_message(self):

        self._spin_right = Twist()
        self._spin_right.linear.x = 0.
        self._spin_right.angular.z = 0.15

        self._spin_left = Twist()
        self._spin_left.linear.x = 0.
        self._spin_left.angular.z = -0.15

        self._right = Twist()
        self._right.linear.x = 0.01
        self._right.angular.z = 0.15

        self._left = Twist()
        self._left.linear.x = 0.01
        self._left.angular.z = -0.15

        self._forward = Twist()
        self._forward.linear.x = 0.01
        self._forward.angular.z = 0.

    def mode_callback(self, msg: Int8):
        rospy.loginfo("mode %s", msg)

        if msg.data == 1:
            self._tracker_on = True
            self.track_pub.publish(1)

        if msg.data == 2:
            self._tracker_on = False

    def spin_right(self):
        rospy.loginfo("spin right")
        self.twist_pub.publish(self._spin_right)

    def spin_left(self):
        rospy.loginfo("spin left")
        self.twist_pub.publish(self._spin_left)

    def right(self):
        rospy.loginfo("right")
        self.twist_pub.publish(self._right)

    def left(self):
        rospy.loginfo("right")
        self.twist_pub.publish(self._left)

    def forward(self):
        rospy.loginfo("forward")
        self.twist_pub.publish(self._forward)

    def stop(self):
        rospy.loginfo("stop")
        self.twist_pub.publish(self._stop)

    def track_callback(self, msg: Int8):
        rospy.loginfo("Start tracking")

        self.init_message()

        rate = rospy.Rate(1)

        while self._tracker_on:
            rospy.loginfo("tracking")

            # When the black line is detected, the corresponding indicator of the tracking module is on, and the port level is LOW.
            # When the black line is not detected, the corresponding indicator of the tracking module is off, and the port level is HIGH.
            trackSensorLeftValue1 = GPIO.input(TrackSensorLeftPin1)
            trackSensorLeftValue2 = GPIO.input(TrackSensorLeftPin2)
            trackSensorRightValue1 = GPIO.input(TrackSensorRightPin1)
            trackSensorRightValue2 = GPIO.input(TrackSensorRightPin2)

            rospy.loginfo("L1 %s L2 %s R1 %s R2 %s",
                          trackSensorLeftValue1, trackSensorLeftValue2,
                          trackSensorRightValue1, trackSensorRightValue2)

            # 4 tracking pins level status
            # 0 0 X 0
            # 1 0 X 0
            # 0 1 X 0
            # Turn right in place,speed is 50,delay 80ms
            # Handle right acute angle and right right angle
            if (trackSensorLeftValue1 == 0 or trackSensorLeftValue2 == 0) and trackSensorRightValue2 == 0:
                self.spin_right()
            # time.sleep(0.08)

            # 4 tracking pins level status
            # 0 X 0 0
            # 0 X 0 1
            # 0 X 1 0
            # Turn right in place,speed is 50,delay 80ms
            # Handle left acute angle and left right angle
            elif trackSensorLeftValue1 == 0 and (
                    trackSensorRightValue1 == 0 or trackSensorRightValue2 == 0):
                self.spin_left()
            # time.sleep(0.08)

            # 0 X X X
            # Left_sensor1 detected black line
            elif trackSensorLeftValue1 == 0:
                self.spin_left()

            # X X X 0
            # Right_sensor2 detected black line
            elif trackSensorRightValue2 == 0:
                self.spin_right()

            # 4 tracking pins level status
            # X 0 1 X
            elif trackSensorLeftValue2 == 0 and trackSensorRightValue1 == 1:
                self.left()

            # 4 tracking pins level status
            # X 1 0 X
            elif trackSensorLeftValue2 == 1 and trackSensorRightValue1 == 0:
                self.right()

            # 4 tracking pins level status
            # X 0 0 X
            elif trackSensorLeftValue2 == 0 and trackSensorRightValue1 == 0:
                self.forward()

            # When the level of 4 pins are 1 1 1 1 , the car keeps the previous running state.

            rate.sleep()

        rospy.loginfo("End tracking")
        self.stop()


if __name__ == '__main__':
    tracker = Tracker()
    rospy.spin()
