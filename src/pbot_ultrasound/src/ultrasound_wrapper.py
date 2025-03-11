#!/usr/bin/env python

import ultrasound
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Range


class UltrasoundWrapper:
    def __init__(self):
        self.ultrasound = ultrasound.Ultrasound()

        rospy.init_node('pbot_ultrasound')

        # Настройка подписчика на сообщения Int
        self.mode_sub = rospy.Subscriber('/pbot/sultrasound_mode', Int8, self.mode_callback)
        self.ultrasound_sub = rospy.Subscriber('/pbot/ultrasound', Int8, self.ultrasound_callback)

        # Настройка издателя для сообщений Range
        self.range_pub = rospy.Publisher('/pbot/range', Range, queue_size=10)
        self.ultrasound_pub = rospy.Publisher('/pbot/ultrasound', Int8, queue_size=1)

    def cleanup(self):
        del self.ultrasound

    def mode_callback(self, msg: Int8):
        rospy.loginfo("mode %s", msg)

        if msg.data == 1:
            self._ultrasound_on = True
            self.ultrasound_pub.publish(1)

        if msg.data == 2:
            self._ultrasound_on = False

    def ultrasound_callback(self):
        rospy.loginfo("Start ultrasound")

        rate = rospy.Rate(5)

        while self._ultrasound_on:
            rospy.loginfo("Calculate distance")

            min_dist, max_dist, avg_dist = self.ultrasound.distance_test()

            # Создаем сообщение
            range_msg = Range()

            # Заполняем поля сообщения
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "base_link"
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 15 * (3.141592653589793 / 180)  # Примерно 0.2618 радиан
            range_msg.min_range = min_dist
            range_msg.max_range = max_dist
            range_msg.range = avg_dist

            self.range_pub.publish(range_msg)
            rate.sleep()

def start():
    s = UltrasoundWrapper()

    rospy.on_shutdown(s.cleanup)

    rospy.loginfo("Ultrasound node started")
    rospy.spin()


if __name__ == '__main__':
    start()
