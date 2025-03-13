import time

import RPi.GPIO as GPIO
import rospy


class Ultrasound:
    def __init__(self):

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self._echo_pin = 0
        self._trig_pin = 1

        GPIO.setup(self._echo_pin, GPIO.IN)
        GPIO.setup(self._trig_pin, GPIO.OUT)

    def request_distance(self):
        GPIO.output(self._trig_pin, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(self._trig_pin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self._trig_pin, GPIO.LOW)

        t3 = time.time()
        while not GPIO.input(self._echo_pin):
            t4 = time.time()
            if (t4 - t3) > 0.03:
                return -1
        t1 = time.time()
        while GPIO.input(self._echo_pin):
            t5 = time.time()
            if (t5 - t1) > 0.03:
                return -1

        t2 = time.time()
        # print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
        time.sleep(0.01)
        return ((t2 - t1) * 340 / 2) * 100

    def distance_test(self):
        num = 0
        distance_array = []
        while num < 5:
            distance = self.request_distance()
            while int(distance) == -1:
                distance = self.request_distance()
                rospy.loginfo("Tdistance is %f", distance)
            while int(distance) >= 500 or int(distance) == 0:
                distance = self.request_distance()
                rospy.loginfo("Edistance is %f", distance)
            distance_array.append(distance)
            num = num + 1
            time.sleep(0.01)

        min_dist = min(distance_array)
        max_dist = max(distance_array)
        avg_dist = sum(distance_array) / len(distance_array)

        rospy.loginfo("distance is %f", avg_dist)
        return min_dist, max_dist, avg_dist
