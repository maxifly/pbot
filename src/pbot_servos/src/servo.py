import time

import RPi.GPIO as GPIO
import rospy

def cam_h_servo():
    return Servo(23, 0, 180)

def cam_v_servo():
    return Servo(21, 0, 100)




class Servo:
    def __init__(self, pwm_pin, min_bound, max_bound):
        self._pwm_pin = pwm_pin
        self._min_bound = min_bound
        self._max_bound = max_bound

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pwm_pin, GPIO.OUT, initial=GPIO.LOW)

        self._pwm = GPIO.PWM(self._pwm_pin, 50)
        self._pwm.start(0)

    def normalize(self, pos):
        return max(min(pos, self._max_bound), self._min_bound)

    def servo_appointed_detection(self, pos):
        n_pos = self.normalize(pos)
        # for i in range(18):
        self._pwm.ChangeDutyCycle(2.5 + 10 * n_pos / 180)
        time.sleep(0.02)
        self._pwm.ChangeDutyCycle(0)
        return n_pos
