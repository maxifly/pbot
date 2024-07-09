import time

import RPi.GPIO as GPIO
import rospy

def cam_h_servo():
    return Servo(23)

def normal_pos(pos):
    if pos < 0:
        return 0
    if pos > 180:
        return 180
    return pos


class Servo:
    def __init__(self, pwm_pin):
        self._pwm_pin = pwm_pin

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pwm_pin, GPIO.OUT, initial=GPIO.LOW)

        self._pwm = GPIO.PWM(self._pwm_pin, 50)
        self._pwm.start(0)

    def servo_appointed_detection(self, pos):
        n_pos = normal_pos(pos)
        for i in range(18):
            self._pwm.ChangeDutyCycle(2.5 + 10 * n_pos / 180)
            time.sleep(0.02)
            self._pwm.ChangeDutyCycle(0)
        return n_pos
