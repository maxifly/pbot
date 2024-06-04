#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO


class Beeper:

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(32, GPIO.OUT)
        self.p = GPIO.PWM(32, 240)


    def beep(self):
        self.p.start(50)
        self.p.ChangeDutyCycle(10)
        time.sleep(1.1)
        self.p.stop()

    def cleanup(self):
        GPIO.cleanup()

