#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO

notes = dict(
    BL1=248,
    BL2=278,
    BL3=294,
    BL4=330,
    BL5=371,
    BL6=416,
    BL7=467,

    B1=495,
    B2=556,
    B3=624,
    B4=661,
    B5=742,
    B6=833,
    B7=935,

    BH1=990,
    BH2=1112,
    BH3=1178,
    BH4=1322,
    BH5=1484,
    BH6=1665,
    BH7=1869,

    NTC1=262,
    NTC2=294,
    NTC3=330,
    NTC4=350,
    NTC5=393,
    NTC6=441,
    NTC7=495,

    NTCL1=131,
    NTCL2=147,
    NTCL3=165,
    NTCL4=175,
    NTCL5=196,
    NTCL6=221,
    NTCL7=248,

    NTCH1=525,
    NTCH2=589,
    NTCH3=661,
    NTCH4=700,
    NTCH5=786,
    NTCH6=882,
    NTCH7=990,

    NTD0=-1,
    NTD1=294,
    NTD2=330,
    NTD3=350,
    NTD4=393,
    NTD5=441,
    NTD6=495,
    NTD7=556,

    NTDL1=147,
    NTDL2=165,
    NTDL3=175,
    NTDL4=196,
    NTDL5=221,
    NTDL6=248,
    NTDL7=278,

    NTDH1=589,
    NTDH2=661,
    NTDH3=700,
    NTDH4=786,
    NTDH5=882,
    NTDH6=990,
    NTDH7=1112,

    NTE1=330,
    NTE2=350,
    NTE3=393,
    NTE4=441,
    NTE5=495,
    NTE6=556,
    NTE7=624,

    NTEL1=165,
    NTEL2=175,
    NTEL3=196,
    NTEL4=221,
    NTEL5=248,
    NTEL6=278,
    NTEL7=312,

    NTEH1=661,
    NTEH2=700,
    NTEH3=786,
    NTEH4=882,
    NTEH5=990,
    NTEH6=1112,
    NTEH7=1248
)


class Beeper:

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(32, GPIO.OUT)
        self.p = GPIO.PWM(32, 240)

    def beep_tune(self, tune, duration):
        notes.get(tune)
        self.p.ChangeFrequency(notes.get(tune))
        time.sleep(duration)

    def beep_melody(self, melody):
        self.p.start(1)
        for (tune, duration) in melody:
            self.beep_tune(tune, duration)
        self.p.stop()

    def beep(self):
        self.beep_melody([('BH4', 1.),
                         ('NTC7', 1.)])

    def note_exists(self, tone):
        if tone in notes:
            return True
        else:
            return False

    def cleanup(self):
        GPIO.cleanup()
