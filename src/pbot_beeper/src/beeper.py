#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO

notes1 = dict(
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

notes = dict(
    PAUSE=1,
    B0=31,
    C1=33,
    CS1=35,
    D1=37,
    DS1=39,
    E1=41,
    F1=44,
    FS1=46,
    G1=49,
    GS1=52,
    A1=55,
    AS1=58,
    B1=62,
    C2=65,
    CS2=69,
    D2=73,
    DS2=78,
    E2=82,
    F2=87,
    FS2=93,
    G2=98,
    GS2=104,
    A2=110,
    AS2=117,
    B2=123,
    C3=131,
    CS3=139,
    D3=147,
    DS3=156,
    E3=165,
    F3=175,
    FS3=185,
    G3=196,
    GS3=208,
    A3=220,
    AS3=233,
    B3=247,
    C4=262,
    CS4=277,
    D4=294,
    DS4=311,
    E4=330,
    F4=349,
    FS4=370,
    G4=392,
    GS4=415,
    A4=440,
    AS4=466,
    B4=494,
    C5=523,
    CS5=554,
    D5=587,
    DS5=622,
    E5=659,
    F5=698,
    FS5=740,
    G5=784,
    GS5=831,
    A5=880,
    AS5=932,
    B5=988,
    C6=1047,
    CS6=1109,
    D6=1175,
    DS6=1245,
    E6=1319,
    F6=1397,
    FS6=1480,
    G6=1568,
    GS6=1661,
    A6=1760,
    AS6=1865,
    B6=1976,
    C7=2093,
    CS7=2217,
    D7=2349,
    DS7=2489,
    E7=2637,
    F7=2794,
    FS7=2960,
    G7=3136,
    GS7=3322,
    A7=3520,
    AS7=3729,
    B7=3951,
    C8=4186,
    CS8=4435,
    D8=4699,
    DS8=4978
)


class Beeper:

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(32, GPIO.OUT)
        self.p = GPIO.PWM(32, 240)

    def beep_tune(self, tune, duration):
        if tune == 'PAUSE':
            time.sleep(duration)
        else:
            self.p.ChangeFrequency(notes.get(tune))
            self.p.start(1)
            notes.get(tune)
            time.sleep(duration)
            self.p.stop()
    # def beep_tune(self, tune, duration):
    #     notes.get(tune)
    #     self.p.ChangeFrequency(notes.get(tune))
    #     time.sleep(duration)

    # def beep_melody(self, melody):
    #     self.p.start(1)
    #     for (tune, duration) in melody:
    #         self.beep_tune(tune, duration)
    #     self.p.stop()

    def beep_melody(self, melody, tempo):
        for (tune, duration) in melody:
            self.beep_tune(tune, duration)
            time.sleep(duration * tempo)

    def beep(self):
        self.beep_melody([('AS4', 1.),
                          ('F4', 1.)], 0.25)

    def note_exists(self, tone):
        if tone in notes:
            return True
        else:
            return False

    def cleanup(self):
        GPIO.cleanup()
