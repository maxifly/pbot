import RPi.GPIO as GPIO


def left_motor():
    return Motor(38, 40, 36)


def right_motor():
    return Motor(35, 37, 33)


def normal_speed(speed):
    abs_speed = abs(speed)

    if abs_speed < 0:
        return 0
    if abs_speed > 100:
        return 100

    return abs_speed


class Motor:
    def __init__(self, forward_pin, backward_pin, pwm_pin):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin
        self._pwm_pin = pwm_pin

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._forward_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._backward_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        self._pwm = GPIO.PWM(self._pwm_pin, 2000)
        self._pwm.start(0)

    def forward(self, speed):
        if speed == 0:
            self.stop()
            return
        GPIO.output(self._forward_pin, GPIO.HIGH)
        GPIO.output(self._backward_pin, GPIO.LOW)
        self._pwm.ChangeDutyCycle(normal_speed(speed))

    def backward(self, speed):
        if speed == 0:
            self.stop()
            return
        GPIO.output(self._forward_pin, GPIO.LOW)
        GPIO.output(self._backward_pin, GPIO.HIGH)
        self._pwm.ChangeDutyCycle(normal_speed(speed))

    def stop(self):
        GPIO.output(self._forward_pin, GPIO.LOW)
        GPIO.output(self._backward_pin, GPIO.LOW)
        self._pwm.ChangeDutyCycle(0)

    def cleanup(self):
        GPIO.cleanup()
