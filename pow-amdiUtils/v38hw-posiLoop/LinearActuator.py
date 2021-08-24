#
#  This work is protected under applicable local and international
#  copyright laws.  Copying, transmitting, using, or any other activity
#  is expressly forbidden without prior signed written approval from
#  the copyright owner.
#
#  Copyright(c) 2019, 2020 Autonomous Medical Device, Incorporated,
#  ALL RIGHTS RESERVED.
#

import RPi.GPIO as GPIO
from time import sleep

class LinearActuator:
    def __init__(self, pin, move_time=0):
        """
        :param pin: Linear Actuiator pin number. Will be set as a PWM output
        :param move_time: time it takes for the linear actuiator to complete moving.
        """
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin,1000) # 1000 = Frequency
        self.pin = pin
        self.move_time = move_time

        # Enable the PWM pin
        GPIO.output(self.pin, GPIO.HIGH)
        self.pwm.start(0) # Default to min position

    # TODO: Full stroke time, % of move, optimize the 'sleep' time to as little as possible.

    def position(self, pos=None):
        """
        Get or set the position of the linear actuiator.
        :param pos: Int value - from 0 to 100 to determine position
        :return: current linear actuator position
        """
        if pos is None:
            return self.pos

        # Set the PWM pin
        self.pwm.start(pos)
        # And hold for as long as it takes to move
        sleep(self.move_time)
        # Turn off the PWM pin
        #self.pwm.stop()
        #GPIO.output(self.pin, GPIO.LOW)

        self.pos = pos
        return self.pos



