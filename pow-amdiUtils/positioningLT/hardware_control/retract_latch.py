#!/usr/libexec/platform-python

import RPi.GPIO as GPIO
from settings import LATCH_PIN, LATCH_STATE_PIN
from settings import EXTEND_LATCH, RETRACT_LATCH, LATCH_EXTENDED 
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(LATCH_STATE_PIN, GPIO.IN)
GPIO.setup(LATCH_PIN, GPIO.OUT)

GPIO.output(LATCH_PIN, RETRACT_LATCH)
sleep(1)

if GPIO.input(LATCH_STATE_PIN) == LATCH_EXTENDED:
   exit(1)
else:
   exit(0)
