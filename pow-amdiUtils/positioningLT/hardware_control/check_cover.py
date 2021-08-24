#!/usr/libexec/platform-python

import RPi.GPIO as GPIO
from settings import *

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(COVER_CHECK_PIN, GPIO.IN)

if GPIO.input(COVER_CHECK_PIN) == COVER_OPEN:
   exit(1)
else:
   exit(0)

