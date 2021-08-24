#!/usr/bin/env python

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
import os  # I really didn't want to have to use this as a way of
           # shuffling a true global around via an environment variable
           # but I can't use from settings import <foo> as that doesn't
           # get set into the global namespce
import logging
import json


# Disk colour via an environment variable
if '_DISK_COLOUR' not in os.environ:
    os.environ['_DISK_COLOUR'] = 'UK'  # Unknown colour to be set by the assay QR code

# This is for testing when not on an RPi. You
# would also need to enable/disable import RPi.GPIO
RPI_PRESENT            = 1
CALIBRATION_FILE       = '/data/calibration_offsets.txt'
CALIBRATION_FILE2      = '/data/calibration_offset42.json'
CAL_DIR                = '/data/'        # '/data/check/'

PENDING_DIR            = '/data/pending/'
#OCI_APIGW_URL          = 'https://bgepkrbxrghxomxnazib2zey4q.apigateway.us-ashburn-1.oci.customer-oci.com/alpha/image'

OCI_DEST               = 'cs-qa-iad'     # cs-prod-iad for production
                                         # cs-dev-iad for development
                                         # cs-qa-iad for QA
OCI_APIGW_URL          = 'https://' + OCI_DEST + '.api.amdilabs.com/v1/image'


POSI_FILE              = '/data/posi_file.txt'
SERIAL_FILE            = '/sys/devices/virtual/dmi/id/product_serial'

LOGGING_FILE           = '/var/log/assay.log'
LOGGING_LEVEL          = (
                          # logging.NOTSET   # Almost everything and the kitchen sink
                          # logging.DEBUG    # MOST debug messages (see next variable)
                          logging.INFO     # Informational notices
                          # logging.WARNING  # Some attention required
                          # logging.ERROR    # Exception notices and some recoverable errors
                          # logging.CRITICAL # Game Over. Give up messages
                         )
DEBUGGING              = 1 # When crtl-c is used, drop into pdb debugger
TRINAMIC_DEBUGGING     = 0 # More verbose output about Trinamic controller commands (LOTS OF)

PROXY_FILE             = '/data/https_proxy'
VERSION_FILE           = '/etc/amdi-release'
HARDWARE_MODEL         = 'autolab20'
UUID_FILE              = '/data/deviceUUID.txt'
MOTOR_INITIALIZED      = '/data/motor_initialized'
SEND_DIRECTLY          = (
                          0  #  0 - dump into the pending directory for later upload
                          # 1  #  1 - send directly to server synchronously
                         )
DEFAULT_RECIPE         = 'MA1010-CD4536'  # default recipe when stuck

# DEFAULTS! some of these get overridden

# Encoder SI units. Although the encoder is a SI1000 is is QUAD encoded ie it
# has 1000 x4 positions it can register
SI_COUNT               = 4000  # Slot Indexes in one 360 turn of the encoder

# GPIO Pin definitions
# Pin numbers are BCM
LASER_STANDBY          = 22    # PWM position control
LASER_1_OUT_PIN        = 27    # Laser output on/off
LASER_2_OUT_PIN        = 17    # Laser 2 output on/off
LASER1_INTENSITY       = 500   # default laser power setting
LASER2_INTENSITY       = 500   # default laser power setting
INTENSITY_WET          = 800   # 'witness the power of this fully armed and operational battle station'
INTENSITY_DRY          = 500   # when no liquid is behind a valve


if RPI_PRESENT:
    LASER_ON           = GPIO.HIGH
    LASER_OFF          = GPIO.LOW

# Laser Addresses
LASER_1                = 0x51
LASER_2                = 0x52
LASER_TEMP_MULTIPLEXER = 0x70
LASER_TEMP_SENSOR_ADDR = 0x48
LASER_ENABLE           = 22
if RPI_PRESENT:
    LASER_POWER_ON     = GPIO.HIGH
    LASER_POWER_OFF    = GPIO.LOW

#lid photosensor 
LV1_PS_PIN = 23
LV2_PS_PIN = 24
if RPI_PRESENT:
    LV1_PS_ON          = GPIO.LOW
    LV1_PS_OFF         = GPIO.HIGH
    LV2_PS_ON          = GPIO.LOW
    LV2_PS_OFF         = GPIO.HIGH


# Camera/optical
CAMERA_SWITCH          = 5
CAMERA_BOOT_TIMEOUT    = 3       # time in sec camera should have booted up within
WHITE_LED              = 26
ORANGE_LED             = 20
if RPI_PRESENT:
    LED_ON             = GPIO.HIGH
    LED_OFF            = GPIO.LOW
    ORANGE_LED_ON      = LED_ON
    ORANGE_LED_OFF     = LED_OFF
    WHITE_LED_ON       = LED_ON
    WHITE_LED_OFF      = LED_OFF
    CAMERA_ON          = GPIO.HIGH
    CAMERA_OFF         = GPIO.LOW

# A20 components
MAGNETIC_INDEX_PIN     = 4      # Motor position input GPIO pin
BUMP                   = 100    # The offset in ticks that the homing pin
                                # activates before truly aligned with
                                # the hall sensor (typically 9 deg. HUGE!)
                                # Need to understand what this anomoly really is

COVER_CHECK_PIN        = 6
if RPI_PRESENT:
    COVER_OPEN         = GPIO.LOW
    COVER_CLOSED       = GPIO.HIGH
LATCH_STATE_PIN        = 16
if RPI_PRESENT:
    LATCH_EXTENDED     = GPIO.HIGH
    LATCH_RETRACTED    = GPIO.LOW
LATCH_PIN              = 21
if RPI_PRESENT:
    EXTEND_LATCH       = GPIO.LOW
    RETRACT_LATCH      = GPIO.HIGH

HOME_POLARITY          = 'AL'   # Active Low

# A20 Asssociated errors
LID_OPEN               = 100    # Lid open detected prior to test running (restartable)
LID_OPEN_TEST          = 101    # Lid open detected while test running (fail)
BAD_DISK               = 102    # invalid / unknown disk type
T74_MISSING            = -999   # impossible temperature to denote the T74 temperature sensor HW is missing
MPLX_MISSING           = -999   # Multiplexer HW missing

# Used in Motor.py (pid info is held in Motor.py itself. Change it there if need be)
MAX_VELOCITY           = 9000   # Never exceed this upper threshold

MAX_CURRENT            = 2500   # Max motor current allowed
MAX_START_CURRENT      = 625    # Maximum starting current [mA] when using hall commutation (15W motor on 24V)
I2T_CONSTANT           = 13200  # I²t winding constant -- UNMEASURED!!! FIND THIS FOR REAL OR BURN THE MOTOR!
I2T_LIMIT              = 211200 # (See page 54 of the 1640 firmware manual (section 9)
HOME_ACCEL             = 45     # RPM (in/de)crese per second used in .home()
HOME_SPEED             = 45     # turning speed rpm/min (the rpm is read in jumps of 15rpm)
NONSTOP                = 0      # Keep on trucking...
GENTLE                 = 1      # Slow down by friction
HARSH                  = 2      # EMF slowdown (can be very harsh)
CALCULATED             = 3      # spin to zero based on current rpm/acceleration

OVERCURRENT            = 0      # cmd: GAP 154, bit  0 = Overcurrent flag
UNDERVOLTAGE           = 1      # cmd: GAP 154, bit  1 = Undervoltage flag
OVERVOLTAGE            = 2      # cmd: GAP 154, bit  2 = Overvoltage flag
OVERTEMP               = 3      # cmd: GAP 154, bit  3 = Overtemperature flag
HALTED                 = 4      # cmd: GAP 154, bit  4 = Motor halted (switched off)
HALLERROR              = 5      # cmd: GAP 154, bit  5 = Hall sensor error flag
VELOCITY_MODE_ACTIVE   = 9      # cmd: GAP 154, bit  9 = Velocity mode active flag
POSITION_MODE_ACTIVE   = 10     # cmd: GAP 154, bit 10 = Position mode active flag
TORQUE_MODE_ACTIVE     = 11     # cmd: GAP 154, bit 11 = Torque mode active flag
MOVE_COMPLETED         = 14     # cmd: GAP 154, bit 14 = Position End Flag
I2T_EXCEEDED           = 17     # cmd: GAP 154, bit 17 = I²t exceeded flag (overload)

SETTLE                 = 0      # possible settling time for the trinamic board
                                # to be happy about it's positioning
                                # after a move is completed

# Foreword on the BURN_PATTERN
# Patterns are based like so:
#  A---------B---------C----------D----------E
#
# C - pinhole burn, fastest but the hole is small (~9.5 sec)
# BD - two pinpoint burns in one move (~8 sec)
# BD,DB - as above but attempts to connect the two pinpoints (poorly) (~11.5 sec)
# AE,EA - simple back and forth, disadvantage, endpoints get cooked, two moves (~12 sec)
# CE,EA,AC - start in middle, back and forth, 3 moves but cooks the middle (~15 sec)
# BE,EA,AD - as above but the start and endpoints are offset to try avoiding cooking the middle (~15 sec)
# None - do stop and start perforations (old faithful) (~15.5 sec)

BURN_PATTERN_L1        = 'C'    # 'C' Seems reasonably accurate
BURN_PATTERN_L2        = 'C'    # but when the target is smaller it can
                                # occasionally miss so you may want to use a sweep?

CHECK_LASER_POS        = 0      # Set to 1 to pause just before we start cutting
                                # each valve to check laser positioning
                                # you will need a terminal/keyboard to use this
                                # it will set the laser to it's LOWEST setting
                                # but immeadiately you press return it will switch
                                # to the higher setting and continue so DONT LOOK
                                # AT THE LIGHT when you go to press return!

def read_calibration_file(self):
    '''
    The Json file looks like this
    {
        "camera": {
            "exposure": 850000,
            "gain": 1,
            "gamma": 1,
            "offset": 2
        },
        "laser1": {
            "offset": 1755,
            "power": 1233
        },
        "laser2": {
            "offset": 2209,
            "power": 600
        }
    }

    Of the above variables only the laser poer/intensities are used in
    the settings.py file
    '''
    global LASER1_INTENSITY
    global LASER2_INTENSITY
    try:
        with open(CALIBRATION_FILE2,'r') as cal_file:
            calibration = json.load(cal_file)  # returns dict variable

            if 'camera' in calibration:
                try:
                    self.camera['offset']   = int(calibration['camera']['offset'])
                    self.camera['exposure'] = int(calibration['camera']['exposure'])
                    self.camera['gain']     = int(calibration['camera']['gain'])
                    self.camera['gamma']    = int(calibration['camera']['gamma'])
                except Exception as e:
                    print('Problem with %s: %s'
                          % (calibration['device'][i]['component'],
                            str(e)))
            if 'laser1' in calibration.keys():
                try:
                    self.laser1['offset']   = int(calibration['laser1']['offset'])
                    self.laser1['power']    = int(calibration['laser1']['power'])
                    LASER1_INTENSITY = self.laser1['power']
                except Exception as e:
                    print('Problem with %s: %s'
                          % (calibration['device'][i]['component'],
                            str(e)))
            if 'laser2' in calibration.keys():
                try:
                    self.laser2['offset']   = int(calibration['laser2']['offset'])
                    self.laser2['power']    = int(calibration['laser2']['power'])
                    LASER2_INTENSITY = self.laser2['power']

                except Exception as e:
                    print('Problem with %s: %s'
                          % (calibration['component'],
                            str(e)))
    except IOError:
        print("Unable to read %s - caution using generic DEFAULTS" % CALIBRATION_FILE2)


class calibration():
    def __init__(self):
        # The default values for the callibration variables
        self.camera = {
            'offset'   : 2585,
            'exposure' : 850000,
            'gain'     : 1,
            'gamma'    : 1,
            }
        self.laser1 = {
            'offset'   : 1755,
            'power'    : LASER1_INTENSITY,
            }
        self.laser2 = {
            'offset'   : 2209,
            'power'    : LASER2_INTENSITY,
            }
        # But try and use the calibration file if it exists
        read_calibration_file(self)

    def toJSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)


# Note you can only write about variables are they exist so this has
# to appear at this point in the code
def write_calibration_file(calibrations):
    try:
        with open(CALIBRATION_FILE2, 'w') as cal_file:
            if calibrations is not None:
                cal_file.write(calibrations.toJSON())
            else:
                print('Bad things happened, there is nothing to write')
    except IOError:
        print("Unable to write file %s" % CALIBRATION_FILE2)

'''
==============================================================================
This is simply to allow you to test the above routines 'standalone'
ie just run:
       python3 settings.py
by itself
------------------------------------------------------------------------------
'''

if __name__ == '__main__':
    x = calibration()  # access the callibration object
    print('THE ORIGINAL CALIBRATION OBJECT DEFAULTS:')
    print(x.toJSON())

    # Alternative way, just directly twiddle the object without referance
    print('----')
    print(calibration().laser1['power'])
    print('----')

    # Test: can re read in a json file?
    read_calibration_file(x)

    print
    print('CHECK we can read a global var')
    print('GLOBAL: LASER1_INTENSITY = %d' % LASER1_INTENSITY)
    print('CHECK: (obj).laser1[\'power\'] = %d' % x.laser1['power'])

    # Test: can we change a variable on the fly?
    print
    print('ATTEMPT TO CHANGE LASER POWER THROUGH OBJECT')
    LASER1_INTENSITY = x.laser1['power'] = 1234
    print('LASER1_INTENSITY %d' % LASER1_INTENSITY)

    # Test: can we write out a formatted file ok?
    print
    write_calibration_file(x)
    print(x.toJSON())
