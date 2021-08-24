'''
#
#  This work is protected under applicable local and international
#  copyright laws.  Copying, transmitting, using, or any other activity
#  is expressly forbidden without prior signed written approval from
#  the copyright owner.
#
#  Copyright(c) 2019, 2020 Autonomous Medical Device, Incorporated,
#  ALL RIGHTS RESERVED.
#

#           E20 Hardware Control Class
#
#
# Hardware includes:
#  - Brushless DC Motor  - Spin the sample.  - PWM
#  - Laser               - Laser on/off      - GPIO : DO
#  - Magnetic indicator  - Index             - GPIO : DI
#  - LED Ring            - Lighting          - GPIO:  DO
#  - Camera              -                   - USB
#
'''

from time import sleep
from time import time
import atexit
import signal
import re
import string
import logging
import serial
import inspect
import RPi.GPIO as GPIO
from Laser import Laser
from Camera import Camera
from Motor import Motor
from settings import LOGGING_FILE, SI_COUNT, CALIBRATION_FILE
from settings import LASER_1, LASER_1_OUT_PIN, LASER1_INTENSITY, INTENSITY_WET
from settings import LASER_2, LASER_2_OUT_PIN, LASER2_INTENSITY, INTENSITY_DRY
from settings import MAGNETIC_INDEX_PIN, LATCH_PIN, LATCH_STATE_PIN
from settings import EXTEND_LATCH, RETRACT_LATCH, LATCH_EXTENDED
from settings import COVER_CHECK_PIN, COVER_OPEN, LID_OPEN
from settings import MOVE_COMPLETED

logging.basicConfig(
        filename=LOGGING_FILE,
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(module)s - %(funcName)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S')

def lineno():  # returns string
    """Returns the current line number in our program."""
    return (__file__.split('/')[len(__file__.split('/'))-1]
            + ': '
            + str(inspect.currentframe().f_back.f_code.co_name)
            + '() +'
            + str(inspect.currentframe().f_back.f_lineno))

class E20:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(COVER_CHECK_PIN, GPIO.IN)
        GPIO.setup(LATCH_STATE_PIN, GPIO.IN)
        GPIO.setup(LATCH_PIN, GPIO.OUT, initial=RETRACT_LATCH)

        # Ugh I really need 0 to mean zero or pass -1 or something that
        # REALLY stops the laser lasing when initialized
        self.laser1 = Laser(LASER_1_OUT_PIN, LASER_1, 0)
        self.laser2 = Laser(LASER_2_OUT_PIN, LASER_2, 0)
        self.laser1.off()
        self.laser2.off()

        self.camera = Camera()

        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            #baudrate=115200,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.motor = Motor(self.ser, MAGNETIC_INDEX_PIN)
        atexit.register(self.cleanup, None, None)
        signal.signal(signal.SIGTERM, self.cleanup)
        signal.signal(signal.SIGINT, self.cleanup)

    def extend_latch(self):
        logging.info('Extending latch')
        GPIO.output(LATCH_PIN, EXTEND_LATCH)
        sleep(1)

    def retract_latch(self):
        logging.info('Retracting latch')
        GPIO.output(LATCH_PIN, RETRACT_LATCH)
        sleep(1)

    def cover_open(self):
        if GPIO.input(COVER_CHECK_PIN) == COVER_OPEN:
            logging.info('Cover open')
            return True
        else:
            logging.info('Cover closed')
        return False

    def latch_extended(self):
        if GPIO.input(LATCH_STATE_PIN) == LATCH_EXTENDED:
            logging.info('Latch extended')
            return True
        else:
            logging.info('Latch retracted')
            return False

    def run_assay(self, filename=None, version='01'):
        if self.cover_open():
            print('ERROR: Please close lid before starting test')
            logging.info('Test stopped, lid/cover open')
            return LID_OPEN

        self.extend_latch()

        if (self.cover_open() and not self.latch_extended()):
            logging.info('Test stopped, lid/cover open')
            print('ERROR: Please close lid before starting test')
            return LID_OPEN

        import importlib
        assay = importlib.import_module('assays.' + filename[0:13] + '-' +  version)
        assay_return = assay.run_assay(self, filename)

        self.retract_latch()

        return 0

    def fire_laser1(self, t, laser_intensity=INTENSITY_WET):
        self.laser1.set_intensity(LASER_1, 200)
        self.laser1.on()
        sleep(t)
        self.laser1.off()

    def fire_laser2(self, t, laser_intensity=INTENSITY_WET):
        self.laser2.set_intensity(LASER_2, 200)
        self.laser2.on()
        sleep(t)
        self.laser2.off()
        


    def laser_sweep(self, start_tick, width, delay=0.0, steps=None, laser_intensity=INTENSITY_WET):
        '''
        Sweep the laser through the disk from start_tick-width to start_tick+width
        All angles are absolute and measured /clockwise/ eg 0->90 deg -> 12->3 o'clock
        but are in encoder SI ticks
        '''
        # Idea here is simply to move slowly one degree at a time as moving from say 35-42 deg
        # at the motors slowest rotation speed is too quick to make a useful cut.
        smallest  = (start_tick - width) #% SI_COUNT  # smallest tick value # NO MORE MODULOS
        largest   = (start_tick + width) #% SI_COUNT  # largest tick value # NO MORE MODULOS
        middle    = start_tick #% SI_COUNT
        
        tolerance = round(SI_COUNT/360)  # is one degree max tolerance acceptable?
        self.motor.set_acceleration(40)
		
		
        # This is just for self interest
        start_time = time()
             
        self.motor.position_abs(smallest)
        
        # disable null/encoder reset
        #self.motor.encoder_set_null(1)

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass

        # Calculate the step size or try and use the value passsed (if any)
        # angular increment should not exceed our preset tolerance of 1 deg
        if steps is not None:
            step = 4 #round(width/steps)  # needs to be an integer value
        else:
            step = 4 #round(width/4)  # number of steps to move

            # hehehe sorry bryce but we found a step size of 4 with the back & forth sweep cut the valve well and in (~35 seconds)

        # Sanity check
        if step == 0 or step > round(tolerance):
            logging.info('Calculated step size (%d) was either too small or too big, using (%d) instead'
                         % (step, round(tolerance)))
            step = round(tolerance)

        self.laser1.set_intensity(LASER_1, laser_intensity)
        self.laser1.on()

        # Cut equal distance either side of the start angle
        increment = smallest
        while increment <= largest: # we can go back to using largest since we do not have to limit ourselves to modulos
            increment += step
            logging.info('Laser1 ON: rotating cartridge clockwise to: %d (%d deg)' % (increment, increment/(SI_COUNT/360)))
            print('Laser1 ON: cutting through %d (%d deg)' % (increment, increment/(SI_COUNT/360)))
            self.motor.position_rel(step)
            while not self.motor.info(check_bit=MOVE_COMPLETED): # instead of a sleep delay, wait for motor to provide move completed status before moving to next step
                pass
            #sleep(delay)  # wait a moment for the laser to actually puncture the valve

        # sweep back across cut
        while increment >= smallest: 
            increment -= step
            logging.info('Laser1 ON: rotating cartridge clockwise to: %d (%d deg)' % (increment, increment/(SI_COUNT/360)))
            print('Laser1 ON: cutting through %d (%d deg)' % (increment, increment/(SI_COUNT/360)))
            self.motor.position_rel(-1*step)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            #sleep(delay)  # wait a moment for the laser to actually puncture the valve

        self.laser1.off()

        # re-enable null/encoder reset
        #self.motor.encoder_set_null(1)         # zero at next N channel event

        # Just for self interest
        elapsed_time = time() - start_time
        logging.info('Valve cut took %0.2f sec' % elapsed_time)
        print('Valve cut took %0.2f sec' % elapsed_time)

        return 0

    def laser2_sweep(self, start_tick, width, delay=0.0, steps=None, laser_intensity=INTENSITY_WET):
        '''
        Sweep the laser through the disk from start_tick-width to start_tick+width
        All angles are absolute and measured /clockwise/ eg 0->90 deg -> 12->3 o'clock
        but are in encoder SI ticks
        '''
        # Idea here is simply to move slowly one degree at a time as moving from say 35-42 deg
        # at the motors slowest rotation speed is too quick to make a useful cut.
        smallest  = (start_tick - width) % SI_COUNT  # smallest tick value
        largest   = (start_tick + width) % SI_COUNT  # largest tick value
        middle    = start_tick % SI_COUNT

        tolerance = round(SI_COUNT/360)  # is one degree max tolerance acceptable?
        self.motor.acceleration(40)

        # This is just for self interest
        start_time = time()

        self.motor.position_abs(smallest)

        # disable null/encoder reset so we can count past 4000 for relative movement
        # self.motor.encoder_set_null(0)

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass

        # Calculate the step size or try and use the value passsed (if any)
        # angular increment should not exceed our preset tolerance of 1 deg
        if steps is not None:
            step = round(width/steps)  # needs to be an integer value
        else:
            step = round(width/4)  # number of steps to move

        # Sanity check
        if step == 0 or step > round(tolerance):
            logging.info('Calculated step size (%d) was either too small or too big, using (%d) instead' 
                         % (step, round(tolerance)))
            step = round(tolerance)

        self.laser2.set_intensity(LASER_2, laser_intensity)
        self.laser2.on()

        # Cut equal distance either side of the start angle
        increment = smallest
        while increment <= (start_tick + width):  # DONT compare with modulo (largest)!
            increment += step
            logging.info('Laser2 ON: rotating cartridge clockwise to: %d (%d deg)' % (increment, increment/(4000/360)))
            print('Laser2 ON: cutting through %d (%d deg)' % (increment, increment/(4000/360)))
            self.motor.position_rel(step)
            sleep(delay)  # wait a moment for the laser to actually puncture the valve

        self.laser2.off()

        # re-enable null/encoder reset
        # self.motor.encoder_set_null(1)         # zero at next N channel event

        # Just for self interest
        elapsed_time = time() - start_time
        logging.info('Valve cut took %0.2f sec' % elapsed_time)
        print('Valve cut took %0.2f sec' % elapsed_time)
        return 0

    def cleanup(self, signum, frame):
        self.laser1.off()
        self.laser2.off()
        self.motor.motor_release()
        sleep(0.25) # allow controller to respond
        self.ser.close()
        self.camera.close()
        self.retract_latch()


    def get_calibration_from_file(self, find_cal_for='CAMERA_OFFSET'):
        '''
        Find the calibration for camera/laser1/laser2
        CALIBRATION_FILE is defined in settings.py
        Expected formati in ticks:
        CAMERA_OFFSET=0
        LV1_OFFSET=121
        LV2_OFFSET=88
        returns either a matched value or a computed value
        and True/False if a calibration file was used
        '''
        cal = 2666  # Default

        try:
            calibration_file = open(CALIBRATION_FILE, 'r')
            for line in calibration_file:
                if re.match(find_cal_for, line):
                    cal = int(line.split('=')[1])  # grab any number after the '='
            calibration_file.close()

        except Exception as e:
            # return 'perfect' defaults if the calibration file
            # is missing anything. These are hard coded so its really
            # not a good thing
            # Maybe log something about missing calibration?
            # print('Error: ' + str(e))
            logging.info('No configuration file found,.. winging it!')
            print('No configuration file found,.. winging it!')
            if find_cal_for == 'CAMERA_OFFSET':
                # camera at 270, but 1st well is @ 30 deg
                cam = (((270 - 30) * SI_COUNT/360))  # 2666 240deg
                cam = (cam + (SI_COUNT/2) + SI_COUNT) % SI_COUNT  # sigh ok so they take the OTHER well measurement
                print('perfect cam = %d (%d deg)' % (cam, (cam/(SI_COUNT/360) % SI_COUNT)))
                return (cam, False)

            if find_cal_for == 'LV1_OFFSET':
                # Laser1 is at 180 & lv1 is @ 11 deg
                lv1 =  (((180 - 11) * SI_COUNT/360) + SI_COUNT) % SI_COUNT  # 1877 167deg
                print('perfect lv1 = %d (%d deg)' % (lv1, (lv1/(SI_COUNT/360) % SI_COUNT)))
                return (lv1, False)

            if find_cal_for == 'LV2_OFFSET':
                # Laser2 is at 75 & lv2 is @ 48 deg (90-48)=42
                # but the lv2 position is a further 75 deg, all mirrored
                lv2 = (((360 - 105 - 48)* SI_COUNT/360) + SI_COUNT) % SI_COUNT  # 2300 207deg
                print('perfect lv2 = %d (%d deg)' % (lv2, (lv2/(SI_COUNT/360) % SI_COUNT)))
                return (lv2, False)

        return cal, True
