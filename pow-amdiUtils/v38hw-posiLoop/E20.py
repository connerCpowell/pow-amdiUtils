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
import sys
import atexit
import signal
import re
import string
import logging
import inspect
from pathlib import Path
import serial
import RPi.GPIO as GPIO
from Laser import Laser
from Camera import Camera
from Motor import Motor
from settings import LOGGING_FILE, SI_COUNT, CALIBRATION_FILE
from settings import LASER_1, LASER_1_OUT_PIN, LASER1_INTENSITY
from settings import LASER_2, LASER_2_OUT_PIN, LASER2_INTENSITY
from settings import BURN_PATTERN_L1, BURN_PATTERN_L2
# ~ from settings import LASER_ENABLE, LASER_POWER_ON, LASER_POWER_OFF
from settings import MAGNETIC_INDEX_PIN, LATCH_PIN, LATCH_STATE_PIN
from settings import EXTEND_LATCH, RETRACT_LATCH, LATCH_EXTENDED
from settings import COVER_CHECK_PIN, COVER_OPEN, LID_OPEN
from settings import MOVE_COMPLETED, SETTLE
from settings import DEFAULT_RECIPE, CHECK_LASER_POS
from settings import HOME_ACCEL, HOME_SPEED

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

    
    # define cover_open() ahead of where I need to use it as a callback in __init__
    def cover_open(self):
        # no matter what, if the cover is open at all, shut off the lasers
        if GPIO.input(COVER_CHECK_PIN) == COVER_OPEN:
            self.laser1.off()
            self.laser2.off()
            # if the test is done then we can safely open the cover without
            # aborting the test ( ie start a new one while other processes deal
            # with uploading the completed test data
            if self.end_of_test_operations == False:
                logging.info('Cover opened, laser power disabled, motor stopped')
                logging.info('Unfinished test! ABORT! (reject the disk and start over)')
                print('The lid MUST be closed throughout the test')
                sys.exit(-3)  # exit will call cleanup() used -3 to identify it came from here
            else:
                print('things are fine (somehow) continuing!')
                return True
        else:
            logging.info('Cover closed, laser power enabled')
            self.laser1.init_laser(self.laser1.devaddr)
            self.laser2.init_laser(self.laser2.devaddr)
        return False

    


    def __init__(self):

        
        self.end_of_test_operations = False

        # Ugh I really need 0 to mean zero or pass -1 or something that
        # REALLY stops the laser lasing when initialized
        self.laser1 = Laser(LASER_1_OUT_PIN, LASER_1, 0)
        self.laser2 = Laser(LASER_2_OUT_PIN, LASER_2, 0)
        self.laser1.off()
        self.laser2.off()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(COVER_CHECK_PIN, GPIO.IN)
        GPIO.setup(LATCH_STATE_PIN, GPIO.IN)
        GPIO.setup(LATCH_PIN, GPIO.OUT, initial=RETRACT_LATCH)

        # when a rising or falling edge is detected on the LATCH_STATE_PIN
        # jump to the defined callback routines
        # The edge must be set to RISING, FALLING or BOTH, we'll use BOTH
        GPIO.add_event_detect(COVER_CHECK_PIN, GPIO.BOTH, callback=lambda x: self.cover_open(), bouncetime=100)

        self.camera = Camera()

        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            #baudrate=115200,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        self.motor = Motor(self.ser, MAGNETIC_INDEX_PIN)
        atexit.unregister(self.cleanup)  # Make sure there is only the one registration
        atexit.register(self.cleanup, None, None)
        signal.signal(signal.SIGTERM, self.cleanup)
        signal.signal(signal.SIGINT, self.cleanup)


    def end_operations(self):
        GPIO.remove_event_detect(COVER_CHECK_PIN)
        self.end_of_test_operations = True

    def extend_latch(self):
        logging.info('Extending latch')
        GPIO.output(LATCH_PIN, EXTEND_LATCH)
        sleep(1)

    def retract_latch(self):
        logging.info('Retracting latch')
        GPIO.output(LATCH_PIN, RETRACT_LATCH)
        self.end_of_test_operations == True
        sleep(1)

    def latch_extended(self):
        if GPIO.input(LATCH_STATE_PIN) == LATCH_EXTENDED:
            logging.info('Latch extended')
            self.end_of_test_operations == False
            return True
        else:
            logging.info('Latch retracted')
            self.end_of_test_operations == True
            return False

    def run_assay(self, filename=None, version='01'):
        if self.cover_open():
            print('ERROR: Please close lid before starting test')
            logging.info('Test stopped, lid/cover open')
            return LID_OPEN

        # Turn the camera on preemptively
        self.camera.power_on_camera()
        self.extend_latch()

        if (self.cover_open() and not self.latch_extended()):
            logging.info('Test stopped, lid/cover open')
            print('ERROR: Please close lid before starting test')
            return LID_OPEN

        import importlib
        try:
            assay = importlib.import_module('assays.' + filename[0:13] + '-' + version)
            assay_return = assay.run_assay(self, filename)
        except ModuleNotFoundError:
            # Ugh, fine, we don't have a recipe module for <MAxxxx-CDyyyy>
            # Can we try the default if it exists? If it doesn't then bailout.
            # We also need to change the
            default = importlib.import_module('assays.' + DEFAULT_RECIPE + '-' + version)
            if default is not None:
                print('Requested %s recipe not found. Using default [%s]' % (filename[0:13], DEFAULT_RECIPE))
                logging.info('Requested %s recipe not found. Using default [%s]' % (filename[0:13], DEFAULT_RECIPE))
                default_return = default.run_assay(self, filename)
            else:
                raise ModuleNotFoundError

        self.retract_latch()

        return 0

    def laser_sweep(self,
                    start_tick,
                    width,
                    delay=0.0,
                    steps=None,
                    laser_intensity=LASER1_INTENSITY,
                    pattern=BURN_PATTERN_L1,
                    lv='0'):

        '''
        Sweep the laser through the disk from start_tick - width to start_tick + width
        All angles are absolute and measured /clockwise/ eg 0->90 deg -> 12->3 o'clock
        but are in encoder SI ticks
        '''
        #                  start_tick
        #  |-------------------0---------------------|
        #  smallest          middle            largest
        #  |----- width -------|

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

        smallest  = (start_tick - width) % SI_COUNT  # smallest tick value
        largest   = (start_tick + width) % SI_COUNT  # largest tick value
        middle    = start_tick % SI_COUNT

        # This is just for self interest
        start_time = time()

        self.motor.set_velocity(HOME_SPEED)
        self.motor.set_acceleration(HOME_ACCEL)
        # print('inside sweep')
        # logging.info('insder sweeper')

                # --------------------------------------------------------------
        if pattern == 'C':
            # Move to the starting position, wait until the move is completed (A)
            self.motor.position_abs(middle)
            # print('inside C')
            # logging.info('insder C')
            
           
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            posi2 = self.motor.get_actual_encoder_count()
            # print('posi', posi2)
            # logging.info('insder %d' % posi2)

            logging.info('POSI: position b4 lv #%s C: %d' % (lv, posi2))

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'C\': %d Actually at: %d' \
                      % (middle, self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser1.set_intensity(LASER_1, laser_intensity)
            # Pause laser at lowest intensity to check position
            self.laser1.on()

            posi = self.motor.get_actual_encoder_count()
            logging.info('POSI: position af lv#%s C: %d' % (lv, posi))

            period = time()
            while (time() - period) <= (delay * 10):
                pass

        # --------------------------------------------------------------
        elif pattern == 'BD':
            # Move to the starting position, wait until the move is completed (B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'B\': %d Actually at: %d' \
                      % (round(middle - (width/2)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser1.set_intensity(LASER_1, laser_intensity)
            self.laser1.on()

            # Return to origin and wait until the move is completed (B-D)
            self.motor.position_rel(round(width + (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'BD,DB':
            # Move to the starting position (B), wait until the move is completed (B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'B\': %d Actually at: %d' \
                      % (round(middle - (width/2)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser1.set_intensity(LASER_1, laser_intensity)
            self.laser1.on()

            # move to (D) and wait until the move is completed (B-D)
            self.motor.position_rel(round(width + (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to the starting position (B), wait until the move is completed (D-B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'AE,EA':
            # Move to the starting position, wait until the move is completed (A)
            self.motor.position_abs(smallest)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'A\': %d Actually at: %d' \
                      % (smallest,
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser1.set_intensity(LASER_1, laser_intensity)
            self.laser1.on()

            # cut to the other edge and wait until the move is completed (A-E)
            self.motor.position_rel(2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to origin and wait until the move is completed (E-A)
            self.motor.position_rel(-2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'CA,AE,EC':
            # Move to the starting position, wait until the move is completed (C)
            self.motor.position_abs(middle)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'C\': %d Actually at: %d' \
                      % (middle,
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser1.set_intensity(LASER_1, laser_intensity)
            self.laser1.on()

            # cut one way and wait until the last move is completed (C-A)
            self.motor.position_rel(-width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # cut to the other edge and wait until the move is completed (A-E)
            self.motor.position_rel(2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to origin and wait until the move is completed (E-C)
            self.motor.position_rel(-width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'BE,EA,AD':
            # Move to the starting position, wait until the move is completed (B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'B\': %d Actually at: %d' \
                      % (round(middle - (width/2)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser1.set_intensity(LASER_1, laser_intensity)
            self.laser1.on()

            # cut one way and wait until the last move is completed (B-E)
            self.motor.position_rel(round((width/2) + width))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # cut to the other edge and wait until the move is completed (E-A)
            self.motor.position_rel(-2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to origin and wait until the move is completed (A-D)
            self.motor.position_rel(round(width + (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern is None:
            tolerance = round(SI_COUNT/360)  # is one degree max tolerance acceptable?

            # This is just for self interest
            start_time = time()

            self.motor.position_abs(smallest)

            # Wait until the last move is completed
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Calculate the step size or try and use the value passsed (if any)
            # angular increment should not exceed our preset tolerance of 1 deg
            if steps is not None:
                step = round(width/steps)  # needs to be an integer value
            else:
                step = round(width/4)  # number of steps to move

            # Sanity check
            if step == 0 or step > round(tolerance):
                msg = 'Calculated step size (%d) was either too small or too big, using (%d) instead' \
                      % (step,
                         round(tolerance))
                logging.info(msg)
                step = round(tolerance)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser1.set_intensity(LASER_1, 0)
                self.laser1.on()
                print('Supposed to be at pos \'A\': %d Actually at: %d' \
                      % (smallest,
                      self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            self.laser1.set_intensity(LASER_1, laser_intensity)
            self.laser1.on()

            # Cut equal distance either side of the start angle
            increment = smallest
            while increment <= (start_tick + width):  # DONT compare with modulo (largest)!
                increment += step
                msg = 'Laser1 ON: cutting through %d (%d deg)' \
                      % (increment,
                         increment/(SI_COUNT/360))
                logging.info(msg)
                print(msg)
                self.motor.position_rel(step)
                sleep(delay)  # wait a moment for the laser to actually puncture the valve

        # --------------------------------------------------------------
        else:
            msg = 'Unknown cutting pattern! [%s] cheque fer typ0s!' \
                  % pattern
            logging.info(msg)
            print(msg)

        # Tidyup and some info
        self.laser1.off()
        elapsed_time = time() - start_time

        if pattern is None:
            msg = 'Valve perforation cut took %0.2f sec @ %dma pwr' \
                  % (elapsed_time,
                     laser_intensity)
        else:
            msg = 'Valve cut using pattern %s took %0.2f sec @ %dma pwr' \
                  % (pattern,
                     elapsed_time,
                     laser_intensity)

        logging.info(msg)
        print(msg)

        self.motor.set_acceleration(self.motor.acceleration_pos_ctrl)
        self.motor.set_velocity(0)

        return 0

    def laser2_sweep(self,
                     start_tick,
                     width,
                     delay=0.0,
                     steps=None,
                     laser_intensity=LASER2_INTENSITY,
                     pattern=BURN_PATTERN_L2,
                     lv='0'):
        '''
        Sweep the laser through the disk from start_tick - width to start_tick + width
        All angles are absolute and measured /clockwise/ eg 0->90 deg -> 12->3 o'clock
        but are in encoder SI ticks
        '''
        #                  start_tick
        #  |-------------------0---------------------|
        #  smallest          middle            largest
        #  |----- width -------|

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

        smallest  = (start_tick - width) % SI_COUNT  # smallest tick value
        largest   = (start_tick + width) % SI_COUNT  # largest tick value
        middle    = start_tick % SI_COUNT

        # This is just for self interest
        start_time = time()

        self.motor.set_velocity(HOME_SPEED)
        self.motor.set_acceleration(HOME_ACCEL)

        # --------------------------------------------------------------
        if pattern == 'C':
            # Move to the starting position, wait until the move is completed (A)
            self.motor.position_abs(middle)
            
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle
            posi2 = self.motor.get_actual_encoder_count()
            logging.info('POSI: position b4 lv #%s C: %d' % (lv, posi2))

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'C\': %d Actually at: %d' \
                      % (middle,
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser2.off()

            # Setup the laser to start cutting
            self.laser2.set_intensity(LASER_2, laser_intensity)
            self.laser2.on()

            posi = self.motor.get_actual_encoder_count()
            logging.info('POSI: position af lv#%s C: %d' % (lv, posi))

            period = time()
            while (time() - period) <= (delay * 10):
                pass

        # --------------------------------------------------------------
        elif pattern == 'BD':
            # Move to the starting position, wait until the move is completed (B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'B\': %d Actually at: %d' \
                      % (round(middle - (width/2)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser2.off()

            # Setup the laser to start cutting
            self.laser2.set_intensity(LASER_2, laser_intensity)
            self.laser2.on()

            # Return to origin and wait until the move is completed (B-D)
            self.motor.position_rel(round(width + (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'BD,DB':
            # Move to the starting position (B), wait until the move is completed (B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'B\': %d Actually at: %d' \
                      % (round(middle - (width/2)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser2.off()

            # Setup the laser to start cutting
            self.laser2.set_intensity(LASER_2, laser_intensity)
            self.laser2.on()

            # move to (D) and wait until the move is completed (B-D)
            self.motor.position_rel(round(width + (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to the starting position (B), wait until the move is completed (D-B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'AE,EA':
            # Move to the starting position, wait until the move is completed (A)
            self.motor.position_abs(smallest)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'A\': %d Actually at: %d' \
                      % (round(middle - (width)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser2.set_intensity(LASER_2, laser_intensity)
            self.laser2.on()

            # cut to the other edge and wait until the move is completed (A-E)
            self.motor.position_rel(2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to origin and wait until the move is completed (E-A)
            self.motor.position_rel(-2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'CA,AE,EC':
            # Move to the starting position, wait until the move is completed (C)
            self.motor.position_abs(middle)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'C\': %d Actually at: %d' \
                      % (middle,
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser2.set_intensity(LASER_2, laser_intensity)
            self.laser2.on()

            # cut one way and wait until the last move is completed (C-A)
            self.motor.position_rel(-width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # cut to the other edge and wait until the move is completed (A-E)
            self.motor.position_rel(2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to origin and wait until the move is completed (E-C)
            self.motor.position_rel(-width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern == 'BE,EA,AD':
            # Move to the starting position, wait until the move is completed (B)
            self.motor.position_abs(round(middle - (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Captain SLOW.... to.. the .. rescuuuuuueee
            self.motor.set_velocity(0)
            self.motor.set_acceleration(1)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'B\': %d Actually at: %d' \
                      % (round(middle - (width/2)),
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser1.off()

            # Setup the laser to start cutting
            self.laser2.set_intensity(LASER_1, laser_intensity)
            self.laser2.on()

            # cut one way and wait until the last move is completed (B-E)
            self.motor.position_rel(round((width/2) + width))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # cut to the other edge and wait until the move is completed (E-A)
            self.motor.position_rel(-2*width)
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            # Return to origin and wait until the move is completed (A-D)
            self.motor.position_rel(round(width + (width/2)))
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

        # --------------------------------------------------------------
        elif pattern is None:
            tolerance = round(SI_COUNT/360)  # is one degree max tolerance acceptable?

            # This is just for self interest
            start_time = time()

            self.motor.position_abs(smallest)

            # Wait until the last move is completed
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass
            sleep(SETTLE)  # let trinamic board positioning settle

            # Calculate the step size or try and use the value passsed (if any)
            # angular increment should not exceed our preset tolerance of 1 deg
            if steps is not None:
                step = round(width/steps)  # needs to be an integer value
            else:
                step = round(width/4)  # number of steps to move

            # Sanity check
            if step == 0 or step > round(tolerance):
                msg = 'Calculated step size (%d) was either too small or too big, using (%d) instead' \
                      % (step,
                         round(tolerance))
                logging.info(msg)
                step = round(tolerance)

            # Check if we pause laser at lowest intensity to check position
            if CHECK_LASER_POS == 1:
                self.laser2.set_intensity(LASER_2, 0)
                self.laser2.on()
                print('Supposed to be at pos \'A\': %d Actually at: %d' \
                      % (smallest,
                         self.motor.get_actual_encoder_count()))
                input('Press RETURN to move on after checking position')
                self.laser2.off()

            self.laser2.set_intensity(LASER_2, laser_intensity)
            self.laser2.on()

            # Cut equal distance either side of the start angle
            increment = smallest
            while increment <= (start_tick + width):  # DONT compare with modulo (largest)!
                increment += step
                msg = 'Laser2 ON: rotating cartridge clockwise to: %d (%d deg)' \
                      % (increment,
                         increment/(SI_COUNT/360))
                logging.info(msg)
                print(msg)
                self.motor.position_rel(step)
                sleep(delay)  # wait a moment for the laser to actually puncture the valve

        # --------------------------------------------------------------
        else:
            msg = 'Unknown cutting pattern! [%s] cheque fer typ0s!' \
                  % pattern
            logging.info(msg)
            print(msg)

        # Tidyup and some info
        self.laser2.off()
        elapsed_time = time() - start_time

        if pattern is None:
            msg = 'Valve perforation cut took %0.2f sec @ %dma pwr' \
                  % (elapsed_time,
                     laser_intensity)
        else:
            msg = 'Valve cut using pattern %s took %0.2f sec @ %dma pwr' \
                  % (pattern,
                     elapsed_time,
                     laser_intensity)

        logging.info(msg)
        print(msg)

        self.motor.set_acceleration(self.motor.acceleration_pos_ctrl)
        self.motor.set_velocity(0)

        return 0

    def cleanup(self, signum, frame):
        atexit.unregister(self.cleanup)
        self.laser1.off()
        self.laser2.off()
        self.motor.motor_release()
        self.end_operations()
        self.camera.close()
        self.retract_latch()
        self.ser.close()
        sys.exit(-1)

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
            msg = 'No configuration file found,.. winging it!'
            logging.info(msg)
            print(msg)
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
