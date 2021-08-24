#!/usr/libexec/platform-python
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
#
'''

import time
import sys
import logging
import inspect  # for line numbers
import RPi.GPIO as GPIO
import os
import re
from TrinamicMotor import TrinamicMotor
from pathlib import Path
from settings import LOGGING_FILE, LOGGING_LEVEL, TRINAMIC_DEBUGGING
from settings import SI_COUNT, HOME_ACCEL, HOME_SPEED
from settings import NONSTOP, GENTLE, HARSH, CALCULATED
from settings import MOVE_COMPLETED, HALTED, VELOCITY_MODE_ACTIVE, POSITION_MODE_ACTIVE
from settings import MOTOR_INITIALIZED, SETTLE
from settings import MAX_CURRENT, MAX_START_CURRENT, MAX_VELOCITY
from settings import I2T_CONSTANT, I2T_LIMIT

if not logging.getLogger().hasHandlers():
    logging.getLogger('').setLevel(LOGGING_LEVEL)

    def create_logfile(filename, level=logging.INFO):
        handler = logging.FileHandler(filename)
        handler.setLevel(level)
        formatter = logging.Formatter('%(asctime)s %(levelname) -8s %(module) -16s - %(funcName)s: %(message)s',
                                      datefmt='%Y-%m-%d %H:%M:%S')
        handler.setFormatter(formatter)
        logging.getLogger('').addHandler(handler)

    create_logfile(filename=LOGGING_FILE, level=LOGGING_LEVEL)

def lineno():  # returns string
    """Returns the current line number in our program."""
    return (__file__.split('/')[len(__file__.split('/'))-1]
            + ': '
            + str(inspect.currentframe().f_back.f_code.co_name)
            + '() +'
            + str(inspect.currentframe().f_back.f_lineno)
            + ' via '
            + str(inspect.currentframe().f_back.f_back.f_code.co_name)
            + '() +'
            + str(inspect.currentframe().f_back.f_back.f_lineno))

class Motor:
    '''
    High level control of the Allmotion Motor
    '''
    def change_pid(self, alias='low'):
        '''
        Change the torque/velocity/position p/i values
        based off a list of preset pid values held in self.pids=[]
        alias: <alias string>
        '''
        # default is to go slow but you should really always define it
        found = 0
        # Hunt for requested pid alias
        for idx in self.pids:
            if idx.get('alias') == alias:
                torque_p     = idx.get('torque_p')
                torque_i     = idx.get('torque_i')
                velocity_p   = idx.get('velocity_p')
                velocity_i   = idx.get('velocity_i')
                position_p   = idx.get('position_p')
                max_velocity = idx.get('max_velocity')
                found = 1
                break
        if found == 1:
            # Only change what /needs/ to be changed in an effort to
            # minimize changes where the pid is changed to the same
            # alias a second or third time or where pids aliases share
            # common values. Also keep an eye on how long it took
            start_time = time.time()

            # Fixup the torque/velocity and position_p PI values
            if self.torque_p != torque_p:
                self.torque_p = torque_p
                self.controller.set_torque_p(torque_p)
            if self.torque_i != torque_i:
                self.torque_i = torque_i
                self.controller.set_torque_i(torque_i)

            if self.velocity_p != velocity_p:
                self.velocity_p = velocity_p
                self.controller.set_velocity_p(velocity_p)
            if self.velocity_i != velocity_i:
                self.velocity_i = velocity_i
                self.controller.set_velocity_i(velocity_i)

            if self.position_p != position_p:
                self.position_p = position_p
                self.controller.set_position_p(position_p)

            # try and limit overspeeding through the velocity ramp limits
            if self.max_velocity != max_velocity:
                self.max_velocity = max_velocity
                self.controller.set_max_abs_ramp_velocity  (max_velocity)

            self.active_pid = alias
            elapsed_time = time.time() - start_time
            msg = 'PID changeover to [%s] took % 0.3f sec' %  (alias, elapsed_time)
            logging.debug(msg)
            return 0
        else:
            return -1

    def get_pid(self):
        msg = '--- pid '
        msg = msg + ''.ljust(80 - len(msg), '-')
        logging.info(msg)
        print(msg)
        msg = 'Torque    : P: %-5d    I: %-5d' % (self.torque_p, self.torque_i)
        logging.info(msg)
        print(msg)
        msg = 'Velocity  : P: %-5d    I: %-5d' % (self.velocity_p, self.velocity_i)
        logging.info(msg)
        print(msg)
        msg = 'Position_P: P: %-5d' % (self.position_p)
        logging.info(msg)
        print(msg)
        msg = 'Speed: %-5d  Current position: %-5d  Target position %-5d' % \
              (self.get_velocity(),
               self.get_actual_encoder_count(),
               self.controller.get_target_position())
        logging.info(msg)
        print(msg)
        msg = 'Accel: %-5d Target speed %-5d' % \
              (self.get_acceleration(),
               self.controller.get_target_speed())

    def motor_init(self):
        # Initalize the motor
        # LEAVE THESE VALUES ALONE! (Sasha will hurt you!)
        # These are the initial settings
        # use the 'low' pid
        self.torque_p = 250
        self.torque_i = 150
        self.velocity_p = 3500
        self.velocity_i = 9000
        self.position_p = 200
        self.max_velocity = 1000  # maximum ramp velocity

        # create some pids, normally you would select by alias
        # eg  <>.change_pid('low')
        # but in the spin function we make an automatic selection based
        # on the rpm range the pid is suitable for
        self.pids                        = [ {
                                             'alias'        : 'low',
                                             'range'        :  (0, 99),
                                             'torque_p'     :  250,
                                             'torque_i'     :  150,
                                             'velocity_p'   :  3500,
                                             'velocity_i'   :  9000,
                                             'position_p'   :  100,
                                             'max_velocity' :  1000,
                                             },
                                             {
                                             'alias'        : 'medium',
                                             'range'        :  (100, 1999),
                                             'torque_p'     :  250,
                                             'torque_i'     :  150,
                                             'velocity_p'   :  3500,
                                             'velocity_i'   :  9000,
                                             'position_p'   :  200,
                                             'max_velocity' :  4000,
                                             },
                                             {
                                             'alias'        : 'high',
                                             'range'        :  (2000, 10000),
                                             'torque_p'     :  250,
                                             'torque_i'     :  150,
                                             'velocity_p'   :  3500,
                                             'velocity_i'   :  9000,
                                             'position_p'   :  200,
                                             'max_velocity' :  9000,
                                             }, ]

        # Initalize the motor with the lower pid as the first thing we
        # do is detect the home position at low speed
        setting = 'low'
        if self.change_pid(alias=setting) == -1:
            msg = 'Could not find a matching pid for [%s]! I MUST abort!' % setting
            logging.critical(msg)
            print(msg)
            sys.exit(-2)

        self.velocity_max                = MAX_VELOCITY      # NEVER exceed (RPM/min)
                                                             # the motor may do 10K but the
                                                             # cassette laminate won't
        self.max_current                 = MAX_CURRENT       # max rating for *motor* in mA (PSU trips if over 3A)
        self.starting_current            = MAX_START_CURRENT # starting current in mA for controlled commutation
        self.number_of_poles             = 4                 # the default was 8 ?!

        self.mvp_target_reached_velo     = 500               # movement positioning enabled at this speed (rpm)
        self.mvp_target_reached_distance = 2                 # Maximum distance from true at which the position
                                                             # end flag is set (in ticks)
        self.encoder_steps               = SI_COUNT          # ticks per revolution

        self.reset_pid                   = 1                 # flag used when something has
                                                             # changed the pid defaults

        # EXPECTED DEFAULTS THAT SHOULD NOT CHANGE (MUCH)
        self.acceleration                = 1000        # RPM/s

        # TO PREVENT BUBBLING MOLTEN PLASTIC EVENTS
        self.i2t_constant                = I2T_CONSTANT      # I will use these when I know they're sane
        self.i2t_limit                   = I2T_LIMIT         # as they have to be measured in the lab

        # FOR SPINNING

        # FOR POSITIONING
        self.velocity_max_pos_ctrl       = HOME_SPEED  # rpm/s position can only happen below this speed
        self.acceleration_pos_ctrl       = HOME_ACCEL  # rpm/s postioning movement can only happen below or @ X rpm/s
        self.velocity_p_pos_ctrl         = 4000        # rpm/min speed (P of PID)
        self.velocity_i_pos_ctrl         = 2000        # rpm/min speed (I of PID)

        # Lets implement the above settings
        # reference page 53
        # https://www.trinamic.com/fileadmin/assets/Products/Modules_Documents/TMCM-1640_TMCL_firmware_manual.pdf
        self.stop()
        # ~ self.motor_release()

        # I use a global here to detect if certain motor setup calls have
        # been initialized as we want to only ever do them once
        # when first powered on.
        # Note: The current design seems to run the python process once
        # and then terminates and restarts the program so we need to use
        # permenant storage as an indicator of state. When the device is
        # started, this file should not exist

        if os.path.isfile(MOTOR_INITIALIZED):
            logging.info('Motor pre-initialized. skipping setup.')
            pass
        else:
            #self.controller.BLDC_reinitialization()
            self.controller.set_velocity_ramp_generator(1)  # don't turn off! It's horrible if you do.
            self.controller.set_sine_initialization_speed(200)
            msg = 'Max motor current limit is     %dmA' % self.controller.get_max_current()
            logging.info(msg)
            print(msg)
            self.controller.set_max_current(self.max_current)  # set it to the safe maximum
            msg = 'Max motor current limit is now %dmA' % self.controller.get_max_current()
            print(msg)
            logging.info(msg)

            # rearranged to be the same order the TMCL IDE would use
            self.controller.set_communication_mode()
            self.controller.set_max_current(self.max_current)                                  # max current
            self.controller.set_start_current(self.starting_current)                           # start current
            self.controller.set_i2t_limit(I2T_LIMIT)                                           # I²t limit
            self.controller.set_number_of_motor_poles(self.number_of_poles)                    # motor poles
            self.controller.set_hall_sensor_invert(0)                                          # hall sensor invert
            self.controller.set_hall_interpolation(1)                                          # hall interpolation
            self.controller.set_encoder_steps(self.encoder_steps)                              # encoder steps
            self.controller.set_encoder_direction(1)                                           # encoder direction
            self.controller.set_encoder_init_mode(1)                                           # use hall encoder init mode
            self.controller.set_torque_p(self.torque_p)                                        # torque P
            self.controller.set_torque_i(self.torque_i)                                        # torque I
            self.controller.set_velocity_p(self.velocity_p)                                    # velocity P
            self.controller.set_velocity_i(self.velocity_i)                                    # velocity I
            self.controller.set_position_p(self.position_p)                                    # position P

            self.controller.set_max_abs_ramp_velocity(self.velocity_max)                       # SAP  4
            self.controller.set_mvp_target_reached_velocity(self.mvp_target_reached_velo)      # SAP  7
            self.controller.set_mvp_target_reached_distance(self.mvp_target_reached_distance)  # SAP 10


            logging.info('Motor initialized.')
            Path(MOTOR_INITIALIZED).touch()

        # These may or may not be set with the right values on powerup so just pin them
        self.controller.set_acceleration(self.acceleration)
        self.home()

    def __init__(self, ser, home_pin):
        self.controller = TrinamicMotor(ser, 0.000) # 0.3)
        self.home_pin = home_pin
        GPIO.setmode(GPIO.BCM)
        #GPIO.setmode(GPIO.BOARD)
        GPIO.setup(home_pin, GPIO.IN)
        self.found_home = 0
        self.motor_init()
        self.test = 0  # for the motor tests

    def set_rs485_baud_rate(self, baud):
        # Baud rates are denoted thus
        # 0 - 9600
        # 1 - 14400
        # 2 - 19200
        # 3 - 28800
        # 4 - 38400
        # 5 - 57600
        # 6 - 76800
        # 7 - 115200
        self.controller.set_rs485_baud_rate(baud)

    def get_rs485_baud_rate(self):
        return self.controller.get_rs485_baud_rate()

    def spin(self, rpm, spin_time=0, rpm_end_tgt=0, stopping=NONSTOP):
        '''
        rpm         : target rpm to REACH
        spin_time   : 0 - spin forever, else timer in seconds
        rpm_end_tgt : what speed should we exit? (default 0) only when CALCULATED
        stopping    : NONSTOP    : constanst speed on exit
                      GENTLE     : Turn off power to motor and freewheel
                      HARSH      : turns motor off (massive back EMF) may reset some motor features
                      CALCULATED : return to defined exit speed [rpm_end_tgt] (default 0)
        home        : None       : don't re-home to find 0 any other value will reset the encoder
                                   to 0 when it finds the magnetic pin
        '''
        # Find an acceptable pid based on the rpm range
        found = 0
        for pid in self.pids:
            range_low, range_high = pid.get('range')
            if range_low <= abs(rpm) <= range_high:
                self.change_pid(pid.get('alias'))
                found = 1
                break

        # The motor will try and keep going so we'll just log this as a warning
        if found != 1:
            msg = 'No PID found with a range to match the rpm [%d] asked for!' % rpm
            logging.warn(msg)
            print(msg)

        # Configure the speed control param to the max
        # ~ self.controller.set_max_abs_ramp_velocity(self.velocity_max)

        if rpm >= 0:
            self.controller.rotate_left(rpm)
            #print('spinning left (-)')
            dir = 1
        else:
            self.controller.rotate_right(-rpm)
            #print('spinning right (+)')
            dir = -1

        if spin_time > 0:
            t_end = time.time() + spin_time
            while time.time() < t_end:
                continue  # to wait
            if stopping is GENTLE:
                self.motor_release() # Gentle spindown by friction
            if stopping is HARSH:
                self.stop()  # HARSH! (avoid, seems to unset other settings)
                # print('position = %d' % self.get_actual_encoder_count())
            if stopping is CALCULATED:
                accel = self.get_acceleration()
                time_to_complete = abs(round(((rpm - rpm_end_tgt) / accel), 2))  # to 2 decimal places is good enough
                # ~ print('Calc time to reach %d from %d rpm is %d sec' % (rpm_end_tgt, rpm, time_to_complete))
                if dir == 1:
                    self.controller.rotate_left(rpm_end_tgt)  # just in case direction causes issues
                else:
                    self.controller.rotate_right(rpm_end_tgt)

                if  time_to_complete > 0:
                    t_end = time.time() + time_to_complete
                    while time.time() < t_end:
                        continue  # to wait
        else:
            pass

    def get_target_speed(self):
        return self.controller.get_target_speed()

    def get_actual_speed(self):
        return self.controller.get_actual_speed()

    def flag_info(self, check_bit):  # check for flag at bit postion N
        flags = self.controller.get_status_info()
        if TRINAMIC_DEBUGGING: # DBG!
            # DBG: show bits. The value is a 32bit number but only the
            # first 18 bits are used, format it in groups of 4 for easier reading
            bit_string = '{0:020b}'.format(flags)
            # 'mask' unused bits with an underscore
            for i in (7, 8, 12, 13, 15, 16, 18, 19):
                bit_string = bit_string[:(19-i)] + '_' + bit_string[(19-i)+1:]
            bit_string = '.'.join(bit_string[i:i+4] for i in range(0, len(bit_string), 4))
            msg = '1640 status register bits (19 -> 0) [%s]' % bit_string
            logging.debug(msg)
            print(msg)
        return (flags & (1 << check_bit)) != 0

    def pid_reset(self):
        DEBUG=0  # VERY SLOW don't use unless you really have to
        if DEBUG:
            print('=== PID DEBUG (warning: slow) ========')
            print('                          Curr  Chg to')
            print('max_abs_ramp_velocity   = %4d   %4d' % (self.controller.get_max_abs_ramp_velocity(), self.velocity_max_pos_ctrl))
            print('mvp_target_reached_velo = %4d   %4d' % (self.controller.get_mvp_target_reached_velocity(), self.mvp_target_reached_velo))
            print('acceleration (rpm/sec)  = %4d   %4d' % (self.controller.get_acceleration(), self.acceleration_pos_ctrl))
            print('velocity     (rpm)      = %4d   %4d' % (self.controller.get_velocity(), self.controller.get_velocity()))
            print('velocity_p_pos          = %4d   %4d' % (self.controller.get_velocity_p(), self.velocity_p_pos_ctrl))
            print('velocity_i_pos          = %4d   %4d' % (self.controller.get_velocity_i(), self.velocity_i_pos_ctrl))
            print('--------------------------------------')
        self.reset_pid = 0  # reset flag
        return  # resetting the pid seems to make oscillations more common?!
        self.controller.set_max_abs_ramp_velocity(self.velocity_max_pos_ctrl)         # HOME_SPEED = 40
        self.controller.set_mvp_target_reached_velocity(self.mvp_target_reached_velo) # 5
        self.controller.set_acceleration(self.acceleration_pos_ctrl)                  # HOME_ACCEL = 40
        self.controller.set_velocity_p(self.velocity_p_pos_ctrl)                      # 4000
        self.controller.set_velocity_i(self.velocity_i_pos_ctrl)                      # 2000
        self.reset_pid = 0  # reset flag

    def position_abs(self, ticks):
        # Some commands meddle with the default values and we have to
        # reset them here, eg MST (motor.stop())
        if self.reset_pid:
            self.pid_reset()

        self.controller.position_abs(ticks)

    def position_rel(self, ticks):
        # Configure position control param
        # Some commands meddle with the default values and we have to
        # reset them here, eg MST (motor stop)
        if self.reset_pid:
            self.pid_reset()

        self.controller.position_rel(ticks)

    def brake(self):
        # Sergay: change to release for now 9/29/2020
        # ~ self.set_target_current_to_zero()   # This release the motor
        pass

    def motor_release(self):
        self.set_target_current_to_zero()

    def stop(self):
        # affects the PID by setting target velocity to zero (MST)
        self.controller.stop()
        self.reset_pid = 1

    def home(self):
        '''
        find the magnetic homing pin but because it's not a 'point'
        and has a small dead zone we try and measure the homing point
        from one side. Note that the controller finds the homing pin
        but just spins past it.
        '''
        # Take a note of the acceleration and velocity values (just in case)
        acceleration = self.get_acceleration()

        # We need to start *QUICKLY* but not too roughly
        # It's the VELOCITY that matters most for positioning
        # and we will be using 65 rpm as explained below
        if acceleration < 500:
            self.set_acceleration(500)

        self.set_velocity(0)
        while not self.flag_info(check_bit=HALTED):        # wait until we are halted before homeing
            pass

        halfway = int(SI_COUNT/2)                          # What is a half revolution?
        self.controller.set_actual_encoder_count(halfway)  # wherever were were is now a 'half turn' away
                                                           # eg 2187678 -> 2000 without actually rotating

        self.change_pid('low')                             # prepare for low speed positioning movement
        # Configure the position control param
        if self.reset_pid:
            self.pid_reset()

        # /just in case/, try once more if we didn't see it the first time.
        for loop in range(1,2):
            # index reset the counter at next N channel event. and then ignore from then on
            self.controller.set_encoder_set_null(1)            # Reset to 0 when the sensor is triggered
            self.controller.set_encoder_clear_set_null(1)      # reset/clear the zero event trigger when it next happens

            self.set_acceleration(HOME_ACCEL)
            self.spin(65,                                      # we now hunt for zero by spinning one revolution
                      spin_time=1,                             # 60 = 1 rev/sec just go a little further (65)
                      stopping=CALCULATED)

            if self.controller.get_encoder_set_null() == 0:
                break

            if loop == 2 and self.controller.get_encoder_set_null() != 0:  # magnetic pin not seen after a full revolution
                msg = 'No cartridge detected after 2 spins, or is it upside down and leaking?'
                logging.critical(msg)
                print(msg)
                sys.exit(-3)

        # restore acceleration
        self.set_velocity(0)
        self.set_acceleration(acceleration)

    def rotate_right(self, velo):
        # ~ print(lineno())
        # Some commands meddle with the default values and we have to
        # reset them here, eg MST (motor stop)
        if self.reset_pid:
            print('Rotate right - Reseting PID')
            self.controller.set_velocity_p(self.velocity_p)
            self.controller.set_velocity_i(self.velocity_i)
            self.reset_pid = 0  # reset flag
        self.controller.rotate_right(velo)

    def rotate_left(self, velo):
        # ~ print(lineno())
        # Some commands meddle with the default values and we have to
        # reset them here, eg MST (motor stop)
        if self.reset_pid:
            print('Rotate left - Reseting PID')
            self.controller.set_velocity_p(self.velocity_p)
            self.controller.set_velocity_i(self.velocity_i)
            self.reset_pid = 0  # reset flag

        self.controller.rotate_left(velo)

    def set_acceleration(self, accel):
        self.controller.set_acceleration(accel)

    def get_acceleration(self):
        return self.controller.get_acceleration()

    def set_target_current_to_zero(self):
        self.controller.set_target_current_to_zero()

    def set_encoder_set_null(self, value):
        self.controller.set_encoder_set_null(value)

    def set_encoder_clear_set_null(self, value):
        self.controller.set_encoder_clear_set_null(value)

    def set_position_p(self, value):
        self.controller.set_position_p(value)

    def set_max_abs_ramp_velocity(self, velo_max):
        self.controller.set_max_abs_ramp_velocity(velo_max)

    def set_mvp_target_reached_velocity(self, velo):
        self.controller.set_mvp_target_reached_velocity(velo)

    def get_mvp_target_reached_velocity(self):
        velocity = self.controller.get_mvp_target_reached_velocity()
        return velocity

    def set_velocity_ramp_generator(self, value):
        self.controller.set_velocity_ramp_generator(value)

    def get_velocity_ramp_generator(self):
        value = self.controller.get_velocity_ramp_generator()
        return value

    def set_velocity(self, value):
        self.controller.set_velocity(value)

    def get_velocity(self):
        value = self.controller.get_velocity()
        return value

    def wait_for_event(self, condition, timeout):   # 1=position, 2=refsw, 3=limsw; timout=0: no time out
        self.controller.wait_for_event(condition, timeout)

    def get_actual_encoder_angle(self):  # 0: actual, 1: controlled
        angle = self.controller.get_actual_encoder_angle()
        return angle

    def get_actual_encoder_count(self):  # 0: actual, 1: controlled
        count = self.controller.get_actual_encoder_count()
        return count

    def set_actual_encoder_count(self, ticks):
        self.controller.set_actual_encoder_count(ticks)

    def get_encoder_set_null(self):
        result = self.controller.get_encoder_set_null()
        return result

    def get_encoder_clear_set_null(self):
        result = self.controller.get_encoder_clear_set_null()
        return result

    def wobble(self, shake=300, duration=5):
        '''
        Purposefully create oscillation to shake contents
        wobble  - alters the velocity ramp to create the oscillation
                  values of 150-400 seem good
                  try not to exceed 1000
        duration - time in seconds  eg 4 or 4.23
        '''
        position = self.get_actual_encoder_count()        # huh? where am I?
        if position is None:
            position = 0                                  # in case we didn't get a position
        # self.set_acceleration(HOME_ACCEL)                 # (40)
        # self.set_velocity(0)                              # stop spinning I'm dizzy
        self.set_max_abs_ramp_velocity(shake)             # prevent it spinning WAY out of control
        self.set_acceleration(10000)                      # some absurd outside value that will never let the ramp settle
        self.position_rel(2000)                           # 2000 = 1/2 turn
        time.sleep(duration)                              # wait a bit while it destroys the motor bearings
        self.set_acceleration(500)                        # end this quickly
        self.position_abs(position)                       # the oscillation will lose the position if you needed it
        self.set_max_abs_ramp_velocity(self.velocity_max) # back to 9000    self.velocity_max
        time.sleep(2)                                     # give the higher acceleration a chance to work
        self.set_acceleration(HOME_ACCEL)

    def read_current_regulator(self):
        '''
        This is just to read all the current regulator variables
        It's basically page 50 of the TMCM 1640 TMCL firmware manual
        '''
        DEBUG=1  # VERY SLOW don't use unless you really have to
        if DEBUG:
            I_ACTUAL  = self.controller.get_actual_current()
            I_TARGET  = self.controller.get_target_current()
            I_Max     = self.controller.get_max_current()
            e_SUM     = self.controller.get_current_pid_error_sum()
            P_PARAM   = self.controller.get_torque_p()
            I_PARAM   = self.controller.get_torque_i()

            print('=== Position regulation DEBUG (warning: slow) ==================')
            print('                                                         Curr')
            print('I.ACTUAL    Actual motor current ....................... %4d mA' % I_ACTUAL)
            print('I.TARGET    Target motor current ....................... %4d mA' % I_TARGET)
            print('I.Max       Max. motor current ......................... %4d mA' % I_Max)
            print('e.SUM       Error sum for integral calculation ......... %4d' % e_SUM)
            print('P.PARAM     Current P parameter ........................ %4d' % P_PARAM)
            print('I.PARAM     Current I parameter ........................ %4d' % I_PARAM)
            print('----------------------------------------------------------------')

    def read_velocity_regulator(self):
        '''
        This is just to read all the velocity regulator variables
        It's basically page 51 of the TMCM 1640 TMCL firmware manual
        '''
        DEBUG=1  # VERY SLOW don't use unless you really have to
        if DEBUG:
            v_ACTUAL  = self.controller.get_actual_speed()
            v_RAMPGEN = self.controller.get_ramp_generator_speed()
            v_Max     = self.controller.get_max_abs_ramp_velocity()
            e_SUM     = self.controller.get_velocity_pid_error_sum()
            P_PARAM   = self.controller.get_velocity_p()
            I_PARAM   = self.controller.get_velocity_i()
            I_Max     = self.controller.get_max_current()
            I_Target  = self.controller.get_target_current()

            print('=== VELOCITY REGULATOR DEBUG (warning: slow) ===================')
            print('                                                         Curr')
            print('v.ACTUAL    Actual motor velocity ...................... %4d rpm' % v_ACTUAL)
            print('v.RAMPGEN   Target velocity of ramp generator .......... %4d rpm' % v_RAMPGEN)
            print('v.Max       Max. target velocity ....................... %4d rpm' % v_Max)
            print('e.SUM       Error sum for integral calculation ......... %4d errors' % e_SUM)
            print('P.PARAM     Velocity P parameter ....................... %4d ' % P_PARAM)
            print('I.PARAM     Velocity I parameter ....................... %4d ' % I_PARAM)
            print('I.Max       Max. target current ........................ %4d mA' % I_Max)
            print('I.Target    Target current for current PID regulator ... %4d mA' % I_Target)
            print('----------------------------------------------------------------')

    def read_positioning_regulator(self):
        '''
        This is just to read all the position regulator variables
        It's basically page 51 of the TMCM 1640 TMCL firmware manual
        '''
        DEBUG=1  # VERY SLOW don't use unless you really have to
        if DEBUG:
            n_ACTUAL  = self.controller.get_actual_encoder_count()
            n_TARGET  = self.controller.get_target_position()
            P_PARAM   = self.controller.get_position_p()
            V_MAX     = self.controller.get_max_abs_ramp_velocity()
            V_TARGET  = self.controller.get_ramp_generator_speed()

            print('=== VELOCITY REGULATOR DEBUG (warning: slow) ===================')
            print('                                                         Curr')
            print('n.ACTUAL    Actual motor position ...................... %4d ticks' % n_ACTUAL)
            print('n.TARGET    Target motor position ...................... %4d ticks' % n_TARGET)
            print('P.PARAM     Position P parameter ....................... %4d ' % P_PARAM)
            print('V.MAX       Max. allowed velocity ...................... %4d rpm' % V_MAX)
            print('V.TARGET    New target velocity for ramp generator ..... %4d rpm' % V_TARGET)
            print('----------------------------------------------------------------')


# ----------------------------------------------------------------------

if __name__ == '__main__':
    import time
    import serial
    import curses
    # bit of borrowed code
    import threading
    import time
    import RPi.GPIO as GPIO
    import random
    import math
    from settings import MAGNETIC_INDEX_PIN, MOVE_COMPLETED
    from settings import WHITE_LED, WHITE_LED_ON, WHITE_LED_OFF
    from settings import SI_COUNT, BUMP, HOME_ACCEL, HOME_SPEED
    from settings import CALIBRATION_FILE

    DEBUGGING = 1
    if DEBUGGING:
        import sys
        import signal
        import pdb
        import os

        def debugging(signum, frame):
            # enter pdb in the /previous/ frame rather than *here*
            signal.alarm(0)
            pdb.Pdb().set_trace(sys._getframe().f_back)
            # to simulate a crtl-c in code, use:
            #    os.kill(os.getpid(), signal.SIGINT)
            # or be boring with pdb.set_trace()

        signal.signal(signal.SIGTERM, debugging)
        signal.signal(signal.SIGINT, debugging)
        ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
        )

    motor = Motor(ser, MAGNETIC_INDEX_PIN)

    motor.motor_init()  # Use our default setup settings

    class RepeatedTimer(object):
        def __init__(self, interval, function, *args, **kwargs):
            self._timer = None
            self.interval = interval
            self.function = function
            self.args = args
            self.kwargs = kwargs
            self.is_running = False
            self.next_call = time.time()
            self.start()

        def _run(self):
            self.is_running = False
            self.start()
            self.function(*self.args, **self.kwargs)

        def start(self):
            if not self.is_running:
                self.next_call += self.interval
                self._timer = threading.Timer(self.next_call - time.time(), self._run)
                self._timer.start()
                self.is_running = True

        def stop(self):
            self._timer.cancel()
            self.is_running = False

    # get the curses screen window
    screen = curses.initscr()

    # turn off input echoing
    curses.noecho()

    # respond to keys immediately (don't wait for enter)
    curses.cbreak()

    # map arrow keys to special values
    screen.keypad(True)

    # print doesn't work with curses, use addstr instead
    # Turn off the cursor!
    curses.curs_set(0)

    print('Welcome to the motor tests')
    #print('Telegram is: %d' % motor.controller.get_telegram_pause_time())

    velocity = motor.get_target_speed()
    acceleration = motor.get_acceleration()

    top                 = 3                   # screen shift panel output (not position or ticker)
    step                = 100                 # incremental step size
    spinning            = 0                   # Flag. Are we spinning or not? 0 = no 1= yes
    velocity_tgt        = 3000                # starting speed since starting from 0 and going up is annoying
    spin_increment      = 1000                # rpm increase
    acceleration_tgt    = motor.acceleration  # starting acceleration since starting from 0 and going up is annoying
    accel_increment     = 1000                # acceleration (rpm/s) increase
    action              = ''                  # What did I do?! I'm innocent (maybe)!
    well                = [2566, 566]         # The IGG/IGM wells (approx position)
    well_selected       = 0                   # 0 or 1, (abuse bitwise toggling since there are only 2)

    def repaint_screen():
        screen.addstr(0, 29, 'Ticks          Deg     rpm')
        screen.addstr(1, 15, 'Position', curses.A_REVERSE)

        screen.addstr(top +  0,  0, 'Velocity     :  % 6.1d  Tgt:% 6.1d' % (motor.get_target_speed(), velocity_tgt))
        screen.addstr(top +  1,  0, 'Acceleration :  % 6.1d' % motor.get_acceleration())


        screen.addstr(top +  3,  0, 'CURRENT ACTION:', curses.A_REVERSE)
        screen.addstr(top +  4,  0, 'Last action:')

        screen.addstr(top +  8,  0, 't/T - random hold test (14 random points) T = wells/valves with spins')
        screen.addstr(top +  9,  0, 'h - detect home position (DOES NOT STOP AT 0!)')
        screen.addstr(top + 10,  0, '0 - move to zero ( may not be 100% due to MVP position tolerance')
        screen.addstr(top + 11,  0, 'r - cut power to the motor - free spin (dragging to a magnetic pole likely)')
        screen.addstr(top + 12,  0, 'b - use driver to hold current position (b)rake on')
        screen.addstr(top + 13,  0, 'c - move to calibration point (3600 ticks)')
        screen.addstr(top + 14,  0, 'w - toggle move to approx IGG/IGM well positions')
        screen.addstr(top + 15,  0, 'square brackets to decrease/increase acceleration')
        screen.addstr(top + 16,  0, 'left/right shifts +/- %d ticks    up/down change spin speed +/-%d' % (step, spin_increment))
        screen.addstr(top + 17,  0, 'q - quit program')

        screen.refresh()

    def current_pid(currents=(3000), velocity_pid=(200,10), true_max=motor.max_current): # some defaults that aren't insane
        # *Make sure disk is taped down*
        # PARAMETERIZING THE CURRENT REGULATOR SET
        # 1. Set the P parameter and the I parameter to zero.
        # 2. Start the motor by using a low target current (e.g. 1000 mA).
        # 3. Modify the current P parameter. Start from a low value and
        #    go to a higher value, until the actual current nearly reaches
        #    50% of the desired target current.
        # 4. Do the same with the current I parameter.
        #
        # crap instructions,..
        # basically step 4 neglects to say WHILE KEEPING torque_p at the 1/2 power value
        # increase torque_i until the full current is achieved.
        # What they ALSO neglect to mention is that velocity_p/i and position_p have to have
        # values as well but of course we have no idea what those should be

        screen.addstr(13 + top,  8, 'Make sure the disk is taped down/immobilized')
        screen.addstr(14 + top,  8, 'Press <enter/rtn> key when ready')
        curses.flushinp()  # Flush all input buffers. This throws away
                           # any typeahead that has been typed by the
                           # user and has not yet been processed by the program.
        char = screen.getch()
        curses.endwin()    # abandon usng curses because I'm lazy

        result=[]
        velocity_p, velocity_i = velocity_pid

        motor.controller.set_velocity_ramp_generator(0)  # turn off ramp generator
        motor.controller.set_velocity_p(velocity_p)      # stop this velocity setting interfering (use homing values)
        motor.controller.set_velocity_i(velocity_p)      # stop this velocity setting interfering (use homing values)
        motor.controller.set_position_p(0)               # likewise for the postitioning
        torque_p = 0
        torque_i = 0
        motor.controller.set_torque_p(torque_p)          # go limp
        motor.controller.set_torque_i(torque_i)          # go limp

        val_found = 0

        for tgt_current in currents:                     # list of currents to use (typically just 3000)
            print('Starting test for when using % 04d mA power (first, find 50%%) ' % tgt_current)
            print('-------------------------------------------------------------------')
            torque_p = 0
            torque_i = 0
            motor.controller.set_torque_p(torque_p)      # go limp
            motor.controller.set_torque_i(torque_i)      # go limp
            motor.controller.set_target_current(0)       # try and disipate any built up coil energy

            # set the max allowed current but don't exceed the PSU's true max
            motor.controller.set_max_current(tgt_current if tgt_current < true_max else true_max)
            print('Max current allowed is currently set as: %d mA' % motor.controller.get_max_current())

            # set where we are as 0 then move 1/2 way around
            motor.controller.set_actual_encoder_count(0)
            motor.position_abs(round(SI_COUNT/2))

            avg = 0
            #import pdb; pdb.set_trace()
            # The relationship with current and toeque when graphed
            # is supposed to be linear ie y=mx+b but when measuring using
            # the trinamic board the current values are haphazard
            #
            # see: https://www.anaheimautomation.com/ for the BLWR111S-24V-10000
            # spec sheet (https://tinyurl.com/ynvbrkmk)
            for torque_p in range(1000, 4000, 50):
                motor.controller.set_torque_p(torque_p)
                avg = abs(motor.controller.get_actual_current())
                tgt_lvl = int(tgt_current /2)  # 50%
                if avg > tgt_lvl:  # are we close? if so start taking avgs
                    avg = 0
                    avg_list = []
                    for loop in range(1, 8+1):           # +1 or it breaks out at 7
                        value = abs(motor.controller.get_actual_current())
                        avg += value
                        avg_list.append(value)
                    avg = round(avg/loop)
                    spread = abs(max(avg_list) - min(avg_list))
                    if avg > tgt_lvl:
                        print('Found candidate value, spread of values over %d readings was %d mA' % (loop, spread))
                        val_found = 1
                else:
                    msg = 'Actual current is too low (%dmA when using %d:%d) tgt: %d skipping avg loop' % \
                          (avg,
                           torque_p,
                           torque_i,
                           tgt_lvl)
                    print(msg)
                    continue
                print('Avg current is %dmA when torque_p = %d' % (avg, torque_p))
                if val_found == 1: # break out of while if we found the value
                    val_found = 0  # reset flag
                    break

            print('Attempting to find usable torque_i value to make up the full value')

            # Step 4. do it all again but for torque_i
            for torque_i in range(0, 50, 1):             # upper bound of 50 is somewhat excessive...
                motor.controller.set_torque_i(torque_i)
                tgt_lvl = int(tgt_current * 0.99)        # Within 99% of full value (to avoid PSU trip)
                avg = abs(motor.controller.get_actual_current())
                msg = 'Torque p/i %d/%d  avg %dmA (tgt:%d)' % \
                      (torque_p,
                       torque_i,
                       avg,
                       tgt_lvl)
                print(msg)
                if avg > tgt_lvl:    # are we close? if so start taking avgs
                    avg = 0
                    avg_list = []
                    for loop in range(1, 8+1):           # +1 or it breaks out at 7
                        value = abs(motor.controller.get_actual_current())
                        avg += value
                        avg_list.append(value)
                    avg = round(avg/loop)
                    spread = abs(max(avg_list) - min(avg_list))
                    if avg > tgt_lvl:
                        print('Found candidate value, spread of values over %d readings was %d mA' % (loop, spread))
                        val_found = 1
                else:
                    msg = 'Actual current is too low (%dmA when using %d:%d) skipping avg loop' % \
                          (avg,
                           torque_p,
                           torque_i)
                    print(msg)
                    continue
                if val_found == 1:  # break out of while if we found the value
                    val_found = 0   # reset flag
                    break

            print('Avg current is %dmA when torque_i = %d' % (avg, torque_i))
            result.append(tuple([tgt_current, torque_p, torque_i]))

        motor.controller.set_torque_p(0)  # go limp
        motor.controller.set_torque_i(0)  # go limp

        print('')
        print('='*80)
        for current, torque_p, torque_i in result:
            print('Current: % 5d mA  torque_p  : % 5d  torque_i  : % 3d' % \
                  (current,
                   torque_p,
                   torque_i))
        print('-'*80)
        print('Where              velocity_p: % 5d  velocity_i: % 3d' % \
              (velocity_p,
               velocity_i))
        print('                   position_p: % 5d' % \
              (motor.controller.get_position_p()))
        print('-'*80)
        print('')
        input('Press any key to continue to exit')
        return result

    def velocity_pid(velocities=(1000, 2000), torque=(1500,11)):
        #PARAMETERIZING THE VELOCITY REGULATOR SET
        # 1. Set the velocity I parameter to zero.
        # 2. Start the motor by using a medium target velocity (e.g. 2000 rpm).
        # 3. Modify the velocity P parameter. Start from a low value and
        #    go to a higher value, until the actual motor speed reaches
        #    80 or 90% of the target velocity.
        # 4. The lasting 10 or 20% speed difference can be reduced by slowly
        #    increasing the velocity I parameter.

        curses.endwin()    # abandon using curses because I'm lazy
        result=[]
        motor.controller.set_velocity_ramp_generator(1)      # turn off ramp generator it just complicates things
        motor.controller.set_velocity_p(0)                   # stop this velocity setting interfering
        motor.controller.set_velocity_i(0)                   # stop this velocity setting interfering
        motor.controller.set_position_p(0)                   # likewise for the positioning
        motor.controller.set_torque_p(torque[0])             # Use our wanted current that
        motor.controller.set_torque_i(torque[1])             # we worked out before

        for tgt in velocities:                               # velocities we are interested in
            motor.controller.set_max_abs_ramp_velocity(9000) # dont let the motor spin out of control
            motor.controller.get_actual_speed()
            motor.rotate_right(tgt)
            velocity_i= 0  # needs to preexist
            val_found = 0

            avg = 0
            # Annoyingly the relationship with velocity_p and rpm  when graphed
            # is an inverse proportional. As a rough guide. 2000 rpm = 150 / 8
            for velocity_p in range(0, 4001, 50):  # 40 = 2000-ish everyone else uses around 30+
                motor.controller.set_velocity_p(velocity_p)
                avg = abs(motor.controller.get_actual_speed())
                tgt_lvl = int(tgt * 0.95)
                if avg > tgt_lvl:                            # are we close? is so start taking avgs
                    print('Checking velocity_p canidate value % 04d' % velocity_p)
                    avg = 0
                    for loop in range(1, 8+1):               # +1 or it breaks out at 7
                        avg += abs(motor.controller.get_actual_speed())
                    avg = round(avg/loop)
                else:
                    print('rpm too low (% 04d rpm [p/i:% 04d/% 04d] tgt: % 04d) skipping avg loop' % \
                          (avg,
                           velocity_p,
                           velocity_i,
                           tgt_lvl))
                    continue

                print('A velocity_p of %d = %drpm (avg)' % (velocity_p, avg))
                if avg >= tgt_lvl:
                    break

            motor.rotate_right(tgt)
            for velocity_i in range(0, 100, 1):
                motor.controller.set_velocity_i(velocity_i)
                avg = abs(motor.controller.get_actual_speed())
                tgt_lvl = round(tgt * 0.99)  # 99% is good enough... probably
                if avg >= tgt_lvl:  # is our current speed close enough?
                                    # if so, check it a few times
                    print('Checking velocity_i canidate value % 04d' % velocity_i)
                    avg = 0
                    for loop in range(1, 8+1):               # +1 or it breaks out at 7
                        avg += abs(motor.controller.get_actual_speed())
                    avg = round(avg/loop)
                else:
                    print('rpm too low (% 04d rpm [p/i:% 04d/% 04d] tgt: % 04d) skipping avg loop' % \
                          (avg,
                           velocity_p,
                           velocity_i,
                           tgt_lvl))
                    continue

                if avg >= tgt_lvl:  # check recomputed avg speed
                    val_found = 1
                    break

            msg = 'For % 04d rpm, velocity_p: % 04d velocity_i % 04d should work' % \
                  (tgt,
                   velocity_p,
                   velocity_i)
            print('='*80)
            print(msg)
            print('-'*80)

            if val_found == 1:
                result.append(tuple([tgt, velocity_p, velocity_i]))
                motor.controller.set_velocity_i(0)  # reset
                continue  # we want to exhaust all the velocities
                          # in the loop rather than break out

        motor.rotate_right(0)
        print('velocity_p should be: %d' % velocity_p)
        print('velocity_i should be: %d' % velocity_i)

        for rpm, velocity_p, velocity_i in result:
            print('RPM: % 5d | velocity_p: % 04d  velocity_i: % 05d' % (rpm, velocity_p, velocity_i))
        print('Where the working current is % 04d and torque_p/i are : % 04d/ % 04d' % \
              (motor.controller.get_max_current(),
               motor.controller.get_torque_p(),
               motor.controller.get_torque_i()))
        print('')
        input('Press any key to continue to exit')

        motor.controller.set_velocity_ramp_generator(1)      # turn ramp generator back on
        return result

    def positioning_pid(velocity_pid=(2500, 20), torque_pid=(1800,1800)):
        # Based on the velocity regulator only the position regulator P has to
        # be parameterized.
        # 1. Disable the velocity ramp generator and set position P
        #    parameter to zero.
        # 2. Choose a target position and increase the position P
        #    parameter until the motor reaches the target position approximately.
        # 3. Switch on the velocity ramp generator. Based on the max.
        #    positioning velocity (axis parameter 4) and the acceleration value
        #    (axis parameter 11) the ramp generator automatically calculates
        #    the slow down point, i.e. the point at which the velocity has
        #    to be reduced in order to stop at the desired target position.
        # 4. Reaching the target position is signaled by setting the position
        #    end flag.

        # Our positioning is always done at 40 rpm as such our pid values are close to
        # torque   p/i = 1800 / 1800
        # velocity p/i = 2500 / 20

        curses.endwin()    # abandon using curses because I'm lazy
        position_p = 0

        # ~ import pdb; pdb.set_trace()
        motor.controller.set_torque_p(torque_pid[0])
        motor.controller.set_torque_i(torque_pid[1])
        motor.controller.set_velocity_p(velocity_pid[0])
        motor.controller.set_velocity_i(velocity_pid[1])

        motor.controller.set_velocity_ramp_generator(0)

        tgt_positions = [2000, 1000, 500, 250]
        tolerance = round(1.5 * (SI_COUNT/360)) # 1.5 deg please

        motor.controller.set_actual_encoder_count(0)  # I swear we're at '0' ... (liez!)
        for tgt in tgt_positions:
            upper = tgt + round(1 + tolerance)    # +/- values either side of 'true'
            lower = tgt + round(1 - tolerance)
            motor.controller.position_abs(tgt)
            while True:
                position = motor.controller.get_actual_encoder_count()
                msg = 'Wanted % 04d got % 04d (diff % -04d)' % \
                      (tgt,
                       position,
                       (tgt - position))
                print(msg)
                if lower < position < upper:
                    # success!
                    msg = 'Reached acceptable threshold % 04d between % 04d and % 04d' % \
                          (position,
                           lower,
                           upper)
                    print(msg)
                    break
                else:
                    # fail,.. increase and move on
                    position_p += 1
                    msg = 'Not within tolerance. value of position_p increased to %d' % \
                          position_p
                    print(msg)
                    motor.controller.set_position_p(position_p)
                    motor.controller.set_actual_encoder_count(0)  # find the postion from zero on loop around
                    if  position_p > 150:
                        msg = 'Exceeded expected range. Giving up'
                        print(msg)
                        break

            # Stage 3: Now we switch on the ramp generator
            motor.controller.set_velocity_ramp_generator(1)

        print('')
        input('Press any key to continue to exit')

    def find_pid():
        # ~ velocities=[40, 175, 3000, 4000, 7000, 7500]
        # ~ velocities=[7500, 7000, 4000, 3000, 175, 40]
        # ~ velocities=[8000, 7500, 7000, 6500, 6000,
                   # ~ 5500, 5000, 4500, 4000, 3500,
                   # ~ 3000, 2500, 2000, 1500, 1000,
                   # ~ 500]
        # ~ velocities=[500,450,400,350,300,250,200,150,100,50]
        velocities=[3000, 2500, 2000, 1500, 1000, 500]

        screen.clear()  # trash the previous screen layout
        screen.addstr( 0 + top,  8, 'PID tests for current, velocities  and position_p:')
        screen.addstr( 1 + top,  8, 'Velocity list %srpm' % velocities)
        screen.addstr( 2 + top,  8, '(you should do these tests in order and amend the')
        screen.addstr( 3 + top,  8, ' Motor.py program accordingly)')

        screen.addstr( 6 + top,  8, '1. tape up/immobilise the disk and start current(I) pid test')

        screen.addstr( 8 + top,  8, '2. remove tape/free spin the disk and start velocity(rpm) PID test')

        screen.addstr(10 + top,  8, '3. attempt to get the positioning test going')

        try:
            while True:
                curses.flushinp()  # Flush all input buffers. This throws away
                                   # any typeahead that has been typed by the
                                   # user and has not yet been processed by the program.
                char = screen.getch()

                if char == ord('q'):  # 'q' to quit
                    break

                if char == ord('1'):  # run the current PID test
                    current_pid(currents=(range(1000, 3001, 250)),
                                velocity_pid=(3500,350),
                                true_max=motor.max_current)

                if char == ord('2'):  # run the velocity PID test
                    # if we have torque value from somewhere, plug in else error out?
                    velocity_pid(velocities,
                                 torque=(1500,13))

                if char == ord('3'):  # run the positioning PID test
                    positioning_pid(velocity_pid=(1500, 13),
                                    torque=(1500,13))

                screen.refresh()

        except Exception as e:
            print('Bad things: ' + str(e))

    def timed_read_pos():
        screen.addstr(23, 0, 'Time: %s' % time.strftime('%H:%M:%S'))
        if motor.test == 0:  # 0 = ok -1 = disable position request commands
            try:
                position = 0 + motor.get_actual_encoder_count()
            except Exception as e:
                position = 0
                pass
            try:
                velocity = 0 + motor.get_actual_speed()
            except Exception as e:
                velocity = 0
                pass
            screen.addstr(1, 24, '[% 9.1d] (% 10.1f) % 6.1d' % (position, (position * (360/SI_COUNT)), velocity))
            if velocity != 0:
                screen.addstr(1, 65, 'Spinning', curses.A_REVERSE)
            else:
                screen.addstr(1, 65, '        ')
        else:
            screen.addstr(1, 24, 'Data from motor suspended for test duration')

        screen.addstr(23, 16, 'P:I Torque [% 05d:%03d]  Velocity [% 05d:%03d]  position_p: %03d' % \
                             (motor.torque_p,
                              motor.torque_i,
                              motor.velocity_p,
                              motor.velocity_i,
                              motor.position_p))

        # flash the tick
        attrs = screen.inch(22, 1)
        ch = chr(attrs & 0xFF)
        # isbold = bool(attrs & curses.A_BOLD)
        # screen.addstr(10, 0, str(isbold))
        if ch == 'O':
            screen.addstr(22, 0, 'TICK')
        else:
            screen.addstr(22, 0, 'TOCK', curses.A_REVERSE)
        screen.refresh()  # repaint the screen

    read_pos = RepeatedTimer(0.1, timed_read_pos)  # threaded timer every 1/2 second

    repaint_screen()
    change = 2

    try:
        while True:
            curses.flushinp()  # Flush all input buffers. This throws away
                               # any typeahead that has been typed by the
                               # user and has not yet been processed by the program.
            char = screen.getch()

            if char == ord('q'):  # 'q' to quit
                break

            elif char == curses.KEY_RIGHT:
                action = 'Move right %d ticks' % step
                screen.addstr(top + 9, 20, action)
                screen.refresh()
                motor.position_rel(step)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                #screen.addstr(6, 0, 'inc left/right step size    = %d' % step)
                change = 1

            elif char == curses.KEY_LEFT:
                action = 'Move right %d ticks' % -step
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.position_rel(-step)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                #screen.addstr(6, 0, 'dec left/right step size    = %d' % step)
                change = 1

            elif char == curses.KEY_UP:
                # switch between velocity, acceleration, from, to, toggle
                # increase velocity
                if abs(velocity_tgt) < 3000:
                    spin_increment = 1000
                if abs(velocity_tgt) < 1000:
                    spin_increment = 500
                if abs(velocity_tgt) < 500:
                    spin_increment = 100
                if abs(velocity_tgt) < 100:
                    spin_increment = 10
                if abs(velocity_tgt) < 50:
                    spin_increment = 5
                velocity_tgt += spin_increment
                action = 'Increase rpm speed by %d to %d' % (spin_increment, velocity_tgt)
                screen.addstr(top + 3, 20, action)
                screen.addstr(top + 0, 28, '% 6.1d' % velocity_tgt)
                motor.spin((velocity_tgt * spinning), spin_time=0)
                screen.refresh()
                change = 1

            elif char == curses.KEY_DOWN:
                # switch between velocity, acceleration, from, to, toggle
                # decrease velocity
                if abs(velocity_tgt) <= 3000:
                    spin_increment = 1000
                if abs(velocity_tgt) <= 1000:
                    spin_increment = 500
                if abs(velocity_tgt) <= 500:
                    spin_increment = 100
                if abs(velocity_tgt) <= 100:
                    spin_increment = 10
                if abs(velocity_tgt) <= 50:
                    spin_increment = 5
                if abs(velocity_tgt) <= 5:
                    spin_increment = -5
                velocity_tgt -= spin_increment
                action = 'Decrease rpm speed by %d to %d' % (spin_increment, velocity_tgt)
                screen.addstr(top + 3, 20, action)
                screen.addstr(top + 0, 28, '% 6.1d' % velocity_tgt)
                motor.spin((velocity_tgt * spinning), spin_time=0)
                screen.refresh()
                change = 1

            elif char == ord('['):
                # decrease acceleration
                if abs(acceleration_tgt) <= 1000:
                    accel_increment = 500
                if abs(acceleration_tgt) <= 500:
                    accel_increment = 100
                if abs(acceleration_tgt) <= 100:
                    accel_increment = 10
                if abs(acceleration_tgt) <= 50:
                    accel_increment = 5
                if abs(acceleration_tgt) <= 5:
                    accel_increment = -5
                acceleration_tgt -= accel_increment
                action = 'Decrease acceleration speed by %d to %d' % (accel_increment, acceleration_tgt)
                screen.addstr(top + 3, 20, action)
                motor.set_acceleration(acceleration_tgt)
                screen.refresh()
                change = 1

            elif char == ord(']'):
                # increase acceleration
                if abs(acceleration_tgt) < 1000:
                    accel_increment = 500
                if abs(acceleration_tgt) < 500:
                    accel_increment = 100
                if abs(acceleration_tgt) < 100:
                    accel_increment = 10
                if abs(acceleration_tgt) < 50:
                    accel_increment = 5
                acceleration_tgt += accel_increment
                action = 'Increase acceleration speed by %d to %d' % (accel_increment, acceleration_tgt)
                screen.addstr(top + 3, 20, action)
                motor.set_acceleration(acceleration_tgt)
                screen.refresh()
                change = 1

            elif char == (ord('l') & 0x1f):
                action = 'Redraw screen'
                screen.addstr(top + 3, 20, action)
                screen.clear()
                screen.refresh()
                change = 1

            elif char == ord('t') or char == ord('T'):  # 10 random position points, see if it holds position
                motor.test = -1  # in this test we cannot have two processes
                                 # trying to asynchronosly read the position data
                                 # or even ask for different data at the same time
                                 # it will fail horribly so we need to disable the
                                 # position status line 8/ (re-enable on exit though)

                test_started = time.time()
                TEST_SETTLE = 2
                # ~ read_pos.stop(); import pdb; pdb.set_trace()

                # Read the config file for offsets? This is from E20.py
                # normally you'd just import it but I want to try and make
                # Motor.py self contained if possible

                def get_calibration_from_file(find_cal_for='CAMERA_OFFSET'):
                    '''
                    Find the calibration for camera/laser1/laser2
                    CALIBRATION_FILE is defined in settings.py
                    Expected format in ticks:
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
                        logging.warn(msg)
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

                CAL_FILE     = get_calibration_from_file(find_cal_for='CAMERA_OFFSET')[1]  # T/F
                CAL_CAMERA   = int(get_calibration_from_file(find_cal_for='CAMERA_OFFSET')[0])
                CAL_INNER    = int(get_calibration_from_file(find_cal_for='LV1_OFFSET')[0])
                CAL_OUTER    = int(get_calibration_from_file(find_cal_for='LV2_OFFSET')[0])

                # 'Perfect' postitions, note that there IS a 'bump of about 100' which I've
                # never been able to explain, but for testing we're more interested in
                # approximate location/slip rather than absolute calibrated location
                bump = 0
                if CAL_FILE is False:  # calibration file missing! self-denial!
                    bump = 0 - BUMP
                TICKS = SI_COUNT/float(360)

                LV1_location = CAL_INNER + (0         * float(TICKS))  # 0   or 0000   ticks
                LV2_location = CAL_INNER + ((360-59)  * float(TICKS))  # 301 or 3344.4 ticks
                LV3_location = CAL_INNER + ((360-99)  * float(TICKS))  # 261 or 2899.9 ticks
                LV4_location = CAL_INNER + ((360-139) * float(TICKS))  # 221 or 2455.5 ticks
                LV5_location = CAL_OUTER + (0         * float(TICKS))  # 0   or 0000   ticks
                LV6_location = CAL_INNER + ((360-156) * float(TICKS))  # 204 or 2266.6 ticks

                positions = [(CAL_CAMERA,    'IG*'),
                             ((LV1_location), 'LV1'),
                             ((LV2_location), 'LV2'),
                             ((LV3_location), 'LV3'),
                             ((LV4_location), 'LV4'),
                             ((LV5_location), 'LV5'),
                             ((LV6_location), 'LV6'),
                            ]
                # Make mirror positions and make them all fall in the 0 - SI_COUNT range
                fixup = []
                for position, label in positions:
                    fixup.append((round((position + bump + SI_COUNT) % SI_COUNT),
                                  round((position + bump +(SI_COUNT/2) + SI_COUNT) % SI_COUNT),
                                  label))
                positions = fixup

                # We should do this at the homeing/acceleration speeds
                # according to sasha we should be limiting using
                # max_abs_ramp_velocity.. which.. hurm... no.
                # ~ motor.set_max_abs_ramp_velocity(HOME_SPEED)
                action = 'Setting velocity and acceleration to %d rpm %d rpm/s' % \
                         (HOME_SPEED,
                          HOME_ACCEL)
                screen.addstr(top + 3, 20, action)
                motor.set_acceleration(HOME_ACCEL)
                motor.set_velocity(HOME_SPEED)

                screen.addstr(top + 4, 20, action)

                if char == ord('T'):
                    spins = 1
                    PERFECT = 1
                    action = 'The %d positions for well and valves (takes approx 3.75 min)'.ljust(len(action)) % (len(positions) * 2)
                else:
                    spins = 0
                    PERFECT = 0
                    action = '14 random position points (takes approx 2.5 min)'.ljust(len(action))
                    positions = []
                    for rnd_pt in range(0, 7):  # locations are paired 180° so 7x2 = 14 points
                        positions.append((random.randint(0, SI_COUNT - 1),
                                          random.randint(0, SI_COUNT - 1),
                                          'random'))

                screen.addstr(top + 3, 20, action)
                screen.refresh()

                res = []      # position wanted, got, after a couple of seconds
                motor.home()  # probably already done but just to be sure.

                pt = 0
                for i, (first, second, name) in enumerate(positions):  # use perfect positions else 0-9
                    if PERFECT == 1:
                        first_position = first
                        second_position = second
                    else:
                        first_position = random.randint(0, SI_COUNT - 1)
                        second_position = random.randint(0, SI_COUNT - 1)

                    for loop in range(0, 2):  # From 0, 2 numbers = 0,1
                        position = first_position if loop == 0 else second_position
                        if spins == 1 and loop == 0:
                            motor.set_acceleration(1000)  # for spinning
                            motor.spin(rpm=4000, spin_time=4, stopping=CALCULATED)
                            motor.home()  # find home position each time
                            motor.set_acceleration(HOME_ACCEL)  # back to finding positions

                        start_time = time.time()
                        motor.position_abs(position)
                        # Wait until the last move is completed
                        while not motor.flag_info(check_bit=MOVE_COMPLETED):
                            pass
                        elapsed_time = time.time() - start_time

                        # Where are we?
                        completed = motor.get_actual_encoder_count()
                        time.sleep(TEST_SETTLE)  # after a few seconds did it move?
                        final = motor.get_actual_encoder_count()
                        res.append((name, position, completed, final, elapsed_time))  # compile list
                        if PERFECT == 1:
                            msg = '%s: wanted: %04d  got: %04d - slew after %ds: %02d    '
                        else:
                            msg = 'Pt #%02d: wanted: %04d  got: %04d - slew after %ds: %02d    '
                        screen.addstr(top + 3, 20, msg \
                                                % (name + '#%d' % (pt % 2) if PERFECT else pt,
                                                   position,
                                                   completed,
                                                   TEST_SETTLE,
                                                   position - final))
                        pt += 1

                # Ok ,.. lets clear and dump the results
                screen.clear()

                # Add a 'rip off' header to denote the start in the logfile
                msg = '=== Position test '
                msg = msg + ''.ljust(80 - len(msg), '=')
                logging.info(msg)
                one_deg = SI_COUNT/360
                test_type  = 'Perfect well and valve test results' if PERFECT else 'Random point test results'
                test_type += ' (where 1° = %6.3f ticks & 1 tick = %.3f°)' % (one_deg, 1/one_deg)
                # ~ underscore = '-' * len(test_type)
                screen.addstr(top + -1, 3, test_type)
                logging.info(test_type)
                # ~ screen.addstr(top +  0, 3, underscore)
                gap = 1  # start scribbling a few lines down
                maximum = 0
                minimum = 0
                f_maximum = 0
                f_minimum = 0
                header  = '        | wanted | got  | diff | final after %2d sec | slew | time taken' % TEST_SETTLE
                screen.addstr(top +  0, 4, header)
                logging.info(header)
                if PERFECT:
                    msg = '%s:  | %4d   | %4d | %3d  | %5d              | %4d |  %7.3f'
                else:
                    msg = 'Pt #%02d: | %4d   | %4d | %3d  | %5d              | %4d |  %7.3f'

                rows, cols = screen.getmaxyx()  # find max rows...

                for i, (name, position, completed, final, elapsed_time) in enumerate(res):
                    if (top + gap + i) < rows -5:  # don't keep going beyond the screen depth
                        screen.addstr(top + gap + i, 4, msg \
                              % (name + '#%d' % (i % 2) if PERFECT else i,
                                 position,
                                 completed,
                                 (position - completed),
                                 final,
                                 (position - final),
                                 elapsed_time))
                    logging.info(msg \
                          % (name + '#%d' % (i % 2) if PERFECT else i,
                             position,
                             completed,
                             (position - completed),
                             final,
                             (position - final),
                             elapsed_time))

                    if (position - completed) > maximum:
                        maximum = position - completed
                    if (position - final) > f_maximum:
                        f_maximum = position - final
                    if (position - completed) < minimum:
                        minimum = position - completed
                    if (position - final) < f_minimum:
                        f_minimum = position - final

                # we need 3 bottom lines
                # Max range, After..,  <press any key> and the status line
                # so we need rows -4
                # ~ read_pos.stop();import pdb; pdb.set_trace()
                msg = 'Max range = %d ticks  %3.0f <-- 0 --> %3.0f (mv completed)' \
                      % ((max(maximum, minimum) - min(maximum, minimum)),
                         minimum,
                         maximum
                        )
                msg += '  Test took %s' % time.strftime("%Mm %Ss", time.gmtime(time.time() - test_started))
                screen.addstr(rows - 5, 0, ' ')
                screen.clrtoeol()
                screen.addstr(rows - 5, 4, msg)
                logging.info(msg)

                msg =  'After %2ds = %d ticks  %3.0f <-- 0 --> %3.0f' \
                       % (TEST_SETTLE,
                         (max(f_maximum, f_minimum) - min(f_maximum, f_minimum)),
                         f_minimum,
                         f_maximum
                        )
                screen.addstr(rows - 4, 0, ' ')
                screen.clrtoeol()
                screen.addstr(rows - 4, 4, msg)
                logging.info(msg)

                msg = '-'*80  # just add a 'rip off' line to denote the end of the test data in logfile
                logging.info(msg)

                screen.addstr(rows - 3, 0, ' ')
                screen.clrtoeol()

                if i < 16:
                    msg = 'Press any key to go back to menu'
                else:
                    msg = 'Press any key to go back to menu - additional %d values in %s' \
                          % (i - 14,
                             os.path.basename(LOGGING_FILE))
                screen.addstr(rows - 2, 0, ' ')
                screen.clrtoeol()
                screen.addstr(rows - 2, 6, msg)

                screen.addstr(rows - 1, 0, ' ')
                screen.clrtoeol()

                curses.flushinp()
                char = screen.getch()
                motor.test = 0  # re-enable the status line
                # Reset the acceleration and speed on exit
                motor.set_acceleration(velocity_tgt)
                motor.set_velocity(0)
                screen.refresh()
                screen.clear()
                repaint_screen()
                change = 1

            elif char == ord('h'):  # key: (h) *FIND/DISCOVER* HOME .. not necessarily GOTO
                action = 'Finding the home location'
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.home()
                change = 1

            elif char == ord('0'):  # key: (0) goto 0!
                action = 'Moving to 0'
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.position_abs(0)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('c'):  # key: (c) goto 0!
                action = 'Moving to 3600 (calibration)'
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.position_abs(3600)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('9'):  # key: (9) goto 90!
                action = 'Moving to 1000 (90deg)'
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.position_abs(1000)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('w'):  # key: (w) toggle between well positions!
                action = 'Moving to ' + str(well[well_selected])
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.position_abs(well[well_selected])
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                well_selected ^= 1
                change = 1

            elif char == ord(' '):  # fire? action? do something?
                # toggle spin/move
                spinning ^= 1
                action = 'Toggle Spinning (%d) @ %d' % (spinning, velocity_tgt)
                motor.spin((velocity_tgt * spinning), spin_time=0)
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                change = 1

            elif char == ord('r'):  # release the hounds smithers
                action = 'Turning power to the motor off (free wheel)'
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.motor_release()
                change = 1

            elif char == ord('b'):  # try and use the motor to lock the current position
                pos = motor.get_actual_encoder_count()
                action = 'Applying brake @ %d' % pos + ' ticks'
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                motor.position_abs(pos)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('g'):  # goto position and stay there
                action = 'Goto absoloute tick position'
                #read_pos.stop()  # stop the screen cursor bouncing around
                pos = ''

                #while not re.match('^-?[0-9]*\.?[0-9]+$', pos):
                #    pos = input(" Enter tick position to goto: ")

                # Max 1 through to 4 numbers 0-3999
                pos = screen.getstr(top + 3, 20, 4)

                pos = float(pos)
                action = action + pos

                #read_pos.start()  # restart the status update
                motor.position_abs(pos)
                while not motor.flag_info(check_bit=MOVE_COMPLETED):
                    pass
                screen.addstr(top + 3, 20, action)
                screen.refresh()
                change = 1

            elif char == ord('p'):  # PID fun
                # Ok lets do this as a seperate function
                # as we need to find the Current P/I
                # the Velocity P/I
                # the Positioning P
                # and all for a pile of different velocities
                read_pos.stop()
                time.sleep(2)
                find_pid()
                screen.clear()
                repaint_screen()
                read_pos.start()

            if change != 0:
                change = 0
                #motor.motor_release()
                # Clear Action messages
                repaint_screen()
                screen.move(top + 3, 20)
                screen.clrtoeol()
                screen.move(top + 4, 20)
                screen.clrtoeol()
                screen.addstr(top + 4, 20, action)
                screen.refresh()

    except Exception as e:
        screen.refresh()
        read_pos.stop()
        print('Bad things: ' + str(e))

    finally:
        # clean down curses environment
        screen.refresh()
        read_pos.stop()

    motor.motor_release()
    time.sleep(0.25)  # allow controller time to respond
    ser.close()
    print('Motor shutdown completed')
    # clean down curses environment
    screen.refresh()
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
