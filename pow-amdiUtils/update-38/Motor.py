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
from settings import LOGGING_FILE, SI_COUNT, HOME_ACCEL, HOME_SPEED
from settings import NONSTOP, GENTLE, HARSH, CALCULATED, MOVE_COMPLETED
from settings import MOTOR_INITIALIZED, SETTLE

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
            + str(inspect.currentframe().f_back.f_lineno)
            + ' via '
            + str(inspect.currentframe().f_back.f_back.f_code.co_name)
            + '() +'
            + str(inspect.currentframe().f_back.f_back.f_lineno))

class Motor:
    '''
    High level control of the Allmotion Motor
    '''

    def motor_init(self):
        # Initalize the motor
        # LEAVE THESE VALUES ALONE! (Sasha will hurt you!)
        self.torque_p                  = 200
        self.torque_i                  = 150
        self.position_p                = 250         # dangerous to meddle with

        self.velocity_p                = 3500        # Positioning speed
        self.velocity_i                = 350  #0     # Positioning integral

        self.velocity_max              = 9000        # NEVER exceed (RPM/min)
                                                     # the motor may do 12K but the
                                                     # cassette laminate won't
        self.max_current               = 3000        # mA (PSU trips over 3A)
        self.number_of_poles           = 4           # the default was 8 ?!

        self.mvp_target_reached_velo   = 5           # movement postioning switchs on at this speed
        self.encoder_steps             = SI_COUNT    # ticks per revolution

        self.reset_pid                 = 1           # flag used when something has
                                                     # changed the pid defaults

        # EXPECTIED DEFAULTS THAT SHOULD NOT CHANGE (MUCH)
        self.acceleration              = 1000        # RPM/s

        # FOR SPINNING

        # FOR POSITIONING
        self.velocity_max_pos_ctrl     = HOME_SPEED  # rpm/s position can only happen below this speed
        self.acceleration_pos_ctrl     = HOME_ACCEL  # rpm/s postioning movement can only happen below or @ X rpm/s
        self.velocity_p_pos_ctrl       = 4000        # rpm/min speed (P of PID)
        self.velocity_i_pos_ctrl       = 2000        # rpm/min speed (I of PID)

        # Lets implement the above settings
        # reference page 53
        # https://www.trinamic.com/fileadmin/assets/Products/Modules_Documents/TMCM-1640_TMCL_firmware_manual.pdf
        self.stop()
        self.motor_release()

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
            self.controller.set_velocity_ramp_generator(1)  # NEW! (might help but you will see bounces)
            self.controller.set_sine_initialization_speed(200)
            
            current = self.controller.get_max_current()
            logging.info('Max current limit is     %dmA' % current)
            print('Max current limit is     %dmA' % current)
            self.controller.set_max_current(self.max_current)  # set it to the safe maximum
            current_limit = self.controller.get_max_current()  # read the real value back
            print('Max current limit is now %dmA' % current_limit)
            logging.info('Max current limit is now %dmA' % current_limit)
            
            self.controller.set_communication_mode()
            self.controller.set_max_current(self.max_current)
            self.controller.set_number_of_motor_poles(self.number_of_poles)
            self.controller.set_hall_sensor_invert()
            self.controller.set_encoder_direction(1)        # spin is to the right
            self.controller.set_encoder_steps(self.encoder_steps)
            self.controller.set_encoder_init_mode()
            self.controller.set_max_abs_ramp_velocity(self.velocity_max)
            self.controller.set_mvp_target_reached_velocity(self.mvp_target_reached_velo)
            self.controller.set_torque_p(self.torque_p)
            self.controller.set_torque_i(self.torque_i)
            self.controller.set_velocity_p(self.velocity_p)
            self.controller.set_velocity_i(self.velocity_i)
            self.controller.set_position_p(self.position_p)

            logging.info('Motor initialized.')
            Path(MOTOR_INITIALIZED).touch()

        # These may or may not be set with the right values on powerup so just pin them
        self.controller.set_acceleration(self.acceleration)
        self.home()

    def __init__(self, ser, home_pin):
        self.controller = TrinamicMotor(ser, 0.3)
        self.home_pin = home_pin
        GPIO.setmode(GPIO.BCM)
        #GPIO.setmode(GPIO.BOARD)
        GPIO.setup(home_pin, GPIO.IN)
        self.found_home = 0
        self.motor_init()

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
        # Configure the speed control param to the max
        self.controller.set_max_abs_ramp_velocity(self.velocity_max)

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
                self.brake() # Gentle spindown by friction
            if stopping is HARSH:
                self.stop()  # HARSH! (avoid, seems to unset other settings)
                # print('position = %d' % self.get_actual_encoder_count())
            if stopping is CALCULATED:
                accel = self.get_acceleration()
                time_to_complete = abs(round(((rpm - rpm_end_tgt) / accel), 2))  # to 2 decimal places is good enough
                print('Calc time to reach %d from %d rpm is %d sec' % (rpm_end_tgt, rpm, time_to_complete))
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

    def info(self, check_bit):  # check for flag at bit postion N
        flags = self.controller.get_status_info()
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
        self.set_target_current_to_zero()   # This release the motor

    def motor_release(self):
        self.set_target_current_to_zero()

    def stop(self):
        # affects the PID by setting target velocity to zero (MST)
        self.controller.stop()
        self.reset_pid = 1

    def home_positioning(self):                                                     # this is called prior to taking images (pre&post), LV1, LV2, LV3, LV4 and LV5
                    
        while self.controller.get_encoder_set_null() != 1:                          # check if ESN != 0, if so try to set it to 1
            self.encoder_set_null(1) 

        print('encoder count %d will be reset to 0' % self.get_actual_encoder_count())

        # this 0's the encoder counter forcing it to do 1 revolution around for the next command
        
        while self.controller.get_encoder_set_null() != 0:                          # okay, so this is just me being very cautious. This whole section is to find "home"
                                                                                    # We do not want say we found "home" until ESN = 0
            while self.get_actual_encoder_count() != (-SI_COUNT):                   # We want to set the encoder count and make sure encounter count is set to "SI_COUNT" prior to going to 0 to find home
                self.set_actual_encoder_count(SI_COUNT)                             # setting this to positive (+) SI_COUNT forces the motor to spin CW when going to 0 and vice versa (neg(-) -> CCW)
        
            self.position_abs(0)                                                    # because the actual encoder count is SI_COUNT, telling the motor to go to 0 ensures that it will spin at most 1 full
                                                                                    # revolution. When the magnetic index passess over the hall sensor, it will reset the encoder count to 0 and the motor 
                                                                                    # will stop spinning
            # Wait until the last move is completed
            
            while not self.info(check_bit=MOVE_COMPLETED):
                pass
        else:
            print('homing finished, encoder %d %d' % (self.get_actual_encoder_count(), self.controller.get_encoder_set_null()))
       
        self.stop()

    def home(self):
        '''
        find the magnetic homing pin but because it's not a 'point'
        and has a small dead zone we try and measure the homing point
        from one side. Note that the controller finds the homing pin
        but just spins past it.
        '''
        # Take a note of the acceleration and velocity values (just in case)
        acceleration = self.get_acceleration()
        velocity = self.get_velocity()

        # We need to start *QUICKLY* but not too roughly
        # It's the VELOCITY that matters most for positioning
        # and we will be using 65 rpm as explained below
        if acceleration < 500:
            self.set_acceleration(500)

        halfway = int(SI_COUNT/2)                          # What is a half revolution?
        self.controller.set_actual_encoder_count(halfway)  # wherever were were is now a 'half turn' away
                                                           # eg 2187678 -> 2000 without actually rotating

        # Configure the position control param
        if self.reset_pid:
            self.pid_reset()

        # index reset the counter at next N channel event. and then ignore from then on
        self.controller.set_encoder_set_null(1)            # Reset to 0 when the sensor is triggered
        self.controller.set_encoder_clear_set_null(1)      # reset/clear the zero event trigger when it next happens

        self.spin(65,                                      # we now hunt for zero by spinning one revolution
                  spin_time=1,                             # 60 = 1 rev/sec just go a little further (65)
                  stopping=CALCULATED)

        # ~ stop_tick = self.get_actual_encoder_count()
        # ~ msg = 'Hunting 0 : Encoder now reads %d' % stop_tick
        # ~ logging.info(msg)
        # ~ print(msg)

        if self.controller.get_encoder_set_null() != 0:  # magnetic pin not seen after a full revolution
            print('No cassette detected or is it upside down and leaking?')
            sys.exit(-3)

        # restore acceleration
        self.set_acceleration(acceleration)

    def rotate_right(self, velo):
        print(lineno())
        # Some commands meddle with the default values and we have to
        # reset them here, eg MST (motor stop)
        if self.reset_pid:
            print('Rotate right - Reseting PID')
            self.controller.set_velocity_p(self.velocity_p)
            self.controller.set_velocity_i(self.velocity_i)
            self.reset_pid = 0  # reset flag
        self.controller.rotate_right(velo)

    def rotate_left(self, velo):
        print(lineno())
        # Some commands meddle with the default values and we have to
        # reset them here, eg MST (motor stop)
        if self.reset_pid:
            print('Rotate left - Reseting PID')
            self.controller.set_velocity_p(self.velocity_p)
            self.controller.set_velocity_i(self.velocity_i)
            self.reset_pid = 0  # reset flag

        self.controller.rotate_left(velo)
        
    def encoder_set_null(self, value):
        self.controller.encoder_set_null(value)

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
    from settings import MAGNETIC_INDEX_PIN, MOVE_COMPLETED
    from settings import WHITE_LED, WHITE_LED_ON, WHITE_LED_OFF

    ser = serial.Serial(
        port='/dev/ttyACM0',
        #baudrate=115200,
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
        )

    motor = Motor(ser, MAGNETIC_INDEX_PIN)

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
        screen.addstr(0, 34, 'Ticks      Deg      rpm')
        screen.addstr(1, 22, 'Position', curses.A_REVERSE)

        screen.addstr(top + 0, 0, 'Velocity     :  % 6.1d  Tgt:% 6.1d' % (motor.get_target_speed(), velocity_tgt))
        screen.addstr(top + 1, 0, 'Acceleration :  % 6.1d' % motor.get_acceleration())
        #screen.addstr(top + 0, 0, 'Velocity     :')
        #screen.addstr(top + 1, 0, 'Acceleration :')

        #screen.addstr(top + 3, 0, 'A -> B settings')
        #screen.addstr(top + 4, 0, 'From         :')
        #screen.addstr(top + 5, 0, 'To           :')
        #screen.addstr(top + 6, 0, 'ACTUAL       :')
        #screen.addstr(top + 7, 0, 'Toggle A-> B :')

        screen.addstr(top + 6, 0, 'CURRENT ACTION:', curses.A_REVERSE)
        screen.addstr(top + 7, 0, 'Last action:')

        screen.addstr(top + 9, 0, 'h - detect home position (DOES NOT STOP AT 0!)')
        screen.addstr(top + 10, 0, '0 - move to zero ( may not be 100% due to MVP position tolerance')
        screen.addstr(top + 11, 0, 'r - cut power to the motor - free spin (dragging to a magnetic pole likely)')
        screen.addstr(top + 12, 0, 'b - use driver to hold current postion (b)rake on')
        screen.addstr(top + 13, 0, 'c - move to calibration point (3600 ticks)')
        screen.addstr(top + 14, 0, 'w - toggle move to approx IGG/IGM well positions')
        screen.addstr(top + 15, 0, 'square brackets to decrease/increase acceleration')
        screen.addstr(top + 16, 0, 'left/right shifts +/- %d ticks    up/down change spin speed +/-%d' % (step, spin_increment))
        screen.addstr(top + 17, 0, 'q - quit program')

        screen.refresh()

    def timed_read_pos():
        screen.addstr(23, 0, 'Time: %s' % time.strftime('%H:%M:%S'))
        position = 0 + motor.get_actual_encoder_count()
        velocity = 0 + motor.get_actual_speed()
        screen.addstr(1, 32, '[% 6.1d] (% 6.1f) % 6.1d' % (position, (position * (360/SI_COUNT)), velocity))
        if velocity != 0:
            screen.addstr(1, 60, 'Spinning', curses.A_REVERSE)
        else:
            screen.addstr(1, 60, '        ')
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

    read_pos = RepeatedTimer(0.5, timed_read_pos)  # threaded timer every 1/2 second

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
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                #screen.addstr(6, 0, 'inc left/right step size    = %d' % step)
                change = 1

            elif char == curses.KEY_LEFT:
                action = 'Move right %d ticks' % -step
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.position_rel(-step)
                while not motor.info(check_bit=MOVE_COMPLETED):
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
                screen.addstr(top + 6, 20, action)
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
                screen.addstr(top + 6, 20, action)
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
                screen.addstr(top + 6, 20, action)
               #screen.addstr(top + 1, 28, '% 6.1d' % acceleration_tgt)
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
                screen.addstr(top + 6, 20, action)
                #screen.addstr(top + 1, 28, '% 6.1d' % acceleration_tgt)
                motor.set_acceleration(acceleration_tgt)
                screen.refresh()
                change = 1

            elif char == (ord('l') & 0x1f):
                action = 'Redraw screen'
                screen.addstr(top + 6, 20, action)
                screen.clear()
                screen.refresh()
                change = 1

            elif char == ord('h'):  # key: (h) *FIND/DISCOVER* HOME .. not necessarily GOTO
                action = 'Finding the home location'
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.home()
                change = 1

            elif char == ord('0'):  # key: (0) goto 0!
                action = 'Moving to 0'
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.position_abs(0)
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('c'):  # key: (c) goto 0!
                action = 'Moving to 3600 (calibration)'
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.position_abs(3600)
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('9'):  # key: (9) goto 90!
                action = 'Moving to 1000 (90deg)'
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.position_abs(1000)
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('w'):  # key: (w) toggle between well positions!
                action = 'Moving to ' + str(well[well_selected])
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.position_abs(well[well_selected])
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                well_selected ^= 1
                change = 1

            elif char == ord(' '):  # fire? action? do something?
                # toggle spin/move
                spinning ^= 1
                action = 'Toggle Spinning (%d) @ %d' % (spinning, velocity_tgt)
                motor.spin((velocity_tgt * spinning), spin_time=0)
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                change = 1

            elif char == ord('r'):  # release the hounds smithers
                action = 'Turning power to the motor off (free wheel)'
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.motor_release()
                change = 1

            elif char == ord('b'):  # try and use the motor to lock the current position
                pos = motor.get_actual_encoder_count()
                action = 'Applying brake @ %d' % pos + ' ticks'
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                motor.position_abs(pos)
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                change = 1

            elif char == ord('g'):  # goto position and stay there
                action = 'Goto absoloute tick position'
                #read_pos.stop()  # stop the screen cursor bouncing around
                pos = ''

                #while not re.match('^-?[0-9]*\.?[0-9]+$', pos):
                #    pos = input(" Enter tick position to goto: ")

                # Max 1 through to 4 numbers 0-3999
                pos = screen.getstr(top +6, 20, 4)

                pos = float(pos)
                action = action + pos

                #read_pos.start()  # restart the status update
                motor.position_abs(pos)
                while not motor.info(check_bit=MOVE_COMPLETED):
                    pass
                screen.addstr(top + 6, 20, action)
                screen.refresh()
                change = 1

            if change != 0:
                change = 0
                #motor.motor_release()
                # Clear Action messages
                repaint_screen()
                screen.move(top + 6, 20)
                screen.clrtoeol()
                screen.move(top + 7, 20)
                screen.clrtoeol()
                screen.addstr(top + 7, 20, action)
                screen.refresh()

    except Exception as e:
        #print('Bad things: ' + str(e))
        screen.refresh()
        read_pos.stop()

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
