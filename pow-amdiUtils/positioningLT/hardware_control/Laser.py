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
'''

from time import sleep
import logging
import inspect
import smbus
import RPi.GPIO as GPIO
from settings import LOGGING_FILE
from settings import LASER_ON, LASER_OFF, LASER_ENABLE
from settings import LASER_POWER_ON, LASER_POWER_OFF
from settings import LASER_TEMP_MULTIPLEXER, LASER_TEMP_SENSOR_ADDR
from settings import WHITE_LED, WHITE_LED_ON, WHITE_LED_OFF

logging.basicConfig(filename=LOGGING_FILE, level=logging.INFO, format='%(asctime)s %(levelname)s %(module)s - %(funcName)s: %(message)s', datefmt='%Y-%m-%d %H:%M:%S')


def lineno():  # returns string
    """Returns the current line number in our program."""
    return (__file__.split('/')[len(__file__.split('/'))-1]
            + ': '
            + str(inspect.currentframe().f_back.f_code.co_name)
            + '() +'
            + str(inspect.currentframe().f_back.f_lineno))

class Multiplexer:
    '''
    The lasers have a temperature sensor attached to them they are  multiplexed
    through a "Pericom PI4MSD5V9540B 2 Channel I2C bus Multiplexer"
    By default on powerup, NO channel is selected. So we need to fix that.

    The address is a bit obscurified because the docs don't explicity state
    where the MSB/LSB's are but the slave address is
    1|1|1|0|0|0|0|r/w
    ie 64+32+16 = 112 = 0x70 with the LSB denoting read/write

    The two TC74 sensors have a +/- 2C range of accuracy.
    Fortunately the scale runs from -65C to +127C using 2's complement
    with a 1:1 scale ie if I read the value 12,that will directly equate
    to 12degC. No weird conversions are needed.

    To check the multiplexer look for 0x70 on the 1st i2c bus
        i2cdetect -y 1
    if you were using the command line:
        i2cset -y 1 0x70 0x00 0x04 # channel a
        # device 0x48(72) shows up   but routes to laser 1
        i2cset -y 1 0x70 0x00 0x05 # channel b
        # device 0x48(72) shows up   now routes to laser 2
        i2cset -y 1 0x70 0x00 0x00
        # device 0x48(72) vanishs and multiplexer enters default state

    Now all this is trickery to swap in and out the two TC74's into the
    SAME i2c address.
    If you look at the TC74 sheets, the TC74 is internally programmed to
    have a default SMBus/I2C address value of 1001 101b = 0x4d but on the
    E20's thats actually on address 0x48

    When the multiplexer has selected a channel you will see a new device
    show up on the i2c tree (0x48)
        i2cdetect -y 1
    you can almost certainly immeadiately read the temperature at this
    point be directly reading register 0x00
        printf "%d C\n" `i2cget -y 1 0x48 0x00`

    Note that there is a physical cut off, if the lid is opened, that removes
    power to the laser circuitry including the TC74's but putting the lid back
    down will repower the circuit and it will almost immeadiately be responsive
    again. eg
        33 C
        33 C
        Error: Read failed
        0 C
        Error: Read failed
        0 C
        33 C
    '''
    def __init__(self, bus, devaddress):
        self.bus = smbus.SMBus(bus)
        self.devaddress = devaddress

    def __enter__(self):
        self.bus = smbus.SMBus(self.bus)
        return self.bus

    def __exit__(self, *args):
        raise Exception("Error with i2c")

    def channel(self, channel):  # values 0-1 indicate the channel, anything else (eg -1) turns off all channels
        if (channel == 0):       # laser 1
            action = 0x04        # x|x|x|x|x|1|0|0
        elif (channel == 1):     # laser 2
            action = 0x05        # x|x|x|x|x|1|0|1
                                 #           |   `---- channel (0/1)
                                 #            `------- enable channel
        else:
            action = 0x00        # you don't know what you're at, switch
                                 # the multiplexer to power on default
                                 # which disables all channels

        self.bus.write_byte_data(self.devaddress,  # 0x04 is the register for switching channels
                                 0x07,
                                 action  )

class TC74:
    '''
    class for the TC74
    '''
    def __init__(self, bus, devaddr):
        self.devaddr         = devaddr  # in the docs it seemed to be 0x4d?
        self.reg_temp        = 0x00     # temp register in 2's complement format
        self.reg_config      = 0x01
        self.shutdown_bit    = 7
        self.data_ready_bit  = 6
        self.tc74_i2c_bus    = smbus.SMBus(bus)

    def read_temp(self):
        temp   = self.tc74_i2c_bus.read_byte_data(self.devaddr, self.reg_temp)
        config = self.tc74_i2c_bus.read_byte_data(self.devaddr, self.reg_config)

        if (config & (1 << self.shutdown_bit)):
            print('TC74 sensor is not available at this time (STANDBY) %x' % config)
            # attempt to bring it online for next time
            config |= (1 << self.shutdown_bit)
            self.tc74_i2c_bus.write_byte_data(self.devaddr, self.reg_config, config)
            return None
        if not (config & (1 << self.data_ready_bit)):
            print('TC74 sensor is not ready at this time (NOT READY) %x' % config)
            # attempt to bring it online for next time
            config |= (1 << self.data_ready_bit)
            self.tc74_i2c_bus.write_byte_data(self.devaddr, self.reg_config, config)
            return None

        if (temp > 127):
            return (128 - temp)

        return temp


class Laser:
    '''
    # I = 5.371*ADC_SAMPLE / Amplifier_Gain (mA).
    # Set register ADCC = 0x04
    # I want the ADC active

    See page 29 of https://www.ichaus.de/upload/pdf/HTG_datasheet_B1en.pdf for overview

    # Reg 0x00, is a bitfield - has an overtemperature flag
    #  76543210
    #  |||||||`- INITRAM : 1;  0-RAM not initialized; 1-RAM initialized;
    #  ||||||`-- PDOVDD  : 1;  0-VDD Power down event not occured; 1-VDD Power down event occurred;
    #  |||||`--- MEMERR  : 1;  0-RAM has not been changed since last validation; 1-RAM has changd and has not been validat
    #  ||||`---- OVT     : 1;  0-No overtemperature event; 1-overtemperature event has occured
    #  |||`----- PDOVBL  : 1;  0-VBL power down not occurred; 1-VBL power down event has occurred
    #  ||`------ OVC     : 1;  0-No overcurrent; 1-overcurrent event has occured (NOTE: latches to prevent laser operating)
    #  |`------- OSCERR  : 1;  0-Oscillator functioning OK; 1-Watchdog timeout set on oscillator failure
    #  `-------- CFGTIMO : 1;  0-Not in config mode or timeout did not happened till now; 1-in config mode and timeout happened.

    # Reg 0x01, is a bitfield - holds status of laser channel
    #  76543210
    #  XXX||||`- MAPC    : 1; channel state (on/off)
    #     |||`-- MONC    : 1; Channel enabled at least once (latched)
    #     ||`--- EC      : 1; The Enable Channel pin digital state (ie are we allowed to lase)
    #     |`---- NMCOK   : 1; MCL, MCH voltage status
    #     `----- RANIN   : 1; read ANIN pin digital state

    # Reg 0x07, is a bitfield but only 1 bit is used
    #  76543210
    #  XXXXXXX`- DRDY : 1; 0-Nothing new since last read, 1- News ADC data available

    # Reg 0x10, is the configuration page register for:
    #  76543210
    #  |||X|||`- EACC    : 1;
    #  ||||||`-- ENAD    : 1;
    #  |||||`--- DISP    : 1;
    #  ||||`---- DISC    : 1;
    #  |||`----- always 1
    #  ```------ ADCC    : 3; lower 3 bits if the (2:0)
    #
    # ADCC function table
    # Code   Function
    # 000    ADC sourced by    V(VDD)÷8  (3 .. 5.5 V)
    # 001    ADC sourced by    V(VBL)÷30 (3 .. 24 V)
    # 010    ADC sourced by    V(VB)÷30  (3 .. 24 V)
    # 011    ADC sourced by    V(MDL)    (0 .. 1.1 V)
    # 100    ADC sourced by    V(MC)     (0 .. 1.1 V)
    # 101    ADC sourced by    V(VRN)÷30 (0 .. 24 V)
    # 110    ADC sourced by    V(VRP)÷30 (0 .. 24 V)
    # 111    ADC sourced by    V(ANIN)   (0 .. 1.1 V)

    # Reg 0x13
    #  76543210
    #  ||X|||``- REF(8:9): 2; 2 MSBs bits of the 10 bit D/A value used for the laser intensity
    #  || ||`--- MCVR    : 1; 0 - MCx Voltage Range is 0 to 5 V, 1 - MCx Voltage Range is VBL−5 V to V
    #  || | `--- ENAM    : 1; ENable Analog Modulation: 0 - disabled, 1 - enabled
    #  || `----- VRNHR   : 1; VRN voltage range: 0 - VRN set from 0V to VBL-1V, 1 - VRN set from 1V to VBL  (wot?!)
    #  ||
    #  |`------- NSW     : 1; CI regulator reference swap: 0 - Inverted control mode, 1 - Standard control mode
    #  `-------- EPNNP   : 1; Laser type: 0 - N-type, 1 - P-type

    # Reg 0x14, lower 8 bits of the 10 bit D/A value used for the laser intensity
    #  76543210
    #  ````````- REF(7:0): 8; LSB 8 bits of the 10 bit D/A value used for the laser intensity

    # Reg 0x16, is a bitfield - has an overtemperature flag
    #  76543210
    #  |X||XX|`- MOSCERR : 1; 0-Oscillator error will be signaled; 1-Oscillator error will not be signaled;
    #  | ||  `-- MMONC   : 1; 0-Enable channel will be signaled; 1-Enable Channel will not be signaled;
    #  | ||  ??? ENADCDIV ???
    #  | ||  ??? LINLOG   ???
    #  | |`----- SOVT    : 1; 0-No overtemperature event is simulated; 1-overtemperature event is simulated;
    #  | `------ SOVC    : 1; 0-No overcurrent event is simulated; 1-overcurrent event is simulated;
    #  |
    #  `-------- SOSCERR : 1; 0-No oscillator error simulated; 1-Oscillator error simulated;

    # Reg 0x1c, is a bitfield but only 3 bits are used
    #  76543210
    #  XXXXX|``-- MODE   : 2; 0-Not allowed; 1-Set operation mode; 2-Set Configuration mode; 3-Not allowed;
    #       |
    #       `---- ANINO  : 1; 0-ANIN pin pulled low (open collector); 1-ANIN pin set to high impedance;

    '''

    def __init__(self, out_pin, devaddr, intensity):
        """
        :param out_pin:
        :param devaddr
        """
        self.out_pin = out_pin
        self.devaddr = devaddr
        self.intensity = intensity
        self.bus = smbus.SMBus(1)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.out_pin, GPIO.OUT)
        GPIO.setup(LASER_ENABLE, GPIO.OUT)
        GPIO.output(LASER_ENABLE, LASER_POWER_ON)
        GPIO.setup(WHITE_LED, GPIO.OUT)
        self.white_led_on = GPIO.input(WHITE_LED)
        self.init_laser(devaddr)
        self.set_intensity(devaddr, intensity)

        self.laser_temp_sensor = Multiplexer(1, 0x70)

        # Status flags @ reg 0x00
        self.status_flag_reg_0 = {
            1:   'INITRAM',
            2:   'PDOVDD',
            4:   'MEMERR',
            8:   'OVT',
            16:  'PDOVBL',
            32:  'OVC',
            64:  'OSCERR',
            128: 'CFGTIMO' }

        # Status flags @ reg 0x01
        self.status_flag_reg_1 = {
            1:   'MAPC',
            2:   'MONC',
            4:   'EC',
            8:   'NMCOK' }

    def init_laser(self, devaddr):
        sleep(0.5)  # let the i2c bus catch up
        # Configure (leave laser power at 0 (0x13, 0x14 xxxx xx00  0000 0000 that still lases))
        # but remember to tell the system that we are a P-type laser (0x13 1xxx xxxx)
        # to reg  0x10  0x11  0x12  0x13  0x14  0x15  0x16  0x17  0x18  0x19  0x1a  0x1b  0x1c
        values = [0x15, 0xf0, 0xff, 0x40, 0x00, 0x40, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]
        self.bus.write_byte_data(devaddr, 0x1c, 0x02)  # Set configuration mode

        # Note: Reg 0x10 is the configuration page
        self.bus.write_i2c_block_data(devaddr, 0x10, values)     # write all the above values and
                                                                 # move to operation mode (0x13, 0x01)

        # A bit counter-intuitive but when powered up some bits are set in a latched
        # state preventing the laser from operating when first configured.
        # Reading the registers forces the driver to read the current states and
        # delatch eg 0x00 bit 4 (OVT) is set as 1 initially, but reg 0x00 needs to be
        # read to get the current status (which is hopefully 0!). Normally you would
        # do a lot of error checking but we're skipping that here.
        # Blanket read 0x00-0x07 and 0x10-0x1c: (see register list on
        # https://www.ichaus.de/upload/pdf/HTG_datasheet_B1en.pdf page 29)
        DEBUG = 0  # just for testing this function
        block = self.bus.read_i2c_block_data(devaddr, 0x00, 7)
        if DEBUG:
            print('===========================================')
            print('laser driver @ 0x%02x' % devaddr)
            print('-------------------------------------------')
            # debug:  What state are we in?
            for i in list(range(0, 0x07)):
                bits_string = '{0:08b}'.format(block[i])
                res = list(bits_string)
                res.insert(4, ' ')
                bits_string = ''.join(res)
                print('0x%02x register value  = 0x%x\t(%s)' % (i, block[i], bits_string))

        block = self.bus.read_i2c_block_data(devaddr, 0x10, 13)
        if DEBUG:
            print('')
            # debug:  What state are we in?
            for i in list(range(0, 0xd)):
                bits_string = '{0:08b}'.format(block[i])
                res = list(bits_string)
                res.insert(4, ' ')
                bits_string = ''.join(res)
                print('0x%02x register value  = 0x%x\t(%s)' % (i+0x10, block[i], bits_string))

    def get_status(self, devaddr):
        # Read Reg 0x00 and 0x01
        reg0 = self.bus.read_byte_data(devaddr, 0x00)
        reg1 = self.bus.read_byte_data(devaddr, 0x01)
        # print('Status: 0x%x' % reg0)
        # print('Status: 0x%x' % reg1)

        # What do I want to pass back here? the bytes paired
        # and figure out the bits elsewhere?
        return (reg1 << 8) + reg0

    def set_intensity(self, devaddr, intensity):
        # byte 1 : 0x40 | byte 2 : 0x00 | level (passed as integer)
        # 0x40 set the top bit EPNNP because our laser is a P-type laser
        # take value and convert it to 10bit (see how REF is composed below)
        value = int("0x40", 16) * 256 + int("0x00", 16) + intensity
        byte1 = value >> 8
        byte2 = value & 255
        reg_0x13 = self.bus.read_byte_data(devaddr, 0x13)  # get the current contents so we mask value
        reg_0x13 &= 0x7C      # just remember the bits 6:2, we don't want to blat them. Bit 7 was set with 0x40 above
                              # mask 0x7C = x111 11xx
        byte1    |= reg_0x13  # fortunately we don't need to do anything more complicated than add the values

        self.bus.write_byte_data(devaddr, 0x1c, 0x02)  # ANIN (ANalogue INput) set to high impedance
        self.bus.write_byte_data(devaddr, 0x13, byte1) # REF(9:8) two bytes for 10 bit D/A
        self.bus.write_byte_data(devaddr, 0x14, byte2) # REF(7:0) range 0x000 to 0x3FF)
        self.bus.write_byte_data(devaddr, 0x1c, 0x01)  # bit 1 - Chip set in operation mode (apply configuration)
        block = self.bus.read_i2c_block_data(devaddr, 0x13, 2)  # Read ANIN pin state (ANalogue INput)
        logging.info("Laser %s enabled and initialized.", self.out_pin)

    def get_power(self, devaddr):
        # We wish to return the power value that was placed into 0x13/0x14 REF(9:8) REF(7:0)
        # which was a 10 bit D/A value (0-0x3ff = 0 - 1024) but we only go as far as 900 because
        # plastic starts melting and its a terrible mess as blood starts flying everywhere.
        # Try it! (we know you want to) and don't forget to take a picture of the carnage!
        reg_0x13 = self.bus.read_byte_data(devaddr, 0x13)
        reg_0x14 = self.bus.read_byte_data(devaddr, 0x14)
        reg_0x13 = (reg_0x13 & 0x03) << 8 # get bottem two bits then shift by 8

        # and is the laser active? 0 = no 1 = yes
        EC = self.bus.read_byte_data(devaddr, 0x01)
        EC = EC >> 2 & 1  # check value of bit 2

        # return tuple (enabled (0/1) and intensity value)
        return (EC, (reg_0x13 + reg_0x14))

    def set_bit(self, value, bit):
        value |= (1 << bit)
        return value

    def clear_bit(self, value, bit):
        value &= ~(1 << bit)
        return value

    def read_bit(self, value, bit):
        return (value >> bit) & 1

    def read_ADC(self, devaddr):
        curses.nocbreak()
        curses.echo()
        curses.endwin()

        # Stop the ADC
        # set ENADCDIV to 1 (bit 2 in reg 16)
        value = self.bus.read_byte_data(devaddr, 0x16)
        self.bus.write_byte_data(devaddr, 0x16, self.set_bit(value, 2))

        # Set the ADCC (reg 10, bits [7:5] to be 0x04 - ADC sourced by V(MC) (0 .. 1.1 V)
        reg = self.bus.read_byte_data(devaddr, 0x10)
        reg |= (0x04 << 5)
        self.bus.write_byte_data(devaddr, 0x10, reg)

        # Start ADC conversion (ENAD set to 1) (reg 10 bit 1).
        # read the register, clear ENAD (to clear DRDY), then set ENAD
        reg = self.bus.read_byte_data(devaddr, 0x10)
        self.bus.write_byte_data(devaddr, 0x10, self.clear_bit(reg, 1<<1))  # clear bit 1 (knock on effect clears DRDY)

        # Set ENAD without disturbing the rest of the register
        self.bus.write_byte_data(devaddr, 0x10, self.set_bit(reg, 1<<1))    # set ENAD (bit 1)

        # Wait for DRDY set to 1
        while not self.bus.read_byte_data(devaddr, 0x07):  # are we ready?
            pass

        # Stop the ADC (ENAD set to 0).
        reg = self.bus.read_byte_data(devaddr, 0x10)
        self.bus.write_byte_data(devaddr, 0x10, self.clear_bit(reg, 1<<1))  # clear bit 1 (knock on effect clears DRDY

        # Read the ADC data.
        msb = self.bus.read_byte_data(devaddr, 0x03)
        lsb = self.bus.read_byte_data(devaddr, 0x04)
        return (msb << 8) + lsb

    def read_uncalibrated_temp(self, devaddr):
        '''
        The laser driver board has an internal temperature sensor but it is
        UNCALIBRATED!

        the Laser will operate beteen -40 and 85C but the temp sensor
        detects from -40 to 125C and scales at 1 = 1C
        '''
        value = self.bus.read_byte_data(devaddr, 0x02)
        # TODO: some calibration offset formula
        # value = ??? I suppose if an overtemp event occurs we know that
        # it hit 85C though that is a terrible way to calibrate
        value = value -40  # wild assumption that 0 = -40C likely badly wrong

        # check that the laser didnt go overtemp or overcurrent!
        value = self.bus.read_byte_data(devaddr, 0x02)

        return value

    def on(self):
        """
        Turn the power to the laser on.
        :return: The state of the laser power
        """
        self.white_led_on = GPIO.input(WHITE_LED)
        if self.white_led_on:  # makes it easier to see
            GPIO.output(WHITE_LED, WHITE_LED_OFF)
            self.white_led_on = 2
        GPIO.output(self.out_pin, LASER_ON)
        self.power = True
        logging.info("Laser GPIO(%s) on.", self.out_pin)
        return self.power

    def off(self):
        """
        Turn the power to the laser off.
        :return: The state of the laser power
        """
        GPIO.output(self.out_pin, LASER_OFF)
        logging.info("Laser GPIO(%s) off.", self.out_pin)
        if self.white_led_on == 2:
            GPIO.output(WHITE_LED, WHITE_LED_ON)
        self.power = False
        return self.power

    def power_state(self):
        """
        Get the state of the laser power
        :return: The state of the laser power
        """
        return self.power

    def position_time(self, position, time):
        """
        Go to a position, and turn the laser on for a set amount of time
        :return: The state of the laser power
        """
        self.position(pos=position)
        self.on()
        sleep(time)
        self.off()
        return self.power

    def on_time(self, time=5):
        self.on()
        sleep(time)
        self.off()

    def disable(self):
        # disable through reg 0x10 bit 3? (0 = on 1 = off?)
        reg = self.bus.read_byte_data(self.devaddr, 0x10)
        self.bus.write_byte_data(self.devaddr, 0x10, self.clear_bit(reg, 1<<3))

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LASER_ENABLE, GPIO.OUT, initial=LASER_POWER_OFF)
        GPIO.output(self.out_pin, LASER_OFF)
        if self.white_led_on == 2:
            GPIO.output(WHITE_LED, WHITE_LED_ON)
        self.power = False
        logging.info("Lasers disabled.")



# I believe in being able to test the code standalone where possible.
# you can run this directly as
#    python3 Laser.py

if __name__ == '__main__':
    import os
    import time
    from settings import LASER_1_OUT_PIN, LASER_2_OUT_PIN
    from settings import LASER_1, LASER_2

    # bit of borrowed code
    import threading
    LASER_PROFILE_LOG = '/data/laser-profile.txt'

    class RepeatedTimer(object):
        '''
        potential use as a threaded timer every 1/2 second
        read_temps = RepeatedTimer(0.5, timed_temp_read, args=(1,2,3,4,),
                                   kwargs={'a_number':1234,})
        '''
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

    # we have two lasers, make two objects to fiddle with at lowest
    # active power (0 = least power BUT DOES LASE!)
    laser1=Laser(LASER_1_OUT_PIN, LASER_1, 0)
    laser2=Laser(LASER_2_OUT_PIN, LASER_2, 0)

    mplx=Multiplexer(1, LASER_TEMP_MULTIPLEXER)    # Multiplexer on bus 1 address 0x70
    mplx.channel(0)                                # Select laser0 temperature sensor
    tc74_sensor = TC74(1, LASER_TEMP_SENSOR_ADDR)  # temp sensor on bus 1 address 0x48
    temp = tc74_sensor.read_temp()                 # read in the temp from whichever sensor
                                                   # was selected

    if temp is None:
        print('Alas, something is preventing me reading the laser temperature sensors')
        print('Is the lid open? Well,. I will keep going anyway.')

    print('Welcome to the laser tests')
    print('')
    print('#           #      #####   #######  ######         #######  #     #  #######  ')
    print('#          # #    #     #  #        #     #        #         #   #   #        ')
    print('#         #   #   #        #        #     #        #          # #    #        ')
    print('#        #     #   #####   #####    ######         #####       #     #####    ')
    print('#        #######        #  #        #   #          #           #     #        ')
    print('#        #     #  #     #  #        #    #         #           #     #        ')
    print('#######  #     #   #####   #######  #     #        #######     #     #######  ')
    print('')
    print('')
    print('#     #     #       #######     #     ######   ######   ')
    print('#     #    # #           #     # #    #     #  #     #  ')
    print('#     #   #   #         #     #   #   #     #  #     #  ')
    print('#######  #     #      #      #     #  ######   #     #  ')
    print('#     #  #######     #       #######  #   #    #     #  ')
    print('#     #  #     #    #        #     #  #    #   #     #  ')
    print('#     #  #     #   #######   #     #  #     #  ######   ')

    print('')
    print('Put on GOGGLES because these lasers can permanently blind you in a flash')
    print('and spare eyeballs don\'t exist.    So GOGGLES ON. NOW!')
    print('')
    print('(New logfile in %s [overwrote last one])' % LASER_PROFILE_LOG)
    sleep(3)

    # manual setting using arrow keys and esc to continue
    import curses

    # get the curses screen window
    screen = curses.initscr()

    # turn off input echoing
    curses.noecho()

    # respond to keys immediately (don't wait for enter)
    curses.cbreak()

    # map arrow keys to special values
    screen.keypad(True)

    # Turn off the cursor!
    curses.curs_set(0)

    active1    = 0                   # laser is at lowest setting by default
    active2    = 0                   # laser is at lowest setting by default
    cur_time   = int(time.time())    # time since last turned on/off in seconds
    log_time   = cur_time + 0.5      # keep an eye on the runtime of the log
    previous1  = cur_time            # last time was..
    previous2  = cur_time            # last time was..
    laser      = 1                   # start off with the inner laser
    power      = 10                  # power increment
    power1     = 0                   # start off low
    power2     = 0                   # start off low
    output1    = 0                   # read this in or calculate somehow #FAKE?
    output2    = 0                   # read this in or calculate somehow #FAKE?

    mplx.channel(0)                # Select laser1 temperature sensor
    temp1    = tc74_sensor.read_temp()
    mplx.channel(1)                # Select laser2 temperature sensor
    temp2    = tc74_sensor.read_temp()

    change  = 0      # flag to see if anything changed
                     # since we can only process one char at a time this
                     # should be good enough
    max_lvl = 900    # maximum laser level (laser will cut out at 1000! and needs reset)
    tick    = 0

    def paint_screen():
        # print doesn't work with curses, use addstr instead
        screen.addstr(22, 0, 'Use \'l\' to reset the time in the logfile')
        screen.addstr(23, 0, 'Use arrow keys, space bar (on/off) and \'q\' to quit')
        screen.addstr(0, 9, 'active\tpower\toutput\tlaser (℃)\tlast changed (sec)')
        screen.addstr(1, 0, 'laser1 - %d\t%d\t%d\t%d' % (active1, power1, output1, temp1))
        screen.addstr(2, 0, 'laser2 - %d\t%d\t%d\t%d' % (active2, power2, output2, temp2))
        screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl)
        screen.addstr(1, 0, 'Laser1', curses.A_REVERSE)

        screen.addstr(6, 0, 'Special note: even at power level 0 the laser WILL lase, you have to turn it off')
        screen.addstr(8, 0, 'TICK')

    paint_screen()
    laser1.set_intensity(LASER_1, 0)
    laser2.set_intensity(LASER_2, 0)


    def timed_temp_read():
        screen.addstr(9, 0, 'Time: %s' % time.strftime('%H:%M:%S'))
        mplx.channel(0)  # Select laser1 temperature sensor
        temp1   = tc74_sensor.read_temp()
        mplx.channel(1)  # Select laser2 temperature sensor
        temp2   = tc74_sensor.read_temp()
        screen.addstr(1, 32, '%d' % temp1)
        screen.addstr(2, 32, '%d' % temp2)
        # update the last states
        cur_time = round(time.time())
        screen.addstr(1, 48, '%-03d' % int(cur_time - previous1))
        screen.addstr(2, 48, '%-03d' % int(cur_time - previous2))
        (active1, power1) = laser1.get_power(LASER_1)
        (active2, power2) = laser2.get_power(LASER_2)
        screen.addstr(1, 9, '%d' % active1)
        screen.addstr(2, 9, '%d' % active2)
        screen.addstr(1, 16, '%d' % power1)
        screen.addstr(2, 16, '%d' % power2)

        with open(LASER_PROFILE_LOG, 'a+') as profile:
            profile.write('%.1f, laser 0 state/pwr/temp, %d, %d, %d, Laser 1 state/pwr/temp, %d, %d, %d\n' % (
                          (round ((time.time() - log_time) * 2) / 2), # half sec resolution
                          active1,
                          power1,
                          temp1,
                          active2,
                          power2,
                          temp2))
        # flash the tick
        attrs = screen.inch(8, 1)
        ch = chr(attrs & 0xFF)
        # isbold = bool(attrs & curses.A_BOLD)
        # screen.addstr(10, 0, str(isbold))
        if ch == 'O':
            screen.addstr(8, 0, 'TICK')
        else:
            screen.addstr(8, 0, 'TOCK', curses.A_REVERSE)
        screen.refresh()  # repaint the screen

    # remove old profile file
    if os.path.exists(LASER_PROFILE_LOG):
        os.remove(LASER_PROFILE_LOG)


    read_temps = RepeatedTimer(0.5, timed_temp_read)  # threaded timer every 1/2 second

    try:
        while True:

            curses.flushinp()      # Flush all input buffers. This throws away
                                   # any typeahead that has been typed by the
                                   # user and has not yet been processed by the program.
            char = screen.getch()  # only proceed if a character was input
            cur_time = time.time()

            if char == ord('q'):   # 'q' to quit
                break

            elif char == curses.KEY_RIGHT:
                power = 10
                #screen.addstr(6, 0, 'power up   = %d' % power)
                change ^= 1

            elif char == curses.KEY_LEFT:
                power = -10
                #screen.addstr(6, 0, 'power down = %d' % power)
                change ^= 1

            elif char == curses.KEY_UP:
                #screen.addstr(4, 0, 'Operating on Laser 1')
                screen.addstr(1, 0, 'Laser1', curses.A_REVERSE)
                screen.addstr(2, 0, 'Laser2')
                laser = 1

            elif char == curses.KEY_DOWN:
                #screen.addstr(4, 0, 'Operating on Laser 2')
                screen.addstr(1, 0, 'Laser1')
                screen.addstr(2, 0, 'Laser2', curses.A_REVERSE)
                laser = 2

            elif char == (ord('l') & 0x1f):
                paint_screen()
                screen.refresh()

            elif char == ord(' '):  # use space to change selected laser on or off
                if laser == 1:
                    previous1 = cur_time
                    active1 ^= 1
                    if active1:
                        laser1.on()
                    else:
                        laser1.off()

                if laser == 2:
                    previous2 = cur_time
                    cur_time = previous2
                    active2 ^= 1
                    if active2:
                        laser2.on()
                    else:
                        laser2.off()

                log_time = cur_time  # for the log, set the time to 0 for easier graphing
                power = 0            # we don't want to change power settings
                change ^= 1

            elif char == ord('l'):   # reset log timer
                log_time = cur_time

            if change:
                change ^= 1          # reset  flag

                if laser == 1:
                    #screen.addstr(10, 0, 'active1 = (power, power1, max_lvl) - %d (%d\t%d\t%d)' % (active1, power, power1, max_lvl))
                    (active1, power1) = laser1.get_power(LASER_1)
                    if  power < 0:   # going -ve
                        if power1 <= 0:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl, curses.A_REVERSE)
                        else:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl)
                            power1 = power1 + power
                    else:            # going +ve
                        if power1 >= max_lvl:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl, curses.A_REVERSE)
                        else:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl)
                            power1 = power1 + power
                    output1 = 0      # laser1.read_ADC(LASER_1)  # What is our output?
                    mplx.channel(0)  # Select laser1 temperature sensor
                    screen.addstr(1, 7, '- %d\t%d\t%d\t%d\t\t%-03d' % (
                                  active1,
                                  power1,
                                  output1,
                                  tc74_sensor.read_temp(),
                                  int(cur_time - previous1)))
                    laser1.set_intensity(LASER_1, power1)

                if laser == 2:
                    (active2, power2) = laser2.get_power(LASER_2)
                    if  power < 0:  # going -ve
                        if power2 <= 0:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl, curses.A_REVERSE)
                        else:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl)
                            power2 = power2 + power
                    else:           # going +ve
                        if power2 >= max_lvl:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl, curses.A_REVERSE)
                        else:
                            screen.addstr(5, 0, 'Maximum laser level range allowed in this test is 0 - %d' % max_lvl)
                            power2 = power2 + power

                    output2 = 0     # laser2.read_ADC(LASER_2)  # What is our output?
                    mplx.channel(1) # Select laser2 temperature sensor
                    screen.addstr(2, 7, '- %d\t%d\t%d\t%d\t\t%-03d' % (
                                  active2,
                                  power2,
                                  output2,
                                  tc74_sensor.read_temp(),
                                  int(cur_time - previous2)))
                    laser2.set_intensity(LASER_2, power2)

    finally:
        # shut down cleanly
        read_temps.stop()
        laser1.off()
        laser2.off()
        curses.curs_set(1); curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()


