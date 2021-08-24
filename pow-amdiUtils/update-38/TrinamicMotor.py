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
from settings import SI_COUNT
import inspect  # for line numbers

# command code values, used to verify command completions
SAP = 5   # Set Axis Paramater
GAP = 6   # Get Axis Paramater
SGP = 9   # Set Global Paramater
GGP = 10  # Get Global Paramater

def lineno():  # returns string
    """Returns the current line number in our program."""
    return (__file__.split('/')[len(__file__.split('/'))-1]
            + ': '
            + str(inspect.currentframe().f_back.f_code.co_name)
            + '() +'
            + str(inspect.currentframe().f_back.f_lineno))

class TrinamicMotor:
    cmd_buffer = ""
    rcv_instruction = 0
    rcv_data = 0
    rcv_checksum_passed = False

    def __init__(self, ser, command_delay=0):
        self.ser = ser
        self.command_delay = command_delay

    def __cmd_buffer(self, cmd):
        self.cmd_buffer = self.cmd_buffer + cmd

    def send(self):
        if (self.ser is not None) and (self.ser.is_open == True):
            # ~ print("send =", repr(self.cmd_buffer))
            self.ser.write(bytes.fromhex(self.cmd_buffer))

            self.cmd_buffer = ""
            sleep(self.command_delay)
            # ~ sleep(0.05)
        else:
            print('Serial line is dead. Unable to send.')

    def receive(self):
        if (self.ser is not None) and (self.ser.is_open == True):
            res = self.ser.read(9)
            sleep(self.command_delay)
            # ~ print("rcv  = ", res.hex()) ####

            rcv_status = res[2]                    # 100
            self.rcv_instruction = res[3]          # 3
            #self.rcv_data = bytearray(res[4:6])    # res[4:7] = b'\xff\xff\xf1'
            self.rcv_data = int.from_bytes(res[4:8], byteorder='big', signed=True)
            rcv_checksum = res[8]

            checksum = sum(bytearray(res[0:8]))
            checksum1 = checksum & 0xFF
            self.rcv_checksum_passed = False         # Fail
            if checksum1 == rcv_checksum:
                if rcv_status == 0x64:               # 100 - command completed ok
                    self.rcv_checksum_passed = True  # Success
                else:
                    print("status error = ", rcv_status)
            else:
                print("checksum error")
        else:
             print('Serial line is dead. Unable to recieve')

    def get_status_info(self):
        header_get_status_info = '01069c00'  # GAP 156 return errors and flags
        self.format_command(header_get_status_info, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    def get_target_speed(self):
        header_get_target_speed = '01060200'  # GAP 2 return TARGET speed
        self.format_command(header_get_target_speed, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    def get_actual_speed(self):
        header_get_actual_speed = '01060300'  # GAP 3 return ACTUAL speed
        self.format_command(header_get_actual_speed, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    def rotate_right(self, velo):
        header_ROR = '01010000'
        self.format_command(header_ROR, velo)
        self.send()
        self.receive()
        return self

    def rotate_left(self, velo):
        header_ROL = '01020000'
        self.format_command(header_ROL, velo)
        self.send()
        self.receive()
        return self

    def stop(self):  # NOTE: sets target velocity to 0
        header_MST = '01030000'
        self.format_command(header_MST, 0)
        self.send()
        self.receive()
        return self

    def position_abs(self, ticks):  # ticks
        header_MVP_ABS = '01040000'
        self.format_command(header_MVP_ABS, -ticks)
        self.send()
        self.receive()
        return self

    def position_rel(self, ticks):  # ticks
        header_MVP_REL = '01040100'
        self.format_command(header_MVP_REL, -ticks)
        self.send()
        self.receive()
        return self

    # ------------------------------------------------------------------
    def set_velocity_ramp_generator(self, value):  # 0:off 1:on
        header_set_velocity_ramp_generator = '01059200' # SAP 146
        self.format_command(header_set_velocity_ramp_generator, value)
        self.send()
        self.receive()
        return self

    def get_velocity_ramp_generator(self):
        header_get_velocity_ramp_generator = '01069200'  # GAP 146
        self.format_command(header_get_velocity_ramp_generator, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_max_abs_ramp_velocity(self, velo_max):
        header_set_max_abs_ramp_velocity = '01050400'  # SAP 4
        self.format_command(header_set_max_abs_ramp_velocity, velo_max)
        self.send()
        self.receive()
        return self

    def get_max_abs_ramp_velocity(self):
        header_get_max_abs_ramp_velocity = '01060400'  # GAP 4
        self.format_command(header_get_max_abs_ramp_velocity, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_mvp_target_reached_velocity(self, velo_max):
        header_set_mvp_target_reached_velo = '01050700'  # SAP 7
        self.format_command(header_set_mvp_target_reached_velo, velo_max)
        self.send()
        self.receive()
        return self

    def get_mvp_target_reached_velocity(self):
        header_get_mvp_target_reached_velocity = '01060700'  # GAP 7
        self.format_command(header_get_mvp_target_reached_velocity, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_acceleration(self, accel):
        header_set_accel = '01050B00'  # SAP 11
        self.format_command(header_set_accel, accel)
        self.send()
        self.receive()
        return self

    def get_acceleration(self):
        header_get_acceleration = '01060B00'  # GAP 11
        self.format_command(header_get_acceleration, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def get_ramp_generator_speed(self):
        header_get_ramp_generator_speed = '01060D00'  # GAP 13 (no SAP pairing)
        self.format_command(header_get_ramp_generator_speed, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def get_velocity_pid_error_sum(self):
        header_get_velocity_pid_error_sum = '0106E500'  # GAP 229 (no SAP pairing)
        self.format_command(header_get_velocity_pid_error_sum, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_position_p(self, pos_P):
        header_set_position_P = '0105E600'  # SAP 230
        self.format_command(header_set_position_P, pos_P)
        self.send()
        self.receive()
        return self

    def get_position_p(self):
        header_get_position_P = '0106E600'  # GAP 230
        self.format_command(header_get_position_P, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_velocity(self, velo):
        header_set_velocity = '01050200'  # SAP 2
        self.format_command(header_set_velocity, velo)
        self.send()
        self.receive()
        return self

    def get_velocity(self):
        header_get_velocity = '01060200'  # GAP 2
        self.format_command(header_get_velocity, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_velocity_p(self, velo_P):
        header_set_velocity_P = '0105EA00'  # SAP 233
        self.format_command(header_set_velocity_P, velo_P)
        self.send()
        self.receive()
        return self

    def get_velocity_p(self):
        header_get_velocity_P = '0106EA00'  # GAP 233
        self.format_command(header_get_velocity_P, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_velocity_i(self, velo_I):
        header_set_velocity_I = '0105EB00'  # SAP 235
        self.format_command(header_set_velocity_I, velo_I)
        self.send()
        self.receive()
        return self

    def get_velocity_i(self):  # I parameter
        header_get_velocity_I = '0106EB00'  # GAP 235
        self.format_command(header_get_velocity_I, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_torque_p(self, torque_P):
        header_set_torque_P = '0105AC00'  # SAP 172
        self.format_command(header_set_torque_P, torque_P)
        self.send()
        self.receive()
        return self

    def get_torque_p(self):  # P parameter
        header_get_torque_P = '0106AC00'  # GAP 172
        self.format_command(header_get_torque_P, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_torque_i(self, torque_I):
        header_set_torque_I = '0105AD00'  # SAP 173
        self.format_command(header_set_torque_I, torque_I)
        self.send()
        self.receive()
        return self

    def get_torque_i(self):
        header_get_torque_I = '0106AD00'  # GAP 173
        self.format_command(header_get_torque_I, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def encoder_set_null(self, zeorOrOne):
        header_set_encoder_set_null = '0105A100'  # SAP 161
        self.format_command(header_set_encoder_set_null, zeorOrOne)
        self.send()
        self.receive()
        return self
    
    def set_encoder_set_null(self, zeorOrOne):
        header_set_encoder_set_null = '0105A100'  # SAP 161
        self.format_command(header_set_encoder_set_null, zeorOrOne)
        self.send()
        self.receive()
        return self

    def get_encoder_set_null(self):
        header_get_encoder_set_null = '0106A100'  # GAP 161
        self.format_command(header_get_encoder_set_null, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_encoder_clear_set_null(self, zeorOrOne):
        header_set_encoder_clear_set_null = '0105A300'  # SAP 163
        self.format_command(header_set_encoder_clear_set_null, zeorOrOne)
        self.send()
        self.receive()
        return self

    def get_encoder_clear_set_null(self):
        header_get_encoder_clear_set_null = '0106A300'  # GAP 163
        self.format_command(header_get_encoder_clear_set_null, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_communication_mode(self):
        header_set_commun_mode = '01059F00'  # SAP 159
        self.format_command(header_set_commun_mode, 7)  # FOC based on encoder
        self.send()
        self.receive()
        return self

    # ------------------------------------------------------------------
    def set_max_current(self, max_current):
        header_max_current = '01050600'  # SAP 6
        self.format_command(header_max_current, max_current)  # max 1000mA peak
        self.send()
        self.receive()
        return self

    def get_max_current(self):
        header_get_max_current = '01060600'  # GAP 6
        self.format_command(header_get_max_current, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_number_of_motor_poles(self, number_of_poles):
        header_number_of_motor_poles = '0105FD00'  # SAP 253
        self.format_command(header_number_of_motor_poles, number_of_poles)
        self.send()
        self.receive()
        return self

    def set_hall_sensor_invert(self):
        header_clear_hall_sensor_invert = '0105FE00'  # SAP 254
        self.format_command(header_clear_hall_sensor_invert, 0)
        self.send()
        self.receive()
        return self

    def set_encoder_direction(self, direction=1):
        header_set_encoder_direction = '0105FB00'  # SAP 251
        self.format_command(header_set_encoder_direction, direction)  # 1 = go right
        self.send()
        self.receive()
        return self

    def set_encoder_steps(self, encoder_steps):
        header_set_encoder_steps = '0105FA00'  # SAP 250
        self.format_command(header_set_encoder_steps, encoder_steps)
        self.send()
        self.receive()
        return self

    def set_encoder_init_mode(self):
        header_set_encoder_init_mode = '0105F900'  # SAP 249
        self.format_command(header_set_encoder_init_mode, 1)  # Use hall sensor
        self.send()
        self.receive()
        return self

    def set_target_current_to_zero(self):
        header_set_target_current = '01059B00'  # SAP 155
        self.format_command(header_set_target_current, 0)
        self.send()
        self.receive()
        return self

    def wait_for_event(self, condition, timeout=0):  # NOT FOR DIRECT MODE!!
        if condition == 1:
            # Wait for target position reached
            header_wait_for_event = '011B0100'
        elif condition == 2:
            # Wait for reference switch
            header_wait_for_event = '011B0200'
        elif condition == 3:
            # Wait for limit switch
            header_wait_for_event = '011B0300'
        self.format_command(header_wait_for_event, timeout)
        self.send()
        self.receive()
        return self

    # ------------------------------------------------------------------
    def set_target_position(self, ticks):  # SAP 0 (in TICKS)
        header_set_target_position = '01050000'
        self.format_command(header_set_target_position, ticks)
        self.send()
        self.receive()
        return self

    def get_target_position(self):  # GAP 0 (in TICKS)
        header_get_target_position = '01060000'
        self.format_command(header_get_target_position, 0)
        self.send()
        self.receive()  # 9 bytes
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                position = -self.rcv_data % SI_COUNT
                return position

    # ------------------------------------------------------------------
    def get_actual_encoder_angle(self):  # GAP 1 (in degrees)
        header_get_actual_encoder_angle = '01060100'
        self.format_command(header_get_actual_encoder_angle, 0)
        self.send()
        self.receive()  # 9 bytes
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == 6:
                #angle = -(360*self.rcv_data/SI_COUNT)%360
                angle = -360 * self.rcv_data / SI_COUNT
                #print("DBG: ANG=%f" % angle)
                return angle

    def get_actual_encoder_angle_modulo(self):  # GAP 1 (in degrees)
        header_get_actual_encoder_angle = '01060100'
        self.format_command(header_get_actual_encoder_angle, 0)
        self.send()
        self.receive()  # 9 bytes
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                angle = -(360*self.rcv_data/SI_COUNT)%360
                return angle

    def get_actual_encoder_count(self):  # GAP 1 (in TICKS)
        header_get_actual_encoder_angle = '01060100'
        self.format_command(header_get_actual_encoder_angle, 0)
        self.send()
        self.receive()  # 9 bytes
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                #position = -self.rcv_data % SI_COUNT
                return -self.rcv_data

    # ------------------------------------------------------------------
    def set_actual_encoder_count(self, ticks):  # SAP 1
        header_set_actual_encoder_count = '01050100'
        self.format_command(header_set_actual_encoder_count, ticks)
        self.send()
        self.receive()
        return self

    def get_actual_encoder_count_modulo(self):  # GAP 1 (in TICKS)
        header_get_actual_encoder_angle = '01060100'
        self.format_command(header_get_actual_encoder_angle, 0)
        self.send()
        self.receive()  # 9 bytes
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                position = -self.rcv_data % SI_COUNT
                return position

    # ------------------------------------------------------------------
    def get_actual_current(self):
        header_get_actual_current = '01069600'  # GAP 150 (no SAP pairing)
        self.format_command(header_get_actual_current, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def set_target_current(self, current):
        header_set_ = '01059B00'  # SAP 155
        self.format_command(header_set_target_current, current)
        self.send()
        self.receive()
        return self

    def get_target_current(self):
        header_get_target_current = '01069B00'  # GAP 155
        self.format_command(header_get_target_current, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def get_encoder_set_null(self):
        header_get_encoder_set_null = '0106A100'  # GAP 161
        self.format_command(header_get_encoder_set_null, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result
        #else:

    def BLDC_reinitialization(self):
        header_BLDC_reinitialization = '01051F00'  # SAP 31 (write only)
        # restart the timer and initialize encoder.
        self.format_command(header_BLDC_reinitialization,0)
        self.send()
        self.receive()
        return self

    # ------------------------------------------------------------------
    def set_telegram_pause_time(self, pause, bank=0):
        '''
        There are three banks, 0,1 are for globals and 2 is for user params
        typically only 0 and 2 are used
        '''
        header_set_telegram_pause_time= '01094b' + "{:0>2}".format(bank) # SGP(9) 75
        self.format_command(header_set_telegram_pause_time, pause)
        self.send()
        self.receive()
        return self

    def get_telegram_pause_time(self, bank=0):
        header_get_telegram_pause_time = '010a4b' + "{:0>2}".format(bank) # GGP(10) 75
        self.format_command(header_get_telegram_pause_time, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GGP:  # remember this is the command used (10)
                result = self.rcv_data
                return result  # (eeprom default seems to be 0 )

    # ------------------------------------------------------------------
    def set_sine_initialization_speed(self, rpm):
        header_set_sine_initialization_speed = '0105F100'  # SAP 241
        self.format_command(header_set_sine_initialization_speed, rpm)
        self.send()
        self.receive()
        return self

    def get_sine_initialization_speed(self):
        header_get_sine_initialization_speed = '0106F100'  # GAP 241
        self.format_command(header_get_sine_initialization_speed, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result  # (eeprom default is +200)

    # ------------------------------------------------------------------
    def get_current_pid_error_sum(self):
        header_get_current_pid_error_sum = '0106C900'  # GAP 201
        self.format_command(header_get_current_pid_error_sum, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result
    # ------------------------------------------------------------------
    def set_encoder_commutation_offset(self, rpm):
        header_set_encoder_commutation_offset = '0105a500'  # SAP 165
        self.format_command(header_set_encoder_commutation_offset, rpm)
        self.send()
        self.receive()
        return self

    def get_encoder_commutation_offset(self):
        header_get_encoder_commutation_offset = '0106a500'  # GAP 165
        self.format_command(header_get_encoder_commutation_offset, 0)
        self.send()
        self.receive()
        if self.rcv_checksum_passed == True:
            if self.rcv_instruction == GAP:
                result = self.rcv_data
                return result

    # ------------------------------------------------------------------
    def format_command(self, header, data):
        if data < 0:
            dataHex = hex((1 << 32) + data)[2:]
        else:
            dataHex = hex(data)[2:]

        dataLen = len(dataHex)
        if dataLen == 1:
            dataBytes = '0000000' + dataHex
        elif dataLen == 2:
            dataBytes = '000000' + dataHex
        elif dataLen == 3:
            dataBytes = '00000' + dataHex
        elif dataLen == 4:
            dataBytes = '0000' + dataHex
        elif dataLen == 5:
            dataBytes = '000' + dataHex
        elif dataLen == 6:
            dataBytes = '00' + dataHex
        elif dataLen == 7:
            dataBytes = '0' + dataHex
        elif dataLen == 8:
            dataBytes = dataHex

        headerAndData = header + dataBytes
        headerAndDataHex = bytes.fromhex(headerAndData)
        checksum = sum(bytearray(headerAndDataHex))
        checksumHex = format(checksum, 'x')

        if len(checksumHex) == 1:
            checksumFinal = '0' + checksumHex
        elif len(checksumHex) > 2:
            checksumFinal = checksumHex[-2:]
        else:
            checksumFinal = checksumHex

        cmd = headerAndData + checksumFinal

        self.__cmd_buffer(cmd)
        return self
