try:
    import RPi.GPIO as GPIO
except Exception:
    print("Failed to load RPi.GPIO. Using Mock")
    import unittest
    from unittest.mock import patch, MagicMock

    MockRPi = MagicMock()
    modules = {
        "RPi": MockRPi,
        "RPi.GPIO": MockRPi.GPIO
    }
    patcher = patch.dict("sys.modules", modules)
    patcher.start()
    import RPi.GPIO as GPIO

try:
    import pymba
except Exception:
    print('Failed to load Vimba. Using Mock')
    MockPymba = MagicMock()
    MockSerial = MagicMock()
    modules = {
        "pymba": MockPymba,
        "serial": MockSerial,
    }
    patcher = patch.dict("sys.modules", modules)
    patcher.start()
    from pymba import Vimba

from E20 import E20
from tkinter import *
from tkinter import font
import tkinter.messagebox as messagebox
from datetime import datetime
from PIL import Image, ImageTk
from time import sleep
import json
import subprocess
import serial
import RPi.GPIO as GPIO
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
#import functions as cf
import cv2
import subprocess
from tkinter import StringVar
from datetime import datetime
from time import sleep
import json
from json2table import convert
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders
import subprocess
import settings
from settings import NONSTOP, GENTLE, HARSH, CALCULATED, MOVE_COMPLETED
from settings import SI_COUNT
from settings import INTENSITY_WET, INTENSITY_DRY
import time

class Window(Frame):

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        self.init_window()
        self.e20 = E20()
    # Create init_window


    def init_window(self):


        REPEATS = 12

        #Camera Settings
        

        # Blood plasma separation
        BPSSS    = 5500
        BPS_RD   = 15.0  # radial distance (mm) of the BPS chamber on the cassette
        BPS_TIME = 60    # length of time to spin to get seperation of plasma (PRP)
        TICKS = SI_COUNT/float(360)  # probably 11.11111 SI_COUNT/float(360)  # probably 11.11111
        
        # How far to swing the laser during valve cutting, speeds and delays etc
        # angles to ticks requires floats and rounds (fractions are ok as we round
        # the result eg 2.5deg = 27.777 ticks -> 28 ticks assigned
        SWING        = round(2.0 * float(TICKS))  # Inner laser swing
        SWING2       = round(2.0 * float(TICKS))  # LV5/7 are further away so the angular cut is smaller
        DELAY        = 1.01  # delay to lase over the same position in seconds
        STEPS_INNER  = 6
        STEPS_OUTER  = 3

        # Relative offsets for laser 1 at inner radius (starts at: 159? )
        # Physical laser is supposed to be at 180 deg
        # LV1 = 11 deg going clockwise from the homing pin
        LV1_location = 0        * float(TICKS)  # 0   or 0000   ticks
        LV2_location = (360-59)  * float(TICKS)  # 301 or 3344.4 ticks
        LV3_location = (360-99)  * float(TICKS)  # 261 or 2899.9 ticks
        LV4_location = (360-139) * float(TICKS)  # 221 or 2455.5 ticks
        LV6_location = (360-156) * float(TICKS)  # 204 or 2266.6 ticks

        # Relative offsets for laser 2 at outer radius
        # Physical laser is @ 105 deg
        # LV5 = 48 deg going clockwise from the homing pin
        LV5_location = 0         * float(TICKS)  # 0   or 0000   ticks
        LV7_location = (360-122) * float(TICKS)  # 122 or 2644.4 ticks

        # 


        def cut(self, lv_positions):
            
            self.e20.motor.home_positioning()

            for i in range(len(lv_positions)):
                position = round(lv_positions[i % len(lv_positions)])

                print('Moving to LV1#%d @ %d ticks (%d deg)'
                    % (i % len(lv_positions),
                        position,
                        position / (SI_COUNT/360)))
                
                if self.e20.laser_sweep(start_tick=position,
                                    width=SWING,
                                    delay=DELAY,
                                    steps=STEPS_INNER,
                                    laser_intensity=INTENSITY_WET) == -1:
                    return -2

        def cut2(self, lv_positions):
            
            self.e20.motor.home_positioning()

            for i in range(len(lv_positions)):
                position = round(lv_positions[i % len(lv_positions)])

                print('Moving to LV1#%d @ %d ticks (%d deg)'
                    % (i % len(lv_positions),
                        position,
                        position / (SI_COUNT/360)))
                
                if self.e20.laser2_sweep(start_tick=position,
                                    width=SWING2,
                                    delay=DELAY,
                                    steps=STEPS_OUTER,
                                    laser_intensity=INTENSITY_WET) == -1:
                    return -2
                
                    
        def lv1_cut():
            LV1_offset = float(varAngLV1.get())
            LV1_SAMPLE = [(LV1_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV1_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
            print(LV1_SAMPLE)

            cut(self, LV1_SAMPLE)

        def lv2_cut():
            LV1_offset = float(varAngLV1.get())
            LV2_WASH1    = [(LV2_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV2_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
            
            print(LV2_WASH1)

            cut(self, LV2_WASH1)

        def lv3_cut():
            LV1_offset = float(varAngLV1.get())
            LV3_ANTIBODY = [(LV3_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV3_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
            print(LV3_ANTIBODY)

            cut(self, LV3_ANTIBODY)
        
        def lv4_cut():
            LV1_offset = float(varAngLV1.get())
            LV4_WASH2    = [(LV4_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV4_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]

            print(LV4_WASH2)

            cut(self, LV4_WASH2)

        def lv6_cut():
            LV1_offset = float(varAngLV1.get())
            LV6_SPIN     = [(LV6_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV6_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]            
            print(LV6_SPIN)

            cut(self, LV6_SPIN)

        def lv5_cut():
            LV2_offset = float(varAngLV5.get())
            LV5_DRY    = [(LV5_location + LV2_offset + SI_COUNT) % SI_COUNT, (LV5_location + LV2_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
            print(LV5_DRY)

            cut2(self, LV5_DRY)

        def lv7_cut():
            LV2_offset = float(varAngLV5.get())
            LV7_RAMJET   = [(LV7_location + LV2_offset + SI_COUNT) % SI_COUNT, (LV7_location + LV2_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
            print(LV7_RAMJET)

            cut2(self, LV7_location)

        def donde():
            count = self.e20.motor.get_actual_encoder_count()
            print('position=',count)

        def move_position():
            pos =float(varAng.get())
            self.e20.motor.position_abs(int(pos))

        def home():
            t0 = time.time()
            self.e20.motor.home_positioning()
            t1 = time.time()
            totalT = t1-t0
            print('Homing time elapased:', totalT)

        def stop():
            self.e20.motor.brake()

        def ImageArray():
            t0 = time.time()
            IGM = float(varPasswd.get())
            IGG = float(IGM+ 2000)
            print('IGM, IGG:', IGM, IGG)
            
            expo = float(varExp.get())*1000000
            gain = float(varGain.get())
            gamma = float(varGamma.get())

            self.e20.motor.home_positioning()
            self.e20.motor.position_abs(IGG)
            sleep(5)
            
            
          
            print('Taking image of IGG Well')
            self.e20.camera.capture(exposure=expo,
                            gain=gain,
                            gamma=gamma,
                            flash=True,
                            filename='/home/autolab/Desktop/images/'+varNombre.get()+'igg.tif'
                            )
            sleep(2)
            self.e20.motor.home_positioning()
            self.e20.motor.position_abs(IGM)
            sleep(5)
            

            print('Taking image of IGM well')
            
            self.e20.camera.capture(exposure=expo,
                            gain=gain,
                            gamma=gamma,
                            flash=True,
                            filename='/home/autolab/Desktop/images/'+varNombre.get()+'igm.tif'
                            )

            print('Images of both arrays taken!')
            t1 = time.time()
            totalT = t1-t0
            print('Image time elapased:', totalT)

        
        def bloodPlasmaSeperation():
            t0 = time.time()
            self.e20.motor.set_acceleration(500)
            self.e20.motor.stop()

            rcf = lambda r: r * 1.118 * ((BPSSS/1000) ** 2)
            print('Blood plasma separation rcf %dg (%d rpm)' % (rcf(BPS_RD), BPSSS))
            self.e20.motor.spin(rpm=5500, spin_time=60, stopping=CALCULATED)
            self.e20.motor.reset_pid = 1
            t1 = time.time()
            totalT = t1-t0
            print('BPS time elapased:', totalT)

        def spin_oscilate():
            
            '''
            Function to oscillate the disk
            '''
            
            self.e20.motor.set_acceleration(3000)
            repeat = float(varRecip.get())
            loops = repeat  # No. of times to repeat (12?)
            print('spin_oscilate(%d)' % loops)
            print('Oscillate cycle')
            t0 = time.time()
            self.e20.motor.spin(rpm=7500, spin_time=10)
            
            while repeat > 0:
                print('Reciprication [%d/%d]' % ((loops+1)-repeat, loops))
                self.e20.motor.spin(rpm=7000, spin_time=1)
                self.e20.motor.spin(rpm=3000, spin_time=1)
                repeat -= 1  # keep decreasing by 1

            self.e20.motor.set_acceleration(3000)

            print('Shuffling fluids 1/4')
            self.e20.motor.spin(rpm=7000, spin_time=4)

            print('Shuffling fluids 2/4')
            self.e20.motor.spin(rpm=175, spin_time=6)

            print('Shuffling fluids 3/4')
            self.e20.motor.spin(rpm=0, spin_time=1)
            t1 = time.time()
            print('Shuffling fluids 4/4')
            self.e20.motor.spin(rpm=4000, spin_time=4, stopping=CALCULATED)
            
            totalT = t1-t0
            print('Osc time elapased:', totalT)


        def spin_fling():
            t0 = time.time()
            self.e20.motor.set_acceleration(500)
            self.e20.motor.stop()

            self.e20.motor.spin(rpm=-7500, spin_time=180, rpm_end_tgt=0, stopping=CALCULATED)
            sleep(2)

            self.e20.motor.set_acceleration(1000)
            self.e20.motor.spin(rpm=3000, spin_time=5, rpm_end_tgt=0, stopping=CALCULATED)
            self.e20.motor.reset_pid = 1
            t1 = time.time()
            totalT = t1-t0
            print('fling time elapased:', totalT)

        def laser1():
            t = float(varTime.get())
            #self.e20.laser1.set_intensity(self.e20.laser.LASER_1, 800)
            self.e20.fire_laser1(t=t)
                
        # Fires laser 2
        def laser2():
            t = float(varTime.get())
            #self.e20.laser2s.set_intensity(self.e20.laser.LASER_2, 800)
            self.e20.fire_laser2(t=t)
             
        
        def laserSweep():
            pos = float(varAng.get())
            self.e20.laser_sweep(start_tick=pos,
                                width=22,
                                delay=1.01,
                                steps=6)

        def laser2Sweep():
            pos = float(varAng.get())
            self.e20.laser2_sweep(start_tick=pos,
                                width=22,
                                delay=1.01,
                                steps=3)

        def led1():
            t = float(varTime.get())
            #self.e20.led1(t)
            self.e20.camera.flash_on_O()
            sleep(t)
            self.e20.camera.flash_off_O()

        def spinner():
            self.e20.motor.set_acceleration(3000)
            t = int(varTime.get())
            v = int(varV.get())
            s = str(varSTP.get())
            self.e20.motor.spin(rpm=v, spin_time=t, stopping=CALCULATED)
        
           
        # LED 2
        # def led2():
        #     t = float(varTime.get())
        #     self.e20.led2(t)

        def led2():
            t = float(varTime.get())
            #self.e20.led1(t)
            self.e20.camera.flash_on_W()
            sleep(t)
            self.e20.camera.flash_off_W()


         #GUI

        # Define title of master widget
        self.master.title("AMDI SEROLOGY READER")

        # Allow the widget to take the full space of the root window
        self.pack(fill=BOTH, expand=2)
        
        # Create font family:
        Title = font.Font(family="Myriad Pro", size=30, weight='bold')
        SubTitle = font.Font(family = 'Myriad Pro', size = 10)
        Buttons = font.Font(family = 'Myriad Pro', size = 16)
        BigButtons = font.Font(family = 'Myriad Pro', size = 20)
        Labels = font.Font(family = 'Myriad Pro', size = 12)
        
        # Title
        self.title = Label(root, text='Autolab - 20', fg = 'Black', font = Title )
        self.title.place(x=230,y=0)
        
        self.rev = Label(root, text='Rev.' +testRev, fg= 'Black', font = SubTitle)
        self.rev.place(x=300,y=50)
        
        self.rev = Label(root, text='RPI4_' + device_num, fg = 'Orange', font = SubTitle )
        self.rev.place(x=10,y=50)
        
        # Logo
        path = "ui_assets/AMDI.png"
        # Creates a Tkinter-compatible photo image, which can be used everywhere Tkinter expects an image object.
        #logo_img = ImageTk.PhotoImage(Image.open(path).resize((50, 50)))

        # The Label widget is a standard Tkinter widget used to display a text or image on the screen.
        #self.logo = Label(root, image=logo_img)
        #self.logo.image = logo_img
        #self.logo.place(x=10, y=10)

        self.varemail = Label(root, text = '...', fg = 'Black', font = Labels )
        self.varemail.place(x = 10, y = 100)
        varEmail = Entry(root)
        #varEmail.insert(END,'')
        varEmail.place(x = 120, y = 100)
        
        self.varvel = Label(root, text = 'IGM pos', fg = 'Black', font = Labels )
        self.varvel.place(x = 10, y = 125)
        varPasswd = Entry(root)
        varPasswd.insert(END,'731')
        varPasswd.place(x = 120, y = 125)  
        
        ### Camera control inputs
        
        # Exposure time
        self.varexp = Label(root, text = 'Exp time [s]', fg = 'Black', font = Labels )
        self.varexp.place(x = 10, y = 150)
        varExp = Entry(root)
        varExp.insert(END,'1')
        varExp.place(x = 120, y = 150)
        
        # Operator Name
        self.varRecip = Label(root, text = 'Recips', fg = 'Black', font = Labels )
        self.varRecip.place(x = 10, y = 75)
        varRecip = Entry(root)
        varRecip.insert(END,'')
        varRecip.place(x = 120, y = 75)
        
        
        # Gain
        self.vargain = Label(root, text = 'Gain [db]', fg = 'Black', font = Labels )
        self.vargain.place(x = 10, y = 175)
        varGain = Entry(root)
        varGain.insert(END,'0.2')
        varGain.place(x = 120, y = 175)
        
        # Gamma
        self.vargamma = Label(root, text = 'Gamma', fg = 'Black', font = Labels )
        self.vargamma.place(x = 10, y = 200)
        varGamma = Entry(root)
        varGamma.insert(END,'1')
        varGamma.place(x = 120, y = 200)
        
        # Filename
        self.varnombre = Label(root, text = 'File Name', fg = 'Black', font = Labels )
        self.varnombre.place(x = 10, y = 225)
        varNombre = Entry(root)
        varNombre.place(x = 120, y = 225)
        
        ### Hardware control inputs
        
        # Velocity for spindle motor
        self.varvel = Label(root, text = 'Velocity [rpm]', fg = 'Black', font = Labels )
        self.varvel.place(x = 10, y = 270)
        varV = Entry(root)
        varV.insert(END,'3000')
        varV.place(x = 120, y = 270)
        
        # Acceleration
        # self.varacc = Label(root, text = 'Acceleration', fg = 'Black', font = Labels )
        # self.varacc.place(x = 10, y = 295)
        # varAcc = Entry(root)
        # varAcc.insert(END,'1000')
        # varAcc.place(x=120, y = 295)

        #Stopping
        self.varstp = Label(root, text = 'Stopping', fg = 'Black', font = Labels )
        self.varstp.place(x = 10, y = 295)
        varSTP = Entry(root)
        varSTP.insert(END,'CALCULATED')
        varSTP.place(x=120, y = 295)
        
        # Time for laser motor
        self.vartime = Label(root, text = 'Time [s]', fg = 'Black', font = Labels )
        self.vartime.place(x = 10, y = 320)
        varTime = Entry(root)
        varTime.insert(END,'2')
        varTime.place(x=120, y = 320)
        
        # Angle for disc
        self.varangle = Label(root, text = 'Angle [tick]', fg = 'Black', font = Labels )
        self.varangle.place(x = 10, y = 345)
        varAng = Entry(root)
        varAng.insert(END,'1989')
        varAng.place(x=120, y = 345)

        self.varlv1 = Label(root, text = 'LV1 [tick]', fg = 'Black', font = Labels )
        self.varlv1.place(x = 10, y = 365)
        varAngLV1 = Entry(root)
        varAngLV1.insert(END,'1989')
        varAngLV1.place(x=120, y = 365)

        self.varlv5 = Label(root, text = 'LV5 [tick]', fg = 'Black', font = Labels )
        self.varlv5.place(x = 10, y = 385)
        varAngLV5 = Entry(root)
        varAngLV5.insert(END,'2200')
        varAngLV5.place(x=120, y = 385)
        
        ### Buttons

        lv1Button = Button(self, text = "LV1", font = Buttons, command = lv1_cut)
        lv1Button.place(x=500, y=25)

        lv2Button = Button(self, text = "LV2", font = Buttons, command = lv2_cut)
        lv2Button.place(x=500, y=60)

        lv3Button = Button(self, text = "LV3", font = Buttons, command = lv3_cut)
        lv3Button.place(x=500, y=95)

        lv4Button = Button(self, text = "LV4", font = Buttons, command = lv4_cut)
        lv4Button.place(x=500, y=120)

        lv6Button = Button(self, text = "LV6", font = Buttons, command = lv6_cut)
        lv6Button.place(x=500, y=150)

        lv5Button = Button(self, text = "LV5", font = Buttons, command = lv5_cut)
        lv5Button.place(x=500, y=185)

        lv7Button = Button(self, text = "LV7", font = Buttons, command = lv7_cut)
        lv7Button.place(x=500, y=215)

        # Spin motor at set rpm
        spinButton = Button(self, text = "Spin", font = Buttons, command = spinner)
        spinButton.place(x=600, y=25)
        
        # Break motor
        stopButton = Button(self, text = 'Brake', font = Buttons, fg='Black', command = stop)
        stopButton.place(x=600, y=75)
        
        # Turn laser 1 on
        laser1Button = Button(self, text = "Lsr1", font = Buttons, command = laser1)
        laser1Button.place(x=600, y=175)
        
        # Turn laser 2 on
        laser2Button = Button(self, text = "Lsr2", font = Buttons, command = laser2)
        laser2Button.place(x=670, y=175)
        
        # Turn led1 on
        led1Button = Button(self, text = "O led", font = Buttons, command = led1)
        led1Button.place(x=600, y=225)
        
        # Turn led2 on
        led2Button = Button(self, text = "W led", font = Buttons, command = led2)
        led2Button.place(x=680, y=225)
        
        # Move disc a set angle
        encoderButton = Button(self, text = "Move Angle", font = Buttons, command = move_position)
        encoderButton.place(x=300, y=325)
        
        # Find index
        findButton = Button(self, text = 'Find Index', font = Buttons, command = home, activebackground="orange")
        findButton.place(x=300, y= 275)

        BOT_ROW = 325
        
        # Encoder position for calibration
        positionButton = Button(self, text = "Position", font = Buttons, command = donde)
        positionButton.place(x = 300, y =375)

        # View results
        resultsButton = Button(self, text = "L1 swp", font = Buttons, command = laserSweep)
        resultsButton.place(x=600,y=260)

        resButton = Button(self, text = "L2 swp", font = Buttons, command = laser2Sweep)
        resButton.place(x=690,y=260)
        
         # Capture camera image
        captureButton = Button(self, text = "Capture", font = Buttons, command = ImageArray)
        captureButton.place(x = 600, y = 125)

        # Run assay
        assayButton = Button(self, text = "spinDry_fling", font = Buttons, fg='Green', command = spin_fling)
        assayButton.place(x = 300, y= 175)

        BOT_ROW_2 = 375

        # QR Scanner
        # QRButton = Button(self, text = "QR Scan", font = Buttons, command = scanQRCode)
        # QRButton.place(x = 600, y= 275)

        
        # Fans
        FansButton = Button(self, text = "spin_Osc", font = Buttons, fg='Red', command = spin_oscilate )
        FansButton.place(x = 300, y= 125)

        BOT_ROW_2 = 375
        
        # Lock lid
        # lockButton = Button(self, text = 'Lock Lid', font = Buttons, command = latch)
        # lockButton.place(x=480, y=325)

        # Unlock lid
        # unlockButton = Button(self, text = 'Unlock Lid', font = Buttons, command = unlatch)
        # unlockButton.place(x=480, y=375)

        # dondeDosButton =  Button(self, text = 'Store v1', font = Buttons, command = dondeDos)
        # dondeDosButton.place(x=10, y=BOT_ROW_2)

        # dondeTresButton =  Button(self, text = 'Store v5', font = Buttons, command = dondeTres)
        # dondeTresButton.place(x=150, y=BOT_ROW_2)

        # dondeQuaButton =  Button(self, text = 'Store array', font = Buttons, command = dondeQua)
        # dondeQuaButton.place(x=325, y=BOT_ROW_2)
        
        # Generate Report
        reportButton = Button(self, text = "BPS", font = Buttons, fg='Blue', command = bloodPlasmaSeperation)
        reportButton.place(x = 300, y= 75)

        BOT_ROW_2 = 375


testRev= '4'
videoOn = False

try:
    serialFile = open('/sys/devices/virtual/dmi/id/product_serial')
    serialNumber = serialFile.readline()[-9:-1]
    serialFile.close()
except:
   serialNumber = '00000000'

device_num = serialNumber
root = Tk()

#size of the window
root.geometry("800x480")

app = Window(root)
root.mainloop()


       

             

            
        

       
            
       





