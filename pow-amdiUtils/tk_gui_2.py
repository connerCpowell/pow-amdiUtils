#
#  This work is protected under applicable local and international
#  copyright laws.  Copying, transmitting, using, or any other activity
#  is expressly forbidden without prior signed written approval from
#  the copyright owner.
#
#  Copyright(c) 2019, 2020 Autonomous Medical Device, Incorporated,
#  ALL RIGHTS RESERVED.
#


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
from settings import *

class Window(Frame):

    v1 = 1795 #-199, -2211
    v5 = 2220 #-161, -1788
    va = 2589  #-127, -1411

    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.master = master
        self.init_window()
        self.e20 = E20()

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(LV1_PS_PIN, GPIO.IN) 
        GPIO.setup(LV2_PS_PIN, GPIO.IN) 


    # Create init_window
    def init_window(self):

        def donde():
            count = self.e20.motor.get_actual_encoder_count()
            msgBox=messagebox.askquestion('Current tick, good?', count)

            print('position=',count)

        # Keeps spinning forever at given rpm
        def spinendlessly():
            #self.e20.motor.spin_settings()
            acc = int(varAcc.get())
            vel = int(varV.get())
            self.e20.motor.set_acceleration(acc)
            self.e20.motor.stop()
            self.e20.motor.spin(vel)

        # Move disc at given angle
        def move_position():
            #self.e20.motor.position_settings()
            pos =float(varAng.get())
            self.e20.motor.position_abs(int(pos))
            

        # Spins disc until index position is found 
        def home():
            #self.e20.motor.position_settings()
            #self.e20.motor.set_acceleration(10)
            #self.e20.motor.spin(500, 3)
            #self.e20.motor.position_abs(0)
            self.e20.motor.home_positioning()

        def psd():
            
            if GPIO.input(LV1_PS_PIN) == LV1_PS_ON:
                print('laser is detected')
                count = self.e20.motor.get_actual_encoder_count()
                print('position=',count)
                i = False
            
            else:
                print('laser is NOT! detected')
                count = self.e20.motor.get_actual_encoder_count()
                print('position=',count)

        def psd2():
            
            if GPIO.input(LV2_PS_PIN) == LV2_PS_ON:
                print('laser is detected')
                count = self.e20.motor.get_actual_encoder_count()
                print('position=',count)
                i = False
            
            else:
                print('laser is NOT! detected')
                count = self.e20.motor.get_actual_encoder_count()
                print('position=',count)

        def photoSearch():
            self.e20.motor.home_positioning()
            self.e20.motor.set_acceleration(10)
            #self.e20.motor.set_max_abs_ramp_velocity(30)

            self.e20.motor.spin(10)

            i = True
            while i is True:
                self.e20.laser1.set_intensity(LASER_1, 400)
                self.e20.laser1.on()
                
                
                if GPIO.input(LV1_PS_PIN) == LV1_PS_ON:
                    print('laser is detected')
                    stop()
                    count = self.e20.motor.get_actual_encoder_count()
                    print('position=',count)
                    sleep(5)
                    self.e20.laser1.off()
                    i = False
                
                else:
                    print('laser is NOT! detected')
                    count = self.e20.motor.get_actual_encoder_count()
                    print('position=',count)


        def photosens():
             
            self.e20.motor.home_positioning()
            self.e20.fire_laser1(5)
            psd()
            print('pause')
            sleep(2)
            
            self.e20.motor.position_abs(1925)
            self.e20.fire_laser1(5)
            psd()
            print('pause2')
            sleep(2)
            
            self.e20.motor.home_positioning()
            self.e20.fire_laser1(5)
            psd()
            print('pause3')
            sleep(2)

            self.e20.motor.position_abs(2360)
            self.e20.fire_laser2(5)
            psd2()
            print('pause4')
            sleep(2)

            self.e20.motor.home_positioning()
            self.e20.fire_laser2(5)
            psd2()
            print('fin')

            

            

        def laser1():
            t = float(varTime.get())
            self.e20.laser1.on_time(t)
            
                
        # Fires laser 2
        def laser2():
            t = float(varTime.get())
            self.e20.laser2.on_time(t)
            
        def laserSweep():
            pos = float(varAng.get())
            self.e20.laser_sweep(start_tick=pos,
                                width=17,
                                delay=1.01,
                                steps=6)


        # LED 1
        # def led1():
        #    t = float(varTime.get())
        #    self.e20.led1(t)i

        def led1():
            t = float(varTime.get())
            #self.e20.led1(t)
            self.e20.camera.flash_on_O()
            sleep(t)
            self.e20.camera.flash_off_O()
           
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
           
 
       # Fans
        def fans():
            t = float(varTime.get())
            self.e20.fans(t) 
            
 
        # Captures image with given input settings    
        def capture():
            exposure = float(varExp.get())*1000000
            gain = float(varGain.get())
            gamma = float(varGamma.get())
            nombre = str(varNombre.get())
            date = str(datetime.now().strftime("%m-%d-%Y_%I:%M:%S_%p"))
            global filename
            filename = "/home/autolab/Desktop/images/%s-%s.tiff" % (nombre, date)
            print("filename=", filename)
            self.e20.camera.capture(exposure=exposure,
                                    gain=gain,
                                    gamma=gamma,
                                    flash=True,
                                    filename=filename)
                                    
            
                       
        def scanQRCode():
            process = subprocess.run('/opt/amdi/bin/QRreader',shell=True)
            
                   
        def assay():
            nombre = str(varNombre.get())
            date = str(datetime.now().strftime("%m-%d-%Y_%I:%M:%S_%p"))
            filename = "/data/test_gui/run_assay/%s-%s.tiff" % (nombre, date)
            self.e20.run_assay(filename)
        
        def test():
            self.e20.run_test

        def latch():
            self.e20.lock_lid()
            

        def unlatch():
            self.e20.unlock_lid()
            

        def stop():
            self.e20.motor.brake()

        def dondeDos():
            self.e20.motor.position_settings()
            self.e20.motor.position_abs(self.v1)
            sleep(5)
            count = self.e20.motor.get_actual_encoder_count_modulo()
            msgBox=messagebox.askquestion('Is this pos. optimal?', count)
            if (msgBox == 'no'):
                messagebox.showinfo('Adjust to position')
                self.e20.motor.brake()
                sleep(5)
                count = self.e20.motor.get_actual_encoder_count_modulo()
                messagebox.showinfo('V1 cal:', count)
                self.v1 = count    
                print('position=', self.v1)
                print('count=', count)
            else:
                return 0    
                 

        def dondeTres():
            self.e20.motor.position_settings()
            self.e20.motor.position_abs(self.v5)
            sleep(5)
            count = self.e20.motor.get_actual_encoder_count_modulo()
            msgBox=messagebox.askquestion('Is this pos. optimal?', count)
            if (msgBox == 'no'):
                messagebox.showinfo('Adjust to position')
                self.e20.motor.brake()
                sleep(3)
                count = self.e20.motor.get_actual_encoder_count_modulo()
                messagebox.showinfo('V5 cal:', count)
                self.v5 = count    
                print('position=', self.v5)
                print('count=', count)
            else:
                return 0 

        def dondeQua():
            #array = -127
            self.e20.motor.position_settings()
            self.e20.motor.position_abs(self.va)
            #messagebox.showinfo('Array=',self.va)
            sleep(2)
            count = self.e20.motor.get_actual_encoder_count_modulo()
            msgBox=messagebox.askquestion('Is this pos. optimal?', count)
            if (msgBox == 'no'):
                messagebox.showinfo('Adjust to position')
                self.e20.motor.brake()
                sleep(3)
                count = self.e20.motor.get_actual_encoder_count_modulo()
                messagebox.showinfo('Array cal:', count)
                self.va = count    
                print('position=', self.va)
                print('count=', count)
            else:
                return 0 
        
        def store():
            print('v1=', self.v1)
            print('v5=', self.v5)
            print('Array=', self.va)

            # motorSer = str(ser)
            # x = motorSer.split(",")
            # y = str(x[0]).split("=", 1)
            # motorID = y[1]
            # print('ser1=', ser)
            # print('x0=', x[0])
            # print("y=", y[1])

            # camSer = str(serB)
            # x2 = camSer.split(",")
            # y2 = str(x2[0]).split("=", 1)
            # camID = y2[1]
            # # print('canInfo=', camSer)
            

            date = str(datetime.now().strftime("%m-%d-%Y_%I:%M:%S_%p"))
            jd = {}
           # jd["Motor Serial Number"] = motorID
           # jd["Camera Serial Number"] = camID
           # jd["Device Number"] = device_num
            jd["Date Time"] = date
            jd["valve1"] = self.v1
            jd["Valve5"] = self.v5
            jd["Array"] = self.va 

            json_file = open("/data/tmp/tmp.json", "w")
            json.dump(jd, json_file)
            json_file.close()
            with open("/data/tmp/tmp.json") as json_file:
                json_object = json.load(json_file)
            json_file.close()
            print(json_object)
        
        def saveReport():
            
            date = str(datetime.now().strftime("%m-%d-%Y_%I:%M:%S_%p"))
            #device_num = cpuserial
            jd = {}
            jd["Device Number"] = device_num
            jd["Date Time"] = date
            jd["Operator Name"] = str(varOperator.get())
            jd["Test Code Revision"] = testRev
            for i in range(0, test_Idx['Max']):
                jd[testNames[i]] = results[i]
            #jo = json.dumps(jd)
            #print(jd)
                
            json_file = open("tmp.json", "w")
            json.dump(jd, json_file)
            json_file.close()
            with open("tmp.json") as json_file:
                json_object = json.load(json_file)
            json_file.close()
            print(json_object)
            build_direction = "LEFT_TO_RIGHT"
            table_attributes = {"style" : "width:100%", "text-align" : "center", "rules" : "all", "border" : "1"}
            json_html = convert(json_object, build_direction=build_direction, table_attributes=table_attributes)
            subject = 'SN_' + device_num + '_' + date
            fname =  subject + ".html"
            f = open(fname, 'wb')
            f.write("<!DOCTYPE html>".encode())
            f.write('\n'.encode())
            f.write("<html>".encode())
            f.write('\n'.encode())
            f.write("<style>".encode())
            f.write("\n".encode())
            f.write("h1 {text-align: center;}".encode())
            f.write("\n".encode())
            f.write("</style>".encode())
            f.write("\n".encode())
            f.write("<head>".encode())
            #f.write("\n".encode())
            f.write("<title>AMDI A-20 ASSEMBLY TEST REPORT</title>".encode())
            #f.write("\n".encode())
            f.write("</head>".encode())
            f.write("\n".encode())
            f.write("<body><p>".encode())
            f.write("<h1>AMDI A-20 ASSEMBLY TEST REPORT</h1>".encode())
            f.write("\n".encode())
            f.write(json_html.encode())
            f.write("</p></body>".encode())
            f.write('\n'.encode())
            f.write("</html>".encode())

            f.close()
            
            msg = MIMEMultipart()
            #msg['Subject'] = f'AMDI test result'
            msg['Subject'] = subject
            msg['From'] = str(varEmail.get())
            msg['To'] = str(varEmail.get())

            part = MIMEBase('application', "octet-stream")
            part.set_payload(open(fname,"rb").read())
            encoders.encode_base64(part)
            s_header_attachment = 'attachment; filename={}'.format(fname)
            #part.add_header('Content-Disposition', 'attachment; filename=fname')
            part.add_header('Content-Disposition', s_header_attachment)
            msg.attach(part)
            
            #attach image
            part = MIMEBase('application', "octet-stream")
            part.set_payload(open(filename,"rb").read())
            encoders.encode_base64(part)
            s_header_attachment = 'attachment; filename={}'.format(filename)
            #part.add_header('Content-Disposition', 'attachment; filename=fname')
            part.add_header('Content-Disposition', s_header_attachment)
            msg.attach(part)

            s = smtplib.SMTP('smtp.office365.com', 587)
            s.ehlo()
            s.starttls()
            s.ehlo()
            if (str(varPasswd.get()) == ""):
                passwd = "!!1234REWQ08Lu"
            else:
                passwd = str(varPasswd.get())
            s.login(str(varEmail.get()), passwd)
            s.send_message(msg)
            s.close()
                            
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

        self.varemail = Label(root, text = 'Email', fg = 'Black', font = Labels )
        self.varemail.place(x = 10, y = 100)
        varEmail = Entry(root)
        varEmail.insert(END,'lab.user@amdilabs.com')
        varEmail.place(x = 120, y = 100)
        
        self.varvel = Label(root, text = 'Password', fg = 'Black', font = Labels )
        self.varvel.place(x = 10, y = 125)
        varPasswd = Entry(root)
        varPasswd.insert(END,'')
        varPasswd.place(x = 120, y = 125)  
        
        ### Camera control inputs
        
        # Exposure time
        self.varexp = Label(root, text = 'Exp time [s]', fg = 'Black', font = Labels )
        self.varexp.place(x = 10, y = 150)
        varExp = Entry(root)
        varExp.insert(END,'1')
        varExp.place(x = 120, y = 150)
        
        # Operator Name
        self.varOperator = Label(root, text = 'Operator', fg = 'Black', font = Labels )
        self.varOperator.place(x = 10, y = 75)
        varOperator = Entry(root)
        varOperator.insert(END,'')
        varOperator.place(x = 120, y = 75)
        
        
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
        self.varvel.place(x = 300, y = 75)
        varV = Entry(root)
        varV.insert(END,'3000')
        varV.place(x = 410, y = 75)
        
        # Acceleration
        self.varacc = Label(root, text = 'Acceleration', fg = 'Black', font = Labels )
        self.varacc.place(x = 300, y = 125)
        varAcc = Entry(root)
        varAcc.insert(END,'1000')
        varAcc.place(x=410, y = 125)
        
        # Time for laser motor
        self.vartime = Label(root, text = 'Time [s]', fg = 'Black', font = Labels )
        self.vartime.place(x = 300, y = 175)
        varTime = Entry(root)
        varTime.insert(END,'2')
        varTime.place(x=410, y = 175)
        
        # Angle for disc
        self.varangle = Label(root, text = 'Angle [deg]', fg = 'Black', font = Labels )
        self.varangle.place(x = 300, y = 225)
        varAng = Entry(root)
        varAng.insert(END,'1795')
        varAng.place(x=410, y = 225)
        
        ### Buttons

        # Spin motor at set rpm
        spinButton = Button(self, text = "Spin Motor", font = Buttons, command = spinendlessly)
        spinButton.place(x=600, y=25)
        
        # Break motor
        stopButton = Button(self, text = 'Brake Motor', font = Buttons, fg='Red', command = stop)
        stopButton.place(x=600, y=75)
        
        # Turn laser 1 on
        laser1Button = Button(self, text = "Laser 1", font = Buttons, command = laser1)
        laser1Button.place(x=600, y=125)
        
        # Turn laser 2 on
        laser2Button = Button(self, text = "Laser 2", font = Buttons, command = laser2)
        laser2Button.place(x=600, y=175)
        
        # Turn led1 on
        led1Button = Button(self, text = "LED O", font = Buttons, command = led1)
        led1Button.place(x=600, y=225)
        
        # Turn led2 on
        led2Button = Button(self, text = "LED W", font = Buttons, command = led2)
        led2Button.place(x=700, y=225)
        
        # Move disc a set angle
        encoderButton = Button(self, text = "Move Angle", font = Buttons, command = move_position)
        encoderButton.place(x=325, y=325)
        
        # Find index
        findButton = Button(self, text = 'Find Index', font = Buttons, command = home, activebackground="orange")
        findButton.place(x=325, y= 275)

        BOT_ROW = 325
        
        # Encoder position for calibration
        positionButton = Button(self, text = "Position", font = Buttons, command = donde)
        positionButton.place(x = 10, y = BOT_ROW)

        # View results
        resultsButton = Button(self, text = "Laser sweep", font = Buttons, command = laserSweep)
        resultsButton.place(x=150,y=BOT_ROW)
        
         # Capture camera image
        captureButton = Button(self, text = "Capture", font = Buttons, command = capture)
        captureButton.place(x = 10, y = 275)

        # Run assay
        assayButton = Button(self, text = "Run Assay", font = Buttons, fg='Green', command = assay)
        assayButton.place(x = 150, y= 275)

        BOT_ROW_2 = 375

        # QR Scanner
        QRButton = Button(self, text = "QR Scan", font = Buttons, command = scanQRCode)
        QRButton.place(x = 600, y= 275)

        BOT_ROW_2 = 375
        
        # Fans
        FansButton = Button(self, text = "Fans", font = Buttons, command = fans)
        FansButton.place(x = 480, y= 275)

        BOT_ROW_2 = 375
        
        # Lock lid
        lockButton = Button(self, text = 'Lock Lid', font = Buttons, command = latch)
        lockButton.place(x=480, y=325)

        # Unlock lid
        unlockButton = Button(self, text = 'Unlock Lid', font = Buttons, command = unlatch)
        unlockButton.place(x=480, y=375)

        dondeDosButton =  Button(self, text = 'Store v1', font = Buttons, command = dondeDos)
        dondeDosButton.place(x=10, y=BOT_ROW_2)

        dondeTresButton =  Button(self, text = 'Store v5', font = Buttons, command = dondeTres)
        dondeTresButton.place(x=150, y=BOT_ROW_2)

        dondeQuaButton =  Button(self, text = 'Store array', font = Buttons, command = dondeQua)
        dondeQuaButton.place(x=325, y=BOT_ROW_2)
        
        # Generate Report
        reportButton = Button(self, text = "photoSense", font = Buttons, fg='Blue', command = photosens)
        reportButton.place(x = 600, y= 325)

        reportButton = Button(self, text = "photoSearch", font = Buttons, fg='Blue', command = photoSearch)
        reportButton.place(x = 600, y= 355)

        BOT_ROW_2 = 375

global root
global testRev
global results
global testNames
global device_num
global videoOn

testRev= '3'
videoOn = False

try:
    serialFile = open('/sys/devices/virtual/dmi/id/product_serial')
    serialNumber = serialFile.readline()[-9:-1]
    serialFile.close()
except:
   serialNumber = '00000000'

device_num = serialNumber

Result_T = {
    'NO'  :  'No Result',
    'PASS':  'Pass',
    'FAIL':  'FAIL'
}

test_Idx = {
    'Laser 1': 0,
    'Laser 2': 1,
    'Capture'   : 2,
    'Move Angle': 3,
    'Find Index': 4,
    'LED Orange': 5,
    'LED White' : 6,
    'Fans'      : 7,
    'Lock Lid'  : 8,
    'Unlock Lid': 9,
    'Scan QR'   : 10,
    'Max'       : 11
}

test_names = ('Laser 1', 'Laser 2', 'Capture','Move Angle', 'Find Index', 'LED Orange',
              'LED White', 'Fans', 'Lock Lid', 'Unlock Lid', 'Scan QR Code')

r = (Result_T['NO'],Result_T['NO'],Result_T['NO'],Result_T['NO'],Result_T['NO'],Result_T['NO'], Result_T['NO'],
     Result_T['NO'],Result_T['NO'],Result_T['NO'],Result_T['NO'],)
results = list(r)
testNames = list(test_names)
root = Tk()

#size of the window
root.geometry("800x480")

app = Window(root)
root.mainloop()
