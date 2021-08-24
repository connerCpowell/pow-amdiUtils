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

import cv2
import RPi.GPIO as GPIO
from time import sleep, time
from typing import Optional
from pymba import Frame, Vimba, VimbaException
import logging
from settings import LOGGING_FILE
from settings import CAMERA_SWITCH, CAMERA_ON, CAMERA_OFF
from settings import ORANGE_LED, ORANGE_LED_ON, ORANGE_LED_OFF
from settings import WHITE_LED, WHITE_LED_ON, WHITE_LED_OFF
from settings import LED_ON, LED_OFF
from settings import CAMARA_BOOT_TIMEOUT

EXPERIMENTAL = 1

logging.basicConfig(filename=LOGGING_FILE,
                    level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(module)s - %(funcName)s: %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

class Camera:
    def __init__(self):
        self.flash_pin = ORANGE_LED
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(CAMERA_SWITCH, GPIO.OUT)
        GPIO.setup(ORANGE_LED, GPIO.OUT)
        GPIO.setup(WHITE_LED, GPIO.OUT)
        self.usb_throughput = 42750000

        # The following values are based off running:
        # python3 /opt/amdi/hardware_control/Camera.py
        # which will let you see what the camera firmware considers
        # sane values for the lighting used.
        # The calibration and checking images are done with white leds

        EXPERIMENTAL = 1  # I have better values than our old ones but
                          # it might trouble Hector's analysis. If we
                          # change it, now would be the time to do so.
        self.led = [ {
                     'colour'  : ORANGE_LED,
                     'exposure': (481186 if EXPERIMENTAL else 1000000),
                     'gain'    : (16     if EXPERIMENTAL else 1),
                     'gamma'   : (1) },
                     {
                     'colour'  : WHITE_LED,
                     'exposure': 82000,
                     'gain'    : 1,
                     'gamma'   : 1 } ]

        logging.info("Camera GPIO setup.")

    def power_on_camera(self):
        GPIO.output(CAMERA_SWITCH, CAMERA_ON)
        logging.info("Camera enabled.")

    def close(self):
        GPIO.output(CAMERA_SWITCH, CAMERA_OFF)
        logging.info("Camera disabled.")

    def capture(self,
                flash=None,
                led_group=ORANGE_LED,
                filename=None,
                leave='off'):
        """
        Capture an image with the camera, and return the frame.
        :param exposure:  in usec
        :param gain:      typical range 1-16
        :param gamma:     typically 1-6 usually left as 1
        :param flash:
        :param led_group: use orange or white LED for illumination
                          ORANGE_LED/WHITE_LED
        :param filename:  ..obvious...
        :param leave:     leave camera on or off after taking picture - default off
        :return:          0 on success -1 on failure
        """
        # Somewhat convoluted
        with Vimba() as vimba:
            retry = 1
            max_retry = 3
            while True:
                try:
                    self.flash_pin = led_group  # Select white or orange LEDs for set_flash(on/off)()
                    GPIO.output(CAMERA_SWITCH, CAMERA_ON)
                    if retry != 1:
                        # I've determined that when we get "-13 Other error"
                        # there is no recovery beyond turning the camera
                        # off and on again,.. sucks.
                        vimba.shutdown()
                        msg = 'Life is hard, I\'ve had to turn the camera physically off and back on again'
                        logging.info(msg)
                        print(msg)
                        GPIO.output(CAMERA_SWITCH, CAMERA_OFF)
                        sleep(0.1)  # let the camera capacitors drain so it's 'off'
                        GPIO.output(CAMERA_SWITCH, CAMERA_ON)
                    vimba.startup()  # We must ALWAYS start off with this
                except VimbaException as e:
                    print('vimba.startup() failed: %d %s' % (e.error_code, e.message))
                    del vimba
                    return -1  # for now, give up as we don't control the library that failed.

                # Naff way of doing it but wait for the camera to be ready (usually ~3 sec)
                cams = vimba.camera_ids()  # what (if any) cameras are available on the system
                start_time = time()
                while not cams:
                    print('Waiting for the camera to be ready [%01d]' % (int(time() - start_time)))
                    sleep(1)
                    cams = vimba.camera_ids()

                    waiting = int(time() - start_time)
                    if waiting > CAMARA_BOOT_TIMEOUT:
                        print('taking a long time,.. problem with camera? [%01d]' % waiting )
                        if waiting > (CAMARA_BOOT_TIMEOUT*2):
                            print('Thats it, I\'m getting me mallet')
                            return -1  # FAILED
                        else:
                            continue

                camera = vimba.camera(0)  # we only have one camera so it must be at 0

                # ------------------------------------------------------

                try:
                    camera.open(adjust_packet_size=False)  # Possibly use adjust_packet_size=False
                except VimbaException as e:
                    msg = 'Error caught opening camera: %d: %s' \
                           % (e.error_code,
                              e.message)
                    logging.info(msg)
                    print(msg)
                    retry += 1
                    if retry > max_retry:
                        msg = 'Was unable to recover the camera for unknown reasons'
                        logging.info(msg)
                        print(msg)
                        return -1  # game over man, game over
                    sleep(1)       # Is it just a matter of waiting?
                    continue       # the while loop
                break              # out of while loop

            # ----------------------------------------------------------
            tries = 3  # Often we get xml reading errors here so when we try
                       # and set the values, if it flunks, try again a few times
            for tried in range(tries):
                try:
                    camera.feature('DeviceLinkThroughputLimit').value = self.usb_throughput
                    # The definitions below are defined in Camera._init_()
                    for idx in self.led:
                        if idx.get('colour') == led_group:
                            camera.feature('ExposureTime').value = idx.get('exposure')
                            camera.feature('Gain').value         = idx.get('gain')
                            camera.feature('Gamma').value        = idx.get('gamma')
                            break

                    temp = camera.feature('DeviceTemperature').value
                    break
                except VimbaException as e:
                    if tried < tries - 1:
                        if e.error_code == VimbaException.ERR_TIMEOUT:
                            sleep(1)
                            continue
                    else:
                        break
                    break
                except KeyError as e:
                    if tried < tries - 1:
                        sleep(1)
                        print('Unable to set some camera features. retrying (%d of %d)' % (tried, tries))
                        continue
                    else:
                        break
                    break
            else:  # no break happened
                pass

            # ----------------------------------------------------------
            if 'temp' in locals():
                msg = 'Camera %s ready. %02.1f°C' % (cams, temp)
                logging.info(msg)
                print(msg)
            else:
                msg = 'Camera connection is unstable? cable issue maybe? or just bad timeouts?'
                logging.info(msg)
                print(msg)
                return -1  # Lord Vader - 'You have failed me for the last time'

            tries = 3
            for tried in range(tries):
                try:
                    camera.arm('SingleFrame')
                    self.flash_on()
                    white_led_on = GPIO.input(WHITE_LED)
                    if white_led_on and (led_group != WHITE_LED) :  # prevent accidental overexposure
                        GPIO.output(WHITE_LED, LED_OFF)             # when using orange led
                        white_led_on = 2
                    frame = camera.acquire_frame()
                    logging.info("Captured image.")
                    if white_led_on == 2:                           # if it was on before
                        GPIO.output(WHITE_LED, LED_ON)              # turn it back on
                    self.flash_off()
                    image = frame.buffer_data_numpy()
                    if filename:
                        cv2.imwrite(filename, image)
                    break
                except VimbaException as e:
                    if tried < tries - 1:
                        self.flash_off()
                        if e.error_code == VimbaException.ERR_TIMEOUT:
                            logging.info(e)
                            camera.disarm()
                            continue
                        else:
                            break
                        break

            tries = 3
            for tried in range(tries):
                try:
                    camera.disarm()
                    camera.close()  # Vimba .close()
                    vimba.shutdown()
                    break
                except VimbaException as e:
                    if tried < tries - 1:
                        if e.error_code == VimbaException.ERR_TIMEOUT:
                            sleep(1)
                            continue
                    else:
                        break
                    break
                except KeyError as e:
                    if tried < tries - 1:
                        sleep(1)
                        print('Unable to set some camera features. retrying (%d of %d)' % (tried, tries))
                        continue
                    else:
                        break
                    break

            # Do we turn the camera off or leave it on?
            if 'off' in leave:
                GPIO.output(CAMERA_SWITCH, CAMERA_OFF)
            del camera  # clean up after ourselves
            del vimba   # clean up after ourselves
            return 0

    def flash_on(self):
        if self.flash_pin is None:
            return -1
        GPIO.output(self.flash_pin, LED_ON)
        logging.info("LED on.")
        self.flash = True
        return self.flash

    def flash_off(self):
        if self.flash_pin is None:
            return -1
        GPIO.output(self.flash_pin, LED_OFF)
        logging.info("LED off.")
        self.flash = False
        return self.flash


# Lets do some standalone tests if we can. We're only taking pictures
# so moving the casstte around isn't really part of the deal here
# I sort of expect folk to hand turn the cassette if necessary

if __name__ == '__main__':

    import cv2
    from pymba import vimba
    from pymba import Frame
    import os
    import sys
    from time import time, sleep
    import RPi.GPIO as GPIO
    import curses
    from pathlib import Path
    from settings import CAMERA_SWITCH, CAMERA_ON, CAMERA_OFF
    from settings import ORANGE_LED, ORANGE_LED_ON, ORANGE_LED_OFF
    from settings import WHITE_LED, WHITE_LED_ON, WHITE_LED_OFF

    TRY_AUTO = 1  # try autogain/exposure
    if os.environ.get('DISPLAY'):
        print("X11 is probably available")
        FB       = 0 # display to Frame buffer of device
        X11      = 1 # throw remote image to an X11 display
    else:
        FB       = 1 # display to Frame buffer of device
        X11      = 0 # throw remote image to an X11 display

    PIXEL_FORMATS_CONVERSIONS = {
        'BayerRG8': cv2.COLOR_BAYER_RG2GRAY,
    }

    maxpic = 2000
    dest = '/data/img'

    # normally camera = Camera() would set this up...
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(CAMERA_SWITCH, GPIO.OUT)
    GPIO.setup(ORANGE_LED, GPIO.OUT)
    GPIO.setup(WHITE_LED, GPIO.OUT)
    GPIO.output(CAMERA_SWITCH, CAMERA_OFF)  # important
    sleep(0.25)
    GPIO.output(CAMERA_SWITCH, CAMERA_ON)
    sleep(2)
    GPIO.output(ORANGE_LED, LED_OFF)
    GPIO.output(WHITE_LED, LED_ON)
    # wait until camera is available

    def get_sample_image(
                         flash,
                         filename,
                         led_group=ORANGE_LED,
                         leave='off'):
        '''
        Wrapper around Camera.capture() to retry should a failure occur
        '''
        # Try and take the picture a few times before giving u
        tries = 3
        for tried in range(tries):
            res = camera.capture(flash=flash,
                                      led_group=led_group,
                                      filename=filename,
                                      leave=leave)
            if res == 0:
                break
            else:
                tried += 1
                msg = '%s(): Erk, didn\'t get a picture, retrying [%d/%d]' \
                      % (sys._getframe().f_code.co_name, tried+1, tries)
                logging.info(msg)
                print(msg)

        if tried >= 5:
            msg = '%s(): Utterly failed to get a picture. As a last resort, attempting suicide' \
                  % sys._getframe().f_code.co_name
            logging.info(msg)
            print(msg)
            sys.exit(-4)  # -4 camera failure
        else:
            return 0


    class AVTCamera:  # Allied Vision Technologies Camera

        vimba=None
        camFactory=None

        def __init__(self,capture_device=0):
            # vimba object
            if AVTCamera.vimba == None:
                AVTCamera.vimba = vimba.Vimba()
                # Start vimba system
                AVTCamera.vimba.startup()
                AVTCamera.camFactory = vimba.camera_ids()
            self.cam_open=False

            cams = [AVTCamera.vimba.camera(id) for id in AVTCamera.camFactory]
            if len(cams) == 0 or len(cams) < capture_device:
                print("No AVT camera found")
                self.cam=None
            else:
                self.cam=cams[capture_device]

        def read(self):
            self.open()
            self.singleframe = 0
            self.cam.start_frame_acquisition()
            while self.singleframe == 0 :
                sleep(0.05)
            self.cam.stop_frame_acquisition()
            return self.image

        def convert_frame(self,  frame: Frame )->None:
            # print('frame {}'.format(frame.data.frameID))
            self.singleframe += 1
            self.image = frame.buffer_data_numpy()
            # print("%s" % frame.pixel_format)
            try:
                self.image = cv2.cvtColor(self.image, PIXEL_FORMATS_CONVERSIONS[frame.pixel_format])
            except KeyError:
                pass

        def release(self):
            self.cam.disarm()
            self.cam.close()
            self.cam_open = False

        # --------------------------------------------------------------
        def preview(self):
            def exposure(value):
                # Hurm can we move in certain step sizes only?
                value = int(value / 102400) * 102400
                setTrackbarPos('Exposure', Frame, value)
                #self.cam.feature('ExposureTime').value = value  # useless with mouse

            def gamma(value):
                if value >= 0 or value <= 16:  # max swing
                    self.cam.feature('Gamma').value = value

            def gain(value):
                if value >= 0 or value <= 24:  # max swing
                    self.cam.feature('Gain').value = value

            if X11:
                cv2.namedWindow("Frame")
                cv2.createButton("GAIN+", gain, None, cv2.QT_PUSH_BUTTON,1)

            while True:
                img=self.read()  #  Camera starts in 2592 x 1944 (Mono8?)

                small = cv2.resize(img, (320, 240))  # for X11
                print('Exposure %02.2f\tGain %02.2f\tCamTemp %02.1f°C' % (self.cam.feature('ExposureTime').value, self.cam.feature('Gain').value, self.cam.feature('DeviceTemperature').value))
                # Something for the framebuffer? the RPi display is *24bit RGB*
                # convert Mono8 bit grey to 24bit rgb ? or is image already 24bit?

                # dd if=/dev/zero of=/dev/fb0  # clear the frame buffer
                # for i in  `seq $((800*480))`; do dd if=/dev/urandom of=/dev/fb0 bs=1 count=1 seek=$i 2>&1 ; echo $i; done
                # ugh,.. fine,.. it uses 3200 bytes for a single line so it's actually 32bpp (4 bytes per pixel not 24bpp
                if FB:
                    blur = img # cv2.medianBlur(img, 13)
                    frame32bpp =  cv2.cvtColor(blur, cv2.COLOR_GRAY2BGRA)
                    fbframe = cv2.resize(frame32bpp, (800, 480), fx=1, fy=1)
                    with open('/dev/fb0', 'rb+') as buf:
                        buf.write(fbframe)

                if X11:
                    cv2.imshow('Frame', small)
                    c=cv2.waitKey(10)
                    if c==ord ('q') :
                        cv2.destroyWindow('Frame')
                        return img;

        # --------------------------------------------------------------
        def open(self):
            if self.cam == None:
                self.image=None
                return self.image
            if self.cam_open == False :
                self.cam.open()
                self.cam_open = True
                if TRY_AUTO:
                    # ~ GPIO.output(ORANGE_LED, LED_ON)
                    # ~ GPIO.output(WHITE_LED, LED_OFF)
                    GPIO.output(ORANGE_LED, LED_OFF)
                    GPIO.output(WHITE_LED, LED_ON)
                    self.cam.feature('ExposureMode').value = 'Timed'
                    self.cam.feature('ExposureAuto').value = 'Continuous'
                    self.cam.feature('GainAuto').value = 'Continuous'
                else:
                    GPIO.output(ORANGE_LED, LED_ON)
                    GPIO.output(WHITE_LED, LED_OFF)
                    #self.cam.feature('ExposureMode').value = 'Timed'  #?
                    #self.cam.feature('AcquisitionFrameRateEnable').value = False
                    #self.cam.feature('AcquisitionMode').value = 'Basic'
                    self.cam.feature('ExposureTime').value = 847000
                    self.cam.feature('Gamma').value = 1
                    self.cam.feature('Gain').value = 1
                self.cam.arm('Continuous', self.convert_frame)

    def cam_test_preview():
        camera=AVTCamera(0)
        camera.preview()
        camera.release()
        GPIO.output(CAMERA_SWITCH, CAMERA_OFF)
        GPIO.output(ORANGE_LED, LED_OFF)
        GPIO.output(WHITE_LED, LED_OFF)

    # ------------------------------------------------------------------

    # get the curses screen window setup
    screen = curses.initscr()

    # turn off input echoing
    curses.noecho()

    # respond to keys immediately (don't wait for enter)
    curses.cbreak()

    # map arrow keys to special values
    screen.keypad(True)

    # Turn off the cursor!
    curses.curs_set(0)

    # print doesn't work with curses, use addstr instead
    choice=['0. Video with auto adjust (with onscreen values) DEFAULT',
            '1. Continuous snapshot (last %d stored in %s)' % (maxpic, dest) ]
    choice_line_pos = 4
    selected = 0  # default

    screen.addstr(0, 0, '============')
    screen.addstr(1, 0, 'Camera tests')
    screen.addstr(2, 0, '------------')

    screen.addstr(choice_line_pos    , 0, choice[0], curses.A_REVERSE)
    screen.addstr(choice_line_pos + 1, 0, choice[1])

    screen.addstr(choice_line_pos + len(choice) + 2, 0, 'Select by using up/down and then pressing <space or return>')
    screen.addstr(choice_line_pos + len(choice) + 3, 0, 'ESC or \'q\' will exit the tests')

    screen.refresh()  # repaint the screen

    try:
        while True:
            curses.flushinp()      # Flush all input buffers. This throws away
                                   # any typeahead that has been typed by the
                                   # user and has not yet been processed by the program.
            char = screen.getch()  # only proceed if a character was input

            if char == ord('q') or char == 0x1b:   # 'q' or ESC to quit
                selected = -1
                break

            elif char == curses.KEY_UP:
                screen.addstr(choice_line_pos    , 0, choice[0], curses.A_REVERSE)
                screen.addstr(choice_line_pos + 1, 0, choice[1])
                selected = 0

            elif char == curses.KEY_DOWN:
                screen.addstr(choice_line_pos    , 0, choice[0])
                screen.addstr(choice_line_pos + 1, 0, choice[1], curses.A_REVERSE)
                selected = 1

            elif char == ord(' ') or char == curses.KEY_ENTER or char == 0x0a:
                break

    finally:
        curses.curs_set(1)
        curses.nocbreak()
        screen.keypad(0)
        curses.echo()
        curses.endwin()


    if selected == 0:
        try:
            cam_test_preview()
        except:
            camera.release()
            GPIO.output(CAMERA_SWITCH, CAMERA_OFF)
            GPIO.output(ORANGE_LED, LED_OFF)
            GPIO.output(WHITE_LED, LED_OFF)

    if selected == 1:
        GPIO.output(WHITE_LED, LED_OFF)
        # just start taking pictures fast as we can
        # we need around 1000 pictures and /tmp barely has enough space

        Path(dest).mkdir(parents=True, exist_ok=True)

        exposure = float(481186)   # For the Orange LEDs use this
        gain     = float(16)       # Lets use the gain to brighten
        gamma    = float(1)        # leave alone for now

        camera = Camera()

        print('Estimated time to run (%d hr and %d min)'
              % (int(maxpic * (5/60)) / 60,
                 int(maxpic * (5/60)) % 60 ))


        for loop in range(1, maxpic):
            print('-----------------------------------------')
            print('Taking picture [%d/%d]' % (loop, maxpic))
            get_sample_image(
                           flash=True,
                           led_group=ORANGE_LED,
                           filename=dest + '/test-img-%03d' % loop + '.tif',
                           leave='on')

        del camera
        GPIO.output(CAMERA_SWITCH, CAMERA_OFF)
        GPIO.output(ORANGE_LED, LED_OFF)
        GPIO.output(WHITE_LED, LED_OFF)

        print('')
        print('Image files will be found under [%s]' % (dest))
        print('')
        print(
'''
I suggest that you create smaller images for easier flipping through
or a short video eg
# ----- %< -----------------------------------------------
TOP=''' + dest + '''
mkdir -p $TOP
pushd $TOP

for i in `seq 1 \`ls -1 test-img*.tif | wc -l\``; do
    sig=3
    echo "Working on run "`printf "%0${sig}d" $((i))`
    convert test-img-`printf "%0${sig}d.tif" $((i))` -resize 320 \
        -font NimbusMonoPS-Regular -gravity SouthWest -pointsize 24 \
        -stroke red label:'Test #'`printf "%0${sig}d" $((i))` -append \
        test-img-`printf "%0${sig}d.png" $((i))`
done

convert test-img-???.png -delay 0 /data/animated.gif   # @25 frames/sec
sync
ffmpeg -y -r 2 -i /data/animated.gif /data/slower.mp4  # @2 fps
popd
# ----- %< -----------------------------------------------
'''
             )
        sys.exit()
