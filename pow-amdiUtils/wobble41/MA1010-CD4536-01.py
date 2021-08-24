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
SHORT = 0               # testing, limited to all images, one short BPSSS spin and
                        # the LV1#0/1 cut. This is for debug purposes only.
                        # *NEVER* use this in production.
                        # I also set the laser intensities to be lowest values
                        # just after the settings imports are pulled in, if enabled

EXPERIMENTAL = 0        # When using experimental, make SURE that the lid is in
                        # place or light pollution will 'whiteout' the picture
                        # since we don't use adaptive exposure.

# situation where we start the box (first time) and there is no config
# file what do we do? do we cry or do we consider the 'perfect' setup?

FOLLOW_CAM_OFFSET = 0   # latch to the camera offset?
                        # Thinking this is a bad idea actually. For this to work
                        # right, I need the LV1/2 postions to be something we can
                        # detect automatically as they're independent from the camera.
                        # ie it's not safe to assume they are perfectly offset
                        # from the camera position

if EXPERIMENTAL:
    import cv2          # This import has to be first due to TLS issues
                        # I refer folk to:
                        # https://bugzilla.redhat.com/show_bug.cgi?id=1722181#c6
    import numpy as np  # importing as np is sort of standard for most folk
    import math         # mainly for atan2 and pi

from time import sleep
import os
import logging
import inspect
from settings import LOGGING_FILE, SI_COUNT, CAL_DIR
from settings import LASER1_INTENSITY, LASER2_INTENSITY
from settings import LASER_1, LASER_1_OUT_PIN
from settings import LASER_2, LASER_2_OUT_PIN
from settings import INTENSITY_DRY, INTENSITY_WET
from settings import NONSTOP, GENTLE, HARSH, CALCULATED, SETTLE  # 4 spin stopping types
from settings import MOVE_COMPLETED, BUMP
from settings import HOME_ACCEL, DEBUGGING
from settings import ORANGE_LED, WHITE_LED

DEBUGGING = 1  # For debugging THIS file in standalone development
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

if SHORT:
    LASER1_INTENSITY = 0
    LASER2_INTENSITY = 0
    INTENSITY_DRY    = 0
    INTENSITY_WET    = 0

def lineno():
    """Returns the current line number in our program."""
    return inspect.currentframe().f_back.f_lineno

logging.basicConfig(
    filename=LOGGING_FILE,
    format='%(asctime)s %(levelname)-8s {%(module)s} [%(funcName)s] - %(message)s',
    level=logging.INFO,
    datefmt='%Y-%m-%d %H:%M:%S')

def get_sample_image(self,
                     flash,
                     filename,
                     led_group=ORANGE_LED,
                     leave='off'):
    '''
    Wrapper around Camera.capture() to retry should a failure occur
    '''
    # Try and take the picture a few times before giving up
    tries = 3  # give up after N tries
    for tried in range(1, tries):
        res = self.camera.capture(flash=flash,
                                  led_group=led_group,
                                  filename=filename,
                                  leave=leave)
        if res == 0:
            break
        else:
            msg = 'Erk, didn\'t get a picture, retrying [%d/%d]' % (tried, tries)
            logging.info(msg)
            print(msg)

    if tried >= tries:
        msg = 'Utterly failed to get a picture. As a last resort, attempting seppiku.'
        logging.info(msg)
        print(msg)
        sys.exit(-4)  # -4 camera failure
    else:
        return 0

def run_assay(self, filename):
    '''
    Perform assey on sample
    '''
    # What disk type are we? MA1003 MA1010 etc?
    assay_id = __file__.split('/')[len(__file__.split('/'))-1]
    cartridge = assay_id.split('-')[0]
    # model = assay_id.split('-')[1]  # normally use this
    model = filename.split('-')[1]  # but with default recipe, we'll use this
                                    # so that what we ASKED for defines the disk colour
                                    # BD = black CD = clear

    # being a bit fancy so you could have a load of optional colours/messages
    colour_code = model[:2]
    colour_dict = {'UK': 'unknown'}, \
                  {'CD': 'clear'}, \
                  {'BD': 'black'}

    for code in colour_dict:
        if code.get(colour_code) is not None:
            os.environ['_DISK_COLOUR'] = code.get(colour_code)  # pass to environment variable as a global
            break
    msg = 'Using %s model %s for the assay test (%s cartridge)' % (cartridge, model, code.get(colour_code))
    logging.info(msg)
    print(msg)

    # === DEBUG ========================================================
    NOPICS   = 1  # set to 1 to stop images being taken
    NOSPIN   = 0  # set to 1 to stop the spin cycles
    NOLASER  = 0  # set to 1 to skip moving and firing the lasers
    VERBOSE  = 0  # be a bit more chatty about whats going on
    CHECKING = 0  # Some extra 'expensive' checks (not for production)
                  # Use CHECKING with EXPERIMENTAL

    # Calibration
    TOLERANCE = 5 # Maximum acceptable variance in deg before using baked defaults from calibration file

    # Resizes for calibration, the full frame from the camera is in fact (2592, 1944)
    FULL_FRAME  = (2592, 1944)
    SAVE_SIZE   = ( FULL_FRAME[0] >> 3, FULL_FRAME[1] >> 3) # save checking image at a reduced size
    RESIZE      = ( FULL_FRAME[0], FULL_FRAME[1] )          # full sized (slower)
    #RESIZE    = ( FULL_FRAME[0] >> 1, FULL_FRAME[1] >> 1)  # (1296, 972) less accurate?
    #RESIZE    = ( FULL_FRAME[0] >> 2, FULL_FRAME[1] >> 2)  # (648, 486) a bit wonky
    #RESIZE    = ( FULL_FRAME[0] >> 3, FULL_FRAME[1] >> 3)  # (324, 243) way too small


    # Blood plasma separation
    BPSSS        = 5500  # blood plasma seperation spin speed
    BPS_RD       = 15.0  # radial distance (mm) of the BPS chamber on the cassette
    if not SHORT:
        BPS_TIME = 20    # length of time to spin to get seperation of plasma (PRP)
    else:
        BPS_TIME = 20

    # for the MA1010 the 0 position is no longer the homing pin but LV1
    # Also we now do everything in encoder ticks (pray we never change
    # it to a different SI encoder)
    # RIGHT NOW I'm using the homing pin and the angular values from an
    # older disk object blueprint but it's all relative and would eventually
    # be fed in as a recipe from the network
    TICKS = SI_COUNT/float(360)  # probably 11.11111

    # How far to swing the laser during valve cutting, speeds and delays etc
    # angles to ticks requires floats and rounds (fractions are ok as we round
    # the result eg 2.5deg = 27.777 ticks -> 28 ticks assigned
    SWING        = round(2.0 * float(TICKS))  # Inner laser swing
    SWING2       = round(1.0 * float(TICKS))  # LV5 cuts are smaller angles
    DELAY        = 0.5  # delay to lase over the same position in seconds
    STEPS_INNER  = 6
    STEPS_OUTER  = 6

    CAL_FILE     = self.get_calibration_from_file(find_cal_for='CAMERA_OFFSET')[1]  # T/F
    CAL_CAMERA   = int(self.get_calibration_from_file(find_cal_for='CAMERA_OFFSET')[0])
    CAL_INNER    = int(self.get_calibration_from_file(find_cal_for='LV1_OFFSET')[0])
    CAL_OUTER    = int(self.get_calibration_from_file(find_cal_for='LV2_OFFSET')[0])

    if not SHORT:
        REPEATS  = 4  ##12  # default number of times the oscillation function repeats
    else:
        REPEATS  = 0

    # Laser intensities were read earlier in E20.py
    LV1_offset   = CAL_INNER
    LV2_offset   = CAL_OUTER

    # With heatsinks this and the extra cooling can /probably/ be removed
    # Just set to 0 for now until confirmed
    if not SHORT:
        LASER_COOL   = 0 ##20   # when not using spins, we need a small break to
                                # let the laser cool down else it could cut out
    else:
        LASER_COOL   = 0

    # Until we figure out how to detect the laser is too hot
    # or not bright enough etc... for now just cool off for an
    # extended period of time (hint: heat pipes / ducted airflow ?)
    # same value but used differently than LASER_COOL

    if not SHORT:
        EXTRA_COOL   = 0  ## 20 (With heatsinks the lasers are cooling adaquately)
    else:
        EXTRA_COOL   = 0

    # DISK OBJECT POSITIONS

    # The idea here is that we just plug in an arbitary list of angular
    # locations. This is done so that we can have 1,2,3,4 or as many valves
    # to shoot at without complicating the code later

    # Relative offsets for laser 1 at inner radius (starts at: 159? )
    # Physical laser is supposed to be at 180 deg
    # LV1 = 11 deg going clockwise from the homing pin
    LV1_location = 0         * float(TICKS)  # 0   or 0000   ticks
    LV2_location = (360-59)  * float(TICKS)  # 301 or 3344.4 ticks
    LV3_location = (360-99)  * float(TICKS)  # 261 or 2899.9 ticks
    LV4_location = (360-139) * float(TICKS)  # 221 or 2455.5 ticks
    LV6_location = (360-156) * float(TICKS)  # 204 or 2266.6 ticks

    # Relative offsets for laser 2 at outer radius
    # Physical laser is @ 105 deg
    # LV5 = 48 deg going clockwise from the homing pin
    LV5_location = 0         * float(TICKS)  # 0   or 0000   ticks
    # LV7_location = (360-122) * float(TICKS)  # 122 or 2644.4 ticks

    # pre and postfix's for the image filenames
    if filename is None:
        filename = 'RUN_ASSAY_TEMP'
    # for now - filename coming in is full pathname so split it and add pre
    preigg   = '/data/pre_' + filename + '_igg.tif'
    preigm   = '/data/pre_' + filename + '_igm.tif'
    igg      = '/data/'     + filename + '_igg.tif'
    igm      = '/data/'     + filename + '_igm.tif'

    # -- beginning of experimental calibration checking ----------------
    if EXPERIMENTAL:
        if False: # DEBUGGING:
            self.motor.read_current_regulator()      # So,... tell me about yourself
            self.motor.read_velocity_regulator()
            self.motor.read_positioning_regulator()

        DEBUG_IMGS = 0                               # Show image processing for detecting
                                                     # the rotation offset of the cassette
        if os.environ.get('DISPLAY') is None:        # test for /probablilty/ that
            DEBUG_IMGS = 0                           # a X11 display is setup/available

        msg = 'Taking the calibration image'
        logging.info(msg)
        print(msg)

        self.motor.set_acceleration(40)
        self.motor.home()  # Should only need to do this once

        # Note: if we have NO calibration file then the perfect position
        # is tricky because a weird bump pushs it out by 9 deg, *just*
        # beyond our max tolerance for detecting the angle
        bump = 0
        if CAL_FILE is False:  # calibration file missing! self-denial!
            bump = 0 - BUMP

        # Experimental self calibration unrelated to the LV locations
        # Our target image should be @ the camera offset -68 deg and
        # minus a further 18 deg = 82 deg (its a magic number!)
        # but we need to offset that by the camera position. Note that
        # I drop 1 tick here as an average loss for the MVP postitioning
        # method the motor driver uses.
        # ie this is math JUST for the camera! nothing else

        # Landmark selection is based on the disk material
        # This is temporary until purpose made marks are available
        marks = [ {
                  'colour'   : 'clear',
                  'landmark' : 'toothed',       # default
                  'offset'   :  68
                  },
                  {
                  'colour'   : 'black',
                  'landmark' : 'siphon',
                  'offset'   :  191
                  } ]

        mark_location = marks[0].get('offset')  # default, in case we get told
                                                # to use a non existant landmark
        for idx in marks:
            if idx.get('colour') == os.environ['_DISK_COLOUR']:
                mark_location = idx.get('offset')
                break

        msg = 'Using %s calibration landmark for a %s cartridge' % (idx.get('landmark'), idx.get('colour'))
        logging.info(msg)
        print(msg)

        # Asolute toothed location
        CAL_location = ((bump
                        + round(CAL_CAMERA - ((mark_location + 18 - 1))
                        * float(TICKS)))
                        + SI_COUNT) % SI_COUNT  # 82 deg for toothed landmark (corrected to be +ve)


        # The magic that makes everything happen
        self.motor.position_abs(CAL_location)       # should be close to /|\ markings

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass
        sleep(SETTLE)  # let trinamic board positioning settle

        # Try and take the picture
        get_sample_image(self,
                         flash=True,
                         led_group=(ORANGE_LED if 'clear' in os.environ['_DISK_COLOUR'] else WHITE_LED),
                         filename='/data/calibrate-' + filename + '.tif',
                         leave=('on'))

        msg = 'Took calibration image'
        logging.info(msg)
        print(msg)

        # Ok some gawd awful crunching using cv2 to figure this out
        img  = cv2.imread('/data/calibrate-' + filename + '.tif')
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        if DEBUG_IMGS:
            orig_title = 'Original image'
            cv2.namedWindow(orig_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(orig_title,
                       cv2.resize(gray, (640, 480)))

        gray = 255 - gray
        if DEBUG_IMGS:
            orig_inv_title = 'Original (inverted) image'
            cv2.namedWindow(orig_inv_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(orig_inv_title,
                       cv2.resize(gray, (640, 480)))

        if 'toothed' in idx.get('landmark'):
            # We dont want the messy top of the image? image[startY:endY, startX:endX]
            height, width = gray.shape
            gray_cropped = gray[int(height*0.3):height, 0:width].copy()  # drop the top 30% of mess
            if DEBUG_IMGS:
                cropped_img_title = 'Cropped image'
                cv2.namedWindow(cropped_img_title, cv2.WINDOW_AUTOSIZE)
                cv2.imshow(cropped_img_title,
                           cv2.resize(gray_cropped, (640, 480)))
            gray = gray_cropped

        # we don't need it to be full size, a quarter size would do fine
        # for testing but in production do it at full scale
        height, width, depth = img.shape
        cv2.resize(img, RESIZE)

        # threshold
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 13, 3)
        thresh = 255 - thresh
        if DEBUG_IMGS:
            thresh_img_title = 'Adaptive threshold image'
            cv2.namedWindow(thresh_img_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(thresh_img_title,
                       cv2.resize(thresh, (640, 480)))

        # Lets sharpen the image to try and smooth the lines
        kernel = np.array([[ 0, -1,  0],
                           [-1,  5, -1],
                           [ 0, -1,  0]], np.float32)
        sharpened = cv2.filter2D(thresh, -1, kernel)
        if DEBUG_IMGS:
            sharpened_img_title = 'Sharpened image'
            cv2.namedWindow(sharpened_img_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(sharpened_img_title,
                        cv2.resize(sharpened, (640, 480)))

        # apply close to connect the white areas
        kernel = np.ones((3,3), np.uint8)
        morph  = cv2.morphologyEx(sharpened, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((1,9), np.uint8)
        morph  = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)
        if DEBUG_IMGS:
            morphed_img_title = 'Morphed image'
            cv2.namedWindow(morphed_img_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(morphed_img_title,
                        cv2.resize(morph, (640, 480)))

        blur = cv2.GaussianBlur(morph, (3,3), 0)
        if DEBUG_IMGS:
            blured_img_title = 'Blured image'
            cv2.namedWindow(blured_img_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(blured_img_title,
                        cv2.resize(blur, (640, 480)))

        smoothed = cv2.addWeighted( blur, 1.5, morph, -0.5, 0)
        #smoothed = cv2.medianBlur(morph, 3)
        smoothed = cv2.adaptiveThreshold(smoothed, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 13, 3)
        smoothed = 255 - smoothed
        if DEBUG_IMGS:
            smoothed_img_title = 'Smoothed image'
            cv2.namedWindow(smoothed_img_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(smoothed_img_title,
                        cv2.resize(smoothed, (640, 480)))


        # apply canny edge detection
        edges = cv2.Canny(smoothed, 150, 200, 1)  # range 150-200 and 1 pixel thick
        if DEBUG_IMGS:
            edges_img_title = 'Edges image'
            cv2.namedWindow(edges_img_title, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(edges_img_title,
                        cv2.resize(edges, (640, 480)))


        # get hough lines
        result = img.copy()              # use .copy() or we end up scribbling over the original

        rho_res   = 8                    # Distance resolution of the accumulator in pixels
        theta     = np.pi/(180 * TICKS)  # Angle resolution of the accumulator in *radians*
                                         # note: almost universally its seen as np.pi/(180)
                                         # which gives the resolution in deg but we need
                                         # use TICKS because our encoder movement is much finer
        threshold = 50                   # votes (intersections) needed before we claim it's
                                         # a line

        lines = cv2.HoughLines(edges,    # use image with cleaned up edges
                               rho_res,
                               theta,
                               threshold,
                               0)

        if lines is None:  # beware of excessive lighting leading to no lines!
            msg = 'The image is probably overexposed, no lines can be found!\n' \
                  'Will use calibration file defaults instead'
            logging.info(msg)
            print(msg)
            sleep(2)  # Leave the message up for a couple of seconds
        else:
            print('Detected %d potential lines' % len(lines))
            # Ok lets make this fancy with transparant marker line
            # We'll need to make a blank canvas first
            sublayer = np.zeros(result.shape, dtype=np.uint8) * 0xFF
            for detected_lines in range(0, 1):  # first in list had the most votes as the strongest line
                for (rho, theta) in lines[detected_lines]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = x0 + 2000 * (-b)
                    y1 = y0 + 2000 * ( a)
                    x2 = x0 - 2000 * (-b)
                    y2 = y0 - 2000 * ( a)

                    #colour = (0xF1, 0xED, 0x38, 0x7f)  # Cyan BGR or BGRA format (A = 0 if undefined)
                    colour = (0x00, 0x00, 0xcc, 0x7f)  # Red
                    # This plots the path and SLOPE not the actual detected line
                    cv2.line(sublayer,
                             (int(round(x1)), int(round(y1))),
                             (int(round(x2)), int(round(y2))),
                             colour, (rho_res << 4) + rho_res,
                             cv2.LINE_AA )

                # Merge the sublayer that we scribbled lines over onto the
                # resulting image
                result = cv2.addWeighted(sublayer, 0.20,  #  20% solid
                                         result,   0.80,  #  80% solid
                                         1.0)

                # What was the line angle?
                # (use * 180.0 / pi to convert radians to degrees)
                # we have p0(x,y) and p1(x,y) Also make sure we measure in ONE direction
                angle = math.atan2(y2 - y1, x2 - x1) * (180.0 / math.pi)

                if angle >= 0:
                    angle = 0 - math.fabs(+90 - angle)
                else:
                    angle = 0 + math.fabs(-90 - angle)

                # Sanity check -- TODO: can we loop a few times if it's badly off?
                if TOLERANCE > angle > -TOLERANCE:
                    msg = 'Skew angle is acceptable for autocorrection (%.2f°)' % angle
                    logging.info(msg)
                    print(msg)
                else:
                    msg = 'Skew tolerance exceeded (%.2f°)' % angle + ' autocorrection may be invalid'
                    logging.info(msg)
                    print(msg)
                    if (angle > (90 - TOLERANCE)) or (angle < (-90 + TOLERANCE)):
                        msg = 'Skew tolerance is excessive, (probably measured a right angle line)\n' \
                              'ABORTING RUN. Check the /data/calibration_offsets.txt file for \n' \
                              'garbage or use QR code to reset/refresh calibration'
                        logging.info(msg)
                        print(msg)
                        return -1  # BAILOUT, no point wasting a cassette.

                fine_offset = round(angle * TICKS)
                print('Camera skew angle: %.2f°' % angle)
                print('Camera tick offset = %d (%.2f°)' % (fine_offset, angle))
                logging.info('Calibration thinks we need to fine tune by an extra ' \
                              + str(fine_offset) + ' ticks')

            if CHECKING:  # just wanna see whats going on
                height, width, depth = img.shape

                # Fonts, text and colours
                font        = cv2.FONT_HERSHEY_SIMPLEX
                font_scaled = 3
                text_colour = (0x19, 0x19, 0xBB, 0x00)  # BGR or BGRA format (A = 0 if undefined)
                text_width  = 8                         # thicker line?
                text_margin = 4                         # add pixel margin around text

                # Lets add a transparent rectangle to make the text easier to read
                # longest text is
                text = 'Camera offset from cfg file: %d' % CAL_CAMERA + ' ticks (orig)'
                (text_bg_width, text_bg_height), baseline = cv2.getTextSize(text, font, font_scaled, text_width)
                x, y, lines = 4, 70, 4  # x,y start and approx lines
                w, h = text_bg_width, int((text_bg_height + baseline) * (lines *1.1)) # hacky but this is for checking only
                sub_img = result[y:y+h, x:x+w]
                rect = np.ones(sub_img.shape, dtype=np.uint8) * 0xFF
                res = cv2.addWeighted(sub_img, 0.5, rect, 0.5, 1.0)

                # Put the transparent rectangle back to its position
                result[y:y+h, x:x+w] = res

                cv2.putText(result, 'Camera offset from cfg file: %d ' % CAL_CAMERA + ' ticks (orig)',
                            (10, 150),
                            font, font_scaled,
                            text_colour, text_width,
                            cv2.LINE_AA)
                cv2.putText(result, 'Desired value: %d' % CAL_location + ' ticks',
                            (10, 250),
                            font, font_scaled,
                            text_colour, text_width,
                            cv2.LINE_AA)
                cv2.putText(result, 'ABS Position:  %d ticks (%.2f deg)' % (
                            self.motor.get_actual_encoder_count(),
                            self.motor.get_actual_encoder_count()/TICKS),
                            (10,350),
                            font, font_scaled,
                            text_colour, text_width,
                            cv2.LINE_AA)
                cv2.putText(result, 'Camera skewed by: %d ticks (%.2f deg)' % (
                            fine_offset,
                            angle),
                            (10, 450),
                            font, font_scaled,
                            text_colour, text_width,
                            cv2.LINE_AA)
                result = cv2.resize(result, SAVE_SIZE)
                cv2.imwrite(CAL_DIR + 'detected-' + filename +'-check.jpg', result)

                if DEBUG_IMGS:
                    detected_img_title = 'Detected image'
                    cv2.namedWindow(detected_img_title, cv2.WINDOW_AUTOSIZE)
                    cv2.imshow(detected_img_title,
                               cv2.resize(result, (640, 480)))

            # And at this point we have a new fine tuned camera offset
            # offset LV1/2 by the same amount (adjust to be +ve values)
            CAL_CAMERA = (CAL_CAMERA + fine_offset + bump + SI_COUNT) % SI_COUNT
            LV1_offset = (LV1_offset + bump + SI_COUNT) % SI_COUNT
            LV2_offset = (LV2_offset + bump + SI_COUNT) % SI_COUNT

            if FOLLOW_CAM_OFFSET:  # add the camera offset to the lv1/2 positions
                LV1_offset = (LV1_offset + fine_offset + SI_COUNT) % SI_COUNT
                LV2_offset = (LV2_offset + fine_offset + SI_COUNT) % SI_COUNT


        if CHECKING:
            # Cheque please! show image with position offset applied
            self.motor.position_abs(round(CAL_CAMERA - (mark_location + 18 -1) * float(TICKS)
                                    + SI_COUNT) % SI_COUNT)

            # Wait until the last move is completed
            while not self.motor.info(check_bit=MOVE_COMPLETED):
                pass

            sleep(SETTLE)  # let trinamic board positioning settle

            get_sample_image(self,
                             flash=True,
                             led_group=(ORANGE_LED if 'clear' in os.environ['_DISK_COLOUR'] else WHITE_LED),
                             filename=CAL_DIR + 'fixed-' + filename + '-.tif',
                             leave='on')
            img  = cv2.imread(CAL_DIR + 'fixed-' + filename + '-.tif')
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            gray = 255 - gray

            # we don't need it to be full size, a quarter size would do fine
            height, width, depth = img.shape

            # Fonts, text and colours
            font        = cv2.FONT_HERSHEY_SIMPLEX
            font_scaled = 3
            text_colour = (0x19, 0x19, 0xBB, 0x00)  # BGR or BGRA format (A = 0 if undefined)
            text_width  = 8                         # thicker line?

            # Lets add a transparent rectangle to make the text easier to read
            # longest text is
            text = 'Camera offset from visual : %d' % CAL_CAMERA + ' ticks (calculated)'
            (text_bg_width, text_bg_height), baseline = cv2.getTextSize(text, font, font_scaled, text_width)
            x, y, lines = 4, 70, 3  # x,y start and approx lines
            w, h = text_bg_width, int((text_bg_height + baseline) * (lines *1.1)) # hacky but this is for checking only
            sub_img = img[y:y+h, x:x+w]
            rect = np.ones(sub_img.shape, dtype=np.uint8) * 0xFF
            res = cv2.addWeighted(sub_img, 0.5, rect, 0.5, 1.0)

            # Put the transparent rectangle back to its position
            img[y:y+h, x:x+w] = res

            cv2.putText(img, 'Camera offset from visual : %d' % CAL_CAMERA + ' ticks (calculated)',
                        (10, 150),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            cv2.putText(img, 'Desired value: %d ticks' % (
                        round(((CAL_CAMERA - ((67 + 18) * float(TICKS))) + SI_COUNT) % SI_COUNT)),
                        (10, 250),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            cv2.putText(img, 'ABS Position:  %d ticks (%.2f deg)' % (
                        self.motor.get_actual_encoder_count(),
                        self.motor.get_actual_encoder_angle()),
                        (10, 350),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            img = cv2.resize(img, SAVE_SIZE)
            cv2.imwrite(CAL_DIR + 'fixed-' + filename + '-check.jpg', img)
            if DEBUG_IMGS:
                corrected_img_title = 'Corrected image - PRESS ANY KEY TO CONTINUE'
                cv2.namedWindow(corrected_img_title, cv2.WINDOW_AUTOSIZE)
                cv2.imshow(corrected_img_title,
                           cv2.resize(img, (640, 480)))
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    # -- end of experimental calibration checking ----------------------

    print('LV1-6:     %04d %04d %04d %04d %04d %04d'
          % (LV1_location,
             LV2_location,
             LV3_location,
             LV4_location,
             LV5_location,
             LV6_location))
    # create valve list with simple 180 deg mirroring
    # Laser1 is 180 deg offset from the homing pin sensor
    # Laser2 has a different position offset
    # the Camera, laser1 and laser2 and probably the homing pin
    # all have slight offsets
    # NOTE (SI_COUNT/2) = 180 deg (SI_COUNT = 360 deg)
    LV1_SAMPLE   = [(LV1_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV1_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
    LV2_WASH1    = [(LV2_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT, (LV2_location + LV1_offset + SI_COUNT) % SI_COUNT]
    LV3_ANTIBODY = [(LV3_location + LV1_offset + SI_COUNT) % SI_COUNT, (LV3_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
    LV4_WASH2    = [(LV4_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT, (LV4_location + LV1_offset + SI_COUNT) % SI_COUNT]
    LV5_DRY      = [(LV5_location + LV2_offset + SI_COUNT) % SI_COUNT, (LV5_location + LV2_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]
    LV6_SPIN     = [(LV6_location + LV1_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT, (LV6_location + LV1_offset + SI_COUNT) % SI_COUNT]
    # LV7_RAMJET   = [(LV7_location + LV2_offset + SI_COUNT) % SI_COUNT, (LV7_location + LV2_offset + (SI_COUNT/2) + SI_COUNT) % SI_COUNT]

    # the camera is offset by 261 deg (in theory, it is 270)
    # The well is 150 deg away from the homing pin and ~18 deg wide
    IGM_location = 0       # 0   or 0000   ticks
    # Move from degrees to ticks and add the offset
    IGM_POSITION = int(IGM_location + CAL_CAMERA + SI_COUNT) % SI_COUNT
    IGG_POSITION = int(IGM_POSITION + (SI_COUNT/2) + SI_COUNT) % SI_COUNT

    # Debugging, you should get something like
    # 1766 1110 0666 0221 2189 0032 0833 - within a few ticks variation
    # due to the slight tolerance shifts in motor positioning
    print('LV1-6+off: %04d %04d %04d %04d %04d %04d'
          % (LV1_SAMPLE[0],
             LV2_WASH1[0],
             LV3_ANTIBODY[0],
             LV4_WASH2[0],
             LV5_DRY[0],
             LV6_SPIN[0]))

    if VERBOSE != 0:
        logging.info('Calibration:')
        logging.info('============')
        logging.info('Camera          = %d' % CAL_CAMERA)
        logging.info('Inner Laser (1) = %d\tIntensity = %d' % (CAL_INNER, LASER1_INTENSITY))
        logging.info('Outer Laser (2) = %d\tIntensity = %d' % (CAL_OUTER, LASER2_INTENSITY))

        logging.info('-----------------------------------------')
        assay_id = __file__.split('/')[len(__file__.split('/'))-1]
        cartridge = assay_id.split('-')[0]

        logging.info('We are using a %s cartridge' , cartridge)

    # ------------------------------------------------------------------
    def spin_oscilate(loops=REPEATS):
        '''
        Function to oscillate the disk
        '''
        self.motor.set_acceleration(3000)
        print('spin_oscilate(x%d)' % loops)
        if NOSPIN == 0 and not SHORT:
            high_spin = 7500
            print('Oscillate cycle starting @ %d rpm' % high_spin)
            self.motor.spin(rpm=high_spin, spin_time=10)
            repeat = loops  # No. of times to repeat (12?)
            while repeat > 0:
                speeds = [('high', 7000, 1),  # text, speed, time (not all used)
                          ('low' , 3000, 1)]
                for (_, speed, duration) in speeds:  # _ means we don't care about that value
                    print('Reciprocation [%d/%d] @%d rpm' % ((loops + 1) - repeat, loops, speed))
                    self.motor.spin(rpm=speed, spin_time=duration)
                repeat -= 1  # keep decreasing by 1

            if loops != 0:  # beware of special case where loops=0 is passed in
                # small list of spins needed to shuffle fluids around the cassette
                # Note: I need it to actually STOP at the end
                # (speed, duration, stopping type)
                SPIN_TASKS = [(7000, 4, 'NONSTOP'),
                              ( 175, 6, 'NONSTOP'),
                              (   0, 1, 'NONSTOP'),
                              (4000, 4, 'CALCULATED') ] #,
                              # ~ (   0, 0, 'CALCULATED') ]
                for (idx, values) in enumerate(SPIN_TASKS, start = 1):
                    print('Shuffling fluids %d/%d' %  (idx, len(SPIN_TASKS)))
                    self.motor.spin(rpm=values[0],
                                    spin_time=values[1],
                                    stopping=values[2])

        else:
            if NOLASER == 0:
                print('Faked Oscillate cycle - cooldown %d sec @ %d'
                       % (LASER_COOL, lineno()))
                sleep (LASER_COOL)
            else:
                print('Faked Oscillate cycle - no laser used @ %d'
                       % lineno())

        # reset the acceleration to default
        self.motor.set_velocity(0)  # stop spinning
        self.motor.set_acceleration(self.motor.acceleration)  # reset to default (1000 rpm?)
        self.motor.reset_pid = 1
        self.motor.home()  # reset the encoder position

    # ------------------------------------------------------------------
    # Pre-calibrate
    if not EXPERIMENTAL:
        self.motor.home()  # Should only need to do this once
                           # In theory we should be able to get the position
                           # then set the position modulo TICKS (SAP 1)
                           # In practice it moves 8/
                           # NOTE!: .home() uses the HOME_ACCEL
                           # value in settings.py

    self.motor.set_acceleration(HOME_ACCEL)  # Use slower acceleration for
                                             # slower movements to prevent sloshing
    # Take Pre-images
    if NOPICS == 0:
        if VERBOSE != 0:
            msg = 'Moving to IGG well position @ %d (%d°)' \
                  % (IGG_POSITION, IGG_POSITION/(SI_COUNT/360))
            logging.info(msg)
            print(msg)

        self.motor.position_abs(IGG_POSITION)

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass
        sleep(SETTLE)  # let trinamic board positioning settle if needed

        print('Taking pre-run image of IGG Well')
        get_sample_image(self,
                         flash=True,
                         filename=preigg,
                         leave='on')

        if EXPERIMENTAL and CHECKING:
            img  = cv2.imread(preigg)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            gray = 255 - gray

            # we don't need it to be full size, a quarter size would do fine
            height, width, depth = img.shape

            # Fonts, text and colours
            font         = cv2.FONT_HERSHEY_SIMPLEX
            font_scaled  = 3
            text_colour  = (0x19, 0x19, 0xBB, 0x00)  # BGR or BGRA format (A = 0 if undefined)
            text_width   = 8                         # thicker line?

            # Lets add a transparent rectangle to make the text easier to read
            # longest text is
            text = ('ABS Position: '
                        + str(self.motor.get_actual_encoder_count())
                        + ' ticks ('
                        + str(self.motor.get_actual_encoder_angle())
                        + ' deg)')
            (text_bg_width, text_bg_height), baseline = cv2.getTextSize(text, font, font_scaled, text_width)
            x, y, lines = 4, 170, 2  # x,y start and approx lines
            w, h = text_bg_width, int((text_bg_height + baseline) * (lines *1.1))  # hacky but this is for checking only
            sub_img = img[y:y+h, x:x+w]
            rect = np.ones(sub_img.shape, dtype=np.uint8) * 0xFF
            res = cv2.addWeighted(sub_img, 0.5, rect, 0.5, 1.0)

            # Put the transparent rectangle back to its position
            img[y:y+h, x:x+w] = res

            cv2.putText(img, 'Desired value: %d' % IGG_POSITION + ' ticks',
                        (10, 250),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            cv2.putText(img, 'ABS Position: '
                        + str(self.motor.get_actual_encoder_count())
                        + ' ticks ('
                        + str(self.motor.get_actual_encoder_angle())
                        + ' deg)',
                        (10, 350),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            img = cv2.resize(img, SAVE_SIZE)
            cv2.imwrite(CAL_DIR + 'preigg-' + filename + '-check.jpg', img)


        if VERBOSE != 0:
            msg = 'Moving to IGM well position @ %d (%d°)' \
                   % (IGM_POSITION, IGM_POSITION / (SI_COUNT / 360))
            logging.info(msg)
            print(msg)

        self.motor.position_abs(IGM_POSITION)

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass
        sleep(SETTLE)  # let trinamic board positioning settle if needed

        print('Taking pre-run image of IGM well')
        get_sample_image(self,
                         flash=True,
                         filename=preigm,
                         leave='off')

        if EXPERIMENTAL and CHECKING:
            img  = cv2.imread(preigm)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            gray = 255 - gray

            # we don't need it to be full size, a quarter size would do fine
            height, width, depth = img.shape
            font        = cv2.FONT_HERSHEY_SIMPLEX
            font_scaled = 3
            text_colour = (0x19, 0x19, 0xBB, 0x00)  # BGR or BGRA format (A = 0 if undefined)
            text_width  = 8                         # thicker line?

            # Lets add a transparent rectangle to make the text easier to read
            # longest text is
            text = ('ABS Position: '
                        + str(self.motor.get_actual_encoder_count())
                        + ' ticks ('
                        + str(self.motor.get_actual_encoder_angle())
                        + ' deg)')
            (text_bg_width, text_bg_height), baseline = cv2.getTextSize(text, font, font_scaled, text_width)
            x, y, lines = 4, 170, 2  # x,y start and approx lines
            w, h = text_bg_width, int((text_bg_height + baseline) * (lines *1.1))  # hacky but this is for checking only
            sub_img = img[y:y+h, x:x+w]
            rect = np.ones(sub_img.shape, dtype=np.uint8) * 0xFF
            res = cv2.addWeighted(sub_img, 0.5, rect, 0.5, 1.0)

            # Put the transparent rectangle back to its position
            img[y:y+h, x:x+w] = res

            cv2.putText(img, 'Desired value: %d' % IGM_POSITION + ' ticks',
                        (10, 250),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            cv2.putText(img, 'ABS Position: '
                        + str(self.motor.get_actual_encoder_count())
                        + ' ticks ('
                        + str(self.motor.get_actual_encoder_angle())
                        + ' deg)',
                        (10, 350),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            img = cv2.resize(img, SAVE_SIZE)
            cv2.imwrite(CAL_DIR + 'preigm-' + filename + '-check.jpg', img)
    else:
        print('Without pictures, need to reset accel and velocities')
        self.motor.set_acceleration(self.motor.acceleration)  # otherwise takes forever to spin down
        self.motor.set_max_abs_ramp_velocity(self.motor.velocity_max_pos_ctrl)

    # ------------------------------------------------------------------
    # Blood plasma separation
    self.motor.set_acceleration(500)  # bit slower to spin up
    if NOSPIN == 0:
        # RCF = 1.118 x Radius(mm) x (rpm/1000)^2
        rcf = lambda r: r * 1.118 * ((BPSSS/1000) ** 2)
        print('Blood plasma separation rcf %dg (%d rpm)'
              % (rcf(BPS_RD), BPSSS))
        self.motor.spin(rpm=BPSSS, spin_time=BPS_TIME, stopping=CALCULATED)  # On exit relocate '0'
        self.motor.reset_pid = 1
        self.motor.home()
    else:
        if NOLASER == 0:
            print('Faked Oscillate cycle - cooldown %d sec @ %d'
                   % (LASER_COOL, lineno()))
            sleep (LASER_COOL)
        else:
            print('Faked Oscillate cycle - no laser used @ %d'
                   % lineno())

    # ------------------------------------------------------------------
    # Open laser valve 1 (sample)
    if NOLASER == 0:
        for i in range(len(LV1_SAMPLE)):
            if (i % 2) == 1:
                sleep(EXTRA_COOL)

            if VERBOSE != 0:
                if (i % 2) == 1:
                    msg = 'Sleeping %d seconds to let laser cool' % EXTRA_COOL
                    logging.info(msg)
                    print(msg)

            position = round(LV1_SAMPLE[i % len(LV1_SAMPLE)])
            msg = 'Moving to LV1#%d @ %d ticks (%d°)' \
                   % (i % len(LV1_SAMPLE),
                      position,
                      position / (SI_COUNT/360))
            logging.info(msg)
            print(msg)

            self.wobbleFire(position)

            # self.motor.position_abs(position)
            # sleep(2)
            # self.laser1.set_intensity(LASER_1, 800)
            # self.laser1.on()

            # self.motor.wobble(swingX=50)
            # self.laser1.off()

            # if self.laser_sweep(start_tick=position,
            #                     width=SWING,
            #                     delay=DELAY,
            #                     steps=STEPS_INNER,
            #                     laser_intensity=INTENSITY_WET) == -1:
            #     return -2

    # ------------------------------------------------------------------
    # Apprently opening LV1 and LV2 at the same time is the new way
    # so no need to rattle and roll (reciprocation) anymore at
    # this point? (commented out for now, will remove when confirmed)
    # ~ # Reciprocation
    # ~ self.motor.set_acceleration(40)
    # ~ if NOSPIN == 0:
        # ~ spin_oscilate()
        # ~ self.motor.brake()
    # ~ else:
        # ~ if NOLASER == 0:
            # ~ print('Faked Oscillate cycle - cooldown %d sec @ %d'
                   # ~ % (LASER_COOL, lineno()))
            # ~ sleep (LASER_COOL)
        # ~ else:
            # ~ print('Faked Oscillate cycle - no laser used @ %d'
                   # ~ % lineno())

    if not SHORT:
        # --------------------------------------------------------------
        # Open laser valve 2 (wash buffer 1)
        if NOLASER == 0:
            for i in range(len(LV2_WASH1)):
                if (i % 2) == 1:
                    sleep(EXTRA_COOL)

                if VERBOSE != 0:
                    if (i % 2) == 1:
                        msg = 'Sleeping %d seconds to let laser cool' % EXTRA_COOL
                        logging.info(msg)
                        print(msg)

                position = round(LV2_WASH1[i % len(LV2_WASH1)])
                msg = 'Moving to LV2#%d @ %d ticks (%d°)' \
                       % (i % len(LV2_WASH1),
                          position,
                          position / (SI_COUNT/360))
                logging.info(msg)
                print(msg)

                self.motor.position_abs(position)
                sleep(2)
                self.laser1.set_intensity(LASER_1, 800)
                self.laser1.on()

                self.motor.wobble(swingX=125)
                self.laser1.off()


                # if self.laser_sweep(start_tick=position,
                #                     width=SWING,
                #                     delay=DELAY,
                #                     steps=STEPS_INNER,
                #                     laser_intensity=INTENSITY_WET) == -1:
                #     return -2

        # Reciprocation
        if NOSPIN == 0:
            spin_oscilate()
            self.motor.brake()
        else:
            if NOLASER == 0:
                print('Faked Oscillate cycle - cooldown %d sec @ %d'
                       % (LASER_COOL, lineno()))
                sleep (LASER_COOL)
            else:
                print('Faked Oscillate cycle - no laser used @ %d'
                       % lineno())

        # --------------------------------------------------------------
        # Open laser valve 3 (Secondary Antibody)
        if NOLASER == 0:
            for i in range(len(LV3_ANTIBODY)):
                if (i % 2) == 1:
                    sleep(EXTRA_COOL)

                if VERBOSE != 0:
                    if (i % 2) == 1:
                        msg = 'Sleeping %d seconds to let laser cool' % EXTRA_COOL
                        logging.info(msg)
                        print(msg)

                position = round(LV3_ANTIBODY[i % len(LV3_ANTIBODY)])
                msg = 'Moving to LV3#%d @ %d ticks (%d°)' \
                       % (i % len(LV3_ANTIBODY),
                          position,
                          position / (SI_COUNT/360))
                logging.info(msg)
                print(msg)

                self.motor.position_abs(position)
                sleep(2)
                self.laser1.set_intensity(LASER_1, 800)
                self.laser1.on()

                self.motor.wobble(swingX=25)
                self.laser1.off()


                # if self.laser_sweep(start_tick=position,
                #                     width=SWING,
                #                     delay=DELAY,
                #                     steps=STEPS_INNER,
                #                     laser_intensity=INTENSITY_WET) == -1:
                #     return -2

        # Reciprocation
        if NOSPIN == 0:
            spin_oscilate()
            self.motor.brake()
        else:
            if NOLASER == 0:
                print('Faked Oscillate cycle - cooldown %d sec @ %d'
                       % (LASER_COOL, lineno()))
                sleep (LASER_COOL)
            else:
                print('Faked Oscillate cycle - no laser used @ %d'
                       % lineno())

        # --------------------------------------------------------------
        # Open laser valve 4 (wash buffer 2)
        if NOLASER == 0:
            for i in range(len(LV4_WASH2)):
                if (i % 2) == 1:
                    sleep(EXTRA_COOL)

                if VERBOSE != 0:
                    if (i % 2) == 1:
                        msg = 'Sleeping %d seconds to let laser cool' % EXTRA_COOL
                        logging.info(msg)
                        print(msg)

                position = round(LV4_WASH2[i % len(LV4_WASH2)])
                msg = 'Moving to LV4#%d @ %d ticks (%d°)' \
                       % (i % len(LV4_WASH2),
                          position,
                          position / (SI_COUNT/360))
                logging.info(msg)
                print(msg)

                if self.laser_sweep(start_tick=position,
                                    width=SWING,
                                    delay=DELAY,
                                    steps=STEPS_INNER,
                                    laser_intensity=INTENSITY_WET) == -1:
                    return -2

        # Reciprocation
        if NOSPIN == 0:
            spin_oscilate()
            self.motor.brake()
        else:
            if NOLASER == 0:
                print('Faked Oscillate cycle - cooldown %d sec @ %d'
                       % (LASER_COOL, lineno()))
                sleep (LASER_COOL)
            else:
                print('Faked Oscillate cycle - no laser used @ %d'
                       % lineno())

        # --------------------------------------------------------------
        # Open laser valve 5 (Dry and Spin)
        if NOLASER == 0:
            for i in range(len(LV5_DRY)):
                if (i % 2) == 1:
                    sleep(EXTRA_COOL)

                if VERBOSE != 0:
                    if (i % 2) == 1:
                        msg = 'Sleeping %d seconds to let laser cool' % EXTRA_COOL
                        logging.info(msg)
                        print(msg)

                position = round(LV5_DRY[i % len(LV5_DRY)])
                msg = 'Moving to LV5#%d @ %d ticks (%d°)' \
                       % (i % len(LV5_DRY),
                          position,
                          position / (SI_COUNT/360))
                logging.info(msg)
                print(msg)

                if self.laser2_sweep(start_tick=position,
                                     width=SWING2,
                                     delay=DELAY,
                                     steps=STEPS_INNER,
                                     laser_intensity=INTENSITY_WET) == -1:  # WET VALVE! use more power and steps
                    return -2

        # --------------------------------------------------------------
        # Open laser valve 6 (Spin)

        if NOLASER == 0:
            for i in range(len(LV6_SPIN)):
                # ~ if VERBOSE != 0:
                position = round(LV6_SPIN[i % len(LV6_SPIN)])
                msg = 'Moving to LV6#%d @ %d ticks (%d°)' \
                       % (i % len(LV6_SPIN),
                          position,
                          position / (SI_COUNT/360))
                logging.info(msg)
                print(msg)

                if self.laser_sweep(start_tick=position,
                                    width=SWING,
                                    delay=DELAY,
                                    steps=STEPS_INNER,
                                    laser_intensity=INTENSITY_DRY) == -1:
                    return -2

        # valves 5 & 6 are now open so we can dump everything to
        # the waste chamber
        # Dry the wells
        self.motor.set_acceleration(500)
        if NOSPIN == 0:
            print('Spin drying the wells')
            self.motor.spin(rpm=-7500, spin_time=180, stopping=CALCULATED)
            sleep(2)  # We must wait for a while for the fluid to siphon
        else:
            if NOLASER == 0:
                print('Faked Oscillate cycle - cooldown %d sec @ %d'
                       % (LASER_COOL, lineno()))
                sleep (LASER_COOL)
            else:
                print('Faked Oscillate cycle - no laser used @ %d'
                       % lineno())

        # FLING! -splat- fluids to opposite side of the waste container

        if NOSPIN == 0:
            self.motor.set_acceleration(1000)
            msg = 'Shift fluid to other side of waste chamber'
            logging.info(msg)
            print(msg)
            self.motor.spin(rpm=3000, spin_time=5, stopping=CALCULATED)  # on exit find '0'
            self.motor.reset_pid = 1

        # Turn the camera back on preemptively. We'll need it in a few seconds time
        # 5 sec + home 1sec + positioning 1 sec?
        self.camera.power_on_camera()
        self.motor.set_acceleration(HOME_ACCEL)  # just be sure the accel is sane
        self.motor.home()

    # Take images
    if NOPICS == 0:
        if VERBOSE != 0:
            msg = 'Moving to IGG well position @ %d (%d°)' \
                  % (IGG_POSITION, IGG_POSITION/(SI_COUNT/360))
            logging.info(msg)
            print(msg)

        self.motor.position_abs(IGG_POSITION)

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass
        sleep(SETTLE)  # let trinamic board positioning settle if needed

        print('Taking image of IGG Well for processing)')
        get_sample_image(self,
                         flash=True,
                         filename=igg,
                         leave='on')

        if EXPERIMENTAL and CHECKING:
            img  = cv2.imread(igg)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            gray = 255 - gray

            # we don't need it to be full size, a quarter size would do fine
            height, width, depth = img.shape

            # Fonts, text and colours
            font         = cv2.FONT_HERSHEY_SIMPLEX
            font_scaled  = 3
            text_colour  = (0x19, 0x19, 0xBB, 0x00)  # BGR or BGRA format (A = 0 if undefined)
            text_width   = 8                         # thicker line?

            # Lets add a transparent rectangle to make the text easier to read
            # longest text is
            text = ('ABS Position:  %d ticks (%.2f deg)' % (
                        self.motor.get_actual_encoder_count(),
                        self.motor.get_actual_encoder_angle()))
            (text_bg_width, text_bg_height), baseline = cv2.getTextSize(text, font, font_scaled, text_width)
            x, y, lines = 4, 170, 2  # x,y start and approx lines
            w, h = text_bg_width, int((text_bg_height + baseline) * (lines *1.1)) # hacky but this is for checking only
            sub_img = img[y:y+h, x:x+w]
            rect = np.ones(sub_img.shape, dtype=np.uint8) * 0xFF
            res = cv2.addWeighted(sub_img, 0.5, rect, 0.5, 1.0)

            # Put the transparent rectangle back to its position
            img[y:y+h, x:x+w] = res

            cv2.putText(img, 'Desired value: %d ticks' % (
                        IGG_POSITION),
                        (10, 250),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            cv2.putText(img, 'ABS Position:  %d ticks (%.2f deg)' % (
                        self.motor.get_actual_encoder_count(),
                        self.motor.get_actual_encoder_angle()),
                        (10, 350),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            img = cv2.resize(img, SAVE_SIZE)
            cv2.imwrite(CAL_DIR + 'igg-' + filename + '-check.jpg', img)

        if VERBOSE != 0:
            msg = 'Moving to IGM well position @ %d (%d°)' \
                   % (IGM_POSITION, IGM_POSITION/(SI_COUNT/360))
            logging.info(msg)
            print(msg)

        self.motor.position_abs(IGM_POSITION)

        # Wait until the last move is completed
        while not self.motor.info(check_bit=MOVE_COMPLETED):
            pass
        sleep(SETTLE)  # let trinamic board positioning settle

        print('Taking image of IGM Well for processing')
        get_sample_image(self,
                         flash=True,
                         filename=igm,
                         leave='off')

        if EXPERIMENTAL and CHECKING:
            img  = cv2.imread(igm)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            gray = 255 - gray

            # we don't need it to be full size, a quarter size would do fine
            height, width, depth = img.shape

            # Fonts, text and colours
            font        = cv2.FONT_HERSHEY_SIMPLEX
            font_scaled = 3
            text_colour = (0x19, 0x19, 0xBB, 0x00)  # BGR or BGRA format (A = 0 if undefined)
            text_width  = 8                         # thicker line?

            # Lets add a transparent rectangle to make the text easier to read
            # longest text is
            text = ('ABS Position: %d ticks (%.2f deg)' % (
                        self.motor.get_actual_encoder_count(),
                        self.motor.get_actual_encoder_angle()))
            (text_bg_width, text_bg_height), baseline = cv2.getTextSize(text, font, font_scaled, text_width)
            x, y, lines = 4, 170, 2  # x,y start and approx lines
            w, h = text_bg_width, int((text_bg_height + baseline) * (lines *1.1)) # hacky but this is for checking only
            sub_img = img[y:y+h, x:x+w]
            rect = np.ones(sub_img.shape, dtype=np.uint8) * 0xFF
            res = cv2.addWeighted(sub_img, 0.5, rect, 0.5, 1.0)

            # Put the transparent rectangle back to its position
            img[y:y+h, x:x+w] = res

            cv2.putText(img, 'Desired value: %d' % IGM_POSITION + ' ticks',
                        (10, 250),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            cv2.putText(img, 'ABS Position:  %d ticks (%.2f deg)' % (
                        self.motor.get_actual_encoder_count(),
                        self.motor.get_actual_encoder_angle()),
                        (10, 350),
                        font, font_scaled,
                        text_colour, text_width,
                        cv2.LINE_AA)
            img = cv2.resize(img, SAVE_SIZE)
            cv2.imwrite(CAL_DIR + 'igm-' + filename + '-check.jpg', img)

    if False: # DEBUGGING:
            self.motor.read_current_regulator()      # So,... tell me about yourself
            self.motor.read_velocity_regulator()
            self.motor.read_positioning_regulator()

    self.motor.motor_release()  # let it become free spinning (uses less electric)
    self.end_operations()  # allow the lid to be opened without aborting the test

    return 0
