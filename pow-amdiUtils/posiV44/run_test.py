#!/usr/libexec/platform-python
'''
Run the test
'''
import sys
import logging
import os
import shutil
from pathlib import Path
import time
from datetime import datetime
import requests
import tifftools

from settings import LOGGING_FILE, VERSION_FILE, SERIAL_FILE, PENDING_DIR, POSI_FILE
from settings import HARDWARE_MODEL, OCI_APIGW_URL, PROXY_FILE
from settings import LID_OPEN, LID_OPEN_TEST, BAD_DISK
from settings import SEND_DIRECTLY

from E20 import E20

test_failed = 0 # by default set to OK, if there's an issue set to 1 and at the end exit with it

def get_time():
    dateTime = datetime.now()
    dateTimeString = dateTime.strftime("%d-%b-%Y %H:%M:%S")

    return dateTimeString

# Add a logger
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

    logging.getLogger('').setLevel(logging.INFO)

if len(sys.argv) != 3:
    print("Test failed, internal error. No arguments passed to run_test")
    logging.critical("no arguments passed to run_test")
    sys.exit(1)

diskQRCode = sys.argv[1]
patientQRCode = sys.argv[2]

test_start = time.time()
logging.info("starting test")
print("Initializing hardware for test.")
logging.info("hardware init")

e20 = E20()

logging.info("test running")
print("Running test. Please wait.")
try:
    ret = e20.run_assay(diskQRCode)
    print("Test finished")
    logging.info('Test finished after %s' % time.strftime("%Mm %Ss", time.gmtime(time.time() - test_start)))
except ImportError:
    logging.critical("Invalid Module Name - diskQRcode %s", diskQRCode)
    test_failed = 1
    print("Test failed - unknown disk version %s", diskQRCode)
    sys.exit(BAD_DISK)
except Exception as e:
    logging.error("Exception : %s", e)
    print("Test failed")
    logging.critical("test failed")
    test_failed = 1
    ret = -1

if ret == LID_OPEN:
    print("Lid open detected, please close the lid")
    sys.exit(LID_OPEN)
if ret == LID_OPEN_TEST:
    print("Lid opened during test. Test failed")
    sys.exit(LID_OPEN_TEST)

with open(LOGGING_FILE) as f:
    print('posi file')
    logging.info('log')
    with open(POSI_FILE, "w") as f2:
        print('inside')
        logging.info('inside')
        for line in f:
            #print(line)
            if "POSI:" in line:
                #print(line)
                #logging.info(line)
                f2.write(line)
        print('copped')
        # logging.info('looped')
        f.close()
        f2.close()
        # logging.info('f2 closed')
        os.remove(LOGGING_FILE)

# uploading image to OCI

proxies = {}

if os.path.exists(PROXY_FILE):
    proxy_file = open(PROXY_FILE, 'r')
    proxyline = proxy_file.readline()
    if proxyline:
        proxies = {'https' : proxyline[14:-1]}

# the serial number is used to upload images

serialNumber = '00000000'
try:
    serialFile = open(SERIAL_FILE)
    serialNumber = serialFile.readline()
    serialFile.close()
    if len(serialNumber) > 10:
        serialNumber = serialNumber[-9:-1]
    else:
        serialNumber = '00000000'
except:
    logging.critical('Could not open serial file %s', SERIAL_FILE)

softwareVersion = 'v2.0-43' # default value
try:
    versionFile = open('/etc/amdi-release')
    softwareVersion = versionFile.readline()
    versionFile.close()
except:
    logging.warn('could not read release file %s', VERSION_FILE)

recipeVersion = diskQRCode[0:13]
hardwareModel = HARDWARE_MODEL
diagnosticsUUID = '0000-0000-0000-0000'
imageAnalysis = 'None'

preigg_path = '/data/' + "pre_" + diskQRCode + "_igg.tif"
preigm_path = '/data/' + "pre_" + diskQRCode + "_igm.tif"
igg_path    = '/data/' + diskQRCode + "_igg.tif"
igm_path    = '/data/' + diskQRCode + "_igm.tif"

for tif_file_name in [igg_path, preigg_path, igm_path, preigm_path]:
    print("Processing Image : ", Path(tif_file_name).name)
    logging.info('processing %s', tif_file_name)
    # rename file to .old first, write to regular name
    try:
        os.rename(tif_file_name, tif_file_name + ".old")
    except FileNotFoundError:
        logging.error('file %s not found', tif_file_name)
        test_failed = 201
        continue

    # open old pre-tagged file
    tif_file = tifftools.read_tiff(tif_file_name + ".old")
    tif_file['ifds'][0]['tags'][tifftools.Tag.HostComputer.value] = {'data': serialNumber, 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.Software.value] = {'data': softwareVersion[:-1], 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.Artist.value] = {'data': diskQRCode, 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.DateTime.value] = {'data': get_time(), 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.ImageDescription.value] = {'data': patientQRCode, 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.Make.value] = {'data': recipeVersion, 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.Model.value] = {'data': hardwareModel, 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.PageName.value] = {'data': diagnosticsUUID, 'datatype': tifftools.Datatype.ASCII}
    tif_file['ifds'][0]['tags'][tifftools.Tag.Copyright.value] = {'data': imageAnalysis, 'datatype': tifftools.Datatype.ASCII}

    # write to new file
    tifftools.write_tiff(tif_file, tif_file_name)
    # remove old
    os.remove(tif_file_name + ".old")

    # upload
    if os.path.isfile('/data/testing'):
        print('Test mode: not uploading images (otherwise remove /data/testing)')
    else:
        files = {'image': (Path(tif_file_name).name, open(tif_file_name, 'rb'),
                           'application/octet-stream')}

        response = None

        if SEND_DIRECTLY == 1:
            # send directly (makes test end a lot longer or send to pending for eventual upload every few minutes
            try:
                response = requests.post(OCI_APIGW_URL, data={'qrcode' : serialNumber},
                                         files=files,
                                         proxies=proxies)
                # For testing, uncomment next two lines
                response = None
                raise(requests.exceptions.ConnectionError)
            except requests.exceptions.ProxyError:
                logging.warn('Proxy Error %s' % (proxies.get('https')))
                logging.warn('Failed to upload %s so moved it to the pending upload area.'
                             % Path(tif_file_name).name)
                shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                test_failed = 200
            except requests.exceptions.HTTPError as errh:
                logging.warn('Http Error: %s' % str(errh))
                logging.warn('Failed to upload %s so moved it to the pending upload area.'
                             % Path(tif_file_name).name)
                shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                test_failed = 200
            except requests.exceptions.ConnectionError as errc:
                logging.warn('Error Connecting: %s' % format(errc))
                logging.warn('Failed to upload %s so moved it to the pending upload area.'
                             % Path(tif_file_name).name)
                shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                test_failed = 200
            except requests.exceptions.Timeout as errt:
                logging.warn('Timeout Error: %s' % str(errt))
                logging.warn('Failed to upload %s so moved it to the pending upload area.'
                             % Path(tif_file_name).name)
                shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                test_failed = 200
            except requests.exceptions.RequestException as err:
                logging.warn('Unexpected error: %s' % str(err))
                logging.warn('Failed to upload %s so moved it to the pending upload area.'
                             % Path(tif_file_name).name)
                shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                test_failed = 200
            except Exception as bad:
                logging.warn('Bad things happened: %s' % str(bad))
                logging.warn('Failed to upload %s so moved it to the pending upload area.'
                             % Path(tif_file_name).name)
                shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                test_failed = 200

            if response:
                if response.status_code == 200:
                    logging.info('Uploaded %s' % Path(tif_file_name).name)
                    os.remove(tif_file_name)
                else:
                    logging.warn('Failed to upload %s so moved it to the pending upload area'
                                 % Path(tif_file_name).name)
                    shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)
                    test_failed = 200
            else:
                test_failed = 200
        else:
            # Indirect upload through the pending directory
            logging.info('Copied %s to pending upload area directly' % tif_file_name)
            shutil.move(tif_file_name, PENDING_DIR + Path(tif_file_name).name)


if os.path.isfile('/data/testing'):
    pass
else:
    logging.info('post-test diagnostic')
    os.system("/usr/bin/python3 /opt/amdi/bin/run_diag.py")

if test_failed:
    msg = 'Exited cleanly with %d' % test_failed
    logging.info('Test complete. ' + msg)

sys.exit(test_failed)
