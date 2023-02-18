#!/usr/bin/python3
import time
import threading
import serial
import logging
import sys
from logging.handlers import TimedRotatingFileHandler
 
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

logname = 'log.txt'

handler = TimedRotatingFileHandler(logname, when='h', interval=1, utc=True)
logger.addHandler(handler)


def dolog(device, baudrate):
    ser = serial.Serial(device, baudrate, timeout=20)

    while True:
        reading = ser.readline()
        if (len(reading) > 0):
            data = str(time.time()) + ',' + str(reading[:-1],'utf-8')
            print(data)
            logger.info(data)


devices = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']

baudrates = [9600, 115200]

success = False
for port in devices:
    for baud in baudrates:
        try:
            print("I am Trying", port, baud, "baud")
            ser = serial.Serial(port, baud, timeout=20)
            # reset device
            ser.setDTR(False)
            time.sleep(0.1)
            ser.setDTR(True)
            time.sleep(0.1)
            ser.setDTR(False)
            time.sleep(0.1)
            ser.flushInput()
            reading = ser.readline()
            print(reading)
            ser.close()
            if (len(reading) == 0):
                continue
            if ((str(reading,'utf-8')[0]=='$') or (str(reading,'utf-8')[0]=='#')):
                success = True
                print('Starting thread', port, baud)
                thread = threading.Thread(target=dolog, args=(port, baud, ))
                thread.start()
                break
        except:
            pass                    
if (not success):
    sys.exit("I can not find any tty device.")

while True:
    #sys.stdio.flush()
    None
