#/usr/bin/python3
import time
import serial
import logging
from logging.handlers import TimedRotatingFileHandler

port = '/dev/ttyUSB1'

baud = 115200

ser = serial.Serial(port, baud, timeout=1)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

logname = "/home/ubuntu/data/geodos/geodos.csv"
handler = TimedRotatingFileHandler(logname, when='h', interval=1, utc=True)
#handler.setLevel(logging.INFO)
#handler.suffix = "%Y%m%d%H%M"
logger.addHandler(handler)

while True:
	reading = ser.readline()
	if (len(reading) > 0):
		data = str(int(round(time.time(),2))) + ',' + str(reading,'utf-8')
		#print(data)
		logger.info(data)


