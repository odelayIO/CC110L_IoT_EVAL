#!/usr/bin/env python

#############################################################################################
#############################################################################################
#
#   The MIT License (MIT)
#   
#   Copyright (c) 2016 http://odelay.io 
#   
#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:
#   
#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.
#   
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.
#   
#   Contact : <everett@odelay.io>
#   
#   Description : 
#     Listens to the commands sent from SimpleHub and writes to log file.
#
#   Version History:
#   
#       Date        Description
#     -----------   -----------------------------------------------------------------------
#      19JUN2014     Original Creation
#      17FEB2016     Added MIT License and Posted on Github 
#
#############################################################################################
#############################################################################################

import serial
import signal
import sys
import time
import datetime

FILE_NAME = 'hapSimpleHub.log';

# Write File Header
f = open(FILE_NAME,'a')


# Open up COM4 as Slave
slave = serial.Serial()
slave.port = "/dev/ttyAMA0"
slave.baudrate = 9600
slave.timeout = 1



def signal_handler(signal, frame):
	print 'You Pressed Ctrl-C!'
	slave.close()
	f.close()
	sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
print 'Press CTRL-C to exit'
print ' '
print ' '


# open serial port
slave.open()

iteration = 0;
node = 0;
while(True):

	a = slave.readline()
	if (len(a) > 10):
		#------------------------------------
		# get timestamp
		#------------------------------------
		ts = time.time()
		st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')

		#------------------------------------
		# Print data to terminal
		#------------------------------------
		print "The length of the serial string: " + str(len(a))
		print " "
		print "Address of Sensor (Src): " + str(hex(ord(a[0]))) + ", RSSI: " + str(-1*ord(a[1])) + "dBm"
		print "Address of Hub (Des): " + str(hex(ord(a[2]))) + ", RSSI: " + str(-1*ord(a[3])) + "dBm"

		temperature = (1.0*(ord(a[4])*2**24 + ord(a[5])*2**16 + ord(a[6])*2**8 + ord(a[7])))/10;
		print "Temperature of Self: " + str(temperature) + "F"
		print "Command: " + str(hex(ord(a[8])))
		print "Status: " + str(hex(ord(a[9])))

		print " "
		print " "
		for i in range(0, len(a)):
			print "a[" + str(i) + "]: " + str(hex(ord(a[i])))

		print " "
		print " "

		#------------------------------------
		# Write the file
		#------------------------------------
		f.write("-----------------------------------------------------------------------------\n");
		f.write("                        ***********************                              \n");
		f.write("                          Iteration = " + str(iteration) + "                 \n");
		f.write("                        ***********************                              \n");
		f.write("  Time: " + st + "\n")
		f.write("\n\n");
		f.write("  The length of the serial string: " + str(len(a)) + "\n");
		f.write("\n");
		f.write("  Address of Sensor (Src): " + str(hex(ord(a[0]))) + ", RSSI: " + str(-1*ord(a[1])) + "dBm\n");
		f.write("  Address of Hub (Des): " + str(hex(ord(a[2]))) + ", RSSI: " + str(-1*ord(a[3])) + "dBm\n");
		f.write("  Temperature of Self: " + str(temperature) + "F\n");
		f.write("  Command: " + str(hex(ord(a[8]))) + "\n");
		f.write("  Status: " + str(hex(ord(a[9]))) + "\n");
		f.write("\n");
		f.write("-----------------------------------------------------------------------------\n");
		f.write("                             ############                                    \n");
		f.write("                             ############                                    \n");
		f.flush()

	#------------------------------------
	# Sleep for a second
	#------------------------------------
	time.sleep(1);

	#------------------------------------
	# Now select a different node
	#------------------------------------
	if(node == 0):
		node = 1;
		command = bytearray([0xA1,0x10]);
	elif(node == 1):
		node = 2;
		command = bytearray([0xA1,0x20]);
	elif(node == 2):
		node = 0;
		command = bytearray([0xA1,0x30]);

	#------------------------------------
	# Write the command to the hub
	#------------------------------------
	slave.write(command);

	#------------------------------------
	# Increment the counter
	#------------------------------------
	iteration = iteration + 1;

