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
#     Simple program to configure the TriggerSensor address in Flash for the
#     MSP430G2553 device.  See below comments for list of commands.
#
#
#   Version History :
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

ADDRESS_HUB   = 0xAA
ADDRESS_LOCAL = 0x9C

CMD_WRITE  = 0x77
CMD_READ   = 0x72
CMD_CLEAR  = 0x65


# Open up COM4 as Slave
slave = serial.Serial()
#slave.port = "/dev/ttyAMA0"
slave.port = "COM16"
slave.baudrate = 9600
slave.timeout = 1

s = 'n';
if(len(sys.argv)>1):
  s = sys.argv[1];


# open serial port
slave.open()
slave.flush()
if(s=='p'):
  command = bytearray([CMD_CLEAR]);
  slave.write(command);
  time.sleep(1);
  command = bytearray([CMD_WRITE,ADDRESS_LOCAL,ADDRESS_HUB])
  slave.write(command);
  slave.flush()

elif(s=='r'):
  command = bytearray([CMD_READ]);
  slave.flush()
  slave.write(command);

  time.sleep(1);

  a = slave.readline()
  for i in range(0, len(a)):  
    print "a[" + str(i) + "]: " + str(hex(ord(a[i])))

elif(s=='e'):
  command = bytearray([CMD_CLEAR]);
  slave.write(command);
  slave.flush()

slave.close()
