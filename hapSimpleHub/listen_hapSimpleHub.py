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
#     Listens to the commands sent from SimpleHub and displays information on terminal.
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

# If True, hub will poll nodes for status
TRIGGER_STATUS = False

# Open up COM4 as Slave
slave = serial.Serial()
#slave.port = "/dev/ttyAMA0"  # RPi test header
#slave.port = "/dev/ttyUSB2"  # RPi USB Connector
slave.port = "COM7"           # Windows Machine
#slave.port = "/dev/ttyACM0"
slave.baudrate = 9600
slave.timeout = 1


class PacketStruct():
  def __init__(self):
    self.src_address = 0
    self.src_rssi    = 0
    self.des_address = 0
    self.des_address = 0
    self.node_temp   = 0 
    self.command     = 0
    self.status      = 0
    self.msg         = 0 


def ProcessPacket(b,a):
  temperature   = (1.0*(ord(a[4])*2**24 + ord(a[5])*2**16 + ord(a[6])*2**8 + ord(a[7])))/10;
  b.src_address = ord(a[0])
  b.src_rssi    = -1*ord(a[1])  # dBm
  b.des_address = ord(a[2])
  b.des_rssi    = -1*ord(a[3]) # dBm
  b.node_temp   = temperature  # F
  b.command     = ord(a[8])
  b.status      = ord(a[9])
  b.msg         = []
  for i in range(10, len(a)-1):
    b.msg.append(ord(a[i]))

  return b
  


def signal_handler(signal, frame):
  print 'You Pressed Ctrl-C!'
  slave.close()
  sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
print 'Press CTRL-C to exit'
print ' '
print ' '


# open serial port
slave.open()

iteration = 0;
node = 0;
packet =  PacketStruct()

while(True):

  a = slave.readline()
  if (len(a) > 10):

    #------------------------------------
    # Process Packet
    #------------------------------------
    packet = ProcessPacket(packet,a)


    #------------------------------------
    # get timestamp
    #------------------------------------
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    print "Time: " + str(st)

    #------------------------------------
    # Print data to terminal
    #------------------------------------
    print "The Number of Bytes in Received Packet: " + str(len(a))
    print " "
    print "Address of Sensor (Src): " + str(hex(packet.src_address)) + ", RSSI: " + str(packet.src_rssi) + "dBm"
    print "Address of Hub (Des): " + str(hex(packet.des_address)) + ", RSSI: " + str(packet.des_rssi) + "dBm"
    print "Temperature of Node: " + str(packet.node_temp) + "F"
    print "Command: " + str(hex(packet.command))
    print "Status: " + str(hex(packet.status))
    print "Received Message: " + str(packet.msg)
    print " "
    print " "



  #------------------------------------
  # Now select a different node
  #------------------------------------
  if(TRIGGER_STATUS == True):
    #------------------------------------
    # Sleep for a second
    #------------------------------------
    time.sleep(1);

    if(node == 0):
      node = 1;
      command = bytearray([0xA1,0x10]);
    elif(node == 1):
      node = 2;
      command = bytearray([0xA1,0x20]);
    elif(node == 2):
      node = 3;
      command = bytearray([0xA1,0x30]);
    elif(node == 4):
      node = 5;
      command = bytearray([0xA1,0x40]);
    elif(node == 5):
      node = 0;
      command = bytearray([0xA1,0x50]);

    #------------------------------------
    # Increment the counter
    #------------------------------------
    iteration = iteration + 1;

