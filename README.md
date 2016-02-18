# Simple Wireless Sensor Network using TI MSP430 LaunchPad with TI CC110L RF BoosterPack
<br>
![demo_setup](https://github.com/odelayIO/CC110L_IoT_EVAL/blob/master/images/demo_setup.jpg)
<br>
<br>
<br>

## Introduction
This is the source code used to evaluate the TI CC110L Chipset.  The objective of this evaluation was to determine the maximum distance the signal could propagate using the lowest data rate, at the max transmit power.  In addition, this testbed was used to explore simple protocol(s) to implement guaranteed wireless communication packet system.
This project implements the following features:
1.	Wireless Link using the CC110L in the ISM band + MSP430G2553
2.	Implements a simple SEND-ACK protocol
3.	Implements a Flash programing feature to program device address 
4.	Implements a simple packet struct with a Python host interface
5.	Creates a very simple interrupt interface on wireless sensor to indicate a packet transmission
<br>
<br>

## Acronyms, Abbreviations, Terms and Definitions
HAP : Home Automation Project
RSSI : Received Signal Strength Indicator (dBm)
LQI : Link Quality Indicator
<br>
<br>

## Evaluation Testbed
This testbed has the following components: 
1.	hapTriggerSensor
  * The sensor will transmit the status when one of the two scenario:
    * An Interrupt on the pin of MSP430 Microcontroller
    * The hapSimpleHub requests an update
2.	hapSimpleHub
  * This receives the packets from a number of wireless sensors and sends status to the computer via the serial port. The status information includes: RSSI, LQI, Temperature, and a counter.  The counter is used to determine if the hapSimpleHub missed a packet.
3.  Host
  * This testbed used a Raspberry PI as the host.  The host application was written in Python to display and record the status information for all received packets.
<br>
<br>

## Components

[TI CC110L RF BoosterPack](http://www.ti.com/tool/430boost-cc110l)

[TI MSP430 LaunchPad Value Line Development Kit](http://www.ti.com/tool/msp-exp430g2)

[TI IDE (similar to Arduino IDE)](http://energia.nu/)
<br>
<br>

## CC110L Energia Library
The Energia IDE was used to compile and program the MSP430 LaunchPad using the CC110L Energia Library.  I made some slight changes to the original CC110L library to increase transmit power and to lower the data rate to improve link performance.  The CC110L has been provided in the Github repository.
<br>
<br>

## Overview of the Wireless Protocol
The wireless protocol uses acknowledgments, therefore the hapTriggerSensor will attempt to re-transmit the packet several times if an acknowledgement packet was not received.  This will evaluate the link performance when operating at the edge of service.  The hapSimpleHub sends status information through the serial port when a received packet has been demodulated without errors.
A simple Python script communicates to the hapTiggerSensor through the serial port.  The Python script could be hosted on a Windows PC, but for this experiment a Raspberry PI was used as the host computer.
<br>
<br>
<br>
****
[odelay.io](http://odelay.io) : Meaning to agree wholeheartedly or to express enthusiasm for an idea or object. Equivolent to "Right on" or "Cool".

