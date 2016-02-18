/******************************************************************************************
 ******************************************************************************************
 *  
 *  The MIT License (MIT)
 *  
 *  Copyright (c) 2016 http://odelay.io 
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 *  Contact : <everett@odelay.io>
 *  
 *  Description : Simple Trigger Sensor to send data to SimpleHub.
 *
 *  Version History :
 *  
 *      Date        Description
 *    -----------   -----------------------------------------------------------------------
 *     19JUN2014     Original Creation
 *     17FEB2016     Added MIT License and Posted on Github 
 *
 *
 ******************************************************************************************
 ******************************************************************************************/


#include <SPI.h>
#include <AIR430BoostFCC.h>
#include "MspFlash.h"

#define flash SEGMENT_D

// -----------------------------------------------------------------------------
//
//  Defines, enumerations, and structure definitions
//
// -----------------------------------------------------------------------------

#define ADDRESS_HUB				0xAA

#define ADDRESS_NODE_A		0x10
#define ADDRESS_NODE_B		0x20
#define ADDRESS_NODE_C		0x30
#define ADDRESS_NODE_D		0x40
#define ADDRESS_NODE_E		0x50


#define SEND_STATUS			0xA0
#define ACK_PACKET			0xEE

#define MSG_SIZE				10	

// -----------------------------------------------------------------------------
//
//   Global data
// 
// -----------------------------------------------------------------------------
uint8_t AVERAGE					= 10;
uint8_t i								= 0;
uint32_t IntDegF				= 0;
uint32_t IntDegC				= 0;
volatile int sendStatus	= LOW;
volatile uint8_t ackRx	= true;


//--------------------------------------------------
// Define the Packet Struct
//--------------------------------------------------
struct sPacket
{
  uint8_t		src;								// Local node address that message originated from
  uint8_t		des;								// Local node address that message originated from
	uint8_t		rssi;								// Local RSSI 
	uint32_t	temp;								// Local Temperature
	uint8_t		cmd;								// Received Command
	uint8_t		status;							// Status of the Sensor xFF = Closed, x00 = Open
  uint8_t		message[MSG_SIZE];  // Local node message [MAX. 58 bytes]
};


struct sPacket txPacket;
struct sPacket rxPacket;


// -----------------------------------------------------------------------------
// Main setup() function
// -----------------------------------------------------------------------------

void setup()
{
  // Setup serial for debug printing.
  Serial.begin(9600);

  // Local Address from Flash space
  Flash.read(flash, &txPacket.src, 1);
  Serial.write("Local Address : ");
  Serial.write(txPacket.src);
	Serial.write("\n\r");

  // Hub Address from Flash space
  Flash.read(flash+1, &txPacket.des, 1);
  Serial.write("Local Address : ");
  Serial.write(txPacket.des);
	Serial.write("\n\r");

  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
  //Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);
  Radio.begin(txPacket.src, CHANNEL_1, POWER_MAX);

	// Allocate memory to rx/tx buffers
  memset(rxPacket.message, 0, sizeof(rxPacket.message));
  memset(txPacket.message, 0, sizeof(txPacket.message));

	// Rx Buffer
  rxPacket.src									= 0;
  rxPacket.des									= 0;
	rxPacket.rssi									= 0;
	rxPacket.temp									= 0;
	rxPacket.cmd									= 0;
	rxPacket.status								= 0;
	rxPacket.message[MSG_SIZE-2]	= '\n';
	rxPacket.message[MSG_SIZE-1]	= '\r';

	// Tx Buffer
  //txPacket.src									= ADDRESS_LOCAL;
  //txPacket.des									= 0;
	txPacket.rssi									= 0;
	txPacket.temp									= 0;
	txPacket.cmd									= 0;
	txPacket.status								= 0;
	txPacket.message[MSG_SIZE-2]	= '\n';
	txPacket.message[MSG_SIZE-1]	= '\r';
 
	// Debuging, setup the sensor pin on buttonPin
	pinMode(PUSH2, INPUT_PULLUP);

	// Now attach an interrupt to the PUSH2 pin
	attachInterrupt(PUSH2, sensorTrig, CHANGE);  // Interrupt is fired whenever	

	// Setup the Internal Temperature Sensor
  analogReference(INTERNAL1V5);
  analogRead(A10); // first reading usually wrong
	 

	//--------------------------------------------------
  //  Setup LED for example demonstration purposes.
	// 
  //  Note: Set radio first to ensure that GDO2 
	//  line isn't being driven by the 
  //  MCU as it is an output from the radio.
	//--------------------------------------------------
  pinMode(RED_LED, OUTPUT);       // Use red LED to display message reception
  digitalWrite(RED_LED, LOW);

	// Debug
	Serial.println("Radio Up.\n");

	// Ack Received
	ackRx = true;

}



//--------------------------------------------------
// Print a decmal value for the serial port
//--------------------------------------------------
void printDec(uint32_t ui) {
  Serial.print(ui/10, DEC);
  Serial.print(".");
  Serial.print(ui%10, DEC);
}


//--------------------------------------------------
// Calculate the temperature
//--------------------------------------------------
uint32_t getIntTempF(void) {
	uint32_t t = 0;

	for(i=0; i<AVERAGE; ++i) {

    //--------------------------------------------------
    // Fahrenheit
    //--------------------------------------------------
    t += ((uint32_t)analogRead(A10)*48724 - 30634388) *10 >> 16;

  }

	return t/AVERAGE;
}


//--------------------------------------------------
// Process the received packet
//--------------------------------------------------
void processRxPacket()  {

	switch(rxPacket.cmd) {
    //--------------------------------------------------
		// Transmit the Status of the Sensor
    //--------------------------------------------------
		case SEND_STATUS:
			txStatus();
      break;
		case ACK_PACKET:
			ackRx		= true;
			break;
	}
}

//--------------------------------------------------
// Transmit the Status Struct
//--------------------------------------------------
void txStatus() {
  //txPacket.des		= ADDRESS_HUB;
	txPacket.rssi		= 256-Radio.getRssi();
	txPacket.temp		= getIntTempF();
	txPacket.cmd		= SEND_STATUS;
	txPacket.status = digitalRead(PUSH2);
  //Radio.transmit(ADDRESS_HUB, (unsigned char*)&txPacket, sizeof(txPacket));
  Radio.transmit(txPacket.des, (unsigned char*)&txPacket, sizeof(txPacket));
  while(Radio.busy());
	ackRx						= false;
}

void loop()
{
  
  
	//--------------------------------------------------
	// Wait for the radio to RX a signal.  If a packet
	// is received, then process packet.  If no packet
	// received, then check if an interrupt occourred.
	//--------------------------------------------------
  if (Radio.receiverOn((unsigned char*)&rxPacket, sizeof(rxPacket), 2000) > 0)
  {
		//--------------------------------------------------
		// Check if the packet address matches
		//--------------------------------------------------
    if(rxPacket.des == txPacket.src)
	    processRxPacket();
  }
  else if(Serial.available() > 0)
  {
    switch( Serial.read() )
    {
      case 'r' : reportID();  break;
      case 'w' : progID();    break;
      case 'e' : eraseID();   break;
      default  : displayHelp();
    }
  }
	else
	{
		//--------------------------------------------------
		// check the status of the internal FSM.
		//--------------------------------------------------
		
		//--------------------------------------------------
		// This will be true if interrupt happened
		//--------------------------------------------------
		if(sendStatus) 
		{
			txStatus();
			sendStatus = LOW;
		}
		//--------------------------------------------------
		// Check if module received ACK
		//--------------------------------------------------
		else if(ackRx == false)
		{
			txStatus();
		}
	}
}

//--------------------------------------------------
// eraseID()
//
//--------------------------------------------------
void eraseID()
{
  Serial.println("Erase Flash"); 
  Flash.erase(flash); 
  Serial.println("Done.");  
}


//--------------------------------------------------
// displayHelp()
//
// Display Help Menu
//--------------------------------------------------
void displayHelp()
{
	Serial.println("r for read current Local Address");
	Serial.println("w for write new Local Address into flash");
	Serial.println("e erase flash");
}


//--------------------------------------------------
// reportID()
//
// Displays the current ID to Serial Port
//--------------------------------------------------
void reportID()
{
	//Serial.write("ID (LOCAL)= ");
  Serial.write(txPacket.src);
	//Serial.write("\n\r");
	//Serial.write("\n\r");
	//Serial.write("ID (HUB)= ");
  Serial.write(txPacket.des);
	//Serial.write("\n\r");
	//Serial.write("\n\r");
}


//--------------------------------------------------
// progID()
//
// Program ID into Flash
//--------------------------------------------------
void progID()
{
	//Serial.write("Enter ID (LOCAL): ");
  Serial.flush();
  while(true)
  {
    if(Serial.available() > 0)
      break;
  }
  txPacket.src = Serial.read();
  Flash.write(flash, (unsigned char*)&txPacket.src,1);
	//Serial.write("\n\r");
	//Serial.write("Writing : ");
  //Serial.write(txPacket.src);
  //Serial.println("\n\rDone.\n");

	//Serial.write("Enter ID (HUB): ");
  Serial.flush();
  while(true)
  {
    if(Serial.available() > 0)
      break;
  }
  txPacket.des = Serial.read();
  Flash.write(flash+1, (unsigned char*)&txPacket.des,1);
	//Serial.write("\n\r");
	//Serial.write("Writing : ");
  //Serial.write(txPacket.des);
  //Serial.println("\n\rDone.\n");
}


//--------------------------------------------------
// This is the interrupt function.  Just setting
// the flag high at the moment, since the uC can
// not handle delay function (etc).  
//--------------------------------------------------
void sensorTrig ()
{
	sendStatus = HIGH;
}
