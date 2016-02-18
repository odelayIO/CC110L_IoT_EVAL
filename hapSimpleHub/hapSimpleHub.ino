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
 *  Description : Polls TriggerSensors and transmits data to serial port.
 *
 *  Version History:
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

#if defined(__MSP430G2553__) 
#else
#error Board not supported
#endif

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

#define ADDRESS_LOCAL		ADDRESS_HUB


#define SEND_STATUS			0xA0
#define POLL_STATUS			0xA1
#define ACK_PACKET			0xEE

#define MSG_SIZE				10	// 8 bytes + return + lf

// -----------------------------------------------------------------------------
//
//   Global data
// 
// -----------------------------------------------------------------------------
uint8_t AVERAGE		= 10;
uint8_t i					=  0;
uint8_t ack				=  0;
uint32_t IntDegF	=  0;
uint32_t IntDegC	=  0;
uint8_t serialBuf[15];


/**
 *  sPacket - packet format.
 */
struct sPacket
{
  uint8_t		src;								// 1, Local node address that message originated from
  uint8_t		des;								// 2, Local node address that message originated from
	uint8_t		rssi;								// 3, Local RSSI 
	uint32_t	temp;								// 7, Local Temperature
	uint8_t		cmd;								// 8, Received Command
	uint8_t		status;							// 9, Status of the Sensor xFF = Closed, x00 = Open
  uint8_t		message[MSG_SIZE];  // 12, Local node message [MAX. 58 bytes]
};


//--------------------------------------------------
// Define the Packet Struct
//--------------------------------------------------
struct sPacket rxPacket;
struct sPacket txPacket;


// -----------------------------------------------------------------------------
// Main example
// -----------------------------------------------------------------------------
void setup()
{
  // The radio library uses the SPI library internally, this call initializes
  // SPI/CSn and GDO0 lines. Also setup initial address, channel, and TX power.
  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);

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
  txPacket.src									= ADDRESS_LOCAL;
  txPacket.des									= 0;
	txPacket.rssi									= 0;
	txPacket.temp									= 0;
	txPacket.cmd									= 0;
	txPacket.status								= 0;
	txPacket.message[MSG_SIZE-2]	= '\n';
	txPacket.message[MSG_SIZE-1]	= '\r';
 
	// Setup the Internal Temperature Sensor
  analogReference(INTERNAL1V5);
  analogRead(A10); // first reading usually wrong
	
 
  // Setup serial for debug printing.
  Serial.begin(9600);

	//--------------------------------------------------
  //  Setup LED for example demonstration purposes.
	// 
  //  Note: Set radio first to ensure that GDO2 
	//  line isn't being driven by the 
  //  MCU as it is an output from the radio.
	//--------------------------------------------------
  pinMode(RED_LED, OUTPUT);       // Use red LED to display message reception
  digitalWrite(RED_LED, LOW);
	delay(1000);
	Serial.println("Ready.\n");
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
// Print a 32-bit value, 4 HEX values
//--------------------------------------------------
void print_uint32(uint32_t ui) {
	Serial.write(ui>>24);
	Serial.write(ui>>16);
	Serial.write(ui>>8);
	Serial.write(ui>>0);
}

//--------------------------------------------------
// Calculate the temperature
//--------------------------------------------------
uint32_t getIntTempF(void) {
	uint32_t t = 0;

	for(i=0; i<AVERAGE; ++i) {

    //----------------------------------------------------------------------
    // Fahrenheit
    //----------------------------------------------------------------------
    t += ((uint32_t)analogRead(A10)*48724 - 30634388) *10 >> 16;

  }

	return t/AVERAGE;
}

//--------------------------------------------------
// Send ACK packet to HUB
//--------------------------------------------------
void sendAck() {
  txPacket.src		= ADDRESS_LOCAL;
  txPacket.des		= rxPacket.src;
	txPacket.rssi		= Radio.getRssi();
	txPacket.temp		= getIntTempF();
	txPacket.cmd		= ACK_PACKET;
	txPacket.status = 0x00;
  Radio.transmit(txPacket.des, (unsigned char*)&txPacket, sizeof(txPacket));
  while(Radio.busy());
}


//--------------------------------------------------
// Process Incoming Serial Port Buffer
//--------------------------------------------------
void processSerialBuf() {

	if(serialBuf[0] == POLL_STATUS) {
		txPacket.src		= ADDRESS_LOCAL;
		txPacket.des		= serialBuf[1];
		txPacket.rssi		= 0;
		txPacket.temp		= 0;
		txPacket.cmd		= SEND_STATUS;
		txPacket.status = 0x00;
		Radio.transmit(txPacket.des, (unsigned char*)&txPacket, sizeof(txPacket));
		while(Radio.busy());
	}

}



void loop()
{
	int rxRssi = 0;

	//--------------------------------------------------
	// Wait for the radio to RX a signal.  If a packet
	// is received, then process packet.  If no packet
	// received, then check if an interrupt occourred.
	// Or if a command came through from the RS232.
	//--------------------------------------------------
  if (Radio.receiverOn((unsigned char*)&rxPacket, sizeof(rxPacket), 3000) > 0)
  {
		if(rxPacket.des == ADDRESS_LOCAL) 
		{
			// turn on RED LED
			digitalWrite(RED_LED, HIGH);

			//--------------------------------------------------
			// KLUDGE: Save the RX RSSI
			//--------------------------------------------------
			rxRssi	= Radio.getRssi();
			rxRssi  = rxRssi * -1;

			//--------------------------------------------------
			// Send ACK
			//--------------------------------------------------
			sendAck();

			//--------------------------------------------------
			// Now send the packet to the host 
			//--------------------------------------------------
			Serial.write(rxPacket.src);
			Serial.write(rxPacket.rssi);
			Serial.write(rxPacket.des);
			Serial.write(rxRssi);  
			print_uint32(rxPacket.temp);
			Serial.write(rxPacket.cmd);
			Serial.write(rxPacket.status);
			for(i=0; i<MSG_SIZE+1; i++) 
			{
				Serial.write(rxPacket.message[i]);
			}

			// Send terminated Char
			//Serial.print('\n');
			//Serial.print('\r');

			// trun off RED LED
			digitalWrite(RED_LED, LOW);
		}
  }
	//--------------------------------------------------
	// Check if anything came through RS232 Interface
	//--------------------------------------------------
	else if(Serial.available() > 0) 
	{
		// turn on RED LED
		digitalWrite(RED_LED, HIGH);

		// Serial Flush
		i=0;
		while(Serial.available()) 
		{
			serialBuf[i] = Serial.read();
			i = i + 1;
		}
		processSerialBuf();

			
		// trun off RED LED
		digitalWrite(RED_LED, LOW);
	}
}

