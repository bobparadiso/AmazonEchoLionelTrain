//running on Feather HUZZAH ESP8266
//using WeMo emulator code by Aruna Tennakoonfrom: https://github.com/kakopappa/arduino-esp8266-alexa-multiple-wemo-switch

#include "NRF24L01.h"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <functional>
#include "switch.h"
#include "UpnpBroadcastResponder.h"
#include "CallbackFunction.h"

//==-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-
#define TX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  32

#define MESSAGE_LEN (TX_ADR_WIDTH + TX_PLOAD_WIDTH)
#define NUM_MESSAGES 2

uint8_t msgs_fwd[] = {
0x11,0x27,0x21,0x94,0x2A,0x1B,0x95,0x51,0x00,0x00,0x7F,0xFF,0xB7,0xE1,0x5B,0x4A,0x75,0x47,0x2D,0x76,0x14,0xAA,0xBA,0xA2,0xF5,0x55,0x55,0x55,0x55,0x56,0xD6,0x22,0x4E,0x43,0x28,0x54,0x06,
0x11,0x27,0x21,0x94,0x2A,0x18,0x95,0x51,0x02,0x80,0x7D,0x7F,0xC6,0xAB,0xB5,0x5D,0x6C,0x9C,0x2A,0xAA,0x92,0xC5,0x55,0x35,0xD0,0x55,0x55,0x55,0x55,0x7D,0xFC,0x44,0x9C,0x86,0x50,0xA8,0x00,
};

uint8_t msgs_rev[] = {
0x11,0x27,0x21,0x94,0x2A,0x1B,0x95,0x51,0x00,0x00,0x7F,0xFF,0xB7,0xE1,0x5B,0x5B,0x03,0x25,0x4D,0x0C,0x8A,0x89,0xAA,0xAB,0xF5,0x55,0x55,0x55,0x55,0x76,0xFE,0x22,0x4E,0x43,0x28,0x54,0x06,
0x11,0x27,0x21,0x94,0x2A,0x18,0x95,0x11,0x02,0x00,0x7D,0xFF,0xF5,0x01,0x52,0xB9,0xA1,0x02,0x8B,0x75,0x0A,0xC5,0x75,0xF5,0xD0,0x2A,0xAA,0xAA,0xAA,0xAE,0xFE,0x22,0x4E,0x43,0x28,0x54,0x00,
};

uint8_t msgs_stop[] = {
0x11,0x27,0x21,0x94,0x2A,0x1B,0x95,0x51,0x00,0x00,0x7F,0xFF,0xB7,0xE1,0x51,0x8D,0xA6,0x55,0xA2,0xEF,0xAA,0xAA,0xAA,0xA2,0xD5,0x55,0x55,0x55,0x55,0x56,0xD5,0x11,0x27,0x21,0x94,0x2A,0x03,
0x11,0x27,0x21,0x94,0x2A,0x18,0x95,0x51,0x00,0x00,0x7F,0xFF,0xE7,0x73,0xBD,0xBB,0x0D,0x20,0xD2,0x6D,0x14,0x55,0x57,0xF5,0xEA,0xAA,0xAA,0xAA,0xAA,0xAE,0xD5,0x11,0x27,0x21,0x94,0x2A,0x00,
};
//==-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-

unsigned char TX_ADDRESS[TX_ADR_WIDTH];

unsigned char rx_buf[TX_PLOAD_WIDTH] = {0}; // initialize value
unsigned char tx_buf[TX_PLOAD_WIDTH] = {0};

//==-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-
// Change these before you flash
#define FIRST_RUN
const char* ssid = "MOTOROLA-5A281";
const char* password = "18f988078e95b9883401";
//const char* ssid = "TP-LINK_B102A4";
//const char* password = "36572816";

#define NUM_SWITCHES 3
UpnpBroadcastResponder upnpBroadcastResponder;
Switch *sw[NUM_SWITCHES];
//==-=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=--=-=-=-=-=-=-=-=-

//
void setup() 
{
	pinMode(CE,  OUTPUT);
	pinMode(SCK, OUTPUT);
	pinMode(CSN, OUTPUT);
	pinMode(MOSI,  OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(IRQ, INPUT);
	//  attachInterrupt(1, _ISR, LOW);// interrupt enable
	Serial.begin(115200);
	init_io();                        // Initialize IO port
	unsigned char status=SPI_Read(STATUS);
	Serial.print("status = ");    
	Serial.println(status,HEX);     // There is read the mode’s status register, the default value should be ‘E’
	Serial.println("*******************TX_Mode Start****************************");
	TX_Mode();                       // set TX mode

	setupWiFi();
	setupSwitches();
	Serial.println("ready");
}

//
void sendMessage(uint8_t *msg, uint8_t payloadLen, uint8_t padding)
{
    //using addr to hold beginning of msg, must be reversed though
    for (int i = 0; i < TX_ADR_WIDTH; i++)
      TX_ADDRESS[i] = msg[TX_ADR_WIDTH-1-i];
    
    TX_Mode(); //apply new addr

    //padding
    for(int i = 0; i<32; i++) tx_buf[i] = padding;

    for(int i = 0; i < payloadLen; i++)
      tx_buf[i] = msg[TX_ADR_WIDTH + i];

    unsigned char status = SPI_Read(STATUS);                   // read register STATUS's value
    //if(status&TX_DS)                                           // if receive data ready (TX_DS) interrupt
    SPI_RW_Reg(FLUSH_TX,0);                                  
    SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);       // write playload to TX_FIFO
    SPI_RW_Reg(WRITE_REG+STATUS,status);                     // clear RX_DR or TX_DS or MAX_RT interrupt flag
}

//
uint8_t *msgs = NULL;
void loop() 
{
	if (WiFi.status() != WL_CONNECTED)
		return;
	
	upnpBroadcastResponder.serverLoop();
	for (int i = 0; i < NUM_SWITCHES; i++)
		sw[i]->serverLoop();

	if (Serial.available())
	{
		char c = Serial.read();
		switch (c)
		{
		case 'f': msgs = msgs_fwd; Serial.println("forward"); break;
		case 'r': msgs = msgs_rev; Serial.println("reverse"); break;
		case 's': msgs = msgs_stop; Serial.println("stop"); break;
		}
	}

	if (!msgs)
		return;

	for (int i = 0; i < NUM_MESSAGES; i++)
	{    
		sendMessage(msgs + MESSAGE_LEN*i, 32, 0xff);
		delay(1);
	}

	delay(10);
}

// connect to wifi – returns true if successful or false if not
void setupWiFi()
{
	boolean state = true;

#ifdef FIRST_RUN
	Serial.println("configuring WiFi chip");
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);	  
	WiFi.begin(ssid, password);
	Serial.println("WiFi chip configured");
#endif
	
	Serial.println("");
	Serial.println("Connecting to WiFi");

	// Wait for connection
	Serial.print("Connecting ...");
	while (WiFi.status() != WL_CONNECTED)
	{
		delay(100);
		Serial.print(".");
	}

	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
}

//
void setupSwitches()
{
	upnpBroadcastResponder.beginUdpMulticast();

	// Define your switches here. Max 14
	// Format: Alexa invocation name, local port no, on callback, off callback
	sw[0] = new Switch("train", 80, trainOn, trainOff);
	sw[1] = new Switch("forward", 81, forwardOn, trainOff);
	sw[2] = new Switch("reverse", 82, reverseOn, trainOff);

	Serial.println("Adding switches upnp broadcast responder");
	for (int i = 0; i < NUM_SWITCHES; i++)
		upnpBroadcastResponder.addDevice(*sw[i]);
}

//
void trainOn()
{
	Serial.print("train turn on ...");
	msgs = msgs_fwd;
}

//
void trainOff()
{
	Serial.print("train turn off ...");
	msgs = msgs_stop;
}
//
void forwardOn()
{
	Serial.print("forward turn on ...");
	msgs = msgs_fwd;
}

//
void reverseOn()
{
	Serial.print("reverse turn on ...");
	msgs = msgs_rev;
}

//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
  digitalWrite(IRQ, 0);
  digitalWrite(CE, 0);			// chip enable
  digitalWrite(CSN, 1);                 // Spi disable	
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  unsigned char i;
  for(i=0;i<8;i++)                      // output 8-bit
  {
    if(Byte&0x80)
    {
      digitalWrite(MOSI, 1);
    }
    else
    {
      digitalWrite(MOSI, 0);
    }
    digitalWrite(SCK, 1);
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(MISO) == 1)
    {
      Byte |= 1;       	                // capture current MISO bit
    }
    digitalWrite(SCK, 0);
  }
  return(Byte);           	        // return read unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(CSN, 0);                   // CSN low, init SPI transaction
  status = SPI_RW(reg);                   // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(CSN, 1);                   // CSN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(CSN, 0);           // CSN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  digitalWrite(CSN, 1);          // CSN high, terminate SPI communication
  
  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSN, 0);                  // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(CSN, 1);                   // Set CSN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(CSN, 0);                  // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                   // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: TX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * TX mode, set TX address, set RX address for auto.ack,
 * fill TX payload, select RF channel, datarate & TX pwr.
 * PWR_UP is set, CRC(2 unsigned chars) is enabled, & PRIM:TX.
 * 
 * ToDo: One high pulse(>10us) on CE will now send this
 * packet and expext an acknowledgment from the RX device.
 **************************************************/
void TX_Mode(void)
{
  digitalWrite(CE, 0);

  SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01

  //disable ESB
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);
  
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, 02);        // Select RF channel 02

  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0b00000110);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0b00000010);     // Set PWR_UP bit, Prim:TX

  digitalWrite(CE, 1);

}
