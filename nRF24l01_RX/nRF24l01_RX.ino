#include "NRF24L01.h"

//***************************************************
#define TX_ADR_WIDTH    5
#define TX_PLOAD_WIDTH  32

uint8_t addr1[] = {0x82,0x73,0x56,0x45,0xc3};
uint8_t addr2[] = {0x11,0x27,0x21,0x94,0x2a};
unsigned char TX_ADDRESS1[TX_ADR_WIDTH];
unsigned char TX_ADDRESS2[TX_ADR_WIDTH];

unsigned char rx_buf[TX_PLOAD_WIDTH];
unsigned char tx_buf[TX_PLOAD_WIDTH];
//***************************************************
void setup() 
{
  for (int i = 0; i < TX_ADR_WIDTH; i++)
  {
    TX_ADDRESS1[i] = addr1[TX_ADR_WIDTH-1-i];
    TX_ADDRESS2[i] = addr2[TX_ADR_WIDTH-1-i];
  }
  
  pinMode(CE,  OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(MOSI,  OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(IRQ, INPUT);
  //  attachInterrupt(1, _ISR, LOW); // interrupt enable
  Serial.begin(115200);
  init_io();                        // Initialize IO port
  unsigned char status=SPI_Read(STATUS);
  Serial.print("status = ");
  Serial.println(status,HEX);      // There is read the mode’s status register, the default value should be ‘E’  
  Serial.println("*****************RX_Mode start******************************R");
  RX_Mode();                        // set RX mode
}

uint32_t lastEvent;

//
void loop() 
{
  for(;;)
  {
    unsigned char status = SPI_Read(STATUS);                         // read register STATUS's value
    uint8_t pipe_num = ( status >> 1 ) & 0x07;

    uint32_t curTime = micros();
    if (pipe_num != 7)                                                 // if receive data ready (TX_DS) interrupt
    {
      //char tbuf[64];
      //sprintf(tbuf, "%6ld : ", micros() - lastEvent);
      //Serial.print(tbuf);
      
      uint8_t *rx_addr;
      if (pipe_num == 0)
        rx_addr = addr1;
      else
        rx_addr = addr2;

      for(int i=0; i<TX_ADR_WIDTH; i++)
      {
          //Serial.print("0x");
          if (rx_addr[i]<0x10) Serial.print("0");
          Serial.print(rx_addr[i],HEX);                              // print rx_buf
          Serial.print(" ");
          //Serial.print(",");
      }
      Serial.print(" : ");
      
      //memset(rx_buf, 0, TX_PLOAD_WIDTH);
      SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);             // read playload to rx_buf
      SPI_RW_Reg(FLUSH_RX,0);                                        // clear RX_FIFO
      for(int i=0; i<32; i++)
      {
          //Serial.print("0x");
          if (rx_buf[i]<0x10) Serial.print("0");
          Serial.print(rx_buf[i],HEX);                              // print rx_buf
          Serial.print(" ");
          //Serial.print(",");
      }
      Serial.println(" ");

      SPI_RW_Reg(WRITE_REG+STATUS,status);                             // clear RX_DR or TX_DS or MAX_RT interrupt flag
      lastEvent = curTime;
    }

    /*
    if (curTime - lastEvent > 1000000)
    {
      Serial.print("status:");
      Serial.println(status, BIN);
      lastEvent = curTime;
    }
    */
  }
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
      digitalWrite(MOSI, 1);    // output 'unsigned char', MSB to MOSI
    }
    else
    {
      digitalWrite(MOSI, 0);
    }
    digitalWrite(SCK, 1);                      // Set SCK high..
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(MISO) == 1)
    {
      Byte |= 1;       	                // capture current MISO bit
    }
    digitalWrite(SCK, 0);         	// ..then set SCK low again
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

  digitalWrite(CSN, 0);                   // Set CSN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(CSN, 1);                  // Set CSN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: RX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * RX Mode, set RX address, writes RX payload width,
 * select RF channel, datarate & LNA HCURR.
 * After init, CE is toggled high, which means that
 * this device is now ready to receive a datapacket.
/**************************************************/
void RX_Mode(void)
{
  digitalWrite(CE, 0);
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS1, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P1, TX_ADDRESS2, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device

  //disable ESB
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);
  
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0b00000011);  // Enable Pipes 0 and 1
  SPI_RW_Reg(WRITE_REG + RF_CH, 02);        // Select RF channel 02

  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RX_PW_P1, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0b00000110);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0b00000011);     // Set PWR_UP bit, Prim:RX
  digitalWrite(CE, 1);                             // Set CE pin high to enable RX device
}
/**************************************************/

