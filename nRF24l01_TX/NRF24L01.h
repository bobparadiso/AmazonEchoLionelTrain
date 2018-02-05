#ifndef NRF24L01_h
#define NRF24L01_h

#include "API.H"

//---------------------------------------------
#define CE       16
// CE_BIT:   Digital Input     Chip Enable Activates RX or TX mode
#define CSN      4
// CSN BIT:  Digital Input     SPI Chip Select
#define SCK      14
// SCK BIT:  Digital Input     SPI Clock
#define MOSI     13
// MOSI BIT: Digital Input     SPI Slave Data Input
#define MISO     12
// MISO BIT: Digital Output    SPI Slave Data Output, with tri-state option
#define IRQ      5
// IRQ BIT:  Digital Output    Maskable interrupt pin
//*********************************************
#endif
