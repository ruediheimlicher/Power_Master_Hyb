//
//  spi_master.h
//  AVR_Starter
//
//  Created by Ruedi Heimlicher on 19.10.2014.
//
//

#ifndef AVR_Starter_spi_master_h
#define AVR_Starter_spi_master_h

//#define SPI_BUFFERSIZE 8


#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB

#define SPI_CS       3
#define SPI_SS       4

#define SPI_MOSI     5
#define SPI_MISO     6
#define SPI_SCK      7



// Schieberegister HC595

#define SPI_SR_PORT     PORTB
#define SPI_SR_PIN      PINB
#define SPI_SR_DDR      DDRB

// Strom-Shunt
#define SRA_CS          0

#define SRA_CS_HI        SPI_SR_PORT |= (1<<SRA_CS)
#define SRA_CS_LO        SPI_SR_PORT &= ~(1<<SRA_CS)

// 7-Seg-Anzeige
#define SRB_CS          1

#define SRB_CS_HI        SPI_SR_PORT |= (1<<SRB_CS)
#define SRB_CS_LO        SPI_SR_PORT &= ~(1<<SRB_CS)

// http://www.ermicro.com/blog/?p=1050
// MCP23S17 SPI Slave Device
#define SPI_SLAVE_ID    0x40
#define SPI_SLAVE_ADDR  0x00      // A2=0,A1=0,A0=0
#define SPI_SLAVE_WRITE 0x00
#define SPI_SLAVE_READ  0x01


// DAC MCP8421
#define MCP_PORT  PORTC
#define MCP_DDR   DDRC
#define MCP_DAC_U_CS 6
#define MCP_DAC_I_CS 7

#define MCP_U_CS_HI        MCP_PORT |= (1<<MCP_DAC_U_CS)
#define MCP_U_CS_LO        MCP_PORT &= ~(1<<MCP_DAC_U_CS)

#define MCP_I_CS_HI        MCP_PORT |= (1<<MCP_DAC_I_CS)
#define MCP_I_CS_LO        MCP_PORT &= ~(1<<MCP_DAC_I_CS)


//#define MCP_LOAD_PORT PORTA
//#define MCP_LOAD_DDR  DDRA
//#define MCP_LOAD  4

#define MCP_LOAD_PORT PORTC
#define MCP_LOAD_DDR  DDRC
#define MCP_LOAD  5

#define MCP_LOAD_HI        MCP_LOAD_PORT |= (1<<MCP_LOAD)
#define MCP_LOAD_LO        MCP_LOAD_PORT &= ~(1<<MCP_LOAD)

volatile uint8_t spi_rxbuffer[SPI_BUFFERSIZE];
volatile uint8_t spi_txbuffer[SPI_BUFFERSIZE];

uint8_t set_SR(uint8_t outData);
uint8_t set_SR_7Seg(uint8_t outData);
void setSPI_Teensy(void);
unsigned char SPI_get_put_char(uint8_t cData);

#endif
