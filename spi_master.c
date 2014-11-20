/**************************************************************
Es soll alle halbe Sekunde im Wechsel 0 bzw. 1 gesendet werden.
Am korrespondierenden Slave soll zur Indikation jeweils die 
LEDs an bzw. aus gehen
Verdrahtung:	MISO(Master) --> MISO(Slave)
				MOSI(Master) --> MOSI(Slave)
				SCK(Master)  --> SCK(Slave)
				PB0(Master)	 --> SS(Slave)
**************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#include "spi_master.h"
unsigned char status = 0;
volatile unsigned char count;


volatile uint8_t			spistatus=0;

#define TEENSY_SEND  7
#define TEENSY_RECV  6


uint16_t spiwaitcounter = WHILEMAX; // 5 ms

extern volatile uint8_t arraypos;


void timer1 (void);
void master_init (void);
void master_transmit (unsigned char data);

ISR (SPI_STC_vect)
{
	return;
}

/*
ISR (TIMER1_OVF_vect)
{						//Senderoutine
	if (count == 1) {
		master_transmit ('1');
		count--;
		return;
	}
	if (count == 0) {
		master_transmit ('0');
		count++;
	}
}
*/

void spi_master_init (void)
{
   /*
    #define SPI_CS       1
    #define SPI_SS       2
    
    #define SPI_MOSI     3
    #define SPI_MISO     4
    #define SPI_SCK      5

    */
   
	SPI_DDR = (1<<SPI_SS) | (1<<SPI_MOSI) | (1<<SPI_SCK);		// setze SCK,MOSI,PB0 (SS) als Ausgang
	SPI_DDR &= ~(1<<SPI_MISO);							// setze MISO als Eingang
	//SPI_PORT = (1<<SPI_SCK) | (1<<SPI_SS);				// SCK und PB0 high (ist mit SS am Slave verbunden)
	//SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);	//Aktivierung des SPI, Master, Taktrate fck/16
	
   SPI_DDR |= (1<<SPI_CS); // Chip select
   //SPI_PORT &= ~(1<<SPI_CS); // LO, invertiert im Optokoppler HI
   SPI_PORT |= (1<<SPI_CS); // HI, ohne Optokoppler
   
   SPCR0 |= (1<<MSTR0);// Set as Master
   
   SPI_PORT &= ~(1<<SPI_MISO); // LO
   
   SPCR0 |= (1<<SPR00);               // div 16 SPI2X: div 8
   //SPCR0 |= (1<<SPR10);               // div 64 SPI2X: div 32
   //SPCR |= (1<<SPR1) | (1<<SPR0);   // div 128 SPI2X: div 64
   //SPCR0 |= (1<<SPI2X0);
   
   SPCR0 |= (1<<SPE0); // Enable SPI
   status = SPSR0;								//Status loeschen
   SPI_DDR |= (1<<(SPI_CS_ADC_PIN));
}

void spi_master_restore(void)
{
   //SPCR0=0;
   SPCR0 &= ~(1<<CPOL0);
   SPCR0 &= ~(1<<CPHA0);

}

void setSPI_Teensy(void)
{
   uint8_t outindex=0;
   SPI_PORT &=  ~(1<<SPI_CS); // CS LO, Start, Slave soll erstes Byte laden
   _delay_us(1);
   
   
   //SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
   //_delay_us(1);
   
   //PORTB &= ~(1<<0);
   for (outindex=0;outindex < SPI_BUFFERSIZE;outindex++)
      //for (outindex=0;outindex < 4;outindex++)
   {
      //OSZI_A_LO;
      // _delay_us(1);
      SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
      //_delay_us(1);
      
      SPDR0 = spi_txbuffer[outindex];
      
      while(!(SPSR0 & (1<<SPIF0)) && spiwaitcounter < WHILEMAX)
      {
          spiwaitcounter++;
      }
      spiwaitcounter=0;
      //_delay_us(1);
      //uint8_t incoming = SPDR0;
      if (spistatus & (1<< TEENSY_SEND))
      {
         spi_rxbuffer[outindex] = SPDR0;
         _delay_us(3);
         //spi_rxbuffer[outindex] = incoming;
      }
      else if (outindex == 0) // slave warten lassen, um code zu laden
      {
         _delay_us(3);
      }
      else if (outindex ==1) // code lesen, spistatus steuern
      {
         spi_rxbuffer[0] = SPDR0;
      }
      
      
      //_delay_us(3);
      SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
      //OSZI_A_HI;
      
      
   }
   //PORTB |=(1<<0);
   arraypos++;
   arraypos &= 0x07;
   //spi_rxbuffer[outindex] = '\0';
   //outbuffer[outindex] = '\0';
   //char rest = SPDR;
   
   // wichtig
   _delay_us(5);
   
   //SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
   //_delay_us(1);
   
   SPI_PORT |= (1<<SPI_CS); // CS HI End, Slave soll  Byte-ZŠhler zurŸcksetzen
   //OSZI_A_HI;

}

unsigned char SPI_get_put_char(uint8_t cData)
{
   /* Start transmission */
   SPDR0 = cData;
   /* Wait for transmission complete */
  // while(!(SPSR & (1<<SPIF)))
      while(!(SPSR0 & (1<<SPIF0)) && spiwaitcounter < WHILEMAX)
      {
         spiwaitcounter++;
      }
      ;
   /* Return data register */
   return SPDR0;
}
/*
void master_transmit (unsigned char data)
{
	PORTB &= ~(1<<PB0);						//SS am Slave Low --> Beginn der Übertragung
	SPDR = data;								//Schreiben der Daten
	//while (!(SPSR & (1<<SPIF)));
   while(!(SPSR & (1<<SPIF)) && spiwaitcounter < WHILEMAX)
   {
      spiwaitcounter++;
   }

	PORTB |= (1<<PB0);							//SS High --> Ende der Übertragung
}
*/
