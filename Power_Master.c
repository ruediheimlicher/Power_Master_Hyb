//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>
#include "AVR_Starter.h"
#include "lcd.c"
#include "adc.c"

#include "soft_spi.c"
#include "spi_adc.c"
 

#include "utils.c"
#include "slaves.c"
#include "spi_master.c"


#include "defines.h"

#include "analog.c"

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;


#define EXT_ADRESSE				0x62
#define TWI_ERR_BIT				7
#define EXT		1

volatile uint8_t					Programmstatus=0x00;
uint8_t Tastenwert=0;
uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Counter fuer Timeout
uint16_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;
//volatile uint8_t		TWI_Flag=0;

// TWI
static volatile uint8_t Write_Err;
static volatile uint8_t Read_Err;




// Update durchfuehren
volatile uint8_t updateOK=0;

// SPI



volatile uint8_t data;

volatile uint8_t textpos=0;
volatile uint8_t inpos=0;

volatile uint8_t data_array[8];
volatile uint8_t arraypos=0;

volatile uint8_t inbuffer[16];
volatile uint8_t outbuffer[16];


volatile uint8_t rxdata =0;
uint8_t erfolg =0;

extern volatile uint8_t lcd_delaycount;
extern volatile uint8_t lcd_status;

extern volatile uint8_t rxbuffer[TWI_BUFFERSIZE];
extern volatile uint8_t txbuffer[TWI_BUFFERSIZE];
volatile uint8_t twi_Call_count0=0;	//	Anzahl TWI-Calls in einer Schleife
volatile uint8_t twi_Reply_count0=0;	//	Anzahl TWI-Replies in einer Schleife
volatile uint8_t twi_Stat_count=0;	//	Anzahl Resets nach erfolglosen TWI-Aufrufen


// Rotary

volatile uint16_t rot_eingang_count=0x00;
volatile uint16_t rot_eingang_plus=0x00;
//volatile uint16_t rot_eingang_A=0x00;
volatile uint16_t rot_count_A=0x00;
volatile uint16_t rot_loopcount_L=0x00;
volatile uint16_t rot_loopcount_H=0x0000;
volatile uint16_t rot_control=0x0000;
volatile uint8_t old_rot_pin=1;
volatile uint8_t new_rot_pin=1;
volatile uint8_t akt_rot_pin=1;
volatile uint8_t rot_changecount=0;

volatile uint16_t disp_loopcount_L=0x00;

volatile uint16_t ist_spannung=0;
volatile uint16_t ist_strom=0;

volatile uint16_t soll_spannung=0;
volatile uint16_t soll_strom=0;

volatile uint16_t ext_spannung=0;
volatile uint16_t ext_strom=0;


// SPI
volatile char incoming=0;
volatile uint8_t outcounter=0;
extern volatile uint8_t spi_rxbuffer[SPI_BUFFERSIZE];
extern volatile  uint8_t spi_txbuffer[SPI_BUFFERSIZE];

volatile uint8_t adc_H=0;
volatile uint8_t adc_L=0;
volatile uint8_t adc_in[2]= {};


// Teensy
volatile uint8_t teensycode = 0;


// Schalter
volatile uint8_t switch_in = 0;
volatile uint8_t switch_out = 0;

// var fuer soft-spi
volatile uint8_t out_H=0;
volatile uint8_t out_L=0;
volatile uint8_t in_H=0;
volatile uint8_t in_L=0;



//volatile uint8_t out[8] = {'H','e','l','l','o',' ',' ',' '};
volatile uint8_t out[8][8] ={
{'H','a','l','l','o',' ',' ',' '},
{'W','i','r',' ',' ',' ',' ',' '},
{'g','e','h','e','n',' ',' ',' '},
{'a','u','f',' ','d','e','n',' '},
{'B','a','c','h','t','e','l',' '},
{'u','n','d',' ','a','u','c','h'},
{'a','u','f',' ','d','e','n',' '},
{'T','u','r','m',' ',' ',' ','*'}};



// TUX

static int16_t measured_val[2]={0,0};
static int16_t set_val[2];


void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
   {
		_delay_ms(0.96);
		ms--;
	}
}

// TWI


// http://www.mikrocontroller.net/attachment/211211/Drehgeber_per_interrupt.c
//#define DEF_BAUDRATE    9600
//#define F_CPU           16000000
/*
// funktionen zur ser. Datenausgabe
void init_serio0(void)
{
   uint8_t temp;
   UCSR0B = 0x0;                               // alles abschalten
   UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);         // default wie nach reset 8 Bit, async, no par.
  //UBRR0=((F_CPU/16L/(DEF_BAUDRATE/10L)+5L)/10L) - 1;    // passend gerundet
   UBRR0=((F_CPU/16L/(DEF_BAUDRATE/10L)+5L)/10L) - 1;    // passend gerundet
   UCSR0B |= (1<<RXEN0) | (1<<TXEN0);          // RX + TX freigeben
   temp = UDR0;
}

void ser_putchar(char c)
{
   while(!(UCSR0A & (1<<UDRE0)));
   UDR0 = c;
}

void ser_string(char *s)
{
   while(*s) ser_putchar(*s++);
   ser_putchar(13);
   ser_putchar(10);
}
 
 // ausgeben
 ser_string(ergebnis_str);               // und per UART ausgeben
*/



void device_init(void)
{
   //Loop LED
   
   LOOPLED_DDR	|= (1<<LOOPLED_PIN);
   //LOOPLED_PORT	PORTD
   //LOOPLED_PIN	4
   
   
	//LCD
	/* INITIALIZE */
	LCD_DDR |=(1<<LCD_RSDS_PIN);
	LCD_DDR |=(1<<LCD_ENABLE_PIN);
	LCD_DDR |=(1<<LCD_CLOCK_PIN);
   
   OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	
   SWITCH_DDR &= ~(1<< SWITCH_0); // Eingang Switch Bit 0
   SWITCH_DDR &= ~(1<< SWITCH_1); // Eingang Switch Bit 1
   SWITCH_DDR &= ~(1<< SWITCH_2); // Eingang Switch Bit 2

   SWITCH_PORT  |= (1<< SWITCH_0); // Eingang Switch Bit 0
   SWITCH_PORT  |= (1<< SWITCH_1); // Eingang Switch Bit 1
   SWITCH_PORT  |= (1<< SWITCH_2); // Eingang Switch Bit 2

   
   ADMIN_DDR &= ~(1<< TEENSY_DETECTED); // Eingang fuer Anmeldung Teensy
   ADMIN_PORT &= ~(1<< TEENSY_DETECTED); //LO
 //  ADMIN_DDR |= (1<< TEENSY_LED); // Ausgang fuer Anzeige Teensy present
 //  ADMIN_PORT &= ~(1<< TEENSY_LED); //LO
  
   /*
   // TWI vorbereiten
	TWI_DDR |= (1<<SDA_PIN);//Bit 4 von PORT C als Ausgang für SDA
	TWI_PORT |= (1<<SDA_PIN); // HI
	
	TWI_DDR |= (1<<SCL_PIN);//Bit 5 von PORT C als Ausgang für SCL
	TWI_PORT |= (1<<SCL_PIN); // HI
   */
   
   
   
   
   
   //
   SOFT_SPI_DDR |= (1<<SOFT_SWITCH_LOAD); // Ausgang fuer LOAD SWITCH
   SOFT_SPI_PORT |= (1<<SOFT_SWITCH_LOAD); // HI
   
   SOFT_SPI_DDR |= (1<<SOFT_SWITCH_CS); // Ausgang fuer CS SWITCH
   SOFT_SPI_PORT |= (1<<SOFT_SWITCH_CS); // HI

}

void rotary_init(void)
{
   /*
   ROTARY_DDR &= ~(1<<ROTARY_A_PIN0);   // Eingang 0
   ROTARY_PORT |= (1<<ROTARY_A_PIN0); // HI
   ROTARY_DDR &= ~(1<<ROTARY_A_PIN1); //Eingang 1
   ROTARY_PORT |= (1<<ROTARY_A_PIN1);
   
  
   // PCINT16/RXD PD0
   PCICR |= (1<<PCIE3);
  
   PCIFR |= (1<<PCIF3);
   
   PCMSK3|= (1<<PCINT24);
   //PCMSK2|= (1<<PCINT17);
   */
   
   
   
   EICRA |= (1<<ISC01);
   EIMSK |= (1<<INTF0);
   
   // Rotary A
   ROTARY_DDR &= ~(1<<ROTARY_A_PIN0);  // Interrupt-Eingang INT0
   ROTARY_PORT |= (1<<ROTARY_A_PIN0);  // HI
   ROTARY_DDR &= ~(1<<ROTARY_A_PIN1);  // Sense-Eingang
   ROTARY_PORT |= (1<<ROTARY_A_PIN1);
   
   // Rotary B
   ROTARY_DDR &= ~(1<<ROTARY_B_PIN0);  // Interrupt-Eingang INT1
   ROTARY_PORT |= (1<<ROTARY_B_PIN0);
   ROTARY_DDR &= ~(1<<ROTARY_B_PIN1);  // Sense-Eingang
   ROTARY_PORT |= (1<<ROTARY_B_PIN1);

   /*
#define ROTARY_A_PIN0         2
#define ROTARY_A_PIN1          0
#define ROTARY_B_PIN0         3
#define ROTARY_B_PIN1          1
*/
}


/*
ISR(PCINT3_vect)
{
   //rot_eingang0++;
   //OSZI_A_LO;
   
   uint8_t rot_pin0 = ROTARY_PIN & 0x01; //Eingang 0
   uint8_t rot_pin1 = (ROTARY_PIN & 0x02); //Eingang 1
   
   new_rot_pin = ROTARY_PIN & 0x02;
   
   akt_rot_pin = new_rot_pin ^ old_rot_pin;
   
   uint16_t delta=0x2F;
   //uint16_t deltaA=0x80;
   
   if (rot_pin1 == 0)
   {
      rot_control++;
      if (rot_loopcount_H  > 0x08)
      {
         delta=0x02;
          //deltaA = 0x08;
      }
      rot_loopcount_H=0;
   }
   //deltaA = 10;
   
   
   if ((rot_pin0==1) && (rot_pin1 == 0))
   {
      if (0x0FFF - soll_spannung > delta)
      {
         soll_spannung += delta;
      }
      else
      {
         soll_spannung = 0x0FFF;
      }
      
   }
   else if ((rot_pin0==0) && (rot_pin1 == 0))
   {
      if (soll_spannung > ROTARY_MIN + delta)
      {
         soll_spannung -= delta;
         
      }
      else
      {
         soll_spannung = ROTARY_MIN;
      }
   }
   
   
   
   
  // soll_spannung = rot_eingang_A;
   spi_txbuffer[2] =  (soll_spannung & 0x00FF);
   spi_txbuffer[3] = ((soll_spannung & 0xFF00)>>8);
   
  // OSZI_A_HI;
}
*/



ISR(INT0_vect)
{
   //rot_eingang0++;
   OSZI_A_LO;
   
   //uint8_t rot_pin0 = ROTARY_PIN & 0x04; // Interrupt-Eingang
   uint8_t rot_pin1 = (ROTARY_PIN & (1<<ROTARY_A_PIN1)); // Sense Eingang A
   
  // new_rot_pin = ROTARY_PIN & 0x02;
   
  // akt_rot_pin = new_rot_pin ^ old_rot_pin;
   
   uint16_t delta=0x2F;
   //uint16_t deltaA=0x80;
   
  // if (rot_pin1 == 0)
   {
      rot_control++;
      if (rot_loopcount_H  > 0x08)
      {
         delta=0x02;
         //deltaA = 0x08;
         
      }
      rot_loopcount_H=0;
   }
   //rot_control = rot_loopcount_H;
   //deltaA = 10;
   
   
   if ( (rot_pin1 == 0))
   {
      if (0x0FFF - soll_spannung > delta) // noch nicht auf Max?
      {
         soll_spannung += delta;
      }
      else
      {
         soll_spannung = 0x0FFF;
      }
      
   }
   else
   {
      if (soll_spannung > ROTARY_MIN + delta) // noch genuegend?
      {
         soll_spannung -= delta;
      }
      else
      {
         soll_spannung = ROTARY_MIN;
      }
   }
   
   // soll_spannung an Teensy
   spi_txbuffer[2] =  (soll_spannung & 0x00FF);
   spi_txbuffer[3] = ((soll_spannung & 0xFF00)>>8);
   
   OSZI_A_HI;
}


void timer0 (void) // Grundtakt fuer Stoppuhren usw.
{
   //lcd_puts("timer0\0");
   
   // Timer
   TCCR0B |= (1<<CS01);						// clock	/8
   //TCCR0B |= (1<<CS01)|(1<<CS00);			// clock	/64
   //TCCR0B |= (1<<CS02)| (1<<CS02);			// clock	/256
   //TCCR0B |= (1<<CS00)|(1<<CS02);			// clock /1024
   
   //TCCR0B |= (1 << CS02);// /256
   //TCCR0B |= (1 << CS00); // no prescaling
   
   
   //OCR0A = 0x02;
   
   TIFR0 |= (1<<TOV0);
   //TIFR |= (1<<TOV0);		//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK0 |= (1<<TOIE0);	//Overflow Interrupt aktivieren
   TCNT0 = 0;					//Rücksetzen des Timers

    }

#pragma mark TIMER0_OVF
ISR (TIMER0_OVF_vect)
{
   rot_loopcount_L++;
   //OSZI_B_TOGG;
    if (rot_loopcount_L >= UPDATE_COUNT)
   {
      //OSZI_A_TOGG;
      //updateOK =1;
      updateOK |= (1<<UPDATE_MEAS);
      rot_loopcount_L = 0;
      
      if (rot_loopcount_H < ROT_HI) // hochzaehlen bis max
      {
         rot_loopcount_H ++;
      }
   }
   
   disp_loopcount_L++;
   if (disp_loopcount_L > 0x8FF)
   {
      updateOK |= (1<<UPDATE_DISP);
      disp_loopcount_L=0;
   }

   
}

// the control loop changes the dac:
static void control_loop(void)
{
   int16_t tmp;
   //LEDON0;
   
   // Data-Array's [2]
   // target_val: int value that is requested (control loop calibrates to this).
   // analog_result: adc measurement results (11bit ADC):
   
   tmp=soll_strom - ist_strom; // current diff
   if (tmp <0) // Spannungsabfall an Shunt ist zu gross
   {
      // ** current control:
      //
      // stay in current control if we are
      // close to the target. We never regulate
      // the difference down to zero otherweise
      // we would suddenly hop to voltage control
      // and then back to current control. Permanent
      // hopping would lead to oscillation and current
      // spikes.
      if (tmp>-2) tmp=0; // Differenz ignorieren
      
      currentcontrol=10; // I control
      if (TEST)
      {
      lcd_gotoxy(19,0);
      lcd_putc('I');
      }
      
      
      if (ist_spannung > soll_spannung) // Ausgangsspannung ist höher als am DAC eingestellt
      {
         // oh, voltage too high, get out of current control:
         tmp =- 20;
         currentcontrol=0; // U control
      }
   }
   else
   {
      // ** voltage control:
      //
      // if we are in current control then we can only go
      // down (tmp is negative). To increase the current
      // we come here to voltage control. We must slowly
      // count up.
      if (TEST)
      {
         lcd_gotoxy(18,0);
         lcd_putc('U');
      }
      tmp=1+ soll_spannung  - ist_spannung; // voltage diff
      if (currentcontrol)
      {
         currentcontrol--;
         // do not go up immediately after we came out of current control:
         if (tmp>0)
         {
            tmp=0;
         }
      }
   }
   if (tmp> -3 && tmp<4)
   { // avoid LSB bouncing if we are close
      tmp=0;
   }
//   lcd_gotoxy(9,0);
//   lcd_putsignedint(tmp);

   //LEDOFF0;
   if (tmp==0) return; // nothing to change
   
   // put a cap on increase
   if (tmp>1)
   {
      tmp=1;
   }
   // put a cap on decrease
   if (tmp<-200)
   {
      tmp=-20;
   }
   else if (tmp<-1)
   {
      tmp=-1;
   }
   //lcd_gotoxy(0,1);
   //lcd_putsignedint(tmp);

   dac_val+=tmp;
   
   if (dac_val>0x0FFF)
   {
      dac_val=0x0FFF; //max, 12bit
   }
   if (dac_val<200)
   {  // the output is zero below 400 due to transistor threshold
      dac_val=200;
   }
 //  lcd_gotoxy(14,1);
 //  lcd_putint12(dac_val);

   setDAC7612(dac_val); // hier ODER in main!
   
   
}


int main (void)
{
   
   //JTAG deaktivieren (datasheet 231)
   MCUCR |=(1<<7);
   MCUCR |=(1<<7);
   
   uint8_t ch = MCUSR;
   MCUSR = 0;
   cli();
   wdt_disable();
   MCUSR &= ~(1<<WDRF);
   
   //23.01.10
   wdt_reset();
   WDTCSR |= (1<<WDCE) | (1<<WDE);
   WDTCSR = 0x00;

   // Fuses: h: D9	l: EF  JTAG OFF
	device_init();
	
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	_delay_ms(1000);
	lcd_cls();
	lcd_puts("READY\0");
	
	_delay_ms(1000);
   lcd_cls();
	uint8_t i=0;
   
   rotary_init();
   sei();
  // i2c_init();
   spi_master_init();
   
   dac_init();
   
   uint16_t spiloop =0;
   uint8_t twiloop=0;
   uint8_t errloop=0;

   
   //DDRB |=(1<<0);
   device_init();
   initADC();
   
//   spi_txbuffer[0]= '$';
   uint8_t k=0;
   for (k=0;k<SPI_BUFFERSIZE;k++)
   {
      spi_txbuffer[k]= 0x00;
      spi_rxbuffer[k]= 0x00;
   }

//   volatile uint16_t U_I_Array[4];
   
   timer0();
   sei();
   
   //spistatus |= (1<< TEENSY_SEND);
   
   soll_spannung = 1000;
   currentcontrol=1; // 0=voltage control, 1 current control
   //lcd_gotoxy(6,0);
   //lcd_putsignedint(-12);
   soll_strom = 800;
   
   uint16_t teststrom = 0;
   uint16_t teststrom_mittel[4]={};
   uint8_t stromschleifecounter=0;
#pragma mark while
	while (1)
	{
		//OSZI_A_TOGG;
      
      // Teensy angeschlossen?
      
      if (ADMIN_PIN & (1<<TEENSY_DETECTED))// || OHNE_TEENSY) // TEENSY_DETECT ist activ LO!!!
      {
         //spistatus |= (1<< TEENSY_RECV);
        spistatus &= ~(1<< TEENSY_RECV);
         //ADMIN_PIN &= ~(1<<TEENSY_LED); // Teensy LED OFF
               }
      else
      {
         //spistatus &= ~(1<< TEENSY_RECV);
         spistatus |= (1<< TEENSY_RECV);     // active LO
         //ADMIN_PIN |= (1<<TEENSY_LED); // Teensy LED ON

      }

      //      if (TEST && (updateOK & (1<<UPDATE_MEAS)))
      {
         //OSZI_A_TOGG;
         updateOK &= ~(1<<UPDATE_MEAS);
         //updateOK = 0;
         //OSZI_A_TOGG;
         
         spiloop++;
         //if (spiloop>0x01)
         {
            //OSZI_B_LO;
            spiloop=0;
            //OSZI_B_HI;
            // if (ADMIN_PIN & (1<<TEENSY_DETECTED))
            
         //   spistatus |= (1<< TEENSY_RECV);
            
            if (spistatus & (1<< TEENSY_RECV)) // Teensy ist da, abfragen
            {
               lcd_gotoxy(19,0);
               lcd_putc('+');
               //lcd_gotoxy(14,0);
               //lcd_puthex(spi_rxbuffer[0]);
               //OSZI_B_HI;
               setSPI_Teensy();
               
               
               if (OHNE_TEENSY)
               {
                  //spi_txbuffer[0] = 0x81;
                  //spi_txbuffer[1] = 0x81;
                 // spi_txbuffer[2] = 5;
                  //spi_txbuffer[3] = 3;
               }

               
               lcd_gotoxy(16,0);
               lcd_puthex(spi_rxbuffer[0]);
               //lcd_gotoxy(16,1);
               //lcd_puthex(spi_rxbuffer[1]);
               //lcd_puthex(errloop);
               
               teensycode = (spi_rxbuffer[0] & 0x07);    //Bit 7 ist SPI_RUN
               spi_rxbuffer[0] &=  ~(0x7F);
               //ext_spannung = (spi_rxbuffer[1] | (spi_rxbuffer[2]<<8));
               
               switch (teensycode)
               {
                  case WRITE_SPANNUNG: // Externe Spannung
                  {
                     errloop++;
                     cli();
                     ext_spannung = (spi_rxbuffer[1] | (spi_rxbuffer[2]<<8));                     
                     soll_spannung = (spi_rxbuffer[1] | (spi_rxbuffer[2]<<8));
                     sei();
                  }break;
                  case WRITE_STROM: // Externer Strom
                  {
                     
                     cli();
                     ext_strom = (spi_rxbuffer[1] | (spi_rxbuffer[2]<<8));
                     sei();
                  }break;
                     
                     
               }// switch
               spi_rxbuffer[0] &=  ~0x7F;
               //OSZI_B_HI;
            }
            else
            {
               lcd_gotoxy(19,0);
               lcd_putc('-');
               //OSZI_B_HI;
               
            }
            //_delay_us(1);
            _delay_us(1);

            //      switch_in = getSwitch(); // 22us
            //      switch_in = get_SR(0);
            //      switch_in = readRaw8Kanal(2);
            
            
            //OSZI_B_LO;
            switch_in = 7-((SWITCH_PIN & 0x1C)>>2 );
            lcd_gotoxy(0,0);
            lcd_puthex(switch_in);
            
            // Test fuer SR HC595
            
           
           
            // Ausgaenge entsprechend Schalterstellung setzen
            switch_out = 0xFF; // reset der Schalterwerte, alle HI: Messung ueberbruecken
            switch (switch_in)
            {
               case 0x00: // OFF
               {
                  
               }break;
                  
               case 0x01: //100mA
               {
                  
                  switch_out &= ~(1<<0);
                  
               }break;
               case 0x02: //
               {
                  switch_out &= ~(1<<1);
               }break;
               case 0x03: // 300mA
               {
                  switch_out &= ~(1<<2);
               }break;
               case 0x04: // 1A
               {
                  
               }break;
               case 0x05: // 3A
               {
                  
               }break;
               case 0x06: // 10A
               {
                  
               }break;
                  
            }// switch
            //OSZI_B_HI;
            
            //lcd_puthex(switch_out);
            set_SR(switch_out); //
            //_delay_us(100);
            //spi_adc_restore();
#pragma mark ADC
             
//            OSZI_A_LO;
            //_delay_us(1);
 //           OSZI_A_HI;
            //_delay_us(1);
            
            //OSZI_B_LO;
            
  //         setDAC7612(soll_spannung); // hier ODER in Control_loop
            
            // Spannung messen
            //OSZI_B_HI;
            ist_spannung= MCP3208_spiRead(SingleEnd,0);
            //_delay_us(1);
            
            // Strom messen
            if (switch_in)
            {
               uint16_t teststrom = MCP3208_spiRead(SingleEnd,(switch_in)) ;
               stromschleifecounter &= 0x03;
               teststrom_mittel[stromschleifecounter] = MCP3208_spiRead(SingleEnd,(switch_in)) ;
               stromschleifecounter++;
               uint8_t index=0;
               teststrom=0;
               for (index=0;index<4;index++)
               {
                  teststrom += (teststrom_mittel[index]);
               }
               teststrom /= 4;
               //         ist_strom =  MCP3208_spiRead(SingleEnd,(switch_out & 0x07)) ;
               
               lcd_gotoxy(16,1);
               // lcd_putc('i');
               lcd_putint12( teststrom);
            }
            /*
             lcd_gotoxy(6,0);
            lcd_putint12(soll_spannung);
*/
            /*
            if (TEST == 1)
            {
            lcd_gotoxy(6,0);
            //lcd_putc('i');
            lcd_putint12(ist_spannung);
            //lcd_putc(' ');
            //lcd_putc('s');
            lcd_putint12(soll_spannung);
            }
           
            if (TEST == 2)
            {
            lcd_gotoxy(6,1);
            lcd_putc('i');
            lcd_putint12(ist_strom);
            lcd_putc(' ');
            lcd_putc('s');
            lcd_putint12(soll_strom);
            }
             */
           // ist_strom =  0;
            
            /*
             //
             lcd_gotoxy(10,0);
             lcd_putint12((spi_txbuffer[2] | (spi_txbuffer[3]<<8))>>4); // 12 bit
             //lcd_putint16((adc_L | (adc_H<<8)));
             //lcd_putc(' ');
             
             lcd_gotoxy(5,1);
             lcd_putc('e');
             lcd_putint12(ext_spannung);
             
             lcd_gotoxy(10,1);
             lcd_putc('i');
             lcd_putint12(ist_spannung);
             lcd_putc('s');
             //lcd_puthex(switch_in);
             
             lcd_putint12(soll_spannung>>4); // 12 bit
             
             lcd_gotoxy(18,0);
             lcd_puthex(errloop);
             
             lcd_gotoxy(0,0);
             lcd_puthex(spi_rxbuffer[0]);
             lcd_putc(' ');
             lcd_puthex(spi_rxbuffer[1]);
             
             lcd_puthex(spi_rxbuffer[2]);
             
             //lcd_putc(' ');
             lcd_puthex(spi_rxbuffer[3]);
             
             //   lcd_putc('*');
             */
            // ist-Spannung von ADC
            /*
             lcd_gotoxy(0,0);
             lcd_putc('i');
             lcd_putc(' ');
             lcd_putint12(ist_spannung);
             
             
             lcd_gotoxy(0,1);
             lcd_putc('s');
             lcd_putc(' ');
             
             lcd_putint12(soll_spannung); // 12 bit
             
             */
            //currentcontrol = 1;
            
            //           lcd_gotoxy(6,1);
            //          lcd_puthex(currentcontrol);
            
            //           lcd_gotoxy(14,1);
            //           lcd_putint12(dac_val);
            
#pragma mark control_loop
            OSZI_B_LO;
            
            control_loop(); //2.5us
            
            OSZI_B_HI;
            //lcd_puthex(currentcontrol);
            
            
         } // if (spiloop>0x4F)
      }
      
      if (updateOK & (1<<UPDATE_DISP))
      {
         updateOK &= ~(1<<UPDATE_DISP);
         //setDAC_long(ext_spannung<<4);
         
         switch (TEST)
         {
         case 1: // nur Spannung
            {
               lcd_gotoxy(6,0);
               //lcd_putc('i');
               lcd_putint12(ist_spannung);
               //lcd_putc(' ');
               //lcd_putc('s');
               lcd_putint12(soll_spannung);
            }break;
 
         case 2: // nur Strom
            {
               
               lcd_gotoxy(6,1);
               lcd_putc('i');
               lcd_putint12(ist_strom);
               lcd_putc(' ');
               lcd_putc('s');
               lcd_putint12(soll_strom);
            }break;

         case 3:
            {
               lcd_gotoxy(6,0);
               //lcd_putc('i');
               lcd_putint12(ist_spannung);
               //lcd_putc(' ');
               //lcd_putc('s');
               lcd_putint12(soll_spannung);

               
               lcd_gotoxy(6,1);
               lcd_putc('i');
               lcd_putint12(ist_strom);
               lcd_putc(' ');
               lcd_putc('s');
               lcd_putint12(soll_strom);
            }break;
         } // switch

         /*
         lcd_gotoxy(0,0);
         lcd_putc('L');
         lcd_puthex(spi_rxbuffer[2]);
         lcd_putc(' ');
         lcd_putc('H');
         lcd_puthex(spi_rxbuffer[3]);
        */
         /*
         // Stufenschalter
         lcd_gotoxy(5,1);
         lcd_putc('S');
         lcd_puthex(switch_in);
          */
         //lcd_putc(' ');
         //lcd_putc('H');
         //lcd_puthex(adc_H);

         
         /*
         lcd_gotoxy(10,0);
         lcd_putint12((spi_txbuffer[2] | (spi_txbuffer[3]<<8))>>4); // 12 bit
         //lcd_putint16((adc_L | (adc_H<<8)));
         lcd_putc(' ');
       */
         
         
         // Strom
   //      lcd_putint12(ext_strom);
         
         
         lcd_gotoxy(4,1);
         lcd_putc('e');
         lcd_putint12(ext_spannung);
 
         lcd_gotoxy(10,1);
         lcd_putc('i');
         lcd_putint12(ist_spannung);
         //lcd_putc('s');
         //lcd_putint12(soll_spannung); // 12 bit
         //lcd_gotoxy(18,0);
         //lcd_puthex(errloop);
         

      }
      
		loopCount0 ++;
		//_delay_ms(2);
		
		if (loopCount0 >=0x00FF)
		{
         //OSZI_B_LO;
         //loopCount2++;
			//OSZI_A_LO;
			LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         
         /*
			loopCount1++;
			if ((loopCount1 >0xF0) )
			{
            LOOPLED_PORT ^= (1<<LOOPLED_PIN);
            loopCount1=0;


			}
			*/
			loopCount0 =0;
         //OSZI_B_HI;
		}
		
      
      
#pragma mark Tastatur 
		/* ******************** */
      if (TASTATUR_ON)
      {
		initADC();
		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
//		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1:											4	5	6
			 2:											7	8	9
			 3:											x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				
				 
				 lcd_gotoxy(17,1);
				 lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 lcd_gotoxy(19,1);
				 lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						
					}break;
						
					case 1:	//	
					{ 
					}break;
						
					case 2://
					{ 
					
							
					}break;
						
					case 3: //	Uhr aus
					{ 
					}break;
						
					case 4://
					{ 
                  uint8_t i=0;

					}break;
						
					case 5://
					{ 
					}break;
						
					case 6://
					{ 
					
					}break;
						
					case 7:// Schalter rückwaerts
					{ 
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9:// Schalter vorwaerts
					{ 

					}break;

					case 10:// *
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12: // # Normalbetrieb einschalten
					{
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}
      } // if TASTATUR_ON
	}
	
	
	return 0;
}
