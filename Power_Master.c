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
volatile uint16_t rot_loopcountA_H=0x0000;
volatile uint16_t rot_loopcountB_H=0x0000;
volatile uint16_t rot_control=0x0000;
volatile uint8_t old_rot_pin=1;
volatile uint8_t new_rot_pin=1;
volatile uint8_t akt_rot_pin=1;
volatile uint8_t rot_changecount=0;

volatile uint16_t disp_loopcount_L=0x00;
volatile uint16_t SR_loopcount_L=0x00;
volatile uint16_t SR_loopcount_H=0x00;

volatile uint16_t ist_spannung=0;
volatile uint16_t ist_strom=0;

volatile uint16_t soll_spannung=0;
volatile uint16_t soll_strom=0;

volatile uint16_t ext_spannung=0;
volatile uint16_t ext_strom=0;

#define CORR_01   80
#define CORR_1   44
#define CORR_10  40

volatile uint16_t strom_corr[3] = {CORR_01,CORR_1,CORR_10};


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
volatile uint8_t strom_mult = 1;
volatile uint8_t strom_kanal = 0;

// var fuer soft-spi
volatile uint8_t out_H=0;
volatile uint8_t out_L=0;
volatile uint8_t in_H=0;
volatile uint8_t in_L=0;

volatile uint8_t ledcounter=0;

// 7-Segment
volatile uint16_t seg_loop=0;


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


// BCD

uint8_t BCD_Array[4]={};

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

void calibrate(void)
{
   uint8_t pos=0;
   for (pos=7;pos>4;pos--)
   {
      uint8_t temp_out=0x1C;
      switch_out |= (1<<pos);
      set_SR_23S17(GPIOB,temp_out);
      _delay_ms(1000);
      uint8_t i=0;
      uint16_t temp_strom = 0 ;
      for (i=0;i<4;i++)
      {
         temp_strom += MCP3208_spiRead(SingleEnd,1) ;
         _delay_us(2);
      }
      temp_strom /= 4;
      strom_corr[pos-5] = temp_strom;
      _delay_ms(500);
   }
   
   
}


void update_BCD_Array(uint8_t* bcdarray, uint16_t inValue)
{
   uint8_t index=0;
   for (index=0; index<4; index++)
   {
      bcdarray[index] = inValue % 10;
      inValue /= 10;
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
   //LOOPLED_PIN	7
   
   
	//LCD
	/* INITIALIZE */
	LCD_DDR |=(1<<LCD_RSDS_PIN);
	LCD_DDR |=(1<<LCD_ENABLE_PIN);
	LCD_DDR |=(1<<LCD_CLOCK_PIN);
   
   OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	
   /*
    // verlegt auf Schieberegister
   SWITCH_DDR &= ~(1<< SWITCH_0); // Eingang Switch Bit 0
   SWITCH_DDR &= ~(1<< SWITCH_1); // Eingang Switch Bit 1
   SWITCH_DDR &= ~(1<< SWITCH_2); // Eingang Switch Bit 2

   SWITCH_PORT  |= (1<< SWITCH_0); // Eingang Switch Bit 0
   SWITCH_PORT  |= (1<< SWITCH_1); // Eingang Switch Bit 1
   SWITCH_PORT  |= (1<< SWITCH_2); // Eingang Switch Bit 2
   */
   
   // Schieberegister Strom-Shunt
   SPI_SR_DDR |= (1<< SRA_CS); // Ausgang CS
   SPI_SR_PORT |= (1<< SRA_CS); // HI

   // Schieberegister 7-Seg-Anzeige
   SPI_SR_DDR |= (1<< SRB_CS);
   SPI_SR_PORT |= (1<< SRB_CS); // HI

   
   ADMIN_DDR &= ~(1<< TEENSY_DETECTED); // Eingang fuer Anmeldung Teensy
   ADMIN_PORT |= (1<< TEENSY_DETECTED); //HI
 //  ADMIN_DDR |= (1<< TEENSY_LED); // Ausgang fuer Anzeige Teensy present
 //  ADMIN_PORT &= ~(1<< TEENSY_LED); //LO
   ADMIN_DDR |= (1<< STROBE); // Ausgang fuer Strobe
   ADMIN_PORT &= ~(1<< STROBE); // LO
   /*
   // TWI vorbereiten
	TWI_DDR |= (1<<SDA_PIN);//Bit 4 von PORT C als Ausgang für SDA
	TWI_PORT |= (1<<SDA_PIN); // HI
	
	TWI_DDR |= (1<<SCL_PIN);//Bit 5 von PORT C als Ausgang für SCL
	TWI_PORT |= (1<<SCL_PIN); // HI
   */
   
   
   
   
   
   //
//   SOFT_SPI_DDR |= (1<<SOFT_SWITCH_LOAD); // Ausgang fuer LOAD SWITCH
//   SOFT_SPI_PORT |= (1<<SOFT_SWITCH_LOAD); // HI
   
//   SOFT_SPI_DDR |= (1<<SOFT_SWITCH_CS); // Ausgang fuer CS SWITCH
 //  SOFT_SPI_PORT |= (1<<SOFT_SWITCH_CS); // HI

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
   EIMSK |= (1<<INT0);
   
   EICRA |= (1<<ISC11);
   EIMSK |= (1<<INT1);
   
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
      if (rot_loopcountA_H  > 0x08)
      {
         delta=0x02;
          //deltaA = 0x08;
      }
      rot_loopcountA_H=0;
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
//   OSZI_A_LO;
   uint8_t rot_pin1 = (ROTARY_PIN & (1<<ROTARY_A_PIN1)); // Sense Eingang A
   
   uint16_t delta=0x2F;
   
   rot_control++;
   if (rot_loopcountA_H  > 0x08)
   {
      delta=0x02;
   }
   rot_loopcountA_H=0;
   
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
      if (soll_spannung > ROTARY_A_MIN + delta) // noch genuegend?
      {
         soll_spannung -= delta;
      }
      else
      {
         soll_spannung = ROTARY_A_MIN;
      }
   }
   // soll_spannung an Teensy
   spi_txbuffer[2] =  (soll_spannung & 0x00FF);
   spi_txbuffer[3] = ((soll_spannung & 0xFF00)>>8);
//   OSZI_A_HI;
}

ISR(INT1_vect)
{
   //rot_eingang0++;
   OSZI_A_LO;
   uint8_t rot_pin1 = (ROTARY_PIN & (1<<ROTARY_B_PIN1)); // Sense Eingang A
   
   uint16_t delta=0x2F;
   
   rot_control++;
   if (rot_loopcountB_H  > 0x08)
   {
      delta=0x02;
   }
   rot_loopcountB_H=0;
   
   if ( (rot_pin1 == 0))
   {
      if (0x0FFF - soll_strom > delta) // noch nicht auf Max?
      {
         soll_strom += delta;
      }
      else
      {
         soll_strom = 0x0FFF;
      }
   }
   else
   {
      if (soll_strom > ROTARY_B_MIN + delta) // noch genuegend?
      {
         soll_strom -= delta;
      }
      else
      {
         soll_strom = ROTARY_B_MIN;
      }
   }
   // soll_spannung an Teensy
//   spi_txbuffer[2] =  (soll_strom & 0x00FF);
//   spi_txbuffer[3] = ((soll_strom & 0xFF00)>>8);
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
      
      if (rot_loopcountA_H < ROT_HI) // hochzaehlen bis max
      {
         rot_loopcountA_H ++;
      }
      if (rot_loopcountB_H < ROT_HI) // hochzaehlen bis max
      {
         rot_loopcountB_H ++;
      }

   }
   
   disp_loopcount_L++;
   if (disp_loopcount_L > 0x8FF)
   {
      updateOK |= (1<<UPDATE_DISP);
      disp_loopcount_L=0;
   }
   
   SR_loopcount_L++;
   if (SR_loopcount_L > 0x0F)
   {
      SR_loopcount_L=0;
      SR_loopcount_H++;
      
      if (SR_loopcount_H %2)
      {
         updateOK |= (1<< UPDATE_SWITCH_SR);
      }
      else
      {
         updateOK |= (1<< UPDATE_7SEG_SR);
      }
   }


   
}

void timer1(void)
{
   DDRD |= 0x30;                      // Set Port D4 and D5 as Output
   
   TCCR1A |= (1<<WGM11); //
   TCCR1A |= (1<<COM1A1) |(1<<COM1B1);   // Set up the two Control registers of Timer1.
                                          // Wave Form Generation is Fast PWM 8 Bit,
   TCCR1B = (1<<WGM12)|(1<<WGM13);                   // OC1A and OC1B are cleared on compare match
   TCCR1B |= (1<<CS11);//|(1<<CS10);                      // and set at BOTTOM. Clock Prescaler is 64.
   
   OCR1A = ist_strom;                       // Dutycycle of OC1A
   OCR1B = ist_spannung>>3;                      // Dutycycle of OC1B
   ICR1H = 0x0F;
   ICR1L = 0xFF;
   
 }

//Interrupt TIMER1 (aller 1msek.)
ISR (TIMER1_COMPA_vect)
{
   //
   
   
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
   MCUCR |=(1<<7);
   MCUCR |=(1<<7);
   
   //uint8_t ch = MCUSR;
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
   
   //dac_init();
   
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
   
   
   
   
   timer1();
   
   uint16_t spannung_mittel[16]={};
   uint8_t spannungschleifecounter=0;

   
   uint16_t strom_mittel[8]={};
   uint8_t stromschleifecounter=0;
   
   init_SR_23S17();
   
   char pattern[32]= {0b00000001,
      0b00000011,
      0b00000110,
      0b00001100,
      0b00011001,
      0b00110011,
      0b01100110,
      0b11001100,
      0b10011000,
      0b00110000,
      0b01100000,
      0b11000000,
      0b10000000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b10000000,
      0b11000000,
      0b01100000,
      0b00110000,
      0b10011000,
      0b11001100,
      0b01100110,
      0b00110011,
      0b00011001,
      0b00001100,
      0b00000110,
      0b00000011,
      0b00000001,
      0b00000000,
      0b00000000,
      0b00000000,
   };


 //calibrate();
  /*
   lcd_gotoxy(10,2);
   for (i=0;i<3;i++)
   {
      lcd_putint(strom_corr[i]);
      lcd_putc(' ');
   }
 */

   
#pragma mark while
	while (1)
	{
		//OSZI_A_TOGG;
      
      // Teensy angeschlossen?
      
      if (ADMIN_PIN & (1<<TEENSY_DETECTED))// || OHNE_TEENSY) // TEENSY_DETECT ist activ LO!!!
      {
         //spistatus |= (1<< TEENSY_RECV);
//        spistatus &= ~(1<< TEENSY_RECV);
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
               if (TEST)
               {
                  lcd_gotoxy(19,0);
                  lcd_putc('+');
               }
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
               
               
               //           lcd_gotoxy(15,0);
               //           lcd_puthex(spi_rxbuffer[0]);
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
               if (TEST)
               {
                  lcd_gotoxy(19,0);
                  lcd_putc('-');
               }
               //OSZI_B_HI;
               
            }
            //_delay_us(1);
            _delay_us(1);

            //      switch_in = getSwitch(); // 22us
            //      switch_in = get_SR(0);
            //      switch_in = readRaw8Kanal(2);
            
            
            //OSZI_B_LO;
            
            // !!! Anschluesse von PORTC vertauscht !!!
#pragma mark SWITCH
            //switch_in = 7-((SWITCH_PIN & 0x07) );
            
            
            lcd_gotoxy(0,0);
            //lcd_puthex(SWITCH_PIN& 0x07);
            lcd_puthex(switch_in);
            //lcd_puthex(errloop++);
            
            // Test fuer SR HC595
            
           
           
            // Ausgaenge entsprechend Schalterstellung setzen
            
            // 0: leer
            // 1: 10A
            // 2: 100mA
            // 3: 1A
#pragma mark switch_out
            
            switch_out = 0x1C; // reset der Schalterwerte, alle LO (NPN fuer Relais)
            strom_mult = 1;
            uint16_t cal  = 0;
            switch (switch_in)
            {
               case 0x00: // OFF
               {
                 
               }break;
                  
               case 0x01: //50mA
               {
                  cal = strom_corr[0];
                  switch_out |= (1<<7);
   //               strom_mult = 2;
                  strom_kanal = 1;
                  
               }break;
               case 0x02: // 100mA
               {
                  cal = strom_corr[0];
                  switch_out |= (1<<7);
                  strom_kanal = 1;
               }break;
               case 0x03: // 500mA
               {
                  cal = strom_corr[1];
                  switch_out |= (1<<6);
    //              strom_mult = 2;
                  strom_kanal = 0;
               }break;
               case 0x04: // 1A
               {
                  cal = strom_corr[1];
                  switch_out |= (1<<6);
                  strom_kanal = 0;
               }break;
               case 0x05: // 5A
               {
                  cal = strom_corr[2];
                  switch_out |= (1<<5);
                  strom_kanal = 2;
       //           strom_mult = 2;
               }break;
               case 0x06: // 10A
               {
                  cal = strom_corr[2];
                  switch_out |= (1<<5);
                  strom_kanal = 2;
               }break;
               default:
               {
                  strom_mult = 1;
               }
                  
            }// switch
            //OSZI_B_HI;
            
            lcd_putc(' ');
            lcd_puthex(switch_out);
            lcd_putc(' ');
            lcd_puthex(strom_mult);
   //         OSZI_A_LO;
   //         set_SR(switch_out); //
   //         OSZI_A_HI;
            //_delay_us(100);
            //spi_adc_restore();
#pragma mark ADC
             
         //   OSZI_A_LO;
         //   _delay_us(10);
         //   OSZI_A_HI;
            //_delay_us(1);
            
            //OSZI_B_LO;
            
     //      setDAC7612(soll_spannung); // hier ODER in Control_loop
            
            
            
            // Spannung messen
            //OSZI_B_HI;
            ist_spannung= MCP3208_spiRead(SingleEnd,2);
            OCR1B = ist_spannung;
   
            spannungschleifecounter &= 0x0F;
            spannung_mittel[spannungschleifecounter] = ist_spannung;
            spannungschleifecounter++;

            
            
            //OCR1B = soll_spannung; // Dutycycle of OC1B // ergibt bei 3 volle Aussteuerung
            
            //_delay_us(1);
            
            // Strom messen
            if (switch_in) // Ein Bereich gewaehlt
            {
               strom_kanal = 1;
               stromschleifecounter &= 0x03;
               uint16_t akt_strom = MCP3208_spiRead(SingleEnd,(strom_kanal)) ;
               
               strom_mittel[stromschleifecounter] = akt_strom;
               
               //strom_mittel[stromschleifecounter] = (MCP3208_spiRead(SingleEnd,strom_kanal)) ;
               if ((strom_mult == 2) && akt_strom < 0x800)
               {
                  akt_strom *= strom_mult;
                  
               }
               else // Bereichsuerberschreitung
               {
                  //switch_out ^= 0x80; // bit 7 toggeln,
               }
               stromschleifecounter++;
               
               

               uint8_t index=0;
               uint32_t mittelstrom = 0;
               for (index=0;index<4;index++)
               {
                  mittelstrom += (strom_mittel[index]);
               }
               mittelstrom /= 4;
               //         ist_strom =  MCP3208_spiRead(SingleEnd,(switch_out & 0x07)) ;
               
               lcd_gotoxy(0,3);
               lcd_putc('i');
               lcd_putint12(mittelstrom);
               lcd_putc('c');
               lcd_putint(cal);
               
               if (mittelstrom > cal)
               {
               mittelstrom -= cal;
               }
               else
               {
                  mittelstrom   = 0;
               }
               
               lcd_putc(' ');
               lcd_putint12( mittelstrom);
               
               OCR1A = mittelstrom;
               //OCR1A = akt_strom;
               //OCR1A = 0x0FFF - mittelstrom;   // Null strom ergibt 0xFF vom ADC -> Dutycycle of OC1A
            }
            else
            {
               OCR1A = 0;
            }
            /*
             lcd_gotoxy(6,0);
            lcd_putint12(soll_spannung);
*/
            
            if (TEST == 1)
            {
            lcd_gotoxy(7,0);
            lcd_putc('i');
            lcd_putint12(ist_spannung);
            lcd_putc(' ');
            lcd_putc('s');
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
         //   OSZI_B_LO;
            
 //           control_loop(); //2.5us
            
          //  OSZI_B_HI;
            //lcd_puthex(currentcontrol);
            
            
         } // if (spiloop>0x4F)
      }
#pragma mark 7SEG
      if (updateOK & (1<<UPDATE_7SEG_SR))
      {
         seg_loop++;
         updateOK &= ~(1<<UPDATE_7SEG_SR);
         
 //        set_SR_23S17_A(pattern[(seg_loop & 0x1F)]);
         
     //    set_SR_23S17_B(seg_loop);
         
         
         //lcd_gotoxy(0,2);
         //lcd_putint12(seg_loop);
         ledcounter++;
         
         //Drehschalter lesen (GPIOA bit 5-7)
         uint8_t schaltpos = get_SR_23S17(GPIOA);
         
         lcd_gotoxy(10,0);
         lcd_puthex(schaltpos);
         lcd_puthex((schaltpos & 0xE0)>>1);
         // shift 5 bit
         switch_in = 7-((schaltpos & 0xE0)>>5);
         lcd_puthex(switch_in);
         uint8_t index=0;
         uint32_t mittelspannung = 0;
         for (index=0;index<16;index++)
         {
            mittelspannung += (spannung_mittel[index]);
         }
         mittelspannung /= 16;
         
         update_BCD_Array(BCD_Array,mittelspannung);
         
         //uint8_t max = get_SR_23S17(GPIOB); // uMax, iMax
         //lcd_puthex(max);
         
         // PORT B: Lesen vor schreiben!!!
         
         uint8_t inputB = get_SR_23S17(GPIOB);
         
         set_SR_23S17(GPIOB,switch_out);
         
         
         //uint8_t inputB = io_SR_23S17(GPIOB,switch_out);
         lcd_gotoxy(14,1);
         lcd_puthex((inputB& 0x03));
         
         // DAC U
         OSZI_B_LO;
         setMCP4821_U(soll_spannung);
         
         _delay_ms(1);
         
         setMCP4821_I(soll_strom);
         lcd_gotoxy(0,2);
         lcd_putint12(soll_spannung);
         lcd_putc(' ');
         lcd_putint12(ist_spannung);
         
         lcd_gotoxy(0,1);
         lcd_putc('i');
         lcd_putint12(ist_strom);
         lcd_putc(' ');
         lcd_putc('s');
         lcd_putint12(soll_strom);
         lcd_putc(' ');
         
         
    //     seg_loop &= 0xFF;
         
         if (seg_loop == 0)
         {
            ////OSZI_A_LO;
         }
         //uint8_t seg_data = ((BCD_Array[seg_loop] & 0x0F));
         /*
          BCD_Array[0]=2;
          BCD_Array[1]=3;
          BCD_Array[2]=4;
          BCD_Array[3]=6;
          */
         uint8_t seg_data = ((BCD_Array[(seg_loop & 0x07)] & 0x0F));
         //uint8_t seg_data = 3;//((BCD_Array[1] & 0x0F));
         
         //lcd_gotoxy(10,3);
         //lcd_putint12(BCD_Array[(seg_loop & 0x07)]);
         //lcd_putc(' ');
         
         
         //lcd_puthex(seg_data);
         
         seg_data |= (1<<((seg_loop & 0x07)+4));
         
         //   seg_data = 13;
         
         //set_SR_7Seg(BCD_Array[seg_loop & 0x03]);
        set_SR_7Seg(seg_data);
        
        
         OSZI_B_HI;
      }//if UPDATE_7SEG_SR

      
      if (updateOK & (1<<UPDATE_DISP))
      {
         updateOK &= ~(1<<UPDATE_DISP);
         //setDAC_long(ext_spannung<<4);
         
         switch (TEST)
         {
         case 1: // nur Spannung
            {
               lcd_gotoxy(5,0);
               //lcd_putc('i');
               lcd_putint12(ist_spannung);
               lcd_putc(' ');
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
               lcd_putc('i');
               lcd_putint12(ist_spannung);
               lcd_putc(' ');
               lcd_putc('s');
               lcd_putint12(soll_spannung);

               
               lcd_gotoxy(6,1);
               lcd_putc('i');
               lcd_putint12(ist_strom);
               lcd_putc(' ');
               lcd_putc('s');
               lcd_putint12(soll_strom);
            }break;
               
            default:
            {
              // lcd_gotoxy(0,2);
             //  lcd_putint12(soll_spannung);
             //  lcd_putc(' ');
             //  lcd_putint12(ist_spannung);
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
         
         /*
         lcd_gotoxy(4,1);
         lcd_putc('e');
         lcd_putint12(ext_spannung);
          
         lcd_gotoxy(10,1);
         //lcd_putc('i');
         lcd_putint12(ist_spannung);
         
         //lcd_putc('s');
         //lcd_putint12(soll_spannung); // 12 bit
         //lcd_gotoxy(18,0);
         //lcd_puthex(errloop);
          
          */
  //       OSZI_A_LO;
         update_BCD_Array(BCD_Array,ist_spannung);
  //       OSZI_A_HI;
         /*
          lcd_gotoxy(0,2);
          lcd_putint1(BCD_Array[3]);
          lcd_putc(' ');
          lcd_putint1(BCD_Array[2]);
          lcd_putc(' ');
          lcd_putint1(BCD_Array[1]);
          lcd_putc(' ');
          lcd_putint1(BCD_Array[0]);
         */
         


      }
      
		loopCount0 ++;
		//_delay_ms(2);
		
		if (loopCount0 >=0x00FF)
		{
         //OSZI_B_LO;
         //loopCount2++;
			//OSZI_A_LO;
         //LOOPLED_DDR	|= (1<<LOOPLED_PIN);
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
