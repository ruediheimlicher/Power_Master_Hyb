/*
 * ----------------------------------------------------------------------------
 * defines
 */

#define SPI_BUFFERSIZE 4
#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms

#define UPDATE_COUNT  0x02
#define ROT_HI  0x1FF

#define UPDATE_MEAS  0
#define UPDATE_DISP  1

#define LOOPLED_PORT	PORTD
#define LOOPLED_DDR	DDRD
#define LOOPLED_PIN	4


#define TASTATUR_ON            0

#define ROTARY_PORT          PORTD
#define ROTARY_DDR           DDRD
#define ROTARY_PIN            PIND
#define ROTARY_PIN0           0
#define ROTARY_PIN1           1




// OSZI
#define OSZIPORT           PORTC
#define OSZIPORTDDR        DDRC
#define OSZIPORTPIN        PINC
#define OSZI_PULS_A        6
#define OSZI_PULS_B        7

#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)




