/*
 * ----------------------------------------------------------------------------
 * defines
 */

#define TEST 1

#define OHNE_TEENSY 1

#define SPI_BUFFERSIZE 4
#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms

#define UPDATE_COUNT  0x0F
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

#define ROTARY_MIN             200

// admin
#define ADMIN_PORT          PORTA
#define ADMIN_DDR           DDRA
#define ADMIN_PIN           PINA
#define TEENSY_DETECTED          7           // Anzeige, dass Teensy vorhanden. Active LO
#define TEENSY_LED          6 

// switch in
#define SWITCH_PORT          PORTA
#define SWITCH_DDR           DDRA
#define SWITCH_PIN           PINA
#define SWITCH_0              2
#define SWITCH_1              3
#define SWITCH_2              4

// code SPI

#define WRITE_SPANNUNG        0x01
#define WRITE_STROM           0x02
#define READ_SPANNUNG         0x03
#define READ_STROM            0x04


#define SPI_RUN_BIT            7   // MASTER soll SPI abfragen

// OSZI
#define OSZIPORT           PORTA
#define OSZIPORTDDR        DDRA
#define OSZIPORTPIN        PINA
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)




