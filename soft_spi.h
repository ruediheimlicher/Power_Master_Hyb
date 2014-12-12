//
//  dac.h
//  Power_Master
//
//  Created by Ruedi Heimlicher on 10.11.2014.
//
//

#ifndef __Power_Master__dac__
#define __Power_Master__dac__

#include <stdio.h>

#endif /* defined(__Power_Master__dac__) */

// Hardware PINs DAC

//#define DAC_PORT  PORTC
//#define DAC_DDR   DDRC



#define SOFT_SWITCH_CS    1
#define SOFT_SWITCH_LOAD    2

#define SOFT_DAC_LOAD       3 // 
#define SOFT_DAC_CS       4

#define SOFT_SCK      5
#define SOFT_MOSI     6
#define SOFT_MISO     7





#define MAXSPANNUNG  756
#define MINSPANNUNG  500


// SOFT-SPI defines

#define SOFT_SPI_PORT   PORTC
#define SOFT_SPI_DDR    DDRC
#define SOFT_SPI_PIN    PINC


#define DAC_LOAD_HI        SOFT_SPI_PORT |= (1<<SOFT_DAC_LOAD)
#define DAC_LOAD_LO        SOFT_SPI_PORT &= ~(1<<SOFT_DAC_LOAD)

#define DAC_CS_HI        SOFT_SPI_PORT |= (1<<SOFT_DAC_CS)
#define DAC_CS_LO        SOFT_SPI_PORT &= ~(1<<SOFT_DAC_CS)

#define SWITCH_LOAD_HI        SOFT_SPI_PORT |= (1<<SOFT_SWITCH_LOAD)
#define SWITCH_LOAD_LO        SOFT_SPI_PORT &= ~(1<<SOFT_SWITCH_LOAD)

#define SWITCH_CS_HI        SOFT_SPI_PORT |= (1<<SOFT_SWITCH_CS)
#define SWITCH_CS_LO        SOFT_SPI_PORT &= ~(1<<SOFT_SWITCH_CS)


#define SCL_HI       SOFT_SPI_PORT |= (1<<SOFT_SCK)
#define SCL_LO       SOFT_SPI_PORT &= ~(1<<SOFT_SCK)

#define DATA_HI      SOFT_SPI_PORT |= (1<<SOFT_MOSI)
#define DATA_LO      SOFT_SPI_PORT &= ~(1<<SOFT_MOSI)



