//
//  analog.c
//  Power_Master
//
//  Created by Ruedi Heimlicher on 25.11.2014.
//
//

#include "analog.h"


static volatile uint8_t currentcontrol=1; // 0=voltage control, otherwise current control
// adc measurement results (11bit ADC):
static volatile int16_t analog_result[2];


// target_val is the value that is requested (control loop calibrates to this).
// We use the same units a the ADC produces.
static volatile int16_t target_val[2];  // datatype int is 16 bit

static volatile int16_t dac_val=600; // the current dac setting


void init_analog(void)
{
   // initialize the adc result to very high values
   // to keep the control-loop down until proper measurements
   // are done:
   analog_result[0]=0; // I
   analog_result[1]=20;  // U
   target_val[0]=0; // initialize to zero, I
   target_val[1]=0; // initialize to zero, U
}

