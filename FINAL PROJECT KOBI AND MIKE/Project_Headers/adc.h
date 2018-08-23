/*
 * adc.h
 *
 *  Created on: Nov 25, 2014
 *      Author: Manuel Alejandro
 */

#ifndef ADC_H_
#define ADC_H_

#include "derivative.h"

/* Prototypes */
void adc_init(void);
int adc_cal(void);
unsigned short adc_read(unsigned char ch);


#define ADC_CHANNEL 0 //where sample from


#endif /* ADC_H_ */
