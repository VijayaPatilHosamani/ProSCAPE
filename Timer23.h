/*
 * Filename: Timer23.h
 *
 * Author: Henry Gilbert
 * 
 * Date: 15 June 2022
 * 
 * Description: External interface for Timer23 module. Features initialize,
 *      delay, and get timestamp functions. 
 * 
 * All rights reserved. Copyright Archangel Systems Inc. 2022
 */

#ifndef TIMER23_H
#define TIMER23_H

/**************  Included Files **************************/
#include <stdint.h>
#include <stdbool.h>


/**************  Function Prototypes *********************/
void Timer23_Initialize(const uint16_t t2config, /* Configuration register value */
        const uint32_t timerPeriod, /* 32 bit timer period: PR3-PR2 */
        const uint32_t configScaleFactor); /* Scale factor to set sampling period to 1ms */

uint32_t Timer23_GetTimestamp_ms(); 

void Timer23_Delay_ms(uint32_t delayInMilliseconds);


#endif
/* End of Timer23.h header file*/
