/* Filename: SoftwareVersion.h
 * 
 * Date: 1 August 2022
 * 
 * Author: Henry Gilbert
 * 
 * Description: Public interface to software version functions
 *
 * 
 * All Rights Reserved. Copyright 2022 Archangel Systems 
 */

#ifndef SOFTWARE_VERSION_H
#define SOFTWARE_VERSION_H


/**************  Included File(s) **************************/
#include <stdint.h>
#include <stdbool.h>
#include "circularBuffer.h"


/**************  Function Prototype(s) *********************/
uint32_t SWVer_GetNextVersionARINCMsg(uint8_t sdi);
void SWVer_GatherSWVersions(circBuffer_t * const adcRxBuff,
        circBuffer_t * const adcTxBuff);

uint8_t asciiConverter( const uint8_t val );

#endif 

/* end SoftwareVersion.h header file */
