/*
 * Filename: Timer23.c
 *
 * Author: Henry Gilbert
 * 
 * Date: 15 June 2022
 * 
 * Description: Module for implementing the 32bit timer combination
 *      for timer 2 and timer 3. The functions were developed specifically
 *      to act as a program's 1ms counter/stopwatch. Useful for timestamping 
 *      messages or getting basic elapsed time with up to 1 millisecond granularity.
 *      This module is not intended for use with interrupts and should operate 
 *      at the highest possible period register.  
 *      
 *      Features an initialize function to set the configuration register,
 *      period register, and scale factor values required to reach a 1ms 
 *      granularity (more below). Get timestamp returns a 32-bit, millisecond 
 *      timestamp based on the current instant the function was called. Delay 
 *      is implemented and features a permanent guard for any loops over 1 second. 
 *      
 *      Directions for use: 
 *          This module configures registers T2CON, PR3, and PR2 of  
 *          dsPIC30F6014A. For proper funcitoning, at minimum, the value 
 *          of T2CON must feature 0x8008, featuring bits:
 *                  15 (timer 2 ON)
 *                   3 (timer 2 and 3 form a 32 bit timer)  
 *          Set bits 5 and 4 (starting from zero) to achieve the prescale values:
 *              Prescale 1:256 : TCKPS<5:4> = 11
 *              Prescale 1:64  : TCKPS<5:4> = 10
 *              Prescale 1:8   : TCKPS<5:4> = 01
 *              Prescale 1:1   : TCKPS<5:4> = 11
 *              
 *              Set T2CONbits.TCS to: 
 *              1 if External Oscillator. CLK will equal Fosc_external . 
 *              0 if Internal Oscillator. CLK will equal Fosc/4 
 * 
 *          The input value to the initialize function timerPeriod should 
 *          equal 0xFFFFFFFF. Otherwise, period resets may occur more than desired. 
 *      
 *      A scale factor is required to convert the CLK counts into actual millisecond
 *      values.
 * 
 *      Must disable interrupts here! Very very important potential oversight. 
 *      If interrupts are required, all status is controlled through T3, and mode 
 *      code must be added to clear the timer interrupt flag in software. 
 * 
 *      This module should be further developed, despite 95% of basic functionality achieved. 
 * 
 *      Prescale value of 1:1, with crystals lower than a certain speed, won't 
 *      interrupt properly if you set the interrupt period too low.  
 * 
 * All rights reserved. Copyright Archangel Systems Inc. 2022
 */


/**************  Included Files **************************/
#include "Timer23.h"
#include "../COM/pic_h/p30F6014A.h"


/**************  Macro Definitions ***********************/
#define MS_WORD_MASK 0xFFFF0000u
#define LS_WORD_MASK 0x0000FFFFu
#define MAX_DELAY_MS 1000u


/**************  Local Variables *************************/
static uint32_t scaleFactor;
static bool isTimer23Initialized = false;


/**************  Function Definitions ********************/

/* Function: Timer23_Initialize
 *
 * Description: Inputs the 16 bit configuration value and 32bit timer period.
 *      The configuration value is written directly to the t2 configuration 
 *      register. The period input is split into two seperate words, each 
 *      written to the respective period register. Sets the module-level 
 *      scale factor based on config data. This allows the timer to be 
 *      reused/configurable if a different prescaler value or Fosc is used. 
 *      
 * Return: None (void)
 * 
 * Requirement Implemented: REL.0104.S.IOP.7.001
 */
void Timer23_Initialize( const uint16_t t2config, /* Configuration register value */
                         const uint32_t timerPeriod, /* 32 bit timer period: PR3-PR2 */
                         const uint32_t configScaleFactor ) /* Scale factor to set sampling period to 1ms */
{
    if (0 == configScaleFactor)
    {
        return; // Invalid scale factor could cause zero division errors. Function should be exited. 
    }

    scaleFactor = configScaleFactor;
    T2CON = t2config;
    PR3 = (uint16_t) ((timerPeriod & MS_WORD_MASK) >> 16);
    PR2 = (uint16_t) (timerPeriod & LS_WORD_MASK);
    isTimer23Initialized = true;

    /* Disable interrupts */
    IEC0bits.T3IE = 0;
    IEC0bits.T2IE = 0;
    return;
}

/* Function: Timer23_GetTimestamp_ms
 *
 * Description: Reads the TMR2 register. Reading TMR2 register causes the TMR3HLD
 *      register to hold the value of TMR3. This ensures an atomic read. Concatenates
 *      the msw and lsw into one single word. Divides by a scale factor to convert 
 *      instruction counts into milliseconds. 
 * 
 *      Note: A future improvement could be to add 1/2 the scale factor to the 
 *      concatenated 32bit word. This would prevent truncation bias when reading
 *      timestamps (similar to adding 0.5f to a float before casting to int). 
 * 
 * Return: Running timestamp in milliseconds 
 * 
 * Requirement Implemented: REL.0104.S.IOP.7.002
 */
uint32_t Timer23_GetTimestamp_ms( )
{
    uint32_t returnVal;
    if ((true == isTimer23Initialized) &&
            (scaleFactor != 0))
    {
        uint16_t lsWord = TMR2;
        uint32_t msWord = TMR3HLD;
        returnVal = (((msWord << 16) | lsWord) / scaleFactor);
    }
    else
    {
        returnVal = 0;
    }
    return returnVal;
}

/* 
 * Function: Timer23_Delay_ms
 * 
 * Description: Performs a delay operation using a while loop. Breaks upon reaching
 *      the user specified delay in milliseconds. 
 * 
 * Important notes: 
 *      1. This function has a built in guard for entering loops 
 *      longer than one second. This function was designed for a very specific
 *      implementation and acts as a safeguard for errenous operations. A more 
 *      verbose function should be developed in the future. 
 * 
 *      2. Not accurate around 1-2 ms. Gets accurate around 3. The configuration
 *      scaleFactor value should be further tested and fine tuned.  
 * 
 * Return: True if a successful wait occured. False if invalid conditions. 
 * 
 * Requirement Implemented: REL.0104.S.IOP.7.003
 */
void Timer23_Delay_ms( uint32_t delayInMilliseconds )
{
    if (false == isTimer23Initialized)
    {
        return;
    }

    /* Guard against entering loops longer than 1 second */
    if (delayInMilliseconds > MAX_DELAY_MS)
    {
        delayInMilliseconds = MAX_DELAY_MS;
    }
    
    uint32_t startTimestamp = Timer23_GetTimestamp_ms( );
    uint32_t currentTimestamp;
    while (true)
    {
        currentTimestamp = Timer23_GetTimestamp_ms( );
        if ((currentTimestamp - startTimestamp) > delayInMilliseconds)
        {
            break;
        }
    }
    return;
}

/* End of Timer23.c source file */
