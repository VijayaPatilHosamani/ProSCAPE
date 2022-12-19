/*
 * Filename: IOPConfig.c
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Module to define configuration data in the configuration block. 
 * 
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
 */


/****************** Included File(s) ******************/
#include "IOPConfig.h"

/* Initialize the union used for configuration data */
__psv__ volatile union configuration_variables IOPConfig = {

    /* Timer 4 Hardware Parameters */
    .hardwareSettings.TMR4CounterConfig = 0x8010,
    .hardwareSettings.TMR4CounterPeriod = 0x47FF,
    .hardwareSettings.TMR4InterruptConfig = 0x000D,

    /* ARINC429 Driver Settings */
    .hardwareSettings.hi3584txvrAconfig = 0xA038,
    .hardwareSettings.hi3584txvrBconfig = 0x8038,

    /* RAM AND ROM TEST SETTINGS */
    .hardwareSettings.RAMTestStartAddress = 0x0800,
    .hardwareSettings.RAMTestEndAddress = 0x1800,
    .hardwareSettings.RAMTestWriteWord1 = 0xA5A5,
    .hardwareSettings.RAMTestReadWord1 = 0xA5A5,
    .hardwareSettings.RAMTestWriteWord2 = 0x5A5A,
    .hardwareSettings.RAMTestReadWord2 = 0x5A5A,
    .hardwareSettings.CRCGenerationKey = 0x04C11DB7,

    /* UART1 Settings */
    .hardwareSettings.UART1InterruptConfig = 0x00BC,
    .hardwareSettings.UART1BaudRate = 0x000F, /* 115200 Baud rate is 0x0007, 57600 Baud rate is 0x000F, 0x30 Baud Rate is 19200, 0xA = 83000 */
    .hardwareSettings.UART1ModeConfig = 0x8000,
    .hardwareSettings.UART1StatusConfig = 0x0400,

    /* UART2 Settings */
    .hardwareSettings.UART2InterruptConfig = 0x00BC,
    .hardwareSettings.UART2BaudRate = 0x000F, /* 115200 Baud rate is 0x0007, 57600 Baud rate is 0x000F, 0x30 Baud Rate is 19200, 0xA = 83000 */
    .hardwareSettings.UART2ModeConfig = 0x8000,
    .hardwareSettings.UART2StatusConfig = 0x0400,

    /* Timer23 Config - sets up the return function to act as 1ms timer*/
    .hardwareSettings.TMR23Config = 0x8038,
    .hardwareSettings.TMR23Period = 0xFFFFFFFF, 
    .hardwareSettings.TMR23ScaleFactor = 114u,


    /************************************ IIR Filter Settings **************************************/
    .iirFilter.IIRFilterK1 = 0.7777678f,
    .iirFilter.IIRFilterK2 = 0.2222322f,


    /************************************ IIR Diff  Settings **************************************/
    .iirDiffSettings.K1 = 0.99f,
    .iirDiffSettings.IIRDiffSampleRate_Hz = 50.0f,
    .iirDiffSettings.IIRDiffUpperDelta = 360.0f,
    .iirDiffSettings.IIRDiffLowerDelta = -360.0f,
    .iirDiffSettings.IIRDiffUpperLimit = 180.0f,
    .iirDiffSettings.IIRDiffLowerLimit = -180.0f,
};

/*   End of IOPConfig.c source file. */