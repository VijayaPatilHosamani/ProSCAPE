/* Filename: IOPConfig.h
 *
 * Author: Henry Gilbert
 * 
 * Date: 2 August 2022
 * 
 * Description: Declaration of configuration variables union. 
 * 
 * 
 * All Rights Reserved. Copyright Archangel Systems 2022  
 */

#ifndef IOP_CONFIG_H
#define IOP_CONFIG_H

#include <stdint.h>

#define CONFIG_BLOCK_START_ADDRESS 0x12000
#define CONFIG_BLOCK_LENGTH 0x5000

typedef struct 
{
    float IIRFilterK1;
    float IIRFilterK2;
} IIRFilterConfigurationVars ;

typedef struct
{
    float K1;
    float IIRDiffSampleRate_Hz;
    float IIRDiffUpperLimit;
    float IIRDiffLowerLimit;
    float IIRDiffUpperDelta;
    float IIRDiffLowerDelta;
} IIRDiffConfigVars ;

/* Hardware configuration settings */
typedef struct
{
    uint16_t TMR4CounterConfig; /* Timer 4 Counter Configuration data. */
    uint16_t TMR4CounterPeriod; /* Timer 4 Counter Period data. */
    uint16_t TMR4InterruptConfig; /* Timer 4 Interrupt Configuration data. */

    uint16_t hi3584txvrAconfig;
    uint16_t hi3584txvrBconfig;

    uint16_t RAMTestStartAddress; /* RAM test Start Address. */
    uint16_t RAMTestEndAddress; /* RAM test End Address. */
    uint16_t RAMTestWriteWord1; /* Ram Test Memory Write Word 1. */
    uint16_t RAMTestReadWord1; /* Ram Test Memory Read Word 1. */
    uint16_t RAMTestWriteWord2; /* Ram Test Memory Write Word 2. */
    uint16_t RAMTestReadWord2; /* Ram Test Memory Read Word 2. */
    uint32_t CRCGenerationKey; /* CRC generation Key. */

    uint16_t UART1InterruptConfig;
    uint16_t UART1BaudRate;
    uint16_t UART1ModeConfig;
    uint16_t UART1StatusConfig;

    uint16_t UART2InterruptConfig;
    uint16_t UART2BaudRate;
    uint16_t UART2ModeConfig;
    uint16_t UART2StatusConfig;

    /* Time 2-3 */
    uint16_t TMR23Config;
    uint32_t TMR23Period;
    uint32_t TMR23ScaleFactor;
} HardwareConfigVars;

typedef struct
{
    uint16_t uart1LoopbackModeSettings;
} maintenanceModeSettings;
;

union configuration_variables
{
    uint8_t byte[CONFIG_BLOCK_LENGTH];

    struct
    {
        IIRFilterConfigurationVars iirFilter;
        IIRDiffConfigVars iirDiffSettings;
        HardwareConfigVars hardwareSettings;
        maintenanceModeSettings mxModeSettings;
    };
};


extern __psv__ volatile union configuration_variables IOPConfig __attribute__((section(".CONFIG"),
        space(psv),
        address(CONFIG_BLOCK_START_ADDRESS)));

extern __prog__ volatile uint32_t u32PM_CRC __attribute__((section(".PM_CRC"), space(prog)));
// TODO add program memory/configuration block CRC? 

#endif 