/* 
 * Filename: ARINC_HI3584.c 
 *
 * Description: Hardware level abstraction layer for the HI3584 parallel
 *      ARINC429 transceiver chip. This module provides a hardware interface
 *      to two ARINC429 transceiver chips. 
 * 
 * The data bus direction is specifically configured before each desired read or 
 * write to the bus. Therefore, the direction does not have to return to a 
 * known value after any operation. 
 * 
 * Author: Henry Gilbert
 * 
 * Date: 28 June 2022
 * 
 * All Rights Reserved: Copyright Archangel Systems 2022
 */

/**************  Include Files  ************************/
#include "ARINC_HI3584.h"

/**************  Macro Definition(s) ***********************/
#define MAX_NUM_REGOCNIZED_LABELS 16 //label filter setup 

/**************  Type Definition(s) ************************/
typedef enum ARINC429_HI3584_DataBusDir_t
{
    ARINC429_HI3584_DATA_BUS_DIR_OUTPUT, /* Specifies the data bus direction as input */
    ARINC429_HI3584_DATA_DIR_INPUT /* Specifies the data bus direction as output */
} ARINC429_HI3584_DataBusDir;

/**************  Local Constant(s) *************************/
static const size_t txvrRxFIFOsize = 32; //32 ; // Size of each HI3584 receiver buffers
static const uint32_t lpTestData = 0xA5A5A500; // Loop back test data
static const uint32_t lpTestRx1ReadbackVal = 0xA5A5A500; // Loop back test rx1 expected read-back value 
static const uint32_t lpTestRx2ReadbackVal = 0xDA5A5AFF; // Loop back test rx2 expected read-back value
static const uint32_t lpTestMaxDelay = 50000; // Software delay for loop back test
static const size_t lpTestNumCycles = 50; // Number of cycles to perform during loop back test


/**************  Static Function Definition(s) *************/

/* This function configures the direction of the 16-bit data bus interfaced with the ARINC transceivers. */
static void Config16bitDataBusDirection( const ARINC429_HI3584_DataBusDir busDirection );

/* This function writes data onto the 16-bit data bus interfaced with the ARINC transceivers. */
static void WriteDataTo16bitDataBus( const uint16_t dataBusWriteValue );

/* This function reads data from the 16-bit data bus interfaced with the ARINC transceivers. */
static uint16_t ReadDataFrom16bitDataBus( void );

/* Reads back the value of transceiver A control register*/
static uint16_t ARINC429_HI3584_txvrA_ReadBackControlRegister( );

/* Reads back the value of transceiver B control register*/
static uint16_t ARINC429_HI3584_txvrB_ReadBackControlRegister( );


/**********************   Functions     ***************************/

/*
 * Function: ARINC429_HI3584_txvrA_Initialize
 *
 * Description: The control and status signals connected to the first of 
 *      two HI-3584 ARINC transceivers are configured as inputs or outputs,
 *      and if configured as an output, their pin states are set to the default state.
 *
 * Return: None 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.004
 */
void ARINC429_HI3584_txvrA_Initialize( )
{
    /**** Configure Outputs ****/
    /* Select signal */
    ARINC429_HI3584_TXVRA_SEL_TRIS = 0;
    ARINC429_HI3584_TXVRA_SEL = 0;

    /* Enable 1 signal */
    ARINC429_HI3584_TXVRA_EN1_TRIS = 0;
    ARINC429_HI3584_TXVRA_EN1 = 1;

    /* Enable 2 signal */
    ARINC429_HI3584_TXVRA_EN2_TRIS = 0;
    ARINC429_HI3584_TXVRA_EN2 = 1;

    /* Latch Enable 1 signal */
    ARINC429_HI3584_TXVRA_PL1_TRIS = 0;
    ARINC429_HI3584_TXVRA_PL1 = 1;

    /* Latch Enable 2 signal */
    ARINC429_HI3584_TXVRA_PL2_TRIS = 0;
    ARINC429_HI3584_TXVRA_PL2 = 1;

    /* Enable Transmit signal */
    ARINC429_HI3584_TXVRA_ENTX_TRIS = 0;
    ARINC429_HI3584_TXVRA_ENTX = 1;

    /* Control Strobe signal */
    ARINC429_HI3584_TXVRA_CWSTR_TRIS = 0;
    ARINC429_HI3584_TXVRA_CWSTR = 1;

    /* Read Status Register signal */
    ARINC429_HI3584_TXVRA_RSR_TRIS = 0;
    ARINC429_HI3584_TXVRA_RSR = 1;

    /**** Configure Inputs ****/
    ARINC429_HI3584_TXVRA_DR1_TRIS = 1; /* Data ready on receiver 1 */
    ARINC429_HI3584_TXVRA_DR2_TRIS = 1; /* Data ready on receiver 2 */
    ARINC429_HI3584_TXVRA_FFT_TRIS = 1; /* Transmit buffer full */

    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT ); /* Configure the direction of the 16 data bus as input */
    return;
}

/* Function: ARINC429_HI3584_txvrB_Initialize
 *
 * Description:  The control and status signals connected to the second 
 *      of two HI-3584 ARINC transceivers are configured as inputs or outputs,
 *      and if configured as an output, their pin states are set to the default state.
 *
 * Return: None 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.011
 */
void ARINC429_HI3584_txvrB_Initialize( )
{
    /**** Configure Outputs ****/
    /* Select signal */
    ARINC429_HI3584_TXVRB_SEL_TRIS = 0;
    ARINC429_HI3584_TXVRB_SEL = 0;

    /* Enable 1 signal */
    ARINC429_HI3584_TXVRB_EN1_TRIS = 0;
    ARINC429_HI3584_TXVRB_EN1 = 1;

    /* Enable 2 signal */
    ARINC429_HI3584_TXVRB_EN2_TRIS = 0;
    ARINC429_HI3584_TXVRB_EN2 = 1;

    /* Latch Enable 1 signal */
    ARINC429_HI3584_TXVRB_PL1_TRIS = 0;
    ARINC429_HI3584_TXVRB_PL1 = 1;

    /* Latch Enable 2 signal */
    ARINC429_HI3584_TXVRB_PL2_TRIS = 0;
    ARINC429_HI3584_TXVRB_PL2 = 1;

    /* Enable Transmit signal */
    ARINC429_HI3584_TXVRB_ENTX_TRIS = 0;
    ARINC429_HI3584_TXVRB_ENTX = 1;

    /* Control Strobe signal */
    ARINC429_HI3584_TXVRB_CWSTR_TRIS = 0;
    ARINC429_HI3584_TXVRB_CWSTR = 1;

    /* Read Status Register signal */
    ARINC429_HI3584_TXVRB_RSR_TRIS = 0;
    ARINC429_HI3584_TXVRB_RSR = 1;

    /**** Configure Inputs ****/
    ARINC429_HI3584_TXVRB_DR1_TRIS = 1; /* Data ready on receiver 1 */
    ARINC429_HI3584_TXVRB_DR2_TRIS = 1; /* Data ready on receiver 2 */
    ARINC429_HI3584_TXVRB_FFT_TRIS = 1; /* Transmit buffer full */

    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT ); /* Configure the direction of the 16 data bus as input */

    return;
}

/* Function: Config16bitDataBusDirection
 * 
 * Description: This function configures the direction of the 16-bit data bus signals. 
 * The direction of the data bus can be configured as digital output/input for either 
 * sending or receiving data from the ARINC device.
 *
 * Return: None 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.001
 */
static void Config16bitDataBusDirection( const ARINC429_HI3584_DataBusDir busDirection ) /* Data Bus Configuration Register. */
{
    uint16_t trisVal = (ARINC429_HI3584_DATA_DIR_INPUT == busDirection) ? 1 : 0;
    DB00_TRIS = trisVal;
    DB01_TRIS = trisVal;
    DB02_TRIS = trisVal;
    DB03_TRIS = trisVal;
    DB04_TRIS = trisVal;
    DB05_TRIS = trisVal;
    DB06_TRIS = trisVal;
    DB07_TRIS = trisVal;
    DB08_TRIS = trisVal;
    DB09_TRIS = trisVal;
    DB10_TRIS = trisVal;
    DB11_TRIS = trisVal;
    DB12_TRIS = trisVal;
    DB13_TRIS = trisVal;
    DB14_TRIS = trisVal;
    DB15_TRIS = trisVal;
    return;
}

/**
 * @brief This function writes the data on to the 16 bit data bus interfaced with the ARINC devices.
 *
 * Description:
 * The data word is loaded into the 16 bus signals interfaced with the ARINC devices.
 * 
 * Return: None 
 * 
 * Requirement(s) Implemented: INT1.0102.S.IOP.6.003.D01
 */
static void WriteDataTo16bitDataBus( const uint16_t dataBusWriteValue ) /* Write data to be loaded into the data bus. */
{
    DB00_WRITE = dataBusWriteValue & 1;
    DB01_WRITE = ((dataBusWriteValue >> 1) & 1);
    DB02_WRITE = ((dataBusWriteValue >> 2) & 1);
    DB03_WRITE = ((dataBusWriteValue >> 3) & 1);
    DB04_WRITE = ((dataBusWriteValue >> 4) & 1);
    DB05_WRITE = ((dataBusWriteValue >> 5) & 1);
    DB06_WRITE = ((dataBusWriteValue >> 6) & 1);
    DB07_WRITE = ((dataBusWriteValue >> 7) & 1);
    DB08_WRITE = ((dataBusWriteValue >> 8) & 1);
    DB09_WRITE = ((dataBusWriteValue >> 9) & 1);
    DB10_WRITE = ((dataBusWriteValue >> 10) & 1);
    DB11_WRITE = ((dataBusWriteValue >> 11) & 1);
    DB12_WRITE = ((dataBusWriteValue >> 12) & 1);
    DB13_WRITE = ((dataBusWriteValue >> 13) & 1);
    DB14_WRITE = ((dataBusWriteValue >> 14) & 1);
    DB15_WRITE = ((dataBusWriteValue >> 15) & 1);
    return;
}

/* Function: ReadDataFrom16bitDataBus
 * 
 * Description: The data bus signals are read and returned as a 16 bit word.
 *
 * Return: None 
 * 
 * Requirement(s) Implemented: INT1.0102.S.IOP.6.003.D02
 */
static uint16_t ReadDataFrom16bitDataBus( void )
{
    /* Read data bus pins bit-by-bit */
    uint16_t busRead = (DB15_READ & 1);
    busRead = (busRead << 1) | (DB14_READ & 1);
    busRead = (busRead << 1) | (DB13_READ & 1);
    busRead = (busRead << 1) | (DB12_READ & 1);
    busRead = (busRead << 1) | (DB11_READ & 1);
    busRead = (busRead << 1) | (DB10_READ & 1);
    busRead = (busRead << 1) | (DB09_READ & 1);
    busRead = (busRead << 1) | (DB08_READ & 1);
    busRead = (busRead << 1) | (DB07_READ & 1);
    busRead = (busRead << 1) | (DB06_READ & 1);
    busRead = (busRead << 1) | (DB05_READ & 1);
    busRead = (busRead << 1) | (DB04_READ & 1);
    busRead = (busRead << 1) | (DB03_READ & 1);
    busRead = (busRead << 1) | (DB02_READ & 1);
    busRead = (busRead << 1) | (DB01_READ & 1);
    busRead = (busRead << 1) | (DB00_READ & 1);
    return busRead;
}

/* Function: ARINC429_HI3584_txvrA_rx1_ReadWord
 *
 * Description: The data is read from the ARINC device by pulsing the EN1 pin. 
 * SEL pin is used to select between the lower upper 16 bit of data.
 *
 * Return: 32bit ARINC429 word 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.009
 */
uint32_t ARINC429_HI3584_txvrA_rx1_ReadWord( void )
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );

    ARINC429_HI3584_TXVRA_EN1 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRA_EN2 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRA_SEL = 0; /* Select the lower 16 bits for read operation. */
    ARINC429_HI3584_TXVRA_EN1 = 0; /* Loads the data bus with the lower 16-bits of the received ARINC Message. */
    uint32_t ARINCwordRead = ReadDataFrom16bitDataBus( ); /* Read the lower 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRA_EN1 = 1; /* Set back to default state */
    ARINC429_HI3584_TXVRA_SEL = 1; /* Select the upper 16 bits for read operation. */
    ARINC429_HI3584_TXVRA_EN1 = 0; /* Loads the data bus with the upper 16-bits of the received ARINC Message. */
    ARINCwordRead |= ((uint32_t) ReadDataFrom16bitDataBus( )) << 16; /* Read the upper 16 bits of the ARINC message */
    ARINC429_HI3584_TXVRA_EN1 = 1; /* Set back to default state */

    return ARINCwordRead;
}

/* Function: ARINC429_HI3584_txvrA_rx2_ReadWord
 *
 * Description: The data is read from the ARINC device by pulsing the EN2 pin. 
 * SEL pin is used to select between the lower upper 16 bit of data.
 *
 * Return: 32bit ARINC429 Word 
 *
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.010
 */
uint32_t ARINC429_HI3584_txvrA_rx2_ReadWord( void )
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );

    ARINC429_HI3584_TXVRA_EN1 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRA_EN2 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRA_SEL = 0; /* Select the lower 16 bits for read operation. */
    ARINC429_HI3584_TXVRA_EN2 = 0; /* Loads the data bus with the lower 16-bits of the received ARINC Message. */
    uint32_t ARINCwordRead = ReadDataFrom16bitDataBus( ); /* Read the lower 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRA_EN2 = 1; /* Set back to default state */
    ARINC429_HI3584_TXVRA_SEL = 1; /* Select the upper 16 bits for read operation. */
    ARINC429_HI3584_TXVRA_EN2 = 0; /* Loads the data bus with the upper 16-bits of the received ARINC Message. */
    ARINCwordRead |= ((uint32_t) ReadDataFrom16bitDataBus( )) << 16; /* Read the upper 16 bits of the ARINC message */
    ARINC429_HI3584_TXVRA_EN2 = 1; /* Set back to default state */

    return ARINCwordRead;
}

/* Function: ARINC429_HI3584_txvrA_TransmitWord
 *
 * Description: The data is loaded into the ARINC device by pulsing the PL1 
 * and PL2 pins to load the lower and higher 16bit words respectively.
 *
 * Return: None 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.008
 */
void ARINC429_HI3584_txvrA_TransmitWord( const uint32_t ARINCword ) /* 32-bit ARINC word to transmit */
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_BUS_DIR_OUTPUT );
    WriteDataTo16bitDataBus( (uint16_t) (ARINCword & 0xFFFF) ); /* Output lower 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRA_PL1 = 0;
    Nop( );
    ARINC429_HI3584_TXVRA_PL1 = 1;

    WriteDataTo16bitDataBus( (uint16_t) ((ARINCword >> 16) & 0xFFFF) ); /* Output upper 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRA_PL2 = 0;
    Nop( );
    ARINC429_HI3584_TXVRA_PL2 = 1;

    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );

    return;
}

/* Function: ARINC429_HI3584_txvrA_LoadCtrlReg
 *
 * Description: The data is loaded into the ARINC device 
 * by pulsing the ( uint16_t ) u16Signal_CWSTR pin, the load operation is
   verified by reading the configuration data back for validation.
 *
 * Return: True if written control register equals the desired written value 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.005
 */
bool ARINC429_HI3584_txvrA_LoadCtrlReg( const uint16_t ctrlRegVal ) /* transceiver control register value */
{
    /* Write Control Register */
    ARINC429_HI3584_TXVRA_SEL = 0; /* Select the configuration data load operation */
    ARINC429_HI3584_TXVRA_CWSTR = 0; /* Release the data bus */
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_BUS_DIR_OUTPUT );
    WriteDataTo16bitDataBus( ctrlRegVal ); /* Write the control register value to the 16-bit data bus. */
    ARINC429_HI3584_TXVRA_CWSTR = 1; /* Upload the data on the data bus into the ARINC transceiver control register. */

    /* Read Control Register */
    uint16_t readBack = ARINC429_HI3584_txvrA_ReadBackControlRegister( );
    return (ctrlRegVal == readBack);
}

/* Function: ARINC429_HI3584_txrA_ReadBackControlRegister
 * 
 * Description: Reads the desired control register and returns the value
 * 
 * Return: Control register value, MSB first
 * 
 * Requirement Implemented: INT1.0101.S.IOP.1.007
 * 
 */

static uint16_t ARINC429_HI3584_txvrA_ReadBackControlRegister( )
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );
    ARINC429_HI3584_TXVRA_SEL = 1; /* Select configuration data read operation. */
    ARINC429_HI3584_TXVRA_RSR = 0; /* Load the 16-bit data bus with the control register data from the ARINC transceiver */
    uint16_t controlRegReadback = ReadDataFrom16bitDataBus( ); /* Read the control register value from the 16-bit data bus. */
    ARINC429_HI3584_TXVRA_RSR = 1; /* Signal the ARINC transceiver to release the 16-bit data bus. */
    ARINC429_HI3584_TXVRA_SEL = 0; /* Set back to default state. */
    return controlRegReadback;
}

/* Function: ARINC429_HI3584_txvrB_rx1_ReadWord
 * 
 * Description: The data is read from the ARINC device by pulsing the 
 * EN1 pin. SEL pin is used to select between the lower upper 16 bit of data.
 *
 * Return: 32bit ARINC429 word from receiver 1
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.016
 */
uint32_t ARINC429_HI3584_txvrB_rx1_ReadWord( void )
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );

    ARINC429_HI3584_TXVRB_EN1 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRB_EN2 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRB_SEL = 0; /* Select the lower 16 bits for read operation. */
    ARINC429_HI3584_TXVRB_EN1 = 0; /* Loads the data bus with the lower 16-bits of the received ARINC Message. */
    uint32_t ARINCwordRead = ReadDataFrom16bitDataBus( ); /* Read the lower 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRB_EN1 = 1; /* Set back to default state */
    ARINC429_HI3584_TXVRB_SEL = 1; /* Select the upper 16 bits for read operation. */
    ARINC429_HI3584_TXVRB_EN1 = 0; /* Loads the data bus with the upper 16-bits of the received ARINC Message. */
    ARINCwordRead |= ((uint32_t) ReadDataFrom16bitDataBus( )) << 16; /* Read the upper 16 bits of the ARINC message */
    ARINC429_HI3584_TXVRB_EN1 = 1; /* Set back to default state */

    return ARINCwordRead;
}

/* Function: ARINC429_HI3584_txvrB_rx2_ReadWord
 * 
 * Description: The data is read from the ARINC device by pulsing the EN2 pin. 
 * SEL pin is used to select between the lower upper 16 bit of data.
 *
 * Return: None 
 *
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.017
 */
uint32_t ARINC429_HI3584_txvrB_rx2_ReadWord( void )
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );

    ARINC429_HI3584_TXVRB_EN1 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRB_EN2 = 1; /* Set to default state */
    ARINC429_HI3584_TXVRB_SEL = 0; /* Select the lower 16 bits for read operation. */
    ARINC429_HI3584_TXVRB_EN2 = 0; /* Loads the data bus with the lower 16-bits of the received ARINC Message. */
    uint32_t ARINCwordRead = ReadDataFrom16bitDataBus( ); /* Read the lower 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRB_EN2 = 1; /* Set back to default state */
    ARINC429_HI3584_TXVRB_SEL = 1; /* Select the upper 16 bits for read operation. */
    ARINC429_HI3584_TXVRB_EN2 = 0; /* Loads the data bus with the upper 16-bits of the received ARINC Message. */
    ARINCwordRead |= ((uint32_t) ReadDataFrom16bitDataBus( )) << 16; /* Read the upper 16 bits of the ARINC message */
    ARINC429_HI3584_TXVRB_EN2 = 1; /* Set back to default state */

    return ARINCwordRead;
}

/* Function: ARINC429_HI3584_txvrB_TransmitWord
 * 
 * Description: The data is loaded into the ARINC device by pulsing the PL1 
 * and PL2 pins to load the lower and higher 16bit words respectively.
 *
 * Return: None 
 *
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.015
 */
void ARINC429_HI3584_txvrB_TransmitWord( const uint32_t ARINCword ) /* 32-bit ARINC word to transmit */
{
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_BUS_DIR_OUTPUT );
    WriteDataTo16bitDataBus( (uint16_t) (ARINCword & 0xFFFF) ); /* Output lower 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRB_PL1 = 0;
    Nop( );
    ARINC429_HI3584_TXVRB_PL1 = 1;

    WriteDataTo16bitDataBus( (uint16_t) ((ARINCword >> 16) & 0xFFFF) ); /* Output upper 16 bits of the ARINC message */

    ARINC429_HI3584_TXVRB_PL2 = 0;
    Nop( );
    ARINC429_HI3584_TXVRB_PL2 = 1;

    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );

    return;
}

/* Function: ARINC429_HI3584_txvrB_LoadCtrlReg
 *
 * Detailed Description: The data is loaded into the ARINC device by pulsing 
 * the ARINC429_HI3584_TXVRB_CWSTR pin, the load operation is verified by reading 
 * the configuration data back for validation.
 *
 * Return: True if written value matches input, false if otherwise. 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.012
 */
bool ARINC429_HI3584_txvrB_LoadCtrlReg( const uint16_t ctrlRegVal ) /* control register value */
{
    /* Write Control Register.*/
    ARINC429_HI3584_TXVRB_SEL = 0; /* Select the configuration data load operation */
    ARINC429_HI3584_TXVRB_CWSTR = 0; /* Release the data bus. */
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_BUS_DIR_OUTPUT );
    WriteDataTo16bitDataBus( ctrlRegVal ); /* Write the control register value to the 16-bit data bus */
    ARINC429_HI3584_TXVRB_CWSTR = 1; /* Upload the data on the data bus into the ARINC transceiver control register. */

    /* Read Control Register.*/
    uint16_t readBackValue = ARINC429_HI3584_txvrB_ReadBackControlRegister( );
    return (ctrlRegVal == readBackValue);
}

/* Function: ARINC429_HI3584_txvrB_ReadBackControlRegister
 * 
 * Description: Reads back the 16 bit control register value for transceiver B.
 *
 * Return: Control register value
 * 
 * Requirement Implemented: INT1.0101.S.IOP.1.014
 */
static uint16_t ARINC429_HI3584_txvrB_ReadBackControlRegister( )
{
    /* Read Control Register.*/
    Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );
    ARINC429_HI3584_TXVRB_SEL = 1; /* Select configuration data read operation */
    ARINC429_HI3584_TXVRB_RSR = 0; /* Load the 16-bit data bus with the control register data from the ARINC transceiver */
    uint16_t controlRegReadback = ReadDataFrom16bitDataBus( ); /* Read the control register value from the 16-bit data bus. */
    ARINC429_HI3584_TXVRB_RSR = 1; /* Pull the RSR pin high to release the 16 bit data bus by the ARINC device. */
    ARINC429_HI3584_TXVRB_SEL = 0; /* Pull the SEL pin low to its default state. */
    return controlRegReadback;
}

/* Function: ARINC429_HI3584_txvrA_LoopbackTest
 *
 * Description: Loop back test is performed by sending a known Message on 
 *      the ARINC transmit and verifying the received values.
 *
 * Return: True if loopback test was successful, false if otherwise.
 *
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.006
 */
bool ARINC429_HI3584_txvrA_LoopbackTest( void )
{
    uint16_t currentCtrRegValue = ARINC429_HI3584_txvrA_ReadBackControlRegister( );
    bool status = ARINC429_HI3584_txvrA_LoadCtrlReg( 0x8000 ); /* Enable self test (i.e. loop back) mode */

    // Flush receiver FIFOs first before sending test words
    size_t currFIFOflushCount = 0;
    while (currFIFOflushCount <= txvrRxFIFOsize)
    {
        ARINC429_HI3584_txvrA_rx1_ReadWord( );
        currFIFOflushCount++;
    }

    currFIFOflushCount = 0;
    while (currFIFOflushCount <= txvrRxFIFOsize)
    {
        ARINC429_HI3584_txvrA_rx2_ReadWord( );
        currFIFOflushCount++;
    }

    // Send test words and verify read-back value
    size_t counter = 0;
    uint32_t rx1readback = lpTestRx1ReadbackVal;
    uint32_t rx2readback = lpTestRx2ReadbackVal;
    while ( (counter < lpTestNumCycles) &&
            ( ( (lpTestRx1ReadbackVal == rx1readback) && (lpTestRx2ReadbackVal == rx2readback) ) || (1 == counter) ) ) // Ignores 1st readback 
        {
            ARINC429_HI3584_txvrA_TransmitWord( lpTestData );

            uint32_t delayCounter = 0;
            while (((1 == ARINC429_HI3584_TXVRA_DR1) || (1 == ARINC429_HI3584_TXVRA_DR2)) && (delayCounter < lpTestMaxDelay))
            {
                delayCounter++;
            }
            rx1readback = ARINC429_HI3584_txvrA_rx1_ReadWord( );
            rx2readback = ARINC429_HI3584_txvrA_rx2_ReadWord( );
            counter++;
        }

    status &= ((lpTestRx1ReadbackVal == rx1readback) && (lpTestRx2ReadbackVal == rx2readback)) ? true : false;
            ARINC429_HI3584_txvrA_LoadCtrlReg( currentCtrRegValue );

    return status;
}

/* Function: ARINC429_HI3584_txvrB_LoopbackTest
 * 
 * Description: Loop back test is performed by sending a known Message 
 *      on the ARINC transmit and verifying the received values.
 *
 * Return: True if loopback test succeeded, false if otherwise. 
 * 
 * Requirement(s) Implemented: INT1.0101.S.IOP.1.013
 */
bool ARINC429_HI3584_txvrB_LoopbackTest( void )
{
    uint16_t currentCtrRegValue = ARINC429_HI3584_txvrB_ReadBackControlRegister( );
            bool status = ARINC429_HI3584_txvrB_LoadCtrlReg( 0x8000 ); /* Enable self test (i.e. loop back) mode */

            // Flush receiver FIFOs before sending test words
            size_t currFIFOflushCount = 0;
    while (currFIFOflushCount <= txvrRxFIFOsize)
    {
        ARINC429_HI3584_txvrB_rx1_ReadWord( );
                currFIFOflushCount++;
    }

    currFIFOflushCount = 0;
    while (currFIFOflushCount <= txvrRxFIFOsize)
    {
        ARINC429_HI3584_txvrB_rx2_ReadWord( );
                currFIFOflushCount++;
    }

    // Send test words and verify read-back value
    uint32_t counter = 0;
            uint32_t rx1readback = lpTestRx1ReadbackVal;
            uint32_t rx2readback = lpTestRx2ReadbackVal;

    while ( (counter < lpTestNumCycles) &&
            ( ( (lpTestRx1ReadbackVal == rx1readback) && (lpTestRx2ReadbackVal == rx2readback) ) || (1 == counter) ) ) // Ignores 1st readback 
        {
            ARINC429_HI3584_txvrB_TransmitWord( lpTestData );

            uint32_t delayCounter = 0;
            //        while (delayCounter < lpTestMaxDelay)
            while (((1 == ARINC429_HI3584_TXVRB_DR1) || (1 == ARINC429_HI3584_TXVRB_DR2)) && (delayCounter < lpTestMaxDelay))
            {
                delayCounter++;
            }

            rx1readback = ARINC429_HI3584_txvrB_rx1_ReadWord( );
            rx2readback = ARINC429_HI3584_txvrB_rx2_ReadWord( );
            counter++;
        }

    status &= ((lpTestRx1ReadbackVal == rx1readback) && (lpTestRx2ReadbackVal == rx2readback)) ? true : false;
            ARINC429_HI3584_txvrB_LoadCtrlReg( currentCtrRegValue );

    return status;
}

/* Function: ARINC429_HI3584_SetupLabelFiltersTxrA
 *
 * Description: Sets the transceiver A label filters to only recognize the 
 *      labels from the ARINC429 Rx message array. Only works when 
 *      the number of messages in the rx array is less than 16. Has three
 *      tries to successfully read back all subscribed labels. 
 * 
 * Return: Returns true if the readback was successful. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.1.018
 */
bool ARINC429_HI3584_SetupLabelFiltersTxvrA( const ARINC429_RxMsgArray * const msgs )
{
    if ((NULL == msgs) ||
            (msgs->numMsgs > MAX_NUM_REGOCNIZED_LABELS))
    {
        return false;
    }

    uint16_t rxLabelsTxrA[MAX_NUM_REGOCNIZED_LABELS]; // Note: These labels are written in hex-flipped format. 
            size_t counter;
    for (counter = 0; counter < msgs->numMsgs; counter++)
    {
        rxLabelsTxrA[counter] = msgs->rxMsgs[counter].msgConfig.label;
    }
    for (; counter < MAX_NUM_REGOCNIZED_LABELS; counter++)
    {
        rxLabelsTxrA[counter] = 0;
    }

    /* Save the current control register value */
    uint16_t currentControlReg = ARINC429_HI3584_txvrA_ReadBackControlRegister( );

            const size_t maxNumRetries = 3;
            size_t retryCounter = 0;
            bool isReadBackValid;
    while (retryCounter < maxNumRetries)
    {
        isReadBackValid = true;
                /* Load Txr B rx label filters */
                ARINC429_HI3584_TXVRA_SEL = 1;
                ARINC429_HI3584_txvrA_LoadCtrlReg( 0x02 ); // 2 allows label filter mode
                Config16bitDataBusDirection( ARINC429_HI3584_DATA_BUS_DIR_OUTPUT );

        for (counter = 0; counter < MAX_NUM_REGOCNIZED_LABELS; counter++)
        {
            ARINC429_HI3584_TXVRA_PL2 = 0;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // 120 ns
                    WriteDataTo16bitDataBus( rxLabelsTxrA[counter] );
                    ARINC429_HI3584_TXVRA_PL2 = 1;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // 150 ns
        }

        /* Read back Txr B rx label filters*/
        Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );
                uint16_t readBackValue;
        for (counter = 0; counter < MAX_NUM_REGOCNIZED_LABELS; counter++)
        {
            ARINC429_HI3584_TXVRA_EN2 = 0;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); //
                    readBackValue = ReadDataFrom16bitDataBus( );
                    isReadBackValid &= (readBackValue == rxLabelsTxrA[counter]);
                    ARINC429_HI3584_TXVRA_EN2 = 1;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // decide on time
        }
        if (isReadBackValid)
        {
            break;
        }
        retryCounter++;
    }

    // If the readback is invalid, remove the label recognition bits 
    if (false == isReadBackValid)
    {
        currentControlReg &= 0x0008; // Clear bit 3 (starting from 0). 
    }

    ARINC429_HI3584_txvrA_LoadCtrlReg( currentControlReg ); // Restore the original control register value 

    return isReadBackValid;
}

/* Function: ARINC429_HI3584_SetupLabelFiltersTxrB
 *
 * Description: Sets the transceiver B label filters to only recognize the 
 *      labels from the ARINC429 Rx message array. Only works when 
 *      the number of messages in the rx array is less than 16. Has three
 *      tries to successfully read back all subscribed labels. 
 * 
 * Return: Returns true if the readback was successful. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.1.019
 */

bool ARINC429_HI3584_SetupLabelFiltersTxvrB( const ARINC429_RxMsgArray * const msgs )
{
    if ((NULL == msgs) ||
            (msgs->numMsgs > MAX_NUM_REGOCNIZED_LABELS))
    {
        return false;
    }

    uint8_t rxLabelsTxrB[MAX_NUM_REGOCNIZED_LABELS]; // Note: These labels are written in hex-flipped format. 
            size_t counter;
    for (counter = 0; counter < msgs->numMsgs; counter++)
    {
        rxLabelsTxrB[counter] = msgs->rxMsgs[counter].msgConfig.label;
    }
    for (; counter < MAX_NUM_REGOCNIZED_LABELS; counter++)
    {
        rxLabelsTxrB[counter] = 0;
    }

    /* Save the current control register value */
    uint16_t currentControlReg = ARINC429_HI3584_txvrB_ReadBackControlRegister( );

            const size_t maxNumRetries = 3;
            size_t retryCounter = 0;
            bool isReadBackValid;
    while (retryCounter < maxNumRetries)
    {
        isReadBackValid = true;
                /* Load Txr B rx label filters */
                ARINC429_HI3584_TXVRB_SEL = 1;
                ARINC429_HI3584_txvrB_LoadCtrlReg( 0x02 ); // 2 allows label filter mode
                Config16bitDataBusDirection( ARINC429_HI3584_DATA_BUS_DIR_OUTPUT );

        for (counter = 0; counter < MAX_NUM_REGOCNIZED_LABELS; counter++)
        {
            ARINC429_HI3584_TXVRB_PL2 = 0;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // 120 ns
                    WriteDataTo16bitDataBus( rxLabelsTxrB[counter] );
                    ARINC429_HI3584_TXVRB_PL2 = 1;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // 150 ns
        }

        /* Read back Txr B rx label filters*/
        Config16bitDataBusDirection( ARINC429_HI3584_DATA_DIR_INPUT );
                uint16_t readBackValue;
        for (counter = 0; counter < MAX_NUM_REGOCNIZED_LABELS; counter++)
        {
            ARINC429_HI3584_TXVRB_EN2 = 0;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // decide on time
                    readBackValue = ReadDataFrom16bitDataBus( );
                    isReadBackValid &= (readBackValue == rxLabelsTxrB[counter]);
                    ARINC429_HI3584_TXVRB_EN2 = 1;
                    Nop( );
                    Nop( );
                    Nop( );
                    Nop( ); // decide on time
        }
        if (isReadBackValid)
        {
            break;
        }
        retryCounter++;
    }

    // If the readback is invalid, remove the label recognition bits 
    if (false == isReadBackValid)
    {
        currentControlReg &= 0x0008; // Clear bit 3 (starting from 0). 
    }

    ARINC429_HI3584_txvrB_LoadCtrlReg( currentControlReg ); // Restore the original control register value 
    return isReadBackValid;
}
/* end ARINC-HI3584.c source file */
