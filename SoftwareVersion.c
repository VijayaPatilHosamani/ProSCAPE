/* 
 * Filename: SoftwareVersion.c
 * 
 * Author: Henry Gilbert
 * 
 * Date: 24 June 2022 
 * 
 * Description: Module for creating and gathering Eclipse's software version ARINC message
 *
 * All Rights Reserved. Copyright Archangel Systems 2022  
 */


/**************  Included File(s) **************************/
#include "SoftwareVersion.h"
#include "EclipseRS422messages.h"
#include "COMUART1.h"
#include "Timer23.h"
#include <string.h>
#include "IOPConfig.h"


/**************  Macro Definition(s) ***********************/
/* Local static array declarations */
#define NUM_BYTES_PER_SCI_VERSION 16
#define NUM_AFC004_SCI 3 //SCI of AFC004, ADC  in that order. 

#define MAX_MSG_IDX_VALUE 16
#define MAX_SYS_IDX_VALUE 3

/* Eclipse-specific macros */
#define ECLIPSE_RS422_VERSION_REQUEST_TXMSG_LENGTH 0x7 // length of entire tx msg. used for both hw and sw. 
#define ECLIPSE_RS422_VERSION_REQUEST_MSG_LENGTH 0x01  // length of data + cmd field
#define ECLIPSE_RS422_ADC_SWVERSION_REPLY_MSG_LENGTH 0x19 // bytes
#define ECLIPSE_RS422_ADC_HWVERSION_REPLY_MSG_LENGTH 0x09 // bytes

/* Message lengths specific to Eclipse's SCI version system */
#define ECLIPSE_RS422_SWVERSION_DATA_LENGTH 12
#define ECLIPSE_RS422_SWVERSION_CRC_POS_OFFSET 8
#define ECLIPSE_RS422_HWVERSION_DATA_LENGTH 4
#define ECLIPSE_RS422_HWVERSION_OFFSET 12

/* ARINC429 Definitions */
#define ARINC429_SDI_SHIFT_VAL 8
#define ARINC429_SUBSYS_IDX_SHIFT_VAL 23
#define ARINC429_MSGSUB_IDX_SHIFT_VAL 18
#define ARINC429_SWVER_DATA_SHIFT_VAL 10
#define ARINC429_SSM_SHIFT_VAL 29
#define ARINC429_SWVERSION_LABEL 0x0000007Fu

/* Miscellaneous */
#define NUM_CHARS_IN_32BIT_CRC 8
#define NUM_BYTES_IN_32BIT_CRC 4

/**************  Type Definition(s) ************************/
enum
{
    CC_AFC004 = 0x12,
    CC_ADC = 0x16,
    CC_PAOA = 0x17
} SW_VER_Subsystem_Index;



/**************  Local Variable(s) *************************/
static uint8_t swVersions[NUM_AFC004_SCI][NUM_BYTES_PER_SCI_VERSION];
static size_t msg_idx = 0; // Should be function-static, but is global for unit testing
static size_t sys_idx = 0; // Should be function-static, but is global for unit testing


/**************  Local Constant(s) *************************/
/* Indices used to index the 2d array*/
static const size_t afcSCIidx = 0;
static const size_t adcSCIidx = 1;
static const size_t paoaSCIidx = 2;
static const uint8_t subsystemVersionArray[NUM_AFC004_SCI] = { CC_AFC004, CC_ADC, CC_PAOA };


/**************  Function Prototype(s) *********************/
static bool SWVer_GatherRequest( EclipseRS422msg * const rs422txMsg, // REquest message
                                 EclipseRS422msg * const rs422rxMsg, // Received reply message
                                 circBuffer_t * const txBuff, // Transmit circular buffer
                                 circBuffer_t * const rxBuff, // Receive circular buffer  
                                 const uint8_t magHeadingSDI ); // SDI of magnetic heading 



/**************  Function Definition(s) *************/

/* Function: asciiConverter
 *
 * Description: Converts a nibble value into the equivalent ASCII value
 * 
 * Return: Formatted ASCII byte value 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.6.004
 */
uint8_t asciiConverter( const uint8_t val )
{
    uint8_t asciiValue = (val < 0xA) ? val + 0x30 : val + 0x37;
    return asciiValue;
}

/*
 * Function: SWVer_GatherRequest
 * 
 * Description: Composes an RS422 transmit message to request a subsystem's 
 *      software and hardware version. The input EclipseRS422 tx message 
 *      is the request message. This txMsg pointer is input to the EclipseRS422_ConstructTxMsg
 *      function, where it composes a valid EclipseRS422 message with the appended 
 *      16 bit CRC. This composed message is flushed into the transmit circular 
 *      buffer and transmission is started. A 5ms wait period is entered, then 
 *      the reply is searched for in the receive circular buffer. Repeat, for a 
 *      maximum of ten times, to search for a valid reply. 
 * 
 *  
 * Return: True if a valid version was received. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.6.001
 * 
 */
static bool SWVer_GatherRequest( EclipseRS422msg * const rs422txMsg, // Request message
                                 EclipseRS422msg * const rs422rxMsg, // Received reply message
                                 circBuffer_t * const txBuff, // Transmit circular buffer
                                 circBuffer_t * const rxBuff, // Receive circular buffer  
                                 const uint8_t magHeadingSDI ) // SDI of magnetic heading 
{
    const size_t maxNumRequestRetries = 10;
    size_t requestCount = 0;
    bool wasReplyFound;

    while (requestCount < maxNumRequestRetries)
    {
        size_t dummyData;
        wasReplyFound = false;

        /* Construct the transmit message: sw version request*/
        EclipseRS422_ConstructTxMsg( rs422txMsg,
                                     txBuff,
                                     NULL,
                                     0,
                                     magHeadingSDI,
                                     ECLIPSE_RS422_VERSION_REQUEST_TXMSG_LENGTH ); //transmit msg always 1
        UART1_TxStart( );
        Timer23_Delay_ms( 5 );
        UART1_ReadToRxCircBuff( );
        wasReplyFound = EclipseRS422_ProcessNewMessage( rxBuff, //receive circular buffer - rx uart
                                                        1, // rx message size. All are singular
                                                        rs422rxMsg, // rx message with data pointer to flush data out 
                                                        &dummyData ); //dummy data
        if (wasReplyFound)
        {
            break;
        }
        else
        {
            requestCount++;
        }
    }
    return wasReplyFound;
}


/**************  Function Definition(s) ********************/

/*
 * Function: SWVer_GatherSWVersions
 * 
 * Description: Function used to send and receive software version requests/replies
 *      from Eclipse's RS422 subsystems. Inputs the transmit and receive circular 
 *      buffers linked to the UART peripherals. 
 * 
 * Note: Here, UART1 is ADC subsystem, UART2 is GPS subsystem 
 * 
 * Return: None 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.6.002
 */
void SWVer_GatherSWVersions( circBuffer_t * const adcRxBuff,
                             circBuffer_t * const adcTxBuff )
{
    if ((NULL == adcTxBuff) ||
        (NULL == adcRxBuff))
    {
        return;
    }

    /* Zero the local array */
    size_t x;
    size_t y;
    for (x = 0; x < NUM_AFC004_SCI; x++)
    {
        for (y = 0; y < NUM_BYTES_PER_SCI_VERSION; y++)
        {
            swVersions[x][y] = 0;
        }
    }

    /*Software Version Request Configuration */
    const EclipseRS422msgConfig swVerADCRequestCfg = {
        .cmd = SOFTWARE_VERSION_CMD,
        .leftSource = LEFT_AHRS,
        .rightSource = RIGHT_AHRS,
        .leftDestination = LEFT_ADC,
        .rightDestination = RIGHT_ADC,
        .length = ECLIPSE_RS422_VERSION_REQUEST_MSG_LENGTH
    };

    const EclipseRS422msgConfig swVerADCReplyCfg = {
        .cmd = SOFTWARE_VERSION_CMD,
        .leftSource = LEFT_ADC,
        .rightSource = RIGHT_ADC,
        .leftDestination = LEFT_AHRS,
        .rightDestination = RIGHT_AHRS,
        .length = ECLIPSE_RS422_ADC_SWVERSION_REPLY_MSG_LENGTH
    };

    const EclipseRS422msgConfig hwVerADCRequestCfg = {
        .cmd = HARDWARE_SERIAL_NUMBER_CMD,
        .leftSource = LEFT_AHRS,
        .rightSource = RIGHT_AHRS,
        .leftDestination = LEFT_ADC,
        .rightDestination = RIGHT_ADC,
        .length = ECLIPSE_RS422_VERSION_REQUEST_MSG_LENGTH
    };

    const EclipseRS422msgConfig hwVerADCReplyCfg = {
        .cmd = HARDWARE_SERIAL_NUMBER_CMD,
        .leftSource = LEFT_ADC,
        .rightSource = RIGHT_ADC,
        .leftDestination = LEFT_AHRS,
        .rightDestination = RIGHT_AHRS,
        .length = ECLIPSE_RS422_ADC_HWVERSION_REPLY_MSG_LENGTH
    };

    /* Holds the actual data received. The ADC has twice the "normal" msg length */
    uint8_t adcSwVersionReplyData[ECLIPSE_RS422_ADC_SWVERSION_REPLY_MSG_LENGTH - 1];
    uint8_t adcHwVersionReplyData[ECLIPSE_RS422_ADC_HWVERSION_REPLY_MSG_LENGTH - 1];

    /* Tx message data. This will be flushed into the tx circular buffer through the construct msg function. */
    uint8_t adcSwVersionRequestData[ECLIPSE_RS422_VERSION_REQUEST_TXMSG_LENGTH];
    uint8_t adcHwVersionRequestData[ECLIPSE_RS422_VERSION_REQUEST_TXMSG_LENGTH];

    /* Set all declared local arrays to zero. */
    size_t i;
    for (i = 0; i < ECLIPSE_RS422_ADC_SWVERSION_REPLY_MSG_LENGTH - 1; i++)
    {
        adcSwVersionReplyData[i] = 0;
        adcHwVersionReplyData[i] = 0;
    }
    for (i = 0; i < ECLIPSE_RS422_VERSION_REQUEST_TXMSG_LENGTH - 1; i++)
    {
        adcSwVersionRequestData[i] = 0;
        adcHwVersionRequestData[i] = 0;
    }


    EclipseRS422msg swVersionRequestADCMsg = {
        .msgConfig = &swVerADCRequestCfg,
        .data = adcSwVersionRequestData
    };

    EclipseRS422msg swVersionReplyADCMsg = {
        .msgConfig = &swVerADCReplyCfg,
        .data = adcSwVersionReplyData
    };

    EclipseRS422msg hwVersionRequesADCtMsg = {
        .msgConfig = &hwVerADCRequestCfg,
        .data = adcHwVersionRequestData
    };
    EclipseRS422msg hwVersionReplyADCMsg = {
        .msgConfig = &hwVerADCReplyCfg,
        .data = adcHwVersionReplyData
    };

    if (true == SWVer_GatherRequest( &swVersionRequestADCMsg,
                                     &swVersionReplyADCMsg,
                                     adcTxBuff,
                                     adcRxBuff,
                                     0x01 ))
    {
        /* Copy the ADC software version to the local array */
        memcpy( &(swVersions[adcSCIidx][0]), &adcSwVersionReplyData, ECLIPSE_RS422_SWVERSION_DATA_LENGTH );

        /* Copy the Pitot/AOA hardware versionresults to the local array */
        uint8_t * pitotAOASwverion = adcSwVersionReplyData + ECLIPSE_RS422_SWVERSION_DATA_LENGTH; //advance 12 bytes
        memcpy( &(swVersions[paoaSCIidx][0]), pitotAOASwverion, ECLIPSE_RS422_SWVERSION_DATA_LENGTH );

    }
    if (true == SWVer_GatherRequest( &hwVersionRequesADCtMsg,
                                     &hwVersionReplyADCMsg,
                                     adcTxBuff,
                                     adcRxBuff,
                                     0x01 ))
    {
        /* Copy the ADC hardware version to the local array*/
        memcpy( &(swVersions[adcSCIidx][ECLIPSE_RS422_HWVERSION_OFFSET]), &adcHwVersionReplyData, ECLIPSE_RS422_HWVERSION_DATA_LENGTH );

        /* Copy the Pitot/AOA hardware version results to the local array */
        uint8_t * pitotAOAHWVerion = adcHwVersionReplyData + ECLIPSE_RS422_HWVERSION_DATA_LENGTH;
        memcpy( &(swVersions[paoaSCIidx][ECLIPSE_RS422_HWVERSION_OFFSET]), pitotAOAHWVerion, ECLIPSE_RS422_HWVERSION_DATA_LENGTH );
    }

    /* Create the AFC004's software version using the program's CRC */
    size_t crcCounter;
    uint8_t msgNibble;
    for (crcCounter = 0; crcCounter < NUM_CHARS_IN_32BIT_CRC; crcCounter++)
    {
        msgNibble = (uint8_t) (((u32PM_CRC << 4 * crcCounter)& (0xF0000000)) >> 28);
        swVersions[afcSCIidx][crcCounter] = asciiConverter( msgNibble );
    }

    /* Store the program memory's true CRC in the CRC slot.*/
    for (crcCounter = 0; crcCounter < NUM_BYTES_IN_32BIT_CRC; crcCounter++)
    {
        swVersions[afcSCIidx][crcCounter + ECLIPSE_RS422_SWVERSION_CRC_POS_OFFSET] = (uint8_t) ((u32PM_CRC >> (8 * crcCounter)) & 0xFF);
    }
}

/*
 * Function: SWVer_GetNextVersionARINCMsg
 * 
 * Description: Composes the software version and places into Eclipse's custom
 *      ARINC429 formatted word. Handles incrementing the message index (msg_idx)
 *      and the subsystem index (sys_idx). msg_idx ranges from 0-15. Upon reaching 
 *      a value of 16, the value is reset to zero. When msg_idx resets to zero, 
 *      sys_idx is incremented. If sys_idx reaches 4, it is reset to zero. 
 * 
 *      During startup, the swVersions array is initialized to zero, so if a 
 *      valid response was not received, only NULL characters (0) will be   
 *      transmitted. 
 * 
 * Return: Formatted 32bit ARINC429 word - Note: parity is calculated in hardware. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.6.003
 */
uint32_t SWVer_GetNextVersionARINCMsg( uint8_t sdi )
{
    uint32_t swVersionARINC429Word = ARINC429_SWVERSION_LABEL; //flipped label value, SSM of valid for DISC messages. 
    uint32_t subSys = (uint32_t) subsystemVersionArray[sys_idx];
    uint32_t msgSubIdx = (uint32_t) msg_idx;
    uint32_t data = (uint32_t) swVersions [sys_idx][msg_idx];

    swVersionARINC429Word |= ((uint32_t) (sdi) << ARINC429_SDI_SHIFT_VAL);
    swVersionARINC429Word |= (subSys << ARINC429_SUBSYS_IDX_SHIFT_VAL);
    swVersionARINC429Word |= (msgSubIdx << ARINC429_MSGSUB_IDX_SHIFT_VAL);
    swVersionARINC429Word |= (data << ARINC429_SWVER_DATA_SHIFT_VAL);

    /* Handle the module level index counter increments*/
    msg_idx++;
    if (MAX_MSG_IDX_VALUE == msg_idx)
    {
        msg_idx = 0;
        sys_idx++;
        if (sys_idx == MAX_SYS_IDX_VALUE)
        {
            sys_idx = 0;
        }
    }
    return swVersionARINC429Word;
}

/* end SoftwareVersion.c source file */