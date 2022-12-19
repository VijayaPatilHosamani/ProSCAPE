/* Filename: IOPMain.c
 *
 * Date: 1 August 2022
 * 
 * Author: Henry Gilbert
 * 
 * Description: Entry point for AFC004's IOP SCI. Performs startup built
 *      in tests, peripheral initialization, and provides the entry point
 *      into the main operating loop. 
 * 
 * All Rights Reserved. Copyright Archangel Systems 2022
 */

/**************  Include Files  ************************/
#include "COMCRCModule.h"
#include "COMHardwareResetConfiguration.h"
#include "COMSystemTimer.h"
#include "COMdsPICunusedISRs.h"
#include "COMRAMTEST.h"
#include "COMUart1.h"
#include "COMUart2.h"
#include "COMVerifyNonVolatileMemoryCRC.h"
#include "../COM/pic_h/p30F6014A.h"
#include "circularBuffer.h"
#include "EclipseRS422messages.h"
#include "ARINC.h"
#include "ArincDownload.h"
#include "calculateNewARINCLabels.h"
#include "ARINC_HI3584.h"
#include "SoftwareVersion.h"
#include "Timer23.h"
#include "maintenanceMode.h"
#include "IOPConfig.h"


/**************  Macro Definition(s) ***********************/
#define NUM_RS422_ADC_RXMSGS 2
#define NUM_RS422_ADC_TXMSGS 1

#define ECLIPSE_RS422_ADC_TX_MSG_LENGTH 27   
#define NUM_ARINC_WORDS_RS422TX_ADC 5

/* Used for AFC004 Tx Msg RS422 to ADC, indices for array address */
#define RS422_GNSS_ALT_IDX 0 
#define RS422_VDOP_IDX 1u
#define RS422_VFOM_IDX 2u
#define RS422_BARO_CORR_IDX 3u
#define RS422_STATUS_IDX 4u

#define RS422_ADC_MSG_INDEX 0

#define RS422_ADC_COMPUTED_DATA_IDX 0 
#define RS422_ADC_STATUS_IDX 1
#define RS422_ADC_TX_CURRENT_DATA_IDX 0

/* ARINC429 message defaults for RS422 transmit */
#define GNSS_ALT_NCD  0x2000007Cu
#define GNSS_ALT_FAIL 0x0000007Cu
#define VDOP_NCD 0x0000007Au 
#define VFOM_NCD 0x2000007Au   
#define VFOM_FAIL 0x0000007Au
#define STATUS_271_FAILURE 0x6000009Du

/************************* Pin Assignments *************************/
/* Fault pin for one shot circuit */
#define FAULT_PIN_LAT           LATGbits.LATG15  
#define FAULT_PIN_TRIS          TRISGbits.TRISG15

/* Strapping pin definitions */
#define STRAP1_Get PORTGbits.RG6
#define STRAP1_TRIS TRISGbits.TRISG6

#define STRAP2_Get PORTGbits.RG7
#define STRAP2_TRIS TRISGbits.TRISG7

#define STRAP3_Get PORTGbits.RG8
#define STRAP3_TRIS TRISGbits.TRISG8

#define STRAP_PARITY_Get PORTGbits.RG9
#define STRAPP_TRIS TRISGbits.TRISG9

/* ARINC429 Line drivers */
#define HI_8586_TXRA_TRIS TRISDbits.TRISD0
#define HI_8586_TXRA_LAT  LATDbits.LATD0

#define HI_8586_TXRB_TRIS TRISCbits.TRISC13
#define HI_8586_TXRB_LAT LATCbits.LATC13

/************************** Extern definitions ****************************/
extern ARINC429_RxMsgArray arincADCarray; /* Rx array for ADC words - populated via RS422 */
extern ARINC429_RxMsgArray arincAHR75array; /* Rx array for AHR75 words */
extern ARINC429_RxMsgArray arincPFDarray; /* Rx array for PFD Input words */
extern EclipseRS422msg ADCRS422rxMsgs[NUM_RS422_ADC_RXMSGS];
extern EclipseRS422msg ADCRS422txMsg [NUM_RS422_ADC_TXMSGS];

/* Structure for IOP system status flags */
static struct
{
    uint8_t RAMTest;
    uint8_t StoredCodeTest;
    uint8_t NoBootFault;
    uint8_t ARINCFault;
    uint8_t InternalFault;
} IOPStatus;

/* Bus status struct */
static struct
{
    bool hasRS422ADCRxBusFailed;
    bool hasAHR75RxBusFailed;
    bool hasPFDRxBusFailed;
} busStatus;


/************************************* Local function prototypes *******************************/
static bool ReadStrapping( uint8_t * const strapping ); /* Strapping result */
static void ConfigureUnusedPinsAsOutputs( void );
static void TransmitAHRSWords( );
static void TransmitADCRS422Words( const uint8_t magHeadingSDI );
static void TransmitA429ADCWords( );
static void CalculateAndTransmitAHRSStatusWords( );

/* Variable automatically located by linker at the very end of used main application program memory space. This is used to
 * determine the CRC calculation end address. */
__prog__ volatile u32 u32PM_CRC __attribute__( (section( ".PM_CRC" ), space( prog )) );

#define LAST_PM_ADDR_USED  ( ( u32 ) &u32PM_CRC - 2 )
#define PM_CRC_ADDR        ( ( u32 ) &u32PM_CRC )

/* Master UART1 and UART2 circular buffers */
#define UART1_RX_BUFF_SIZE 256
#define UART1_TX_BUFF_SIZE 100
#define UART2_RX_BUFF_SIZE 256
#define UART2_TX_BUFF_SIZE 100

/* ADC Receive Circular Buffer */
uint8_t uart1rxCirBuffData[UART1_RX_BUFF_SIZE];
circBuffer_t UART1rxCircBuff = {
    .data = uart1rxCirBuffData,
    .capacity = sizeof (uart1rxCirBuffData),
    .head = 0,
    .tail = 0
};

/* ADC Transmit Circular Buffer */
uint8_t uart1txCirBuffData[UART1_TX_BUFF_SIZE];
circBuffer_t UART1txCircBuff = {
    .data = uart1txCirBuffData,
    .capacity = sizeof (uart1txCirBuffData),
    .head = 0,
    .tail = 0
};

/* Maintenance UARTs */
uint8_t uart2rxCirBuffData[UART1_RX_BUFF_SIZE];
circBuffer_t UART2rxCircBuff = {
    .data = uart2rxCirBuffData,
    .capacity = sizeof (uart2rxCirBuffData),
    .head = 0,
    .tail = 0
};

uint8_t uart2txCirBuffData[UART1_TX_BUFF_SIZE];
circBuffer_t UART2txCircBuff = {
    .data = uart2txCirBuffData,
    .capacity = sizeof (uart2txCirBuffData),
    .head = 0,
    .tail = 0
};

int16_t main( void )
{
    int16_t returnVal = 1;

    /* RAM Test Function. */
    IOPStatus.RAMTest = u16_asmRAMTestResult( IOPConfig.hardwareSettings.RAMTestStartAddress, /* RAM Test Start Address. */
                                              IOPConfig.hardwareSettings.RAMTestEndAddress, /* Ram Test End memory address. */
                                              IOPConfig.hardwareSettings.RAMTestWriteWord1, /* Ram Test Memory Write Word 1. */
                                              IOPConfig.hardwareSettings.RAMTestReadWord1, /* Ram Test Memory Read Word 1. */
                                              IOPConfig.hardwareSettings.RAMTestWriteWord2, /* Ram Test Memory Write Word 2. */
                                              IOPConfig.hardwareSettings.RAMTestReadWord2 ); /* Ram Test Memory Read Word 2. */

    /* Generate the CRC table used for calculating the 32-bit CRC remainders */
    v_Generate32BitCRCTable( IOPConfig.hardwareSettings.CRCGenerationKey ); /* CRC Generation Key. */

#ifdef __DEBUG
    // Skip CRC checks if debugging
    IOPStatus.StoredCodeTest = 1;
#else
    /* Verify CRC of program code */
    IOPStatus.StoredCodeTest = u8_VerifyProgramMemoryCRC( ZERO, /* Program start address */
                                                          LAST_PM_ADDR_USED, /* Last program address used */
                                                          PM_CRC_ADDR ); /* Address of program memory CRC */
#endif

    v_HardwareResetConfiguartion( );
    ADPCFG = 0xFFFF; /* Configure all ANx pins as digital I/O */
    ConfigureUnusedPinsAsOutputs( );

    /* Digital Output Pin for the Fault Signal */
    FAULT_PIN_LAT = 0; // Set to known value
    FAULT_PIN_TRIS = 0; // Configure pin as output

    /* Strapping pin definitions as inputs  */
    STRAP1_TRIS = 1;
    STRAP2_TRIS = 1;
    STRAP3_TRIS = 1;
    STRAPP_TRIS = 1;

    /* ARINC Channel setup - same for both configurations */
    ARINC429_HI3584_txvrA_Initialize( ); // AHR75
    ARINC429_HI3584_txvrB_Initialize( ); // PFD

    /* Perform ARINC Loopback Tests  */
    IOPStatus.ARINCFault = ARINC429_HI3584_txvrA_LoopbackTest( ) ? 1 : 0;
    IOPStatus.ARINCFault &= ARINC429_HI3584_txvrB_LoopbackTest( ) ? 1 : 0;

    IOPStatus.ARINCFault &= ARINC429_HI3584_txvrA_LoadCtrlReg( IOPConfig.hardwareSettings.hi3584txvrAconfig ) ? 1 : 0;
    IOPStatus.ARINCFault &= ARINC429_HI3584_txvrB_LoadCtrlReg( IOPConfig.hardwareSettings.hi3584txvrBconfig ) ? 1 : 0;

    /* Output linedriver Txr A set to low speed transmit*/
    HI_8586_TXRA_TRIS = 0;
    HI_8586_TXRA_LAT = 0;

    /* Output linedriver Txr B set to high speed transmit */
    HI_8586_TXRB_TRIS = 0;
    HI_8586_TXRB_LAT = 1;

    /* Timer 2-3 - millisecond counter */
    Timer23_Initialize( IOPConfig.hardwareSettings.TMR23Config,
                        IOPConfig.hardwareSettings.TMR23Period,
                        IOPConfig.hardwareSettings.TMR23ScaleFactor );

    /* Timer 4: System Frequency Timer used in all modes */
    v_InitializeTMR4( IOPConfig.hardwareSettings.TMR4CounterConfig,
                      IOPConfig.hardwareSettings.TMR4CounterPeriod,
                      IOPConfig.hardwareSettings.TMR4InterruptConfig );

    /* Initialize UART1 for received ADC Msgs */
    UART1_Initialize( IOPConfig.hardwareSettings.UART1InterruptConfig,
                      IOPConfig.hardwareSettings.UART1BaudRate,
                      IOPConfig.hardwareSettings.UART1ModeConfig,
                      IOPConfig.hardwareSettings.UART1StatusConfig,
                      &UART1rxCircBuff,
                      &UART1txCircBuff );

    /* Initialize UART2 for maintenance mode */
    UART2_Initialize( IOPConfig.hardwareSettings.UART2InterruptConfig,
                      IOPConfig.hardwareSettings.UART2BaudRate,
                      IOPConfig.hardwareSettings.UART2ModeConfig,
                      IOPConfig.hardwareSettings.UART2StatusConfig,
                      &UART2rxCircBuff,
                      &UART2txCircBuff );

    /* Turn rate IIR Diff setup*/
    SetupTurnRateIIRDiff( IOPConfig.iirDiffSettings.K1,
                          IOPConfig.iirDiffSettings.IIRDiffSampleRate_Hz,
                          IOPConfig.iirDiffSettings.IIRDiffUpperLimit,
                          IOPConfig.iirDiffSettings.IIRDiffLowerLimit,
                          IOPConfig.iirDiffSettings.IIRDiffUpperDelta,
                          IOPConfig.iirDiffSettings.IIRDiffLowerDelta );

    /* IIR Filter setup */
    SetupNormAccelIIRFilter( IOPConfig.iirFilter.IIRFilterK1,
                             IOPConfig.iirFilter.IIRFilterK2 );

    /* Read Strapping */
    uint8_t strappingValue;

    const bool isStrappingValid = ReadStrapping( &strappingValue );
    if (isStrappingValid)
    {
        if (0x07 == strappingValue)
        {
            /* Enter Level D maintenance mode */
            maintenanceMode( &UART2txCircBuff, &UART2rxCircBuff ); // commented out for flight test 
        }
    }
    /* Deactivate UART 2 */
    U2MODEbits.UARTEN = 0;
    IEC1bits.U2RXIE = 0;
    IEC1bits.U2TXIE = 0;

    IOPStatus.NoBootFault = (IOPStatus.RAMTest & /* RAM Memory Test status bit. */
            IOPStatus.StoredCodeTest & /* Program Memory Test status bit. */
            IOPStatus.ARINCFault /* ARINC Fault Condition */
            );

    if (0 == IOPStatus.NoBootFault)
    {
        while (1);
    }

    IOPStatus.InternalFault = IOPStatus.NoBootFault;

    /*************************************** Main operating code init section ************************************/

    /* Verified ARINC429 messages received from the ADC via RS422. 
     * Does not include msg header, cmd, etc.  */
    uint8_t ADCComputedData_data[ECLIPSE_RS422_ADC_COMPUTED_DATA_MSG_LENGTH - 1];
    uint8_t ADCadcStatusMsg_data[ECLIPSE_RS422_ADC_STATUS_MSG_LENGTH - 1];

    /* Set the ADC RS422 message array to link to the declared array. */
    ADCRS422rxMsgs[RS422_ADC_COMPUTED_DATA_IDX].data = ADCComputedData_data;
    ADCRS422rxMsgs[RS422_ADC_STATUS_IDX].data = ADCadcStatusMsg_data;

    uint8_t AHRSCurrentDataMessage [ECLIPSE_RS422_ADC_TX_MSG_LENGTH];
    ADCRS422txMsg[RS422_ADC_TX_CURRENT_DATA_IDX].data = AHRSCurrentDataMessage;

    SWVer_GatherSWVersions( &UART1rxCircBuff,
                            &UART1txCircBuff );

    /* Setup label filters. Functions return true if label filter setup was successful. Negate this 
     * value to set the internal fault flag */
    IOPStatus.InternalFault &= (ARINC429_HI3584_SetupLabelFiltersTxvrA( &arincAHR75array ));
    IOPStatus.InternalFault &= (ARINC429_HI3584_SetupLabelFiltersTxvrB( &arincPFDarray ));

    uint32_t rateCounter = 0;
    size_t adcMsgIdx;

    /* Main operating loop */
    while (true)
    {
        /* ARINC: AHR75 is channel A, PFD is channel B */
        DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );

        /* Process RS422 ADC Data into ARINC words if a valid message was processed */
        UART1_ReadToRxCircBuff( );
        if (EclipseRS422_ProcessNewMessage( &UART1rxCircBuff,
                                            sizeof (ADCRS422rxMsgs) / sizeof (EclipseRS422msg),
                                            ADCRS422rxMsgs,
                                            &adcMsgIdx ))
        {
            EclipseRS422_CreateARINCWords( ADCRS422rxMsgs,
                                           &arincADCarray,
                                           adcMsgIdx,
                                           sizeof (ADCRS422rxMsgs) / sizeof (EclipseRS422msg) );
        }

        /* Download ARINC Words from PFD - no on event words are expected from PFD, so use NULL and 0 */
        DownloadMessagesFromARINCtxvrBrx2( &arincPFDarray );

        if (u16_ReadSystemFrequencyFlag( ))
        {
            /* 100 Hz Commands */
            FAULT_PIN_LAT = (true == IOPStatus.InternalFault) ? 1 : 0;
            v_ResetSystemFrequencyFlag( );
            rateCounter++;
            /* Process bus failure conditions */
            busStatus.hasRS422ADCRxBusFailed = EclipseRS422_processBusFailure( ADCRS422rxMsgs, sizeof (ADCRS422rxMsgs) / sizeof (EclipseRS422msg) );
            busStatus.hasAHR75RxBusFailed = ProcessARINCBusFailure( &arincAHR75array );
            busStatus.hasPFDRxBusFailed = ProcessARINCBusFailure( &arincPFDarray );

            if (0 == (rateCounter % 4))/* 50 Hz - 20 ms*/
            {
                DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );
                TransmitAHRSWords( );
            }

            if (7 == (rateCounter % 10)) /* 20 Hz - 50 ms */
            {
                DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );
                CalculateAndTransmitAHRSStatusWords( );
                TransmitADCRS422Words( arincAHR75array.rxMsgs[2].data.SDI ); //mag heading SDI 
            }


            if (2 == (rateCounter % 12)) /* 16.67 Hz - 60 ms */
            {
                DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );
                TransmitA429ADCWords( );
            }

            if (3 == (rateCounter % 20)) /* 10 Hz - 100 ms */
            {
                ARINC429_HI3584_txvrB_TransmitWord( SWVer_GetNextVersionARINCMsg( arincAHR75array.rxMsgs[2].data.SDI ) );
                DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );
            }

            DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );

            IOPStatus.InternalFault = IOPStatus.NoBootFault;
            // TODO add other internal fault checks here

            /* Drive the Digital fault line low, at the end of the code execution cycle. Provided there is no system fault. */
            FAULT_PIN_LAT = 0;
        }
    }
    return returnVal; // should never reach here 
}

/* Function: TransmitA429ADCWords
 *
 * Description: Transmits ADC words, if baro correction and PFD receive bus is valid. 
 * 
 * Return:None 
 * 
 */
static void TransmitA429ADCWords( )
{
    /* If baro correction is failed, or if baro correction times out, don't send air data */
    uint32_t baroWord;
    bool isBaroWordValid = ARINC429_GetLatestARINC429Word( &arincPFDarray, 235, &baroWord );
    uint8_t baroSSM = ARINC429_ExtractSSMbits( baroWord );
    bool isAirDataValid = (isBaroWordValid && (ARNIC429_SSM_BCD_PLUS == baroSSM));

    if (isAirDataValid)
    {
        TransmitLatestARINCMsgIfValid( &arincADCarray, 200, A429_CHANNEL_B ); /* Airspeed Rate */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 203, A429_CHANNEL_B ); /* Pressure Altitude */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 204, A429_CHANNEL_B ); /* Baro-Corrected Altitude */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 205, A429_CHANNEL_B ); /* Mach Number */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 206, A429_CHANNEL_B ); /* Equivalent Airspeed */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 210, A429_CHANNEL_B ); /* True Airspeed */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 211, A429_CHANNEL_B ); /* Total Air Temperature */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 212, A429_CHANNEL_B ); /* Altitude Rate */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 213, A429_CHANNEL_B ); /* Static Air Temperature */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 215, A429_CHANNEL_B ); /* Corrected Impact Pressure */
    }

    DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );

    if (isAirDataValid)
    {
        TransmitLatestARINCMsgIfValid( &arincADCarray, 221, A429_CHANNEL_B ); /* Angle of Attack */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 222, A429_CHANNEL_B ); /* Delta P Alpha */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 223, A429_CHANNEL_B ); /* Uncorrected Impact Pressure */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 224, A429_CHANNEL_B ); /* AOA Rate */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 231, A429_CHANNEL_B ); /* Indicated OAT */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 235, A429_CHANNEL_B ); /* Baro Correction */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 242, A429_CHANNEL_B ); /* Total Pressure */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 246, A429_CHANNEL_B ); /* Static Pressure */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 271, A429_CHANNEL_B ); /* STATUS */
        TransmitLatestARINCMsgIfValid( &arincADCarray, 377, A429_CHANNEL_B ); /* Equipment Identification */
    }
    return;
}

/* Function: TransmitAHRSWords
 *
 * Description: Calculates new AHRS words. Transmits air data back to AHR75
 * 
 * Return: None 
 */
static void TransmitAHRSWords( )
{
    /* Newly calculated words */
    ARINC429_HI3584_txvrB_TransmitWord( CalculateTurnRate( &arincAHR75array ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateSlipAngle( &arincAHR75array ) );

    /* Modified ARINC Words */
    ARINC429_HI3584_txvrB_TransmitWord( CalculateNewMagneticHeadingARINCWord( &arincAHR75array ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateNewPitchAngleARINCWord( &arincAHR75array ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateNewRollAngleARINCWord( &arincAHR75array ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateNewBodyLateralAccelARINCWord( &arincAHR75array ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateNewNormalAccelerationARINCWord( &arincAHR75array ) );

    /* Read AHRS FIFO */
    DownloadMessagesFromARINCtxvrArx2( &arincAHR75array );

    /* As-is ARINC words to transmit */
    TransmitLatestARINCMsgIfValid( &arincAHR75array, 331, A429_CHANNEL_B ); /* Body Longitudinal Acceleration */
    TransmitLatestARINCMsgIfValid( &arincAHR75array, 326, A429_CHANNEL_B ); /* Body Pitch Rate */
    TransmitLatestARINCMsgIfValid( &arincAHR75array, 327, A429_CHANNEL_B ); /* Body Roll Rate*/
    TransmitLatestARINCMsgIfValid( &arincAHR75array, 330, A429_CHANNEL_B ); /* Body Yaw Rate */

    /* Transmit air data to AHRS */
    TransmitLatestARINCMsgIfValid( &arincADCarray, 206, A429_CHANNEL_A ); // Calibrated Airspeed
    TransmitLatestARINCMsgIfValid( &arincADCarray, 210, A429_CHANNEL_A ); // True Airspeed
    TransmitLatestARINCMsgIfValid( &arincADCarray, 221, A429_CHANNEL_A ); // Angle of Attack
    return;
}

/* Function: TransmitADCRS422Words
 *
 * Description: Transmits message to ADC 
 *  * 
 *      RS422 Transmit:
 *          Set 76  GNSS Altitude to NCD (no GPS )
 *          Set 102 VDOP to NCD (not found anywhere)
 *          Set 136 VFOM to NCD (no GPS)
 *          Set 235 Baro correction to the received value 
 *          Set 271 Status to status word if valid, Failure if invalid 
 * 
 * Return: None 
 * 
 */
static void TransmitADCRS422Words( const uint8_t magHeadingSDI )
{
    /* Compose RS422 message to transmit to ADC */
    uint32_t arinc429TxWords[NUM_ARINC_WORDS_RS422TX_ADC];

    /* Transmit RS422 message to ADC */
    arinc429TxWords[RS422_GNSS_ALT_IDX] = GNSS_ALT_NCD;
    arinc429TxWords[RS422_VDOP_IDX] = VDOP_NCD;
    arinc429TxWords[RS422_VFOM_IDX] = VFOM_NCD;
    uint32_t arincStatusWord271;
    arinc429TxWords[RS422_BARO_CORR_IDX] = CalculateBaroCorrection( &arincPFDarray );
    arinc429TxWords[RS422_STATUS_IDX] = (true == ARINC429_GetLatestARINC429Word( &arincPFDarray,
                                                                                 271,
                                                                                 &arincStatusWord271 ))
            ? arincStatusWord271 : STATUS_271_FAILURE;

    EclipseRS422_ConstructTxMsg( ADCRS422txMsg,
                                 &UART1txCircBuff,
                                 arinc429TxWords,
                                 NUM_ARINC_WORDS_RS422TX_ADC,
                                 magHeadingSDI,
                                 ECLIPSE_RS422_ADC_TX_MSG_LENGTH );
    UART1_TxStart( );
    return;
}

static void CalculateAndTransmitAHRSStatusWords( )
{
    /* Transmit AHRS status words */
    ARINC429_HI3584_txvrB_TransmitWord( CalculateARINCLabel272( &arincAHR75array,
                                                                busStatus.hasRS422ADCRxBusFailed ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateARINCLabel274( &arincAHR75array,
                                                                busStatus.hasRS422ADCRxBusFailed ) );
    ARINC429_HI3584_txvrB_TransmitWord( CalculateARINCLabel275( &arincAHR75array ) );
}

/* Function: ReadStrapping
 *
 * Description: Strapping pins:
 *      S1: RG6
 *      S2: RG7
 *      S3: RG8
 *      SP: RG9
 * 
 * Return: True if strapping is valid, false if strapping is invalid 
 * 
 * Requirement Implemented: INT1.0103.S.IOP.1.001
 */

static bool ReadStrapping( uint8_t * const strapping ) /* Strapping result */
{
    /* Read strapping values multiple times in a row with a delay between reads */
    const size_t numReads = 10;

    *strapping = 0;

    /* Get first set of reads */
    uint8_t strap1 = STRAP1_Get;
    uint8_t strap2 = STRAP2_Get;
    uint8_t strap3 = STRAP3_Get;
    uint8_t strapParity = STRAP_PARITY_Get;

    size_t index;
    bool isStrappingOK = true;
    /* Now read multiple times and check consistency */
    for (index = 1; index < numReads; index++)
    {
        Timer23_Delay_ms( 10 );
        if ((STRAP1_Get != strap1) ||
                (STRAP2_Get != strap2) ||
                (STRAP3_Get != strap3) ||
                (STRAP_PARITY_Get != strapParity))
        {
            isStrappingOK = false;
            break;
        }
    }

    if (isStrappingOK)
    {
        /* Check for odd parity */
        if (0x1 == ((strap1 + strap2 + strap3 + strapParity) & 0x1))
        {
            *strapping = (strap1 << 2) + (strap2 << 1) + strap3; /* Parity OK */
        }
        else
        {
            isStrappingOK = false; /* Parity error */
        }
    }
    return isStrappingOK;
}

/* Function: ConfigureUnusedPinsAsOutputs
 * 
 * Description: Configures pins RB2, RB3, RB4, RB5, RB15, RD14, RG12, 
 *          RG13, RA6, RD5, RD7, RD1, RC14, RF6, RF7, RF8 as outputs 
 *          and sets the LATH value to zero. 
 * 
 * Return: None (void)
 * 
 * Requirement Implemented: PCS.0402.S.IOP.2.001
 */
static void ConfigureUnusedPinsAsOutputs( void )
{
    /* Pin B2 */
    TRISBbits.TRISB2 = 0;
    LATBbits.LATB2 = 0;

    /* Pin B3 */
    TRISBbits.TRISB3 = 0;
    LATBbits.LATB3 = 0;

    /* Pin B4 */
    TRISBbits.TRISB4 = 0;
    LATBbits.LATB4 = 0;

    /* Pin B5 */
    TRISBbits.TRISB5 = 0;
    LATBbits.LATB5 = 0;

    /* Pin B15 */
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;

    /* Pin D14*/
    TRISDbits.TRISD15 = 0;
    LATDbits.LATD15 = 0;

    /* Pin G12*/
    TRISGbits.TRISG12 = 0;
    LATGbits.LATG12 = 0;

    /* Pin G13*/
    TRISGbits.TRISG13 = 0;
    LATGbits.LATG13 = 0;

    /* Pin A6 */
    TRISAbits.TRISA6 = 0;
    LATAbits.LATA6 = 0;

    /* Pin D5 */
    TRISDbits.TRISD5 = 0;
    LATDbits.LATD5 = 0;

    /* Pin D7 */
    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;

    /* Pin D1 */
    TRISDbits.TRISD1 = 0;
    LATDbits.LATD1 = 0;

    /* Pin C14*/
    TRISCbits.TRISC14 = 0;
    LATCbits.LATC14 = 0;

    /* Pin F6 */
    TRISFbits.TRISF6 = 0;
    LATFbits.LATF6 = 0;

    /* Pin F7 */
    TRISFbits.TRISF7 = 0;
    LATFbits.LATF7 = 0;

    /* Pin F8 */
    TRISFbits.TRISF8 = 0;
    LATFbits.LATF8 = 0;
    return;
}