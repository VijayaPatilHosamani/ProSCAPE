/* 
 * Filename: calculateNewARINCLabels.c
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Module used to calculate all new ARINC labels. 
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
 */


/**************  Included File(s) **************************/
#include "calculateNewARINCLabels.h"
#include "ARINC.h"
#include <math.h> 
#include "COMTrigModule.h"
#include "COMIIRDifferentiator.h"
#include "COMIIRFilter.h"
#include "IOPConfig.h"

/**************  Macro Definition(s) ***********************/
#define PI 3.14159265358979f
#define TWO_PI 6.28318530717959f
#define degToRad(angleInDegrees) ((angleInDegrees) * PI / 180.0f)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0f / PI)

/* Discrete message bit masks */
#define AHRS_STATUS_SDI_SSM_MASK 0x60000300u
#define AHRS_272_BIT_25_SET 0x2000000u
#define SET_BIT_15_AHRS_STATUS 0x4000u
#define AHRS_LABEL_271_MSU_FAIL_MASK 0x400u
#define AHRS_LABEL_270_CAL_MASK 0x400u
#define A429_DISC_SSM_FAIL_MASK 0x60000000



/**************  Local Variable(s) *************************/

/* Filter variables and constants.
 * Important note: These were changed from function-level static variables to global
 * variables for unit testing code coverage. These variables should ONLY be modified by 
 * the functions described in the comments. 
 */
static const size_t filterGoodThreshold = 10;

/* Slip angle filter variables. ONLY MODIFY THESE IN  CalculateSlipAngle()*/
static bool isIIRSlipFilterGood = false;
static size_t iirFilterGoodCount = 0;

/* IIR Differentiator variables. ONLY MODIFY THESE IN  CalculateTurnRate()*/
static bool isIIRDiffGood = false;
static size_t iirDiffGoodCount = 0;


/* Configuration data for transmitted ARINC Words. Transmitted ARINC Words may have different message configurations 
 * based on Eclipse's non-standard systems. These represent the swapped values. The rxMsg configs in the main files 
 * are based on the words we expect to receive. */

/* Slip/Skid Indicated Side Slip Angle */
static const ARINC429_LabelConfig arincLabel250Config = {
    .label = FormatLabelNumber( 250 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .resolution = 0.0439453f,
    .numSigBits = 12,
    .minValidValue = -180.0f,
    .maxValidValue = 180.0f
};

/* Turn Rate */
static const ARINC429_LabelConfig arincLabel340Config = {
    .label = FormatLabelNumber( 340 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 13,
    .resolution = 0.015625f,
    .minValidValue = -128.0f,
    .maxValidValue = 128.0f
};

/* Body Lateral Acceleration */
static const ARINC429_LabelConfig arincLabel332Config = {
    .label = FormatLabelNumber( 332 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 12,
    .resolution = 0.000976563f,
    .minTransmitInterval_ms = 15,
    .maxTransmitInterval_ms = 25
};

/* Body Normal Acceleration */
static const ARINC429_LabelConfig arincLabel333Config = {
    /* Body Normal Acceleration */
    .label = FormatLabelNumber( 333 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 12,
    .resolution = 0.000976563f,
    .minTransmitInterval_ms = 15,
    .maxTransmitInterval_ms = 25
};

/* Eclipse Magnetic Heading configuration. Changed from ASI's 15 sig bits */
static const ARINC429_LabelConfig Eclipse_ARINCLabel320Config = {
    .label = FormatLabelNumber( 320 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 12,
    .resolution = 0.043945f,
    .minValidValue = -180.0f,
    .maxValidValue = 180.0f
};

/* Eclipse Pitch Angle Configuration */
static const ARINC429_LabelConfig Eclipse_ARINCLabel324Config = {
    .label = FormatLabelNumber( 324 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 13,
    .resolution = 0.010986328f,
    .minValidValue = -90.0f,
    .maxValidValue = 90.0f
};

/* Eclipse Roll Angle Configuration */
static const ARINC429_LabelConfig Eclipse_ARINClabel325Config = {
    .label = FormatLabelNumber( 325 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 12,
    .resolution = 0.043945313f,
    .minValidValue = -180.0f,
    .maxValidValue = 180.0f
};

/* Body Normal Acceleration Configuration*/
static const ARINC429_LabelConfig Eclipse_ARINClabel333Config = {
    .label = FormatLabelNumber( 333 ),
    .msgType = ARINC429_STD_BNR_MSG,
    .numSigBits = 12,
    .resolution = 0.000976563f,
    .minValidValue = -3.0f, //+1.0f from offset
    .maxValidValue = 5.0f // +1.0f from offset 
};

/* Baro Correction */
static const ARINC429_LabelConfig arincLabel235Config = {
    .label = FormatLabelNumber( 235 ),
    .msgType = ARINC429_STD_BCD_MSG,
    .numSigBits = 19,
    .resolution = 0.001f,
    .numDiscreteBits = 0,
    .numSigDigits = 5,
};

/********************************** Filter setups **************************************/
static IIRDiff_Filter magHeadingIIRDiff;
static sIIR_struct accelerationZFilter;


/**************  Function Definition(s) ********************/

/* Function: SetupTurnRateIIRFilter
 *
 * Description: Performs an IIR Differentiator setup based on input configuration values. 
 * 
 * Return: None (void) 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.012
 */
void SetupTurnRateIIRDiff( const float k1,
                           const float samplingRate,
                           const float upperLimit,
                           const float lowerLimit,
                           const float upperDelta,
                           const float lowerDelta )
{
    IIRDifferentiatorSetup( &magHeadingIIRDiff,
                            k1,
                            samplingRate,
                            upperLimit,
                            lowerLimit,
                            upperDelta,
                            lowerDelta );
}

/* Function: SetupNormAccelIIRFilter
 *
 * Description: Calls IIRSetup based on input configurable k1 and k2 values 
 * 
 * Return: None (void)
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.013
 * 
 */
void SetupNormAccelIIRFilter( const float k1,
                              const float k2 )
{
    v_IIRSetup( &accelerationZFilter,
                k1,
                k2 );
}

/* Function: CalculateSlipAngle
 * 
 * Description: Slip Angle = arcTan (aY/aZ). aZ will be filtered through an IIR Filter. 
 * 
 * Return: 32bit ARINC429 word for slip angle based on input Tx message
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.002
 */
uint32_t CalculateSlipAngle( const ARINC429_RxMsgArray * const rxMsgArray )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData ayData;
    ARINC429_GetLabelDataReturnStatus readStatusAY = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 332 ), &ayData );
    ARINC429_RxMsgData azData;
    ARINC429_GetLabelDataReturnStatus readStatusAZ = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 333 ), &azData );
    uint32_t slipAngleWord;

    /* Compose ARINC429 Msg */
    ARINC429_TxMsg txMsgSlipAngle;
    txMsgSlipAngle.msgConfig = &arincLabel250Config;
    txMsgSlipAngle.SDI = azData.SDI; // which one should set? Should they be checked to be equal ?
    float slipAngleInDegrees;
    float filteredAZ;

    /* Set validation of Ay. It doesn't need the filter's validation. */
    bool isAYdataValid = ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == readStatusAY) &&
            ayData.isDataFresh && ayData.isNotBabbling &&
            (ARINC429_SSM_BNR_NORMAL_OPERATION == ayData.SM));

    /* Validate AZ*/
    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == readStatusAZ) &&
        azData.isDataFresh &&
        azData.isNotBabbling &&
        (ARINC429_SSM_BNR_NORMAL_OPERATION == azData.SM))
    {
        if (isIIRSlipFilterGood)
        {
            filteredAZ = f32_IIRFilter( azData.engDataFloat, &accelerationZFilter );
            slipAngleInDegrees = radToDeg( f32_ArcTan2( -ayData.engDataFloat, (filteredAZ + 1.0f) ) );
            txMsgSlipAngle.SM = ARINC429_CheckValidityOfARINC_BNR_Data( slipAngleInDegrees, &arincLabel250Config );
        }

        else
        {
            /* First valid msg received */
            txMsgSlipAngle.SM = ARINC429_SSM_BNR_FAILURE_WARNING;

            if (0 == iirFilterGoodCount)
            {
                v_IIRReset( &accelerationZFilter );
                v_IIRPreload( azData.engDataFloat, &accelerationZFilter );
                slipAngleInDegrees = 0;
            }
            else
            {
                filteredAZ = f32_IIRFilter( azData.engDataFloat, &accelerationZFilter );
                slipAngleInDegrees = radToDeg( f32_ArcTan2( -ayData.engDataFloat, (filteredAZ + 1.0f) ) );
            }
            iirFilterGoodCount++;

            if (iirFilterGoodCount > filterGoodThreshold)
            {
                isIIRSlipFilterGood = true;
            }
        }
    }
    else
    {
        /* AZ isn't a valid message. Invalid the tx message's SSM */
        slipAngleInDegrees = 0.0f;
        txMsgSlipAngle.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
        isIIRSlipFilterGood = false;
        iirFilterGoodCount = 0;
    }

    /* Despite the status of AZ, if AY data is invalid, the tx msg is invalid. This has no effect on the spooling/filter setup */
    if (false == isAYdataValid)
    {
        txMsgSlipAngle.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }

    txMsgSlipAngle.engData = slipAngleInDegrees;
    ARINC429_AssembleStdBNRmessage( &txMsgSlipAngle,
                                    &slipAngleWord );
    return slipAngleWord;
}

/*
 * Function: CalculateTurnRate
 * 
 * Description: Takes the IIR filtered derivative of magnetic heading and calculates the turn rate in degrees per second. 
 *              Processes the ARINC429 message with the SDI from the receive AHR75 magnetic heading. 
 * 
 * Return: ARINC429 formatted turn rate 
 * 
 * Requirement: INT1.0101.S.IOP.5.001 
 */
uint32_t CalculateTurnRate( const ARINC429_RxMsgArray * const rxMsgArray )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData magHeadingData;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 320 ), &magHeadingData );
    uint32_t turnRateWord;

    /* Compose ARINC429 Msg */
    ARINC429_TxMsg txMsgTurnRate;
    float turnRate_dps;
    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        magHeadingData.isDataFresh &&
        magHeadingData.isNotBabbling &&
        (ARINC429_SSM_BNR_NORMAL_OPERATION == magHeadingData.SM))
    {
        if (isIIRDiffGood)
        {
            turnRate_dps = IIR_Differentiator_Limited( magHeadingData.engDataFloat, &magHeadingIIRDiff ); // degrees per second 
            txMsgTurnRate.SM = ARINC429_CheckValidityOfARINC_BNR_Data( turnRate_dps, &arincLabel340Config );
        }
        else
        {
            
            // Spooling
            if (0 == iirDiffGoodCount)
            {
                IIRDifferentiatorReset( &magHeadingIIRDiff );
                IIRDifferentiatorPreload( magHeadingData.engDataFloat, /* value to be loaded into past input */
                                          &magHeadingIIRDiff ); /* pointer to the IIR differentiator struct */
                turnRate_dps = 0.0f;
            }
            else
            {
                turnRate_dps = IIR_Differentiator_Limited( magHeadingData.engDataFloat, &magHeadingIIRDiff ); // degrees per second 
            }

            iirDiffGoodCount++;

            if (iirDiffGoodCount >= filterGoodThreshold)
            {
                isIIRDiffGood = true;
            }

            txMsgTurnRate.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
        }
    }
    else
    {
        isIIRDiffGood = false;
        iirDiffGoodCount = 0;
        turnRate_dps = magHeadingIIRDiff.pastOutputOfDiff;
        txMsgTurnRate.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }

    txMsgTurnRate.msgConfig = &arincLabel340Config;
    txMsgTurnRate.SDI = magHeadingData.SDI;
    txMsgTurnRate.engData = turnRate_dps;
    ARINC429_AssembleStdBNRmessage( &txMsgTurnRate,
                                    &turnRateWord );
    return turnRateWord;
}

/*
 * Function: CalculateNewMagneticHeadingARINCWord
 * 
 * Description: Converts Archangel's 15 sig bit magnetic heading into Eclipses 12 bit magnetic heading. 
 *              Accounts for adjusted label configurations between both versions.
 * 
 * Return: ARINC429 formatted magnetic heading word based on Eclipse's msg config
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.006
 */
uint32_t CalculateNewMagneticHeadingARINCWord( const ARINC429_RxMsgArray * const rxMsgArray )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData magHeadingData;
    ARINC429_GetLabelDataReturnStatus magHeadReadStatus = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 320 ), &magHeadingData );
    ARINC429_RxMsgData lbl271Data;
    ARINC429_GetLabelDataReturnStatus lbl271ReadStatus = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 271 ), &lbl271Data );
    uint32_t magHeadingWord;

    ARINC429_TxMsg txMsgMagHeading;
    txMsgMagHeading.msgConfig = &Eclipse_ARINCLabel320Config;
    txMsgMagHeading.SDI = magHeadingData.SDI;
    txMsgMagHeading.engData = magHeadingData.engDataFloat;

    /* If both the magnetic heading data and 271 data are valid, set the SM based on received magnetic heading */
    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == magHeadReadStatus) &&
        magHeadingData.isDataFresh &&
        magHeadingData.isNotBabbling &&
        //       (ARINC429_SSM_BNR_NORMAL_OPERATION == magHeadingData.SM) &&
        (ARINC429_GET_LABEL_DATA_MSG_SUCCESS == lbl271ReadStatus) &&
        lbl271Data.isDataFresh &&
        lbl271Data.isNotBabbling &&
        (ARINC429_SSM_DIS_NORMAL_OPERATION == lbl271Data.SM))
    {
        /* TEST THIS: Set the mag heading message to fail if the MSU has failed, determined from label 271. */
        txMsgMagHeading.SM = (lbl271Data.rawARINCword & AHRS_LABEL_271_MSU_FAIL_MASK)
                ? ARINC429_SSM_BNR_FAILURE_WARNING : magHeadingData.SM;
    }
    else
    {
        txMsgMagHeading.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }

    ARINC429_AssembleStdBNRmessage( &txMsgMagHeading,
                                    &magHeadingWord );
    return magHeadingWord;
}

/*
 * Function: CalculateNewPitchAngleARINCWord
 * 
 * Description: Converts Archangel's pitch angle ARINC configuration (14 sig bits, res = 0.011) and assembles 
 *              an ARINC429 word based on Eclipse's configuration (13 sig bits, res = 0.011). 
 * 
 * Return: ARINC429 formatted pitch angle word. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.004
 */
uint32_t CalculateNewPitchAngleARINCWord( const ARINC429_RxMsgArray * const rxMsgArray ) /* Pitch angle rx msg processed from AHR75 */
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData pitchData;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 324 ), &pitchData );

    ARINC429_TxMsg txMsgPitchAngle;
    txMsgPitchAngle.msgConfig = &Eclipse_ARINCLabel324Config;
    txMsgPitchAngle.SDI = pitchData.SDI;
    txMsgPitchAngle.engData = pitchData.engDataFloat;

    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        pitchData.isDataFresh &&
        pitchData.isNotBabbling)
    {
        /* Compose ARINC429 Msg. PITCH_ANGLE eng data is already calculated */

        txMsgPitchAngle.SM = pitchData.SM;
    }
    else
    {
        txMsgPitchAngle.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }

    uint32_t pitchAngleARINCWord;
    ARINC429_AssembleStdBNRmessage( &txMsgPitchAngle,
                                    &pitchAngleARINCWord );
    return pitchAngleARINCWord;
}

/*
 * Function: CalculateNewRollAngleARINCWord
 * 
 * Description: Converts Arhcangel's roll angle ARINC configuration (14 sig bits, res = 0.011) and assembles
 *              an ARINC429 word based on Eclipse's configuration (12 sig bits, res = 044). 
 * 
 * Return: ARINC429 formatted roll angle word.
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.003
 */
uint32_t CalculateNewRollAngleARINCWord( const ARINC429_RxMsgArray * const rxMsgArray ) /* Roll angle rx msg processed from AHR75 */
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData rollData;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 325 ), &rollData );
    uint32_t rollAngleARINCWord;

    ARINC429_TxMsg txMsgRollAngle;
    txMsgRollAngle.msgConfig = &Eclipse_ARINClabel325Config;
    txMsgRollAngle.SDI = rollData.SDI;
    txMsgRollAngle.engData = rollData.engDataFloat;

    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        rollData.isDataFresh &&
        rollData.isNotBabbling)
    {
        txMsgRollAngle.SM = rollData.SM;
    }

    else
    {
        txMsgRollAngle.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }
    ARINC429_AssembleStdBNRmessage( &txMsgRollAngle,
                                    &rollAngleARINCWord );
    return rollAngleARINCWord;
}

/*
 * Function: CalculateNewBodyLateralAccelARINCWord
 * 
 * Description: Inverts the polarity of the body later acceleration engineering data. Processes the message with the 
 *              same SDI, SSM, and label config. 
 * 
 * Return: ARINC429 formatted body lateral acceleration word. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.007
 */
uint32_t CalculateNewBodyLateralAccelARINCWord( const ARINC429_RxMsgArray * const rxMsgArray )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData bodyLatAccelData;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 332 ), &bodyLatAccelData );
    uint32_t bodyLatAccARINCWord;

    ARINC429_TxMsg txMsgbodyLatAcc;
    txMsgbodyLatAcc.msgConfig = &arincLabel332Config;
    txMsgbodyLatAcc.SDI = bodyLatAccelData.SDI;
    txMsgbodyLatAcc.engData = -(bodyLatAccelData.engDataFloat);

    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        bodyLatAccelData.isDataFresh &&
        bodyLatAccelData.isNotBabbling)
    {

        txMsgbodyLatAcc.SM = bodyLatAccelData.SM;
    }
    else
    {
        txMsgbodyLatAcc.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }

    ARINC429_AssembleStdBNRmessage( &txMsgbodyLatAcc,
                                    &bodyLatAccARINCWord );
    return bodyLatAccARINCWord;
}

/* Function: CalculateNewNormalAccelerationARINCWord
 * 
 * Description: Calculates the body lateral acceleration ARINC word. If the 
 *      az data is fresh, set the SM based on received az's SM. Otherwise, fail
 *      the message. Add 1.0f to the engineering data. 
 * 
 * Return: 32bit ARINC429 word for normal acceleration. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.005
 */
uint32_t CalculateNewNormalAccelerationARINCWord( const ARINC429_RxMsgArray * const rxMsgArray )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData bodyNormAccelData;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 333 ), &bodyNormAccelData );
    uint32_t az;

    ARINC429_TxMsg txMsgNormAcc;
    txMsgNormAcc.msgConfig = &Eclipse_ARINClabel333Config;
    txMsgNormAcc.SDI = bodyNormAccelData.SDI;
    float azOffset = bodyNormAccelData.engDataFloat + 1.0f; // needed to add 1 g instead of minus. 
    txMsgNormAcc.engData = azOffset;

    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        bodyNormAccelData.isDataFresh &&
        bodyNormAccelData.isNotBabbling)
    {
        // If true, check if the message is valid in the first place.
        if (ARINC429_SSM_BNR_NORMAL_OPERATION == bodyNormAccelData.SM)
        {
            txMsgNormAcc.SM = ARINC429_CheckValidityOfARINC_BNR_Data( azOffset, &Eclipse_ARINClabel333Config );
        }
        else
        {
            txMsgNormAcc.SM = bodyNormAccelData.SM;
        }
    }
    else
    {
        txMsgNormAcc.SM = ARINC429_SSM_BNR_FAILURE_WARNING;
    }

    ARINC429_AssembleStdBNRmessage( &txMsgNormAcc,
                                    &az );
    return az;
}

/* Function: CalculateARINCLabel272
 * 
 * Description: Set 25 to zero, 26 to adcTimeout, 12&11 to MSU fail.
 *      Gets the latest label data for label 271 AHRS status. If label 271 
 *      is babbling or not fresh, set all aspects to failure.  
 * 
 * Return: 32bit ARINC429 word for Status 272 
 *
 * Requirement Implemented: INT1.0101.S.IOP.5.009
 */
uint32_t CalculateARINCLabel272( const ARINC429_RxMsgArray * const rxMsgArray,
                                 const bool hasADCTimedOut )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData lbl271Data;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 271 ), &lbl271Data );

    uint32_t label272ARINCWord = 0x0000005D;

    if (lbl271Data.isDataFresh &&
        lbl271Data.isNotBabbling &&
        (ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        (ARINC429_SSM_DIS_NORMAL_OPERATION == lbl271Data.SM))
    {
        label272ARINCWord |= (lbl271Data.rawARINCword & AHRS_STATUS_SDI_SSM_MASK); // Set 272 to the same SSM and SDI as 271
        /* If the ADC has timed out, set bit 25 (starting from 0) to 1. */
        if (hasADCTimedOut)
        {
            label272ARINCWord |= AHRS_272_BIT_25_SET;
        }
        /* If MSU fail (271-bit11), set 272's bits 10 and 11 */
        if (lbl271Data.rawARINCword & AHRS_LABEL_271_MSU_FAIL_MASK)
        {
            label272ARINCWord |= 0xC00u; // set bits 10 and 11 if MSU fail 
        }
    }
    else
    {
        label272ARINCWord |= A429_DISC_SSM_FAIL_MASK; // fail the resultant word. Ignores SDi and data and only fails lbl. 
    }
    return label272ARINCWord;
}

/* Function: CalculateARINCLabel274
 *
 * Description: Set bit 28 if MSU fail. Set bit 11 if MSU calibrating. Set bit 13 if ADC timeout
 * 
 * Return: 32bit ARINC429 word for Eclipse AHRS status 274
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.010
 */
uint32_t CalculateARINCLabel274( const ARINC429_RxMsgArray * const rxMsgArray,
                                 const bool hasADCTimedOut )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    uint32_t label274ARINCWord = 0x0000003Du; // Set the flipped label value initially. 

    ARINC429_RxMsgData lbl271Data;
    ARINC429_GetLabelDataReturnStatus status271 = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 271 ), &lbl271Data );

    ARINC429_RxMsgData lbl270Data;
    ARINC429_GetLabelDataReturnStatus status270 = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 270 ), &lbl270Data );

    if (lbl271Data.isDataFresh &&
        lbl271Data.isNotBabbling &&
        (ARINC429_SSM_DIS_NORMAL_OPERATION == lbl271Data.SM)&&
        (ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status271) &&
        lbl270Data.isDataFresh &&
        lbl270Data.isNotBabbling &&
        (ARINC429_SSM_DIS_NORMAL_OPERATION == lbl270Data.SM)&&
        (ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status270))
    {
        label274ARINCWord |= (lbl271Data.rawARINCword & AHRS_STATUS_SDI_SSM_MASK); // copy 271's SDI and SSM. 

        /* Set bit 28 if MSU fail*/
        if (lbl271Data.rawARINCword & AHRS_LABEL_271_MSU_FAIL_MASK)
        {
            label274ARINCWord |= 0x10000000u;
        }

        /* Set bit 11 if MSU is calibrating*/
        if (lbl270Data.rawARINCword & AHRS_LABEL_270_CAL_MASK)
        {
            label274ARINCWord |= 0x800u;
        }

        /* Set bit 13 if ADC timed out*/
        if (hasADCTimedOut)
        {
            label274ARINCWord |= 0x1000u;
        }
    }

    else
    {

        label274ARINCWord |= A429_DISC_SSM_FAIL_MASK;
    }
    return label274ARINCWord;
}

/* Set 23 to 271 bit 11, 15 to 1 */

/* Function: CalculateARINCLabel275
 *
 * Description: Set bit 23 if MSU fail. Set bit 25 to zero if FPA SSM is invalid. 
 * 
 * Return: 32 bit formatted ARINC429 word for Label 275 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.5.011
 */
uint32_t CalculateARINCLabel275( const ARINC429_RxMsgArray * const rxMsgArray )
{
    if (NULL == rxMsgArray)
    {
        return 0;
    }

    ARINC429_RxMsgData lbl271Data;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 271 ), &lbl271Data );

    // Get flight path acceleration from array. If the SM is failed, set bits 26-24. 
    ARINC429_RxMsgData flightPathAccelData;
    ARINC429_GetLabelDataReturnStatus fpaStatus = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 323 ), &flightPathAccelData );

    uint32_t label275ARINCWord = 0x000040BDu; // Default with label and bit 15 to 1. 

    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        lbl271Data.isNotBabbling &&
        lbl271Data.isDataFresh &&
        (ARINC429_SSM_DIS_NORMAL_OPERATION == lbl271Data.SM) &&
        (ARINC429_GET_LABEL_DATA_MSG_SUCCESS == fpaStatus) &&
        flightPathAccelData.isDataFresh &&
        flightPathAccelData.isNotBabbling)
    {
        label275ARINCWord |= ((lbl271Data.rawARINCword & AHRS_STATUS_SDI_SSM_MASK)); // Extract the SSM and SDI from label 271 

        /* If 271 msu fail, set bit 23*/
        if (lbl271Data.rawARINCword & AHRS_LABEL_271_MSU_FAIL_MASK)
        {
            label275ARINCWord |= 0x400000; //set bit 23
        }
        //indicates low speed tx bus to ahr75 has failed. If SM is not valid, set bit 25 t0 zero. 
        label275ARINCWord |= (ARINC429_SSM_BNR_NORMAL_OPERATION != flightPathAccelData.SM) ? 0x3000000u : 0x2000000u;

    }
    else
    {
        label275ARINCWord |= A429_DISC_SSM_FAIL_MASK; //send the label with failed data SSM
    }
    return label275ARINCWord;
}

/* Function: CalculateBaroCorrection
 * 
 * Description: Calculates baro correction. If the received baro correction
 *      is not valid, invalidate the SSM bits of baro correction.
 * 
 * Return: Formatted ARINC429 word for baro correction 
 *
 * Requirement Implemented: INT1.0101.S.IOP.5.008
 */
uint32_t CalculateBaroCorrection( const ARINC429_RxMsgArray * const rxMsgArray )
{
    ARINC429_RxMsgData baroData;
    uint32_t baroARINCWord;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray, FormatLabelNumber( 235 ), &baroData );

    ARINC429_TxMsg baroMsg;
    baroMsg.msgConfig = &arincLabel235Config;

    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
        baroData.isDataFresh &&
        baroData.isNotBabbling &&
        (ARNIC429_SSM_BCD_PLUS == baroData.SM))
    {
        baroMsg.engData = baroData.engDataFloat;
        baroMsg.SDI = baroData.SDI;
        baroMsg.SM = ARNIC429_SSM_BCD_PLUS;
    }
    else
    {
        baroMsg.engData = 0.0f;
        baroMsg.SDI = 0;
        baroMsg.SM = ARNIC429_SSM_BCD_NO_COMPUTED_DATA;
    }

    ARINC429_AssembleStdBCDmessage( &baroMsg, &baroARINCWord );
    return baroARINCWord;
}
