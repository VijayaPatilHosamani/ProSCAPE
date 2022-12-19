/* Filename: AFC004MessageConfig.c
 *
 * Author: Henry Gilbert
 * 
 * Date: 1 August 2022
 * 
 * Description: Definitions of all AFC004 message (ARINC429 and RS422) to receive
 *      and transmit. Messages are defined in this file and used externally by 
 *      by extern. 
 *      
 * 
 * All Rights Reserved. Copyright Archangel Systems 2022
 */


/**************  Included File(s) **************************/
#include "ARINC.h"
#include "EclipseRS422messages.h"


/**********   ARINC429 (ARINC 706) Receive messages. Received via RS422 ADC **************/
ARINC429_RxMsg arincWordsRxFromRS422ADC[] = {
    {
        /* Label 200 - Airspeed Rate */
        .msgConfig.label = FormatLabelNumber( 200 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 14,
        .msgConfig.resolution = 0.00390625f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 203 - Pressure Altitude */
    {
        .msgConfig.label = FormatLabelNumber( 203 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 17,
        .msgConfig.resolution = 1.0f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 204 - Baro-Corrected Altitude */
    {
        .msgConfig.label = FormatLabelNumber( 204 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 17,
        .msgConfig.resolution = 1.0f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 205 - Mach Number  */
    {
        .msgConfig.label = FormatLabelNumber( 205 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 16,
        .msgConfig.resolution = 0.0000625f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 206 - Equivalent Airspeed */
    {
        .msgConfig.label = FormatLabelNumber( 206 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 14,
        .msgConfig.resolution = 0.0625f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 210 - True Airspeed */
    {
        .msgConfig.label = FormatLabelNumber( 210 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 15,
        .msgConfig.resolution = 0.0625f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 211 - Total Air Temperature */
    {
        .msgConfig.label = FormatLabelNumber( 211 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.125f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 212 - Altitude Rate */
    {
        .msgConfig.label = FormatLabelNumber( 212 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 11,
        .msgConfig.resolution = 16.0f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 213 - Static Air Temperature */
    {
        .msgConfig.label = FormatLabelNumber( 213 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 11,
        .msgConfig.resolution = 0.25f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 215 - Corrected Impact Pressure */
    {
        .msgConfig.label = FormatLabelNumber( 215 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 14,
        .msgConfig.resolution = 0.03125f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 221 - Angle of Attack */
    {
        .msgConfig.label = FormatLabelNumber( 221 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.043995f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 222 - Delta P Alpha */
    {
        .msgConfig.label = FormatLabelNumber( 222 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 18,
        .msgConfig.resolution = 0.000061035f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 223 - Uncorrected Impact Pressure */
    {
        .msgConfig.label = FormatLabelNumber( 223 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 14,
        .msgConfig.resolution = 0.03125f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 224 - AOA Rate */
    {
        .msgConfig.label = FormatLabelNumber( 224 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 13,
        .msgConfig.resolution = 0.015625f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 231 - Indicated OAT */
    {
        .msgConfig.label = FormatLabelNumber( 231 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.125f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 235 - Baro Correction */
    {
        .msgConfig.label = FormatLabelNumber( 235 ),
        .msgConfig.msgType = ARINC429_STD_BCD_MSG,
        .msgConfig.numSigBits = 19,
        .msgConfig.resolution = 0.001f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.numSigDigits = 5,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 242 - Total Pressure */
    {
        .msgConfig.label = FormatLabelNumber( 242 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 16,
        .msgConfig.resolution = 0.03125f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 246 - Static Pressure */
    {
        .msgConfig.label = FormatLabelNumber( 246 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 16,
        .msgConfig.resolution = 0.03125f,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65
    },

    /* Label 271 - STATUS. important label, looped back */
    {
        .msgConfig.label = FormatLabelNumber( 271 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65,
        .msgConfig.numDiscreteBits = 18

    },

    /* Label 377 - Equipment Identification */
    {
        .msgConfig.label = FormatLabelNumber( 377 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.minTransmitInterval_ms = 30,
        .msgConfig.maxTransmitInterval_ms = 65,
        .msgConfig.numDiscreteBits = 10
    }
};

/* Rx array for ADC words - populated via RS422 */
ARINC429_RxMsgArray arincADCarray = {
    .numMsgs = sizeof ( arincWordsRxFromRS422ADC) / sizeof ( ARINC429_RxMsg),
    .rxMsgs = arincWordsRxFromRS422ADC,
    .maxBusFailureCounts = 30u // 150 ms , 2.5 times the standard receive interval. 
};


/**************** ARINC429 (ARINC 705) received from AHR75 ******************/
ARINC429_RxMsg arincWordsRxFromAHR75[] = {
    {
        .msgConfig.label = FormatLabelNumber( 270 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.numSigBits = 19,
        .msgConfig.numDiscreteBits = 4,
        .msgConfig.minTransmitInterval_ms = 450,
        .msgConfig.maxTransmitInterval_ms = 550
    },
    {
        .msgConfig.label = FormatLabelNumber( 271 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.numSigBits = 19,
        .msgConfig.numDiscreteBits = 1,
        .msgConfig.minTransmitInterval_ms = 450,
        .msgConfig.maxTransmitInterval_ms = 550
    },
    {
        /* Magnetic Heading */
        .msgConfig.label = FormatLabelNumber( 320 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 15,
        .msgConfig.resolution = 0.0055f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Pitch Angle */
        .msgConfig.label = FormatLabelNumber( 324 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 14,
        .msgConfig.resolution = 0.010986f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Roll Angle */
        .msgConfig.label = FormatLabelNumber( 325 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 14,
        .msgConfig.resolution = 0.010986f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Body Pitch Rate */
        .msgConfig.label = FormatLabelNumber( 326 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 13,
        .msgConfig.resolution = 0.015625f,
        .msgConfig.maxTransmitInterval_ms = 25,
        .msgConfig.minTransmitInterval_ms = 15
    },
    {
        /* Body Roll Rate */
        .msgConfig.label = FormatLabelNumber( 327 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 13,
        .msgConfig.resolution = 0.015625f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Body Yaw Rate */
        .msgConfig.label = FormatLabelNumber( 330 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 13,
        .msgConfig.resolution = 0.015625f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Body Longitudinal Acceleration */
        .msgConfig.label = FormatLabelNumber( 331 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.000976563f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Body Lateral Acceleration */
        .msgConfig.label = FormatLabelNumber( 332 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.000976563f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Body Normal Acceleration */
        .msgConfig.label = FormatLabelNumber( 333 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.000976563f,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    },
    {
        /* Flight path Acceleration. Used for LS TX flag */
        .msgConfig.label = FormatLabelNumber( 323 ),
        .msgConfig.msgType = ARINC429_STD_BNR_MSG,
        .msgConfig.numSigBits = 12,
        .msgConfig.resolution = 0.001,
        .msgConfig.minTransmitInterval_ms = 15,
        .msgConfig.maxTransmitInterval_ms = 25
    }
};

/* Rx array for AHR75 words */
ARINC429_RxMsgArray arincAHR75array = {
    .numMsgs = sizeof ( arincWordsRxFromAHR75) / sizeof ( ARINC429_RxMsg),
    .rxMsgs = arincWordsRxFromAHR75,
    .maxBusFailureCounts = 10 // 50 ms, 2.5 times the standard receive interval. 
};


/**************** ARINC429 words received from PFD ************/
ARINC429_RxMsg arincWordsRxFromPFD[] = {
    {
        /* Baro Correction */
        .msgConfig.label = FormatLabelNumber( 235 ),
        .msgConfig.msgType = ARINC429_STD_BCD_MSG,
        .msgConfig.numSigBits = 19,
        .msgConfig.resolution = 0.001,
        .msgConfig.numDiscreteBits = 0,
        .msgConfig.numSigDigits = 5,
        .msgConfig.minTransmitInterval_ms = 40,
        .msgConfig.maxTransmitInterval_ms = 60
    },
    {
        /* Phase of Flight */
        .msgConfig.label = FormatLabelNumber( 124 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.numDiscreteBits = 3,
        .msgConfig.minTransmitInterval_ms = 180,
        .msgConfig.maxTransmitInterval_ms = 220
    },
    {
        /* ADC Status Word - loop around label, rs422 transmitted to ADC */
        .msgConfig.label = FormatLabelNumber( 270 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.minTransmitInterval_ms = 45,
        .msgConfig.maxTransmitInterval_ms = 55
    },
    {
        /* AHRS Status Word */
        .msgConfig.label = FormatLabelNumber( 271 ),
        .msgConfig.msgType = ARINC429_DISCRETE_MSG,
        .msgConfig.minTransmitInterval_ms = 45,
        .msgConfig.maxTransmitInterval_ms = 55
    }
};

/* Rx array for PFD Input words */
ARINC429_RxMsgArray arincPFDarray = {
    .numMsgs = sizeof ( arincWordsRxFromPFD) / sizeof ( ARINC429_RxMsg),
    .rxMsgs = arincWordsRxFromPFD,
    .maxBusFailureCounts = 25 // 125 ms, 2.5 times the standard receive interval. 
};



/********************************************** Eclipse RS422 Message Configurations *********************************************/
static const EclipseRS422msgConfig ADCComputedDataMsg_cfg = {
    .cmd = ADC_COMPUTED_DATA_CMD,
    .length = ECLIPSE_RS422_ADC_COMPUTED_DATA_MSG_LENGTH,
    .leftSource = LEFT_ADC,
    .rightSource = RIGHT_ADC,
    .leftDestination = LEFT_AHRS,
    .rightDestination = RIGHT_AHRS,
};

static const EclipseRS422msgConfig ADCStatusMsg_cfg = {
    .cmd = ADC_STATUS_CMD,
    .length = ECLIPSE_RS422_ADC_STATUS_MSG_LENGTH,
    .leftSource = LEFT_ADC,
    .rightSource = RIGHT_ADC,
    .leftDestination = LEFT_AHRS,
    .rightDestination = RIGHT_AHRS,
};

static const EclipseRS422msgConfig AHRSCurrentDataMsgTx_cfg = {
    .cmd = AHRS_CURRENT_DATA_CMD,
    .leftDestination = LEFT_ADC,
    .rightDestination = RIGHT_ADC,
    .leftSource = LEFT_AHRS,
    .rightSource = RIGHT_AHRS,
    .length = ECLIPSE_RS422_AHRS_CURRENT_DATA_MSG_LENGTH
};

/****************** RS422 messages received from ADC ****************/
EclipseRS422msg ADCRS422rxMsgs[] = {
    /* ADC Computed Data Message */
    {
        .msgConfig = &ADCComputedDataMsg_cfg,
        .data = NULL,
        .timeStamp_max_counts = 15u,
        .hasBusFailed = true
    },
    /* ADC Status Message */
    {
        .msgConfig = &ADCStatusMsg_cfg,
        .data = NULL,
        .hasBusFailed = true,
        .timeStamp_max_counts = 30u
    }
};

EclipseRS422msg ADCRS422txMsg = {

    .msgConfig = &AHRSCurrentDataMsgTx_cfg,
    .data = NULL

};

/* end of AFC004MessageConfig.c source file */