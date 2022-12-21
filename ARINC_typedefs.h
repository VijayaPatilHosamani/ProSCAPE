#ifndef ARINC_TYPEDEFS_H
#define ARINC_TYPEDEFS_H

/**************  Included Files  ************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef	__cplusplus
extern "C" {
#endif

    /**************  Type Definitions ************************/
    typedef uint16_t arincLabel; // Holds an ARINC 429 label ()

    /* ARINC 429 Sign/Status Matrix Values 
     * NOTE: When used with a message to be transmitted, these values will be written directly to the SSM bits (bits 31:30). */
    typedef enum ARNIC429_SM_t {
        /* ARINC 429 BCD Numeric Sign/Status Matrix Values */
        ARNIC429_SSM_BCD_PLUS = 0,
        ARNIC429_SSM_BCD_NORTH = 0,
        ARNIC429_SSM_BCD_EAST = 0,
        ARNIC429_SSM_BCD_RIGHT = 0,
        ARNIC429_SSM_BCD_TO = 0,
        ARNIC429_SSM_BCD_ABOVE = 0,
        ARNIC429_SSM_BCD_NO_COMPUTED_DATA = 1,
        ARNIC429_SSM_BCD_FUNCTIONAL_TEST = 2,
        ARNIC429_SSM_BCD_MINUS = 3,
        ARNIC429_SSM_BCD_SOUTH = 3,
        ARNIC429_SSM_BCD_WEST = 3,
        ARNIC429_SSM_BCD_LEFT = 3,
        ARNIC429_SSM_BCD_FROM = 3,
        ARNIC429_SSM_BCD_BELOW = 3,

        /* ARINC 429 BNR Status Matrix Values
         * NOTE: For BNR messages, sign is not indicated in the SSM (bits 31:30). */
        ARINC429_SSM_BNR_FAILURE_WARNING = 0,
        ARINC429_SSM_BNR_NO_COMPUTED_DATA = 1,
        ARINC429_SSM_BNR_FUNCTIONAL_TEST = 2,
        ARINC429_SSM_BNR_NORMAL_OPERATION = 3,

        /* ARINC 429 Discrete Status Matrix Values */
        ARINC429_SSM_DIS_VERIFIED_DATA = 0,
        ARINC429_SSM_DIS_NORMAL_OPERATION = 0,
        ARINC429_SSM_DIS_NO_COMPUTED_DATA = 1,
        ARINC429_SSM_DIS_FUNCTIONAL_TEST = 2,
        ARINC429_SSM_DIS_FAILURE_WARNING = 3,
    } ARINC429_SM;

    /* ARINC 429 received message data and statuses. */
    typedef struct ARINC429_RxMsgData_t {
        uint32_t rawARINCword;
        uint8_t SM : 2; // Status matrix. For BCD messages, the sign of the data may be indicated with this field and should be processed accordingly by the application code.
        uint8_t SDI : 2; // Source/destination identifier
        float engDataFloat; // BCD/BNR message data field converted to engineering units (float). For BCD messages, this will always be positive.
        int32_t engDataInt; // BCD/BNR message data field converted to engineering units (expressed as nearest integer)
        uint32_t discreteBits; // Discrete bits from the data field (starting from bit 11 for BCD/BNR, shifted fully left in Discrete message), if any are specified.
        uint32_t sysTimeLastGoodMsg_ms; // the system time (in ms) when the last valid message was received
        bool isEngDataInBounds; // Indicates whether the BCD/BNR data is within the specified maximum and minimum valid values.
        bool isNotBabbling; // Set to TRUE if the time between the two most recent data receive events is >= the minimum transmit interval. FALSE otherwise.
        bool isDataFresh; /* Indicates whether the time expired since the most recent data was received has exceeded the maximum
                          * transmit interval time. This property is determined when the data is read by the application code using
                          * the ARINC429_GetLatestLabelData() method */
    } ARINC429_RxMsgData;

    /* ARINC 429 Message Types */
    typedef enum ARINC429_MsgType_t {
        ARINC429_STD_BNR_MSG, // Denotes a standard ARINC 429 BNR (two's-complement binary) message
        ARINC429_STD_BCD_MSG, // Denotes a standard ARINC 429 BCD (binary coded decimal) message
        ARINC429_DISCRETE_MSG // Denotes an ARINC 429 discrete data message. Discrete bits are shifted fully left in data field (padding is on LSB side).
    } ARINC429_MsgType;

    typedef enum ARINC429_GetLabelDataReturnStatus_t {
        /* Function arguments were invalid */
        ARINC429_GET_LABEL_DATA_ERROR_INVALID_ARGUMENT = -5,

        /* No matching definition for the received label could be found */
        ARINC429_GET_LABEL_DATA_ERROR_NO_MATCHING_LABEL = -3,

        /* Conditions were not correct for retrieving label data (e.g. module not initialized or configured) */
        ARINC429_GET_LABEL_DATA_ERROR_CONDITIONS_NOT_CORRECT = -1,

        /* Label data was successfully retrieved */
        ARINC429_GET_LABEL_DATA_MSG_SUCCESS = 0,

    } ARINC429_GetLabelDataReturnStatus;

    /* Label Config */
    typedef struct ARINC429_LabelConfig_t {
        uint8_t label; // Changed to 8-bit since in the AFC004 we are pre-converting labels
        ARINC429_MsgType msgType;

        // Standard BNR messages
        uint8_t numSigBits; // Number of significant bits in the BNR data field (max: 18 normally or up to 20 if SDI bits are used as BNR data bits)

        // Standard BCD messages
        uint8_t numSigDigits; // Number of significant digits in a BCD

        // Common to standard BNR and BCD messages
        float resolution; // Data resolution
        float maxValidValue; // Maximum valid value (in engineering units) 
        float minValidValue; // Minimum valid value (in engineering units)

        // Common to standard BNR, BCD, and Discrete messages
        uint8_t numDiscreteBits; // Number of discrete bits in the data field (bits 11-29), if any. Always right aligned. Must set to 0 if not used.

        // Common to all message types
        uint16_t minTransmitInterval_ms; // Minimum transmit interval, in ms
        uint16_t maxTransmitInterval_ms; // Maximum transmit interval, in ms
    } ARINC429_LabelConfig;

    /* Top-level structure for ARINC 429 received messages. Includes configuration, statuses and message data. */
    typedef struct ARINC429_RxMsg_t {
        const ARINC429_LabelConfig msgConfig; /* configuration */
        ARINC429_RxMsgData data; /* received message data and statuses */
    } ARINC429_RxMsg;

    /* Holds an array of received messages and the length of the array */
    typedef struct ARINC429_RxMsgArray_t {
        const size_t numMsgs;
        ARINC429_RxMsg * const rxMsgs;

        /* Added these "bus failure" values back to update status msg. */
        const uint32_t maxBusFailureCounts;
        uint32_t currentCounts;
        bool hasBusFailed;
    } ARINC429_RxMsgArray;

    /* ARINC 429 transmitted message data and statuses. */
    typedef struct ARINC429_TxMsg_t {
        const ARINC429_LabelConfig* msgConfig;
        ARINC429_SM SM; // Status matrix. For BCD messages this should include the sign when appropriate.
        uint8_t SDI : 2; // SDI
        float engData; // Message data field converted to engineering units
        uint32_t discreteBits; // Discrete bits for the data field (starting from bit 11, if configured to use discrete bits)
    } ARINC429_TxMsg;

#ifdef	__cplusplus
}
#endif

#endif

/* End of ARINC_typedefs.h header file. */
