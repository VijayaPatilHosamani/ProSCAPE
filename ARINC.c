/* Filename: ARINC.c
 *
 * Author: Henry Gilbert
 * 
 * Date: 24 June 2022
 * 
 * Description: Module for processing ARINC429 messages. 
 *  
 * All Rights Reserved. Copyright Archangel Systems 2022 
 */

/**************  Included File(s) **************************/
#include "ARINC.h"
#include <math.h>
#include "Timer23.h"


/**************  Local Constant(s) *************************/
static const size_t maxNumRxMsgsInArray = 64; // Maximum number of rx messages that can be defined in a received message array


/**************  Macro Definition(s) ***********************/
#define MAX_OCTAL_LABEL_VALUE 377u


/**************  Static Function Prototypes (s) ************/
static ARINC429_ReadMsgReturnStatus ARINC429_ProcessStdBNRmessage( ARINC429_RxMsg * const thisRxMsg, // Received message, includes message configuration
                                                                   const uint32_t ARINCMsg ); // Received ARINC429 message

static ARINC429_ReadMsgReturnStatus ARINC429_ProcessStdBCDmessage( ARINC429_RxMsg * const thisRxMsg, // Received message, includes message configuration
                                                                   const uint32_t arincMsg ); // Received ARINC429 message

static ARINC429_ReadMsgReturnStatus ARINC429_ProcessDiscreteMessage( ARINC429_RxMsg * const thisRxMsg, // Received message, includes message configuration
                                                                     const uint32_t arincMsg ); // Received ARINC429 message

static bool ARINC429_IsLabelDataNotBabbling( const uint32_t clock_ms, // current clock count
                                             const ARINC429_RxMsg * const rxMsg ); // ARINC Rx message

static bool ARINC429_IsLabelDataFresh( const uint32_t clock_ms, // current clock count 
                                       const ARINC429_RxMsg * const rxMsg ); // ARINC Rx message


/**************  Static Function Definition(s) *************/

/* Function: ARINC429_ProcessStdBNRmessage
 *
 * Description: Parses the fields of a standard ARINC429 binary message. 
 *      Converts the engineering data to a float and integer. Extract the 
 *      SSM and SDI bits and stores in the rxMsg data field. 
 * 
 * Return: ARINC429_ReadMsgReturnStatus based on read status 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.003 
 */
static ARINC429_ReadMsgReturnStatus ARINC429_ProcessStdBNRmessage( ARINC429_RxMsg * const thisRxMsg,
                                                                   const uint32_t ARINCMsg )
{
    thisRxMsg->data.rawARINCword = ARINCMsg; // Store raw ARINC word

    uint32_t rawDataField = ARINCMsg >> (ARINC429_BNR_MAX_DATA_FIELD_SHIFT - thisRxMsg->msgConfig.numSigBits);
    rawDataField &= (UINT32_MAX >> (NUM_BITS_IN_UINT32 - thisRxMsg->msgConfig.numSigBits - 1)); // Mask includes sign bit

    ARINC429_ReadMsgReturnStatus readStatus;
    float dataEng;

    if (EXIT_FAILURE == ARINC429_BNR_ConvertRawMsgDataToEngUnits( thisRxMsg->msgConfig.numSigBits,
                                                                  thisRxMsg->msgConfig.resolution,
                                                                  &dataEng, // result in engineering units
                                                                  rawDataField )) // raw data field (right-aligned)
    {
        readStatus = ARINC429_READ_MSG_ERROR; // Error-- shouldn't happen
    }

    else
    {
        thisRxMsg->data.engDataFloat = dataEng;

        // Calculate the nearest int equivalent of the scaled data as some code needs integer values (e.g. TCAS intruder number)
        // Doing this here helps avoid issues with incorrect conversion of floats to int values in
        // downstream code (a common novice programmer mistake)

        double calcValue = (dataEng < 0.0) ? dataEng - 0.5f : dataEng + 0.5f;

        calcValue = clamp( calcValue, INT32_MIN, INT32_MAX ); // avoid issues with integer overflow during cast
        thisRxMsg->data.engDataInt = (int32_t) calcValue;

        // Extract the discrete bits (if used)
        if (thisRxMsg->msgConfig.numDiscreteBits > 0)
        {
            uint32_t discreteBits = (ARINCMsg >> ARINC429_BNR_BCD_MSG_DISCRETE_BITS_SHIFT_VAL);
            discreteBits &= (UINT32_MAX >> (NUM_BITS_IN_UINT32 - thisRxMsg->msgConfig.numDiscreteBits));
            thisRxMsg->data.discreteBits = discreteBits;
        }
        else
        {
            thisRxMsg->data.discreteBits = 0; // For good measure
        }

        thisRxMsg->data.SM = ARINC429_ExtractSSMbits( ARINCMsg ); /* Get SSM bits */

        /* Get SDI bits. Ignore SDI bits if more than 18 sig bits. */
        thisRxMsg->data.SDI = (thisRxMsg->msgConfig.numSigBits <= ARINC429_BNR_STD_MSG_NUM_SIGBITS_18) ?
                ARINC429_ExtractSDIbits( ARINCMsg ) : 0;

        readStatus = ARINC429_READ_MSG_SUCCESS;
    }

    return readStatus;
}

/* Function: ARINC429_ProcessStdBCDmessage
 *
 * Description: Processes the fields of an ARINC429 BCD message. 
 * 
 * Return: ARINC429_ReadMsgReturnStatus based on process result 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.004 
 */
static ARINC429_ReadMsgReturnStatus ARINC429_ProcessStdBCDmessage( ARINC429_RxMsg * const thisRxMsg, // Received message, includes message configuration
                                                                   const uint32_t arincMsg ) // Received ARINC message
{
    // Check number of significant digits and verify that discrete bit field does not overlap digit data
    if ((thisRxMsg->msgConfig.numSigDigits < 1) ||
            (thisRxMsg->msgConfig.numSigDigits > ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS) ||
            (((thisRxMsg->msgConfig.numSigDigits * 4 - 1) + thisRxMsg->msgConfig.numDiscreteBits) > ARINC429_BCD_STD_DATA_MAX_DATA_FIELD_SIZE))
    {
        return ARINC429_READ_MSG_ERROR_INVALID_MESSAGE; // Error-- Invalid ARINC message configuration
    }

    uint32_t bcdData = arincMsg & ARINC429_BCD_DATAFIELDMASK;
    bcdData >>= ARINC429_BCD_STD_MSG_DATA_FIELD_SHIFT +
            ARINC429_BCD_BITS_PER_DIGIT * (ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS - thisRxMsg->msgConfig.numSigDigits);

    float dataEng;
    if (EXIT_FAILURE == ARINC429_BCD_ConvertBCDvalToEngVal( thisRxMsg->msgConfig.numSigDigits,
                                                            thisRxMsg->msgConfig.resolution,
                                                            &dataEng, // result in engineering units
                                                            bcdData ))
    {
        return ARINC429_READ_MSG_ERROR_INVALID_MESSAGE; // Error-- invalid BCD digit in data field
    }

    thisRxMsg->data.engDataFloat = dataEng;

    // Calculate the nearest integer equivalent of the scaled data as some code may need integer values
    // Doing this here helps avoid issues with incorrect conversion of floats to int values in downstream code (a common novice programmer mistake)
    double calcValue = dataEng + ((dataEng < 0.0) ? -0.5f : 0.5f); // correct way to do rounding to avoid bias issues
    calcValue = clamp( calcValue, INT32_MIN, INT32_MAX ); // avoid issues with integer overflow during cast
    thisRxMsg->data.engDataInt = (int32_t) calcValue;

    // Extract the discrete bits (if used)
    if (thisRxMsg->msgConfig.numDiscreteBits > 0)
    {
        uint32_t discreteBits = arincMsg >> ARINC429_BNR_BCD_MSG_DISCRETE_BITS_SHIFT_VAL;
        discreteBits &= (UINT32_MAX >> (NUM_BITS_IN_UINT32 - thisRxMsg->msgConfig.numDiscreteBits));
        thisRxMsg->data.discreteBits = discreteBits;
    }
    else
    {
        thisRxMsg->data.discreteBits = 0; // For good measure
    }

    thisRxMsg->data.SM = ARINC429_ExtractSSMbits( arincMsg ); /* Get SSM bits */
    thisRxMsg->data.SDI = ARINC429_ExtractSDIbits( arincMsg ); /* Get SDI bits */

    return ARINC429_READ_MSG_SUCCESS;
}

/* Function: ARINC429_ProcessDiscreteMessage
 *
 * Description: Parses the message fields of a discrete ARINC429 message. 
 * 
 * Return: ARINC429_ReadMsgReturnStatus based on process message result. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.002 
 */
static ARINC429_ReadMsgReturnStatus ARINC429_ProcessDiscreteMessage( ARINC429_RxMsg * const thisRxMsg, // Received message, includes message configuration
                                                                     const uint32_t arincMsg ) // Received ARINC message
{
    if ((thisRxMsg->msgConfig.numDiscreteBits < 1) ||
            (thisRxMsg->msgConfig.numDiscreteBits > ARINC429_DISCRETE_MSG_MAX_NUM_BITS))
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_MSG_CONFIG; // Error-- Invalid ARINC message configuration
    }

    thisRxMsg->data.engDataFloat = 0.0f; // Not used with discrete messages
    thisRxMsg->data.engDataInt = 0; // Not used with discrete messages
    thisRxMsg->data.isEngDataInBounds = false; // Not used with discrete messages

    // Extract the discrete bits (if used)
    //    uint32_t discreteBits = arincMsg >> ( ARINC429_DISCRETE_MSG_MAX_DATA_FIELD_SHIFT - thisRxMsg->msgConfig.numDiscreteBits + 1 );
    uint32_t discreteBits = arincMsg >> (10); // temp implementation based on non-standard padding values. All values are padded msb
    discreteBits &= UINT32_MAX >> (NUM_BITS_IN_UINT32 - thisRxMsg->msgConfig.numDiscreteBits);
    thisRxMsg->data.discreteBits = discreteBits;

    thisRxMsg->data.SM = ARINC429_ExtractSSMbits( arincMsg ); /* Get SSM bits */
    thisRxMsg->data.SDI = ARINC429_ExtractSDIbits( arincMsg ); /* Get SDI bits */
    return ARINC429_READ_MSG_SUCCESS;
}


/* Function: ARINC429_IsLabelDataFresh
 *
 * Description: Reports whether a received message is fresh  (i.e. the maximum 
 *      receive interval has not been exceeded). This function should be
 *      called when the application code is accessing the ARINC data that has 
 *      been transferred from the ARINC chip to the microcontroller.
 * 
 * Return: true if message is fresh, false if not fresh 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.011
 */
static bool ARINC429_IsLabelDataFresh( const uint32_t clock_ms,
                                       const ARINC429_RxMsg * const rxMsg )
{
    if (NULL == rxMsg)
    {
        return false; // Error-- invalid function arguments. Assume stale.
    }

    uint32_t elapsedTime_ms = clock_ms - rxMsg->data.sysTimeLastGoodMsg_ms;
    bool returnVal = (elapsedTime_ms <= rxMsg->msgConfig.maxTransmitInterval_ms);

    return returnVal;
}

     
/* Function: ARINC429_IsLabelDataNotBabbling
 *
 * Description: Determines if an rxMsg is babbling (receive interval is faster
 *      than the minimum specified receive interval). If the current timestamp
 *      minus the time since a last good message is greater than the minimum
 *      transmit interval, return true. Function does not require a NULL 
 *      pointer check since it is accessed through ProcessReceivedMessage, and 
 *      can never call this function if a null pointer is detected. 
 * 
 * Return: True if not babbling, false if babbling. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.012 
 */

static bool ARINC429_IsLabelDataNotBabbling( const uint32_t clock_ms,
                                             const ARINC429_RxMsg * const rxMsg )
{
//    if (NULL == rxMsg)
//    {
//        return false; // Error-- invalid function arguments. Assume babbling.
//    }

    uint32_t elapsedTime = clock_ms - rxMsg->data.sysTimeLastGoodMsg_ms;
    bool returnVal = (elapsedTime >= rxMsg->msgConfig.minTransmitInterval_ms);
    return returnVal;
}

/**************  Function Definition(s) ********************/

/* Function: ARINC429_ProcessReceivedMessage
 *
 * Description: Takes a received ARINC429 message and checks each member of the
 *      rxMsgArray for a matching label. If a label match is found, process the 
 *      received message based on the label config type. If any process message
 *      routine fails, return the status through readMsgReturnStatus. If a message
 *      was successfully processed, timestamp the message and check babbling 
 *      status. Returns ARINC429_READ_MSG_SUCCESS upon success. 
 * 
 * Return: ARINC429_ReadMsgReturnStatus based on read message status 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.001
 */
ARINC429_ReadMsgReturnStatus ARINC429_ProcessReceivedMessage( ARINC429_RxMsgArray * const rxMsgArray, /* Pointer to receive message array */
                                                              const uint32_t ARINCMsg ) /* ARINC429 word read from hardware */
{
    if ((NULL == rxMsgArray) ||
            (NULL == rxMsgArray->rxMsgs))
    {
        return ARINC429_READ_MSG_ERROR; // Error-- invalid receive message array for specified receiver
    }

    uint8_t msgLabel = (uint8_t) (ARINCMsg & ARINC429_LBL_MASK);

    ARINC429_ReadMsgReturnStatus readMsgReturnStatus = ARINC429_READ_MSG_SUCCESS;

    /* Look through configured messages and try to match label */
    size_t count = 0;
    bool labelMatchFound = false;

    while ((count < rxMsgArray->numMsgs) &&
            (count < maxNumRxMsgsInArray) &&
            (false == labelMatchFound))
    {
        if (rxMsgArray->rxMsgs[count].msgConfig.label == msgLabel)
        {
            /* Process the message */
            ARINC429_RxMsg * const thisRxMsg = &(rxMsgArray->rxMsgs[count]);
            switch (thisRxMsg->msgConfig.msgType)
            {
                case ARINC429_STD_BNR_MSG:
                    readMsgReturnStatus = ARINC429_ProcessStdBNRmessage( thisRxMsg, // Received message, includes msg config
                                                                         ARINCMsg ); // Received ARINC message
                    break;

                case ARINC429_STD_BCD_MSG:
                    thisRxMsg->data.rawARINCword = ARINCMsg;
                    readMsgReturnStatus = ARINC429_ProcessStdBCDmessage( thisRxMsg, ARINCMsg );
                    break;

                case ARINC429_DISCRETE_MSG:
                    thisRxMsg->data.rawARINCword = ARINCMsg;
                    readMsgReturnStatus = ARINC429_ProcessDiscreteMessage( thisRxMsg, ARINCMsg );
                    break;
                default:
                    readMsgReturnStatus = ARINC429_READ_MSG_ERROR; // Error-- Un-handled message type. This should not happen.
                    break;
            }

            /* If message was successfully processed then update babbling status and record new message receipt time */
            if (ARINC429_READ_MSG_SUCCESS == readMsgReturnStatus)
            {
                uint32_t timestamp_now_ms = Timer23_GetTimestamp_ms( );

                thisRxMsg->data.isNotBabbling = ARINC429_IsLabelDataNotBabbling( timestamp_now_ms, // Check for babbling (do this before updating the last message receipt time)
                                                                                 thisRxMsg );
                thisRxMsg->data.sysTimeLastGoodMsg_ms = timestamp_now_ms;
            }

            labelMatchFound = true;
        }
        else
        {
            count++;
        }
    }

    if (false == labelMatchFound)
    {
        readMsgReturnStatus = ARINC429_READ_MSG_ERROR_NO_MATCHING_LABEL;
    }

    return readMsgReturnStatus;
}



/* Function: ARINC429_AssembleStdBNRmessage
 *
 * Description: Constructs a transmit 32bit ARINC429 word based on the input 
 *      txMsg. Writes the input parameter as the result of the assembled 
 *      ARINC429 word. 
 * 
 * Return: ARINC429_WriteMsgReturnStatus based on write status. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.007 
 */
ARINC429_WriteMsgReturnStatus ARINC429_AssembleStdBNRmessage( const ARINC429_TxMsg * const txMsg,
                                                              uint32_t * const ARINCMsg )
{
    if ((NULL == txMsg->msgConfig) ||
            (NULL == ARINCMsg))
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_ARGUMENT; // Error-- invalid function arguments
    }

    /* Calculate dataField */
    uint32_t dataField = 0;
    bool isDataClipped = false;

    ARINC429_WriteMsgReturnStatus writeMsgReturnStatus;
    if (EXIT_FAILURE == ARINC429_BNR_ConvertEngValToRawBNRmsgData( txMsg->msgConfig->numSigBits,
                                                                   txMsg->msgConfig->resolution,
                                                                   txMsg->engData,
                                                                   &dataField,
                                                                   &isDataClipped ))
    {
        writeMsgReturnStatus = ARINC429_WRITE_MSG_ERROR; // Error-- shouldn't happen
    }

    else
    {
        // If data clipping occurred, indicate it in the return status
        writeMsgReturnStatus = (true == isDataClipped) ? ARINC429_WRITE_MSG_SENT_DATA_CLIPPED : ARINC429_WRITE_MSG_SUCCESS;

        /* Shift data into message data field location, pad and then mask */
        uint32_t dataField_Shifted = dataField << (ARINC429_BNR_MAX_DATA_FIELD_SHIFT - txMsg->msgConfig->numSigBits);

        /* Mask based on whether the SDI field is being used as extra data field bits or not */
        if (ARINC429_BNR_STD_MSG_NUM_SIGBITS_20 == txMsg->msgConfig->numSigBits)
        {
            dataField_Shifted &= ARINC429_BNR_STD_MSG_DATAFIELDMASK_20SIGBITS;
        }
        else if (ARINC429_BNR_STD_MSG_NUM_SIGBITS_19 == txMsg->msgConfig->numSigBits)
        {
            dataField_Shifted &= ARINC429_BNR_STD_MSG_DATAFIELDMASK_19SIGBITS;
        }
        else
        {
            dataField_Shifted &= ARINC429_BNR_STD_MSG_DATAFIELDMASK_UPTO18SIGBITS;
        }

        /* Format discrete bits, if used */
        uint32_t discreteBits_shifted;
        if (txMsg->msgConfig->numDiscreteBits > 0)
        {
            uint32_t discreteBits = txMsg->discreteBits;
            discreteBits &= (UINT32_MAX >> (NUM_BITS_IN_UINT32 - txMsg->msgConfig->numDiscreteBits));
            discreteBits_shifted = (discreteBits << ARINC429_BNR_BCD_MSG_DISCRETE_BITS_SHIFT_VAL);
        }
        else
        {
            discreteBits_shifted = 0;
        }

        /* Assemble ARINC message */
        uint32_t arincMsgTemp = txMsg->msgConfig->label; // Label (already formatted)
        arincMsgTemp |= dataField_Shifted; // Data field
        arincMsgTemp |= discreteBits_shifted; // Discrete bits (if none are used then this will be a ZERO OR op)

        /* Ignore SDI bits if numSigBits is greater than 18, otherwise use. */
        if (txMsg->msgConfig->numSigBits <= ARINC429_BNR_STD_MSG_NUM_SIGBITS_18)
        {
            arincMsgTemp |= (txMsg->SDI & ARINC429_SDI_FIELD_LIMIT_MASK) << ARINC429_SDI_FIELD_SHIFT_VAL; // SDI
        }

        arincMsgTemp |= ((uint32_t) (txMsg->SM & ARINC429_SSM_FIELD_LIMIT_MASK) << ARINC429_SSM_FIELD_SHIFT_VAL); // SSM

        *ARINCMsg = arincMsgTemp; // Write result
    }

    return writeMsgReturnStatus;
}

/* Function: ARINC429_AssembleDiscreteMessage
 *
 * Description: Assembles a standard ARINC429 discrete message
 * 
 * Return: ARINC429_WriteMsgReturnStatus based on write parameters 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.005 
 */
ARINC429_WriteMsgReturnStatus ARINC429_AssembleDiscreteMessage( const ARINC429_TxMsg * const txMsg, // Message to transmit, includes message configuration
                                                                uint32_t * const arincMsg ) // Assembled ARINC message
{
    if ((NULL == txMsg) ||
            (NULL == txMsg->msgConfig) ||
            (NULL == arincMsg))
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_ARGUMENT; // Error-- invalid function arguments
    }

    if ((txMsg->msgConfig->numDiscreteBits < 1) ||
            (txMsg->msgConfig->numDiscreteBits > ARINC429_DISCRETE_MSG_MAX_NUM_BITS))
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_MSG_CONFIG; // Error-- Invalid ARINC message configuration
    }

    /* Mask and shift discrete bits. Discrete bits are always shifted fully left in the data field in discrete messages. */
    uint32_t discreteData_shifted;
    uint32_t discreteData = txMsg->discreteBits;
    discreteData &= (UINT32_MAX >> (NUM_BITS_IN_UINT32 - txMsg->msgConfig->numDiscreteBits));
    discreteData_shifted = discreteData << (ARINC429_DISCRETE_MSG_MAX_DATA_FIELD_SHIFT - txMsg->msgConfig->numDiscreteBits + 1);

    /* Assemble ARINC message */
    uint32_t arincMsgTemp = txMsg->msgConfig->label; // Label
    arincMsgTemp |= discreteData_shifted; // Discrete data
    arincMsgTemp |= (txMsg->SDI & ARINC429_SDI_FIELD_LIMIT_MASK) << ARINC429_SDI_FIELD_SHIFT_VAL; // SDI
    arincMsgTemp |= ((uint32_t) (txMsg->SM & ARINC429_SSM_FIELD_LIMIT_MASK) << ARINC429_SSM_FIELD_SHIFT_VAL); // SSM
    *arincMsg = arincMsgTemp; // Write result
    return ARINC429_WRITE_MSG_SUCCESS;
}

/* Function: ARINC429_AssembleStdBCDmessage
 *
 * Description: Assembles a standard BCD ARINC429 message. 
 * 
 * Return: See ARINC429_ReadMsgReturnStatus enum for return values
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.006 
 */
ARINC429_WriteMsgReturnStatus ARINC429_AssembleStdBCDmessage( const ARINC429_TxMsg * const txMsg, // Message to transmit, includes message configuration
                                                              uint32_t * const arincMsg ) // Assembled ARINC message
{
    if ((NULL == txMsg) ||
            (NULL == txMsg->msgConfig) ||
            (NULL == arincMsg))
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_ARGUMENT; // Error-- invalid function arguments
    }

    if ((txMsg->msgConfig->numSigDigits < 1) ||
            (txMsg->msgConfig->numSigDigits > ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS) ||
            (((txMsg->msgConfig->numSigDigits * 4 - 1) + txMsg->msgConfig->numDiscreteBits) > ARINC429_BCD_STD_DATA_MAX_DATA_FIELD_SIZE))
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_MSG_CONFIG; // Error-- Invalid ARINC message configuration
    }

    if (txMsg->engData < 0) // Engineering data to send must be positive (sign is indicated separately with the SM field)
    {
        return ARINC429_WRITE_MSG_ERROR_INVALID_MSG_DATA; // Error-- engineering data must be non-negative
    }

    /* Calculate dataField */
    uint32_t dataField = 0;
    bool isDataClipped = false;

    if (EXIT_FAILURE == ARINC429_BCD_ConvertEngValToBCD( txMsg->msgConfig->numSigDigits,
                                                         txMsg->msgConfig->resolution,
                                                         ARINC429_BCD_STD_MSG_MAX_NUM_BITS_MSC,
                                                         txMsg->engData,
                                                         &dataField,
                                                         &isDataClipped ))
    {
        return ARINC429_WRITE_MSG_ERROR; // Error-- shouldn't happen
    }

    // If data clipping occurred, indicate it in the return status
    ARINC429_WriteMsgReturnStatus writeMsgReturnStatus = (true == isDataClipped)
            ? ARINC429_WRITE_MSG_SENT_DATA_CLIPPED : ARINC429_WRITE_MSG_SUCCESS;

    /* Shift data into message data field location, accounting for padding for unused digits, and then mask */
    uint32_t dataField_shifted = dataField << (ARINC429_BCD_STD_MSG_DATA_FIELD_SHIFT +
            ARINC429_BCD_BITS_PER_DIGIT * (ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS - txMsg->msgConfig->numSigDigits));
    dataField_shifted &= ARINC429_BCD_DATAFIELDMASK;

    /* Format discrete bits, if used */
    uint32_t discreteBits_shifted;
    if (txMsg->msgConfig->numDiscreteBits > 0)
    {
        uint32_t discreteBits = txMsg->discreteBits;
        discreteBits &= (UINT32_MAX >> (NUM_BITS_IN_UINT32 - txMsg->msgConfig->numDiscreteBits));
        discreteBits_shifted = (discreteBits << ARINC429_BNR_BCD_MSG_DISCRETE_BITS_SHIFT_VAL);
    }
    else
    {
        discreteBits_shifted = 0;
    }

    /* Assemble ARINC message */
    uint32_t arincMsgTemp = txMsg->msgConfig->label; // Label
    arincMsgTemp |= dataField_shifted; // Data field
    arincMsgTemp |= discreteBits_shifted; // Discrete bits (if none are used then this will be a ZERO OR op)
    arincMsgTemp |= (txMsg->SDI & ARINC429_SDI_FIELD_LIMIT_MASK) << ARINC429_SDI_FIELD_SHIFT_VAL; // SDI
    arincMsgTemp |= ((uint32_t) (txMsg->SM & ARINC429_SSM_FIELD_LIMIT_MASK) << ARINC429_SSM_FIELD_SHIFT_VAL); // SSM
    *arincMsg = arincMsgTemp; // Write result
    return writeMsgReturnStatus;
}

/* Function: ARINC429_CheckValidityOfARINC_BNR_Message
 *
 * Description: Checks the upper and lower bounds of ARINC binary engineering 
 *      data. If the  engineering data is out of bounds, the function returns 
 *      an SSM of 00. If data is valid, the function will return a valid SSM (11).
 * 
 * Return: ARINC429_SM failure warning if out of bounds, normal operation if valid. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.008 
 */
ARINC429_SM ARINC429_CheckValidityOfARINC_BNR_Data( const float engData,
                                                    const ARINC429_LabelConfig * const lblCfg )
{
    return ((engData < lblCfg->minValidValue) || (engData > lblCfg->maxValidValue)) ?
            ARINC429_SSM_BNR_FAILURE_WARNING : ARINC429_SSM_BNR_NORMAL_OPERATION;
}

/* Function: ARINC429_GetLatestLabelData
 *
 * Description: Searches an rxMsg array for a matching label. If a label
 *      match is found, set the input return parameter to the data found
 *      in the label match. Timestamps the time and determines if the 
 *      message is fresh. Sets the rxMsgData's isDataFresh parameter 
 *      to the freshness status of the message. 
 * 
 * Return: ARINC429_GetLabelDataReturnStatus status of read. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.010 
 */
ARINC429_GetLabelDataReturnStatus ARINC429_GetLatestLabelData( const ARINC429_RxMsgArray * const rxMsgArray,
                                                               const arincLabel hexFlippedLabel, // The label number of the ARINC data to be retrieved
                                                               ARINC429_RxMsgData * const rxMsgData ) // The latest received data corresponding to the given label and rx number
{
    ARINC429_GetLabelDataReturnStatus getLabelDataReturnStatus;

    if ((NULL == rxMsgArray) ||
            (NULL == rxMsgData))
    {
        getLabelDataReturnStatus = ARINC429_GET_LABEL_DATA_ERROR_INVALID_ARGUMENT; // Error-- invalid function arguments
    }
    else
    {
        /* Lookup label */
        size_t count = 0;
        bool labelMatchFound = false;
        while ((count < rxMsgArray->numMsgs) &&
                (count < maxNumRxMsgsInArray))
        {
            if (rxMsgArray->rxMsgs[count].msgConfig.label == hexFlippedLabel)
            {
                labelMatchFound = true;
                *rxMsgData = rxMsgArray->rxMsgs[count].data;
                uint32_t current_time_ms = Timer23_GetTimestamp_ms( );
                rxMsgData->isDataFresh = ARINC429_IsLabelDataFresh( current_time_ms,
                                                                    &(rxMsgArray->rxMsgs[count]) );
                getLabelDataReturnStatus = ARINC429_GET_LABEL_DATA_MSG_SUCCESS; // Success!
                break;
            }

            count++;
        }

        if (false == labelMatchFound)
        {
            getLabelDataReturnStatus = ARINC429_GET_LABEL_DATA_ERROR_NO_MATCHING_LABEL; // Error-- no matching data could be found for the provided label
        }
    }

    return getLabelDataReturnStatus;
}

/* Function: ARINC429_GetLatestARINC429Word
 *
 * Description: Searches an rxMessageArray for a matching label. If 
 *      ARINC429_GetLatestLabelData finds a matching label, and the 
 *      data is fresh and is not babbling, return the ARINC word. 
 *      Otherwise, return false. This function should not directly
 *      set an ARINC429 word, and should use the return status as 
 *      guidance to what to do when an ARINC word isn't valid. 
 * 
 * Return: true if valid word. False if invalid. Writes the arincWord 
 *         input parameter the ARINC word if valid. 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.009 
 */
bool ARINC429_GetLatestARINC429Word( const ARINC429_RxMsgArray * const rxMsgArray,
                                     const arincLabel octalStdLabel,
                                     uint32_t * const arincWord )
{
    if ((NULL == rxMsgArray) ||
            (0 == octalStdLabel) ||
            (octalStdLabel > MAX_OCTAL_LABEL_VALUE) ||
            (NULL == arincWord))
    {
        return false; //error
    }

    ARINC429_RxMsgData data;
    ARINC429_GetLabelDataReturnStatus status = ARINC429_GetLatestLabelData( rxMsgArray,
                                                                            FormatLabelNumber( octalStdLabel ),
                                                                            &data );
    if ((ARINC429_GET_LABEL_DATA_MSG_SUCCESS == status) &&
            (true == data.isDataFresh) &&
            (true == data.isNotBabbling))
    {
        *arincWord = data.rawARINCword;
        return true;
    }
    else
    {
        return false;
    }
}

/* End of ARINC.c source file. */
