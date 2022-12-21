/* Filename: ARINC.h
 *
 * Date: 1 August 2022
 * 
 * Author: Henry Gilbert
 * 
 * Description: Public interface to ARINC message processing 
 * 
 * All Right Reserved. Copyright Archangel Systems 2022 
 * 
 */

#ifndef ARINC_H
#define ARINC_H

/**************  Included Files  ************************/
#include <stdint.h>
#include <stdbool.h>
#include "ARINC_typedefs.h"
#include "ARINC_common.h"

#ifdef	__cplusplus
extern "C" {
#endif

    /* Return status values for ARINC 429 message read functions */
    typedef enum ARINC429_ReadMsgReturnStatus_t {
        ARINC429_READ_MSG_ERROR_INVALID_ARGUMENT = -9, /* Function arguments were invalid */
        ARINC429_READ_MSG_ERROR_NO_MATCHING_LABEL = -6, /* No matching definition for the received label could be found */
        ARINC429_READ_MSG_ERROR_INVALID_MESSAGE = -5, /* The message being processed contained invalid data. */
        ARINC429_READ_MSG_ERROR = -4, /* An unspecified error was encountered */
        ARINC429_READ_MSG_SUCCESS = 0, /* A message was read successfully */
    } ARINC429_ReadMsgReturnStatus;

    /* Return status values for ARINC 420 message write functions */
    typedef enum ARINC429_WriteMsgReturnStatus_t {
        ARINC429_WRITE_MSG_ERROR_INVALID_ARGUMENT = -6, /* Function arguments were invalid */
        ARINC429_WRITE_MSG_ERROR_INVALID_MSG_DATA = -5, /* The ARINC message contains invalid field data */
        ARINC429_WRITE_MSG_ERROR_INVALID_MSG_CONFIG = -4, /* The ARINC message configuration is invalid */
        ARINC429_WRITE_MSG_ERROR = -3, /* An unspecified error was encountered */
        ARINC429_WRITE_MSG_SENT_DATA_CLIPPED = -1, /* A message was written successfully but provided data value exceeded data field limits 
                                                            * and was clipped to the max/min limit. */
        ARINC429_WRITE_MSG_SUCCESS = 0, /* A message was written successfully */
    } ARINC429_WriteMsgReturnStatus;


    /**************  Function Definitions ************************/

    /* Processes a received message. See ARINC429_ReadMsgReturnStatus for return types. */
    ARINC429_ReadMsgReturnStatus ARINC429_ProcessReceivedMessage(ARINC429_RxMsgArray * const rxMsgArray,
            const uint32_t ARINCMsg);

    ARINC429_WriteMsgReturnStatus ARINC429_AssembleStdBNRmessage(const ARINC429_TxMsg * const txMsg,
            uint32_t * const ARINCMsg);

    ARINC429_WriteMsgReturnStatus ARINC429_AssembleStdBCDmessage(const ARINC429_TxMsg * const txMsg, // Message to transmit, includes message configuration
            uint32_t * const arincMsg); // Assembled ARINC message

    ARINC429_WriteMsgReturnStatus ARINC429_AssembleDiscreteMessage(const ARINC429_TxMsg * const txMsg, // Message to transmit, includes message configuration
            uint32_t * const arincMsg); // Assembled ARINC message

    /* ARINC 429 Check data validity. Returns SSM */
    ARINC429_SM ARINC429_CheckValidityOfARINC_BNR_Data(const float engData,
            const ARINC429_LabelConfig * const lblCfg);

    /* Reports whether a received message is fresh (i.e. the maximum receive interval has not been exceeded). This function should be
     * called when the application code is accessing the ARINC data that has been transferred from the ARINC chip to the microcontroller.
     * */
    ARINC429_GetLabelDataReturnStatus ARINC429_GetLatestLabelData(const ARINC429_RxMsgArray * const rxMsgArray,
            const arincLabel label, // The label number of the ARINC data to be retrieved
            ARINC429_RxMsgData * const rxMsgData); // The latest received data corresponding to the given label and rx number

    bool ARINC429_GetLatestARINC429Word(const ARINC429_RxMsgArray * const rxMsgArray,
            const arincLabel octalStdLabel,
            uint32_t * const arincWord);

#ifdef	__cplusplus
}
#endif

#endif

/* End of ARINC.h header file. */
