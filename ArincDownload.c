/*
 * Filename: ArincDownload.c
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Module used for download and transmitting ARINC429 messages
 *          using the HI3854 chip. Extension to IOPArinc.c
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
 */


/**************  Included File(s) **************************/
#include "ARINC_HI3584.h"
#include "ARINC.h"
#include "ArincDownload.h"


/**************  Macro Definition(s) ***********************/
#define MAX_NUM_RX_MSGS 32u 
#define MAX_OCTAL_LABEL_VALUE 377

/**************  Function Definition(s) ********************/

/* Function: DownloadMessagesFromARINCtxvrArx2
 * 
 * Return: None 
 * 
 * Description: Retrieves all messages from transceiver A FIFO and processes
 *      every message into the input ARINC429_RxMsgArray. If a parity error
 *      is detected, discard the message. If a valid message is processed, 
 *      reset the arinc array's bus counts to zero.  
 * 
 * Requirement Implemented: INT1.0101.S.IOP.3.001
 */
void DownloadMessagesFromARINCtxvrArx2( ARINC429_RxMsgArray * const ARINCMsgArray )
{
    if (NULL == ARINCMsgArray)
    {
        return;
    }

    uint8_t numWordsProcessed = 0;
    uint32_t thisARINCRxMsg;

    while ((ARINC429_HI3584_TXVRA_DR2 == 0) && (numWordsProcessed < MAX_NUM_RX_MSGS))
    {
        thisARINCRxMsg = ARINC429_HI3584_txvrA_rx2_ReadWord( );
      
        if (thisARINCRxMsg & 0x80000000u)
        {
            ; // Parity error check 
        }
        
        else if (ARINC429_READ_MSG_SUCCESS == ARINC429_ProcessReceivedMessage( ARINCMsgArray,
                                                                          thisARINCRxMsg ))
        {
            ARINCMsgArray->currentCounts = 0;
        }
        else
        {
            ; // Nothing 
        }

        numWordsProcessed++;
    }
    return;
}

/*
 * Function: ProcessAHRSTimeout
 * 
 * Description: Increments an ARINC_RxMsgArray's message counter. This 
 *      message counter (currentCounts) is incremented every 10 ms. If
 *      the currentCounts is larger than the configured maxBusFailureCounts,
 *      return true (bus failure).  
 * 
 * Return: true if bus failed, false if bus is valid
 *
 * Requirement Implemented: INT1.0101.S.IOP.3.003
 */
bool ProcessARINCBusFailure( ARINC429_RxMsgArray * ARINCMsgArray )
{
    ARINCMsgArray->currentCounts++;
    return ((ARINCMsgArray->currentCounts >= ARINCMsgArray->maxBusFailureCounts))
            ? true : false;
}

/* Function: DownloadMessagesFromARINCtxvrBrx2
 *
 * Description: Retrieves all messages from transceiver B FIFO and processes
 *      every message into the input ARINC429_RxMsgArray. If a parity error
 *      is detected, discard the message. If a valid message is processed, 
 *      reset the arinc array's bus counts to zero.  
 * 
 * Return: None (void)
 * 
 * Requirement Implemented: INT1.0101.S.IOP.3.002
 */
void DownloadMessagesFromARINCtxvrBrx2( ARINC429_RxMsgArray * const ARINCMsgArray )
{
    if (NULL == ARINCMsgArray)
    {
        return;
    }

    uint8_t numWordsProcessed = 0;
    uint32_t thisARINCRxMsg;
    while ((ARINC429_HI3584_TXVRB_DR2 == 0) && (numWordsProcessed < MAX_NUM_RX_MSGS))
    {
        thisARINCRxMsg = ARINC429_HI3584_txvrB_rx2_ReadWord( );

        if (thisARINCRxMsg & 0x80000000u)
        {
            ; // parity error 
        }
        else if (ARINC429_READ_MSG_SUCCESS == ARINC429_ProcessReceivedMessage( ARINCMsgArray,
                                                                          thisARINCRxMsg ))
        {
            ARINCMsgArray->currentCounts = 0;
        }
        else
        {
            ;
        }
        numWordsProcessed++;
    }
    return;
}

/* Function: TransmitLatestARINCMsgIfValid
 * 
 * Description: Accepts as inputs a pointer to a rxMessage array and a label. Searches the rxArray for a 
 *      matching label and fresh data. If desired data is fresh, performs an ARIN429 Transmit operation
 *      of the raw ARINC429 word.  
 *          
 *      NOTE IMPORTANT: The label input will be assumbed to be in standard format. 
 *      Hex-flip format is performed inside this func. 
 * 
 * 
 * Return: None (void)
 * 
 * Requirement Implemented: INT1.0101.S.IOP.3.004
 */
void TransmitLatestARINCMsgIfValid( ARINC429_RxMsgArray * const rxMsgArray,
                                    uint16_t octalStdLabel,
                                    const ARINC429_TX_CHANNEL channel )
{
    if ((NULL == rxMsgArray) ||
            (octalStdLabel > MAX_OCTAL_LABEL_VALUE))
    {
        return;
    }
    ARINC429_RxMsgData data;
    const uint16_t hexFlippedLabel = FormatLabelNumber( octalStdLabel );
    ARINC429_GetLabelDataReturnStatus readStatus = ARINC429_GetLatestLabelData( rxMsgArray,
                                                                                hexFlippedLabel,
                                                                                &data );
    if ((true == data.isDataFresh) &&
            (true == data.isNotBabbling) &&
            ARINC429_GET_LABEL_DATA_MSG_SUCCESS == readStatus)
    {
        switch (channel)
        {
            case A429_CHANNEL_A:
                ARINC429_HI3584_txvrA_TransmitWord( data.rawARINCword );
                break;
            case A429_CHANNEL_B:
                ARINC429_HI3584_txvrB_TransmitWord( data.rawARINCword );
                break;
            default:
                break;
        }
    }
    return;
}

/* end ArincDownload.c source file*/