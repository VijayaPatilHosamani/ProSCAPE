/*
 * Filename: ArincDownload.h
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Module used for download and transmitting ARINC429 messages
 *          using the HI3854 chip. Extension to IOPArinc.h
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
*/

#ifndef IOPARINCNEW_H
#define IOPARINCNEW_H


/**************  Included File(s) **************************/
#include "ARINC_typedefs.h"
#include <stdbool.h>


/**************  Type Definition(s) ************************/
typedef enum {
    A429_CHANNEL_A,
    A429_CHANNEL_B
} ARINC429_TX_CHANNEL;


/**************  Function Prototype(s) *********************/
void DownloadMessagesFromARINCtxvrArx2(ARINC429_RxMsgArray * const ARINCMsgArray);

void DownloadMessagesFromARINCtxvrBrx2(ARINC429_RxMsgArray * const ARINCMsgArray);

void TransmitLatestARINCMsgIfValid(ARINC429_RxMsgArray * const rxMsgArray,
        uint16_t octalStdLabel,
        const ARINC429_TX_CHANNEL channel);

bool ProcessARINCBusFailure(ARINC429_RxMsgArray * ARINCMsgArray);

#endif
/* end ArincDownload.h header file */