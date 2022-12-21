/*
 * Filename: calculateNewARINCLabels.h
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Header file used to declare the FormatLabelNumber macro,
 *          functions used for calculating new ARINC labels in eng data. 
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
 */

#ifndef CALCULATE_NEW_ARINC_LABELS_H
#define CALCULATE_NEW_ARINC_LABELS_H

/**************  Included File(s) **************************/
#include <stdint.h>
#include "ARINC_typedefs.h"
#include <stdbool.h>

/**************  Function Prototype(s) *********************/
void SetupTurnRateIIRDiff(const float k1,
        const float samplingRate,
        const float upperLimit,
        const float lowerLimit,
        const float upperDelta,
        const float lowerDelta);

void SetupNormAccelIIRFilter(const float k1,
        const float k2);

uint32_t CalculateTurnRate(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateSlipAngle(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateNewMagneticHeadingARINCWord(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateNewPitchAngleARINCWord(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateNewRollAngleARINCWord(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateNewBodyLateralAccelARINCWord(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateNewNormalAccelerationARINCWord(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateARINCLabel272(const ARINC429_RxMsgArray * const rxMsgArray,
        bool hasADCTimedOut);

uint32_t CalculateARINCLabel274(const ARINC429_RxMsgArray * const rxMsgArray,
        bool hasADCTimedOut);

uint32_t CalculateARINCLabel275(const ARINC429_RxMsgArray * const rxMsgArray);

uint32_t CalculateBaroCorrection(const ARINC429_RxMsgArray * const rxMsgArray);

#endif