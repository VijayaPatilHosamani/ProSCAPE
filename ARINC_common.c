/*
 * Filename: ARINC_common.c
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Module used for processing common ARINC functions.
 * 
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
 */


/**************  Include Files  ************************/
#include "ARINC_common.h"
#include <stdlib.h>
#include <math.h>


/**************  Function Definition(s) ********************/

/* Function: ARINC429_BNR_ConvertEngValToRawBNRmsgData
 *
 * Description: Converts a value from engineering units to raw data field values. 
 *      If the data value exceeds the specified size limits of the BNR field then 
 *      the BNR data value will be set to the maximum value that fits within the BNR field.
 * 
 * Return: EXIT_SUCCESS if conversion is successful. Returns EXIT_FAILURE if 
 *          there is an invalid digit in the data or if the input arguments are invalid
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.015 
 */
int32_t ARINC429_BNR_ConvertEngValToRawBNRmsgData( const size_t numSigBits,
                                                   const float resolution,
                                                   const float dataEng,
                                                   uint32_t * const result,
                                                   bool * const isDataClipped )
{
    int32_t success = EXIT_SUCCESS;

    if ((NULL == result) ||
            (NULL == isDataClipped) ||
            (numSigBits < 1) ||
            (numSigBits > ARINC429_BNR_STD_MSG_MAX_NUM_SIGBITS))
    {
        success = EXIT_FAILURE;
    }
    else
    {
        double calcValue = (resolution != 0.0f) ? (dataEng / resolution) : 0.0f;
        calcValue += (calcValue < 0.0f) ? -0.5f : 0.5f; // correct way to do rounding to avoid bias issues
        calcValue = clamp( calcValue, INT32_MIN, INT32_MAX ); // avoid issues with integer overflow during cast
        int32_t calcValueAsInt = (int32_t) calcValue;
        uint32_t calcValueAsUint = (uint32_t) calcValueAsInt;

        // Check for dataField overflow and limit if needed.
        bool isClipped = false;
        const uint32_t datafieldOvfCheckMaskVal = (UINT32_MAX << numSigBits);
        if (calcValueAsUint & INT32_SIGN_BIT_MASK) // Check sign of data.  
        {
            /* Data is negative. The sign bit and all bits to the left of the sign bit should be 1. 
             * If not, then set the data field to the minimum value */
            if ((calcValueAsUint & datafieldOvfCheckMaskVal) != datafieldOvfCheckMaskVal)
            {
                /* Negative Overflow:  Sets sign bit to 1 and all other bits to 0 which equates to min data field value*/
                calcValueAsUint = 0x1 << numSigBits;
                isClipped = true;
            }
            // ELSE no negative overflow. No action required.
        }
        else
        {
            // Data is positive. There should be no bits set above left most data field bit. If so, set data field to max value.
            if (calcValueAsUint & datafieldOvfCheckMaskVal)
            {
                calcValueAsUint = UINT32_MAX >> (NUM_BITS_IN_UINT32 - numSigBits); // Max possible data field value
                isClipped = true;
            }
            // ELSE no positive overflow. No action required.

        }
        *isDataClipped = isClipped;
        *result = calcValueAsUint;
    }
    return success;
}

/* Function: ARINC429_BNR_ConvertRawMsgDataToEngUnits
 *
 * Description: Converts a raw message data to floating point engineering units. 
 * 
 * Return: EXIT_SUCCESS if conversion is successful. Returns EXIT_FAILURE if 
 *          there is an invalid digit in the BNR data or if the input arguments are invalid
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.016 
 */
int32_t ARINC429_BNR_ConvertRawMsgDataToEngUnits( const uint8_t numSigBits,
                                                  const float resolution,
                                                  float * const dataEng,
                                                  const uint32_t rawMsgData )
{
    int32_t success = EXIT_SUCCESS;

    if ((NULL == dataEng) ||
            (numSigBits < 1) ||
            (numSigBits > ARINC429_BNR_STD_MSG_MAX_NUM_SIGBITS))
    {
        success = EXIT_FAILURE;
    }
    else
    {
        uint32_t rawMsgData_SignExt = rawMsgData;

        // Check sign bit and set high bits to 1 if value is negative
        if (0 != (((uint32_t) 0x1 << numSigBits) & rawMsgData))
        {
            rawMsgData_SignExt |= (UINT32_MAX << numSigBits);
        }

        int32_t msgDataAsInt = (int32_t) rawMsgData_SignExt;
        *dataEng = (float) msgDataAsInt * resolution;
    }

    return success;
}

/* Function: ARINC429_ExtractSDIbits
 *
 * Description: Extracts SDI field from an ARINC429 message 
 * 
 * Return: SDI bit value of ARINC Message
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.017 
 */
uint8_t ARINC429_ExtractSDIbits( uint32_t ARINCMsg )
{
    return (ARINCMsg >> ARINC429_SDI_FIELD_SHIFT_VAL) & ARINC429_SDI_FIELD_LIMIT_MASK;
}

/* Function: ARINC429_ExtractSSMbits
 *
 * Description: Extracts SSM field from an ARINC429 message
 * 
 * Return: SSM bit value of ARINC Message
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.018
 */
uint8_t ARINC429_ExtractSSMbits( uint32_t ARINCMsg )
{
    return (ARINCMsg >> ARINC429_SSM_FIELD_SHIFT_VAL) & ARINC429_SSM_FIELD_LIMIT_MASK;
}

/* Function: ARINC429_BCD_ConvertBCDvalToEngVal
 *
 * Description: Converts a value from standard BCD to engineering units.
 *          Any padding should be handled before calling this function.
 *          Also note that sign must be handled by processing the SSM field.
 * 
 * Return: EXIT_SUCCESS if conversion is successful. Returns EXIT_FAILURE if 
 *          there is an invalid digit in the BCD data or if the input arguments are invalid
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.013 
 */
int32_t ARINC429_BCD_ConvertBCDvalToEngVal( const size_t numSigDigits,
                                            const float resolution,
                                            float * const dataEng, // Converted result in engineering units.
                                            const uint32_t rawBCDdata ) // BCD data
{
    int32_t success;

    if ((NULL == dataEng) ||
            (numSigDigits < 1) ||
            (numSigDigits > ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS)) // No BCD message can contain more than 5 digits
    {
        success = EXIT_FAILURE;
    }
    else
    {
        uint32_t calcValue = 0;
        uint32_t tempVal = rawBCDdata;

        size_t count = 0; // Number of digits processed
        uint32_t multVal = 1;
        uint32_t thisDigit;
        while ((tempVal > 0) &&
                (count < numSigDigits))
        {
            // Extract next digit
            thisDigit = tempVal & 0xF;
            if (thisDigit > ARINC429_BCD_MAX_DIGIT_VAL) // Verify that the BCD digit is valid
            {
                break; // Error-- unexpected digit in BCD data. This will result in EXIT_FAILURE return status.
            }

            // Add digit to result
            calcValue += multVal * thisDigit;
            tempVal >>= ARINC429_BCD_BITS_PER_DIGIT;
            multVal *= 10;
            count++;
        }

        success = (0 == tempVal) ? EXIT_SUCCESS : EXIT_FAILURE;
        *dataEng = (0 == tempVal) ? (float) calcValue * resolution : 0.0f;
    }
    return success;
}

/* Function: ARINC429_BCD_ConvertEngValToBCD
 *
 * Description: Converts a BCD engineering value into BCD data format 
 * 
 * Return: EXIT_SUCCESS for successful process, EXIT_FAULIRE for invalid parameters 
 * 
 * Requirement Implemented: INT1.0101.S.IOP.4.014 
 */
int32_t ARINC429_BCD_ConvertEngValToBCD( const size_t numSigDigits,
                                         const float resolution,
                                         const size_t numBitsMSC, // Number of bits in the most-significant BCD character
                                         const float dataEng, // Input value in engineering units
                                         uint32_t * const rawBCDdata, // result
                                         bool * const isDataClipped ) // Indicates whether data was clipped (i.e. its value exceeded the size of the specified BCD field)
{
    int32_t success = EXIT_SUCCESS;

    if ((NULL == rawBCDdata) ||
            (numSigDigits < 1) ||
            (numSigDigits > ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS) || // No BCD message can contain more than 5 digits
            (numBitsMSC < 1) ||
            (numBitsMSC > ARINC429_BCD_BITS_PER_DIGIT))
    {
        success = EXIT_FAILURE;
    }
    else
    {
        float calcValue = (resolution != 0.0f) ? (dataEng / resolution) : 0.0f;
        uint32_t tempValue = (uint32_t) min( calcValue + 0.5f, UINT32_MAX ); // changed from clamp to min 
        uint32_t asBCD = 0;
        size_t count = 0;
        uint32_t thisDigit;
        while ((tempValue > 0) &&
                (count < numSigDigits))
        {
            thisDigit = tempValue % 10;

            if ((numSigDigits == (count + 1)) &&
                    (thisDigit > (UINT32_MAX >> (NUM_BITS_IN_UINT32 - numBitsMSC))))
            {
                break; // Attempting to use out-of-bounds bits
            }

            asBCD += thisDigit << (ARINC429_BCD_BITS_PER_DIGIT * count);
            tempValue /= 10;
            count++;
        }

        // Check for data clipping
        if (0 == tempValue)
        {
            *isDataClipped = false;
        }
        else
        {
            *isDataClipped = true;
            // Data clipped so set to maximum value based on number of significant digits and number of bits in MSC
            asBCD = 0;
            count = 0;
            while (count < numSigDigits)
            {
                thisDigit = (count != (numSigDigits - 1)) ? ARINC429_BCD_MAX_DIGIT_VAL : (UINT32_MAX >> (NUM_BITS_IN_UINT32 - numBitsMSC));
                asBCD += thisDigit << (ARINC429_BCD_BITS_PER_DIGIT * count);
                count++;
            }
        }

        *rawBCDdata = asBCD;
    }

    return success;
}

/* End of ARINC_common.c source file. */
