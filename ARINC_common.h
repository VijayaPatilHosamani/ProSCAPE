/*
 * Filename: ARINC_common.h
 * 
 * Author: Henry Gilbert
 * 
 * Date: 8 April 2022 
 * 
 * Description: Header file used to declare ARINC_common functions, #defines 
 *          for ARINC processing, and the clamp function. 
 * 
 * All rights reserved. Copyright 2022. Archangel Systems Inc.
 */

#ifndef ARINC_COMMON_H
#define ARINC_COMMON_H

/**************  Included Files  ************************/

#include <stdint.h>
#include <stdbool.h>
#include "ARINC_typedefs.h"

#ifdef	__cplusplus
extern "C" {
#endif

    /**************  Type Definitions ************************/

    /* ARINC 429 Octal Label Definitions */
#define ARINC429_LBL_MASK                    0xFF
#define ARINC429_LBL_OCT_SHIFT_ONES_DIG      0
#define ARINC429_LBL_OCT_MASK_ONES_DIG       0x7
#define ARINC429_LBL_OCT_SHIFT_TENS_DIG      3
#define ARINC429_LBL_OCT_MASK_TENS_DIG       0x7
#define ARINC429_LBL_OCT_SHIFT_HUNDREDS_DIG  6
#define ARINC429_LBL_OCT_MASK_HUNDREDS_DIG   0x3

    /* ARINC 429 Word Format Parameters */
#define ARINC429_PARITY_BIT_SHIFT_VAL  31
#define ARINC429_PARITY_BIT_MASK       0x1
#define ARINC429_SSM_FIELD_SHIFT_VAL   29
#define ARINC429_SSM_FIELD_LIMIT_MASK  0x3
#define ARINC429_SDI_FIELD_SHIFT_VAL   8
#define ARINC429_SDI_FIELD_LIMIT_MASK  0x3

    /* ARINC 429 BNR Message Definitions */
#define ARINC429_BNR_MAX_DATA_FIELD_SHIFT          28    // Maximum shift count for BNR data field
#define ARINC429_BNR_STD_MSG_NUM_SIGBITS_18        18    // Normal BNR uses up to 18 significant bits
#define ARINC429_BNR_STD_MSG_NUM_SIGBITS_19        19    // Extended BNR data field using one of the SDI bits
#define ARINC429_BNR_STD_MSG_NUM_SIGBITS_20        20    // Extended BNR data field using both of the SDI bits
#define ARINC429_BNR_STD_MSG_MAX_NUM_SIGBITS       20    // Maximum number of sig bits in BNR message

#define ARINC429_BNR_STD_MSG_DATAFIELDMASK_UPTO18SIGBITS    0x1FFFFC00  // Masks out bits 1-10 and 30-32 of ARINC word
#define ARINC429_BNR_STD_MSG_DATAFIELDMASK_19SIGBITS        0x1FFFFE00  // Masks out bits 1-9 and 30-32 of ARINC word
#define ARINC429_BNR_STD_MSG_DATAFIELDMASK_20SIGBITS        0x1FFFFF00  // Masks out bits 1-8 and 30-32 of ARINC word

#define ARINC429_BNR_BCD_MSG_DISCRETE_BITS_SHIFT_VAL  10  // Shift value to move DISC data into data field of standard BNR/BCD data

    /* ARINC 429 BCD Message Definitions */
#define ARINC429_BCD_MAX_DIGIT_VAL                    9           // Max value of an individual BCD digit (valid values are 0-9 dec)
#define ARINC429_BCD_DATAFIELDMASK                    0x1FFFFC00  // Masks out bits 1-10 and 30-32 of ARINC word
#define ARINC429_BCD_STD_DATA_MAX_DATA_FIELD_SIZE     19       // Maximum size of the data field in a standard BCD message.
#define ARINC429_BCD_STD_MSG_DATA_FIELD_SHIFT         10       // Shift value to move standard BCD data into the data field.
#define ARINC429_BCD_BITS_PER_DIGIT                   4        // Number of bits per binary coded decimal digit (maximum)
#define ARINC429_BCD_STD_MSG_MAX_NUM_SIGDIGITS        5        // Max number of sig digits in a standard BCD message
#define ARINC429_BCD_STD_MSG_MAX_NUM_BITS_MSC         3        // Max number of bits in the most significant character of a 
    //standard BCD message

    /* ARINC 429 Discrete Message Definitions */
#define ARINC429_DISCRETE_MSG_MAX_NUM_BITS               19        // Maximum number of discrete bits in a discrete message
#define ARINC429_DISCRETE_MSG_MAX_DATA_FIELD_SHIFT       28        // Maximum shift count for discrete data field
#define ARINC429_DISCRETE_NONSTD_DATA_SHIFT_VAL          10        // right shift value for non standard DISC msgs (10 bits in SDI and label field)

#define NUM_BITS_IN_UINT32    32             // Total number of bits in a 32-bit UINT
#define INT32_SIGN_BIT_MASK   0x80000000     // Mask to use for checking the sign bit

#define clamp(value, low, high)  (((value) > (high)) ? (high) : (((value) < (low)) ? (low) : (value)))

#define RevBitsInByte(byte) \
   ( ( ( ( ( ( ( ( byte & 0xF0 ) >> 4 ) | ( ( byte & 0x0F ) << 4 ) ) & 0xCC ) >> 2 ) | ( ( ( ( ( byte & 0xF0 ) >> 4) | ( ( byte & 0x0F ) << 4)) & 0x33 ) << 2) ) & 0xAA ) >> 1 ) | \
   ( ( ( ( ( ( ( ( byte & 0xF0 ) >> 4 ) | ( ( byte & 0x0F ) << 4 ) ) & 0xCC ) >> 2 ) | ( ( ( ( ( byte & 0xF0 ) >> 4) | ( ( byte & 0x0F ) << 4)) & 0x33 ) << 2) ) & 0x55 ) << 1 )

#define FormatLabelNumber(labelInOctal) \
      RevBitsInByte ( ( ( (labelInOctal / 100 ) << ARINC429_LBL_OCT_SHIFT_HUNDREDS_DIG ) | ( ( ( labelInOctal / 10 ) - ( (labelInOctal / 100 ) * 10 )) << ARINC429_LBL_OCT_SHIFT_TENS_DIG ) | \
              ( labelInOctal - ( (labelInOctal / 10) * 10 ) ) ) )

    /**************  Function Definitions ************************/

    /* Converts a value from engineering units to raw data field values. If the data value exceeds the specified size limits of the 
     * BNR field then the BNR data value will be set to the maximum value that fits within the BNR field.
     * 
     * Returns EXIT_SUCCESS if conversion was successful. Returns EXIT_FAILURE otherwise if the input arguments are invalid. */
    int32_t ARINC429_BNR_ConvertEngValToRawBNRmsgData(const size_t numSigBits,
            const float resolution,
            const float dataEng,
            uint32_t * const result,
            bool * const isDataClipped);

    /* Converts a raw ARINC message field data value to engineering units.
     * 
     * Returns EXIT_SUCCESS if conversion was successful. Returns EXIT_FAILURE if function arguments were invalid. */
    int32_t ARINC429_BNR_ConvertRawMsgDataToEngUnits(const uint8_t numSigBits,
            const float resolution,
            float * const dataEng,
            const uint32_t rawMsgData);

    /* Extracts the source/destination identifier bits from a message. */
    uint8_t ARINC429_ExtractSDIbits(uint32_t ARINCMsg);

    /* Extracts the sign/status matrix bits from a message. */
    uint8_t ARINC429_ExtractSSMbits(uint32_t ARINCMsg);

    int32_t ARINC429_BCD_ConvertEngValToBCD(const size_t numSigDigits,
            const float resolution,
            const size_t numBitsMSC, // Number of bits in the most-significant BCD character
            const float dataEng, // Input value in engineering units
            uint32_t * const rawBCDdata, // result
            bool * const isDataClipped);

    int32_t ARINC429_BCD_ConvertBCDvalToEngVal(const size_t numSigDigits,
            const float resolution,
            float * const dataEng, // Converted result in engineering units.
            const uint32_t rawBCDdata); // BCD data

#ifdef	__cplusplus
}
#endif

#endif
