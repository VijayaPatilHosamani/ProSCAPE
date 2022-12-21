/* Filename: ARINC_HI3584.h
 * 
 * Author: Henry Gilbert
 * 
 * Date: 1 August 2022
 *  
 * Description: Public interface to the ARINC429 HI-3584 driver. Features pin definitions
 *      for data bus, transceiver control signals, and hardware specific function prototypes. 
 * 
 * All Rights Reserved. Copyright Archangel Systems 2022
 */

#ifndef ARINC_HI3584_H
#define ARINC_HI3584_H

/**************  Include Files  ************************/
#include "stdint.h"
#include "ARINC_typedefs.h"
#include "stdbool.h"
#include "stdlib.h"
#include "../COM/pic_h/p30F6014A.h"

/****************************************
 **** ARINC Transceiver A Configuration *
 ****************************************/

/* Digital INPUT Pins for ARINC transceiver A */

/*  Data ready on ARINC transceiver A, receiver 1 */
#define ARINC429_HI3584_TXVRA_DR1            PORTDbits.RD15   
#define ARINC429_HI3584_TXVRA_DR1_TRIS       TRISDbits.TRISD15

/*  Data ready on ARINC transceiver A, receiver 2 */
#define ARINC429_HI3584_TXVRA_DR2            PORTDbits.RD2     
#define ARINC429_HI3584_TXVRA_DR2_TRIS       TRISDbits.TRISD2

/*  Transmit Buffer Full Status Register signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_FFT            PORTDbits.RD13
#define ARINC429_HI3584_TXVRA_FFT_TRIS       TRISDbits.TRISD13

/* Digital OUTPUT Pins for ARINC transceiver A */
/* Select signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_SEL            LATBbits.LATB14   
#define ARINC429_HI3584_TXVRA_SEL_TRIS       TRISBbits.TRISB14

/* Enable 1 signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_EN1            LATGbits.LATG3  
#define ARINC429_HI3584_TXVRA_EN1_TRIS       TRISGbits.TRISG3

/* Enable 2 signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_EN2            LATGbits.LATG2  
#define ARINC429_HI3584_TXVRA_EN2_TRIS       TRISGbits.TRISG2

/* Latch Enable 1 signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_PL1            LATAbits.LATA14   
#define ARINC429_HI3584_TXVRA_PL1_TRIS       TRISAbits.TRISA14

/*  Latch Enable 2 signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_PL2            LATAbits.LATA15   
#define ARINC429_HI3584_TXVRA_PL2_TRIS       TRISAbits.TRISA15

/* Enable Transmit signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_ENTX           LATDbits.LATD8    
#define ARINC429_HI3584_TXVRA_ENTX_TRIS      TRISDbits.TRISD8

/* Control Strobe signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_CWSTR          LATDbits.LATD9   
#define ARINC429_HI3584_TXVRA_CWSTR_TRIS     TRISDbits.TRISD9 

/* Read Status Register signal for ARINC transceiver A */
#define ARINC429_HI3584_TXVRA_RSR            LATDbits.LATD10   
#define ARINC429_HI3584_TXVRA_RSR_TRIS       TRISDbits.TRISD10

/****************************************
 **** ARINC Transceiver B Configuration *
 ****************************************/

/* Digital Input pins for ARINC transceiver B */

/*  Data ready on ARINC transceiver B, receiver 1 */
#define ARINC429_HI3584_TXVRB_DR1            PORTDbits.RD4    
#define ARINC429_HI3584_TXVRB_DR1_TRIS       TRISDbits.TRISD4

/*  Data ready on ARINC transceiver B, receiver 2 */
#define ARINC429_HI3584_TXVRB_DR2            PORTDbits.RD6    
#define ARINC429_HI3584_TXVRB_DR2_TRIS       TRISDbits.TRISD6

/* Transmit Buffer Full Status Register signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_FFT            PORTAbits.RA7   
#define ARINC429_HI3584_TXVRB_FFT_TRIS       TRISAbits.TRISA7

/* Digital Output Pins for ARINC transceiver B */

/* Select signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_SEL            LATDbits.LATD11  
#define ARINC429_HI3584_TXVRB_SEL_TRIS       TRISDbits.TRISD11

/* Enable 1 signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_EN1            LATDbits.LATD3  
#define ARINC429_HI3584_TXVRB_EN1_TRIS       TRISDbits.TRISD3

/* Enable 2 signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_EN2            LATDbits.LATD12 
#define ARINC429_HI3584_TXVRB_EN2_TRIS       TRISDbits.TRISD12

/*Latch Enable 1 signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_PL1            LATFbits.LATF0  
#define ARINC429_HI3584_TXVRB_PL1_TRIS       TRISFbits.TRISF0 

/* Latch Enable 2 signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_PL2            LATFbits.LATF1  
#define ARINC429_HI3584_TXVRB_PL2_TRIS       TRISFbits.TRISF1

/* Enable Transmit signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_ENTX           LATGbits.LATG1    
#define ARINC429_HI3584_TXVRB_ENTX_TRIS      TRISGbits.TRISG1

/* Control Strobe signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_CWSTR          LATGbits.LATG0  
#define ARINC429_HI3584_TXVRB_CWSTR_TRIS     TRISGbits.TRISG0 

/* Read Status Register signal for ARINC transceiver B */
#define ARINC429_HI3584_TXVRB_RSR            LATGbits.LATG14  
#define ARINC429_HI3584_TXVRB_RSR_TRIS       TRISGbits.TRISG14


/* ARINC Data I/O Pins - Same for both ARINC transceivers  */
/* Data bit 0 */
#define DB00_WRITE         LATCbits.LATC1
#define DB00_READ          PORTCbits.RC1
#define DB00_TRIS          TRISCbits.TRISC1

/* Data bit 1 */
#define DB01_WRITE         LATCbits.LATC2
#define DB01_READ          PORTCbits.RC2
#define DB01_TRIS          TRISCbits.TRISC2

/* Data bit 2 */
#define DB02_WRITE         LATCbits.LATC3
#define DB02_READ          PORTCbits.RC3
#define DB02_TRIS          TRISCbits.TRISC3

/* Data bit 3 */
#define DB03_WRITE         LATCbits.LATC4
#define DB03_READ          PORTCbits.RC4
#define DB03_TRIS          TRISCbits.TRISC4

/* Data bit 4 */
#define DB04_WRITE         LATAbits.LATA12
#define DB04_READ          PORTAbits.RA12
#define DB04_TRIS          TRISAbits.TRISA12

/* Data bit 5 */
#define DB05_WRITE         LATAbits.LATA13
#define DB05_READ          PORTAbits.RA13
#define DB05_TRIS          TRISAbits.TRISA13

/* Data bit 6 */
#define DB06_WRITE         LATBbits.LATB6
#define DB06_READ          PORTBbits.RB6
#define DB06_TRIS          TRISBbits.TRISB6

/* Data bit 7 */
#define DB07_WRITE         LATBbits.LATB7
#define DB07_READ          PORTBbits.RB7
#define DB07_TRIS          TRISBbits.TRISB7

/* Data bit 8 */
#define DB08_WRITE         LATAbits.LATA9
#define DB08_READ          PORTAbits.RA9
#define DB08_TRIS          TRISAbits.TRISA9

/* Data bit 9 */
#define DB09_WRITE         LATAbits.LATA10
#define DB09_READ          PORTAbits.RA10
#define DB09_TRIS          TRISAbits.TRISA10

/* Data bit 10 */
#define DB10_WRITE         LATBbits.LATB8
#define DB10_READ          PORTBbits.RB8
#define DB10_TRIS          TRISBbits.TRISB8

/* Data bit 11 */
#define DB11_WRITE         LATBbits.LATB9
#define DB11_READ          PORTBbits.RB9
#define DB11_TRIS          TRISBbits.TRISB9

/* Data bit 12 */
#define DB12_WRITE         LATBbits.LATB10
#define DB12_READ          PORTBbits.RB10
#define DB12_TRIS          TRISBbits.TRISB10

/* Data bit 13 */
#define DB13_WRITE         LATBbits.LATB11
#define DB13_READ          PORTBbits.RB11
#define DB13_TRIS          TRISBbits.TRISB11

/* Data bit 14 */
#define DB14_WRITE         LATBbits.LATB12
#define DB14_READ          PORTBbits.RB12
#define DB14_TRIS          TRISBbits.TRISB12

/* Data bit 15 */
#define DB15_WRITE         LATBbits.LATB13
#define DB15_READ          PORTBbits.RB13
#define DB15_TRIS          TRISBbits.TRISB13


/************** Function Prototypes ************************/

/* Initializes the PIC microcontroller pins used as discrete signals to/from the first of two HI-3584 ARINC transceivers (ARINC transceiver A). */
void ARINC429_HI3584_txvrA_Initialize();

/* Initializes the PIC microcontroller pins used as discrete signals to/from the second of two HI-3584 ARINC transceivers (ARINC transceiver B). */
void ARINC429_HI3584_txvrB_Initialize();

/* Reads one ARINC message from ARINC transceiver A, receiver 1. */
uint32_t ARINC429_HI3584_txvrA_rx1_ReadWord(void);

/* Reads one ARINC message from ARINC transceiver A, receiver 2. */
uint32_t ARINC429_HI3584_txvrA_rx2_ReadWord(void);

/* Loads data into the ARINC transmit buffer of ARINC transceiver A. */
void ARINC429_HI3584_txvrA_TransmitWord(const uint32_t ARINCword); /* 32-bit ARINC word to transmit */

/* Loads configuration data into ARINC transceiver A. */
bool ARINC429_HI3584_txvrA_LoadCtrlReg(const uint16_t ctrlRegVal); /* transceiver control register value */

/* Reads one ARINC message from ARINC transceiver B, receiver 1. */
uint32_t ARINC429_HI3584_txvrB_rx1_ReadWord(void);

/* Reads one ARINC message from ARINC transceiver B, receiver 2. */
uint32_t ARINC429_HI3584_txvrB_rx2_ReadWord(void);

/* Loads data into the ARINC transmit buffer of ARINC transceiver A. */
void ARINC429_HI3584_txvrB_TransmitWord(const uint32_t ARINCword); /* 32-bit ARINC word to transmit */

/* Loads configuration data into ARINC transceiver B. */
bool ARINC429_HI3584_txvrB_LoadCtrlReg(const uint16_t ctrlRegVal); /* transceiver control register value */

/* Performs a loop back test on ARINC transceiver A, receiver 2. */
bool ARINC429_HI3584_txvrA_LoopbackTest(void);

/* Performs a loop back test on ARINC transceiver B, receiver 2. */
bool ARINC429_HI3584_txvrB_LoopbackTest(void);

/* Configures label filters for both transceivers from configuration data */
bool ARINC429_HI3584_SetupLabelFiltersTxvrB(const ARINC429_RxMsgArray * const msgs);
bool ARINC429_HI3584_SetupLabelFiltersTxvrA(const ARINC429_RxMsgArray * const msgs);


#endif
/* end of ARINC_HI3584.h */
