#include "maintenanceMode.h"
#include "ARINC_HI3584.h"
#include "COMSystemTimer.h"
#include "COMUart2.h"
#include "Timer23.h"



//static void mxProcessNew( );
//
//static struct
//{
//    bool isARINCtxvrALoopbackValid;
//    bool isARINCtxvrBLoopbackValid;
//    bool isUART1LoopbackValid;
//    bool areStrappingPinsConsistent;
//
//} MaintenanceModeMessage;

typedef enum
{
    UART_STATUS,
    ARINC_TXVRA,
    ARINC_TXVRB

} DEV_ID;

void maintenanceMode( circBuffer_t * txBuff, circBuffer_t * rxBuff )
{

    // Test dspic version number
    // uart 2 loopback
    // uart1 loopback
    // arinc loopback a
    // arinc loopback b
    // ram test
    // arinc driver test
    // Test sending a repeat character test
    // Test program memory CRC    

    uint8_t testArray[5] = { 0x01, 0xFF, 0xF2, 0xA5, 0xB2 };



    while (true)
    {
        cb_flushIn( txBuff, testArray, 5 );
        UART2_TxStart( );
        cb_reset( txBuff );
        Timer23_Delay_ms( 100 );
    }
    return;
}

//static void mxProcessNew( )
//{
//
//}