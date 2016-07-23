/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    comms.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "comms.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

COMMS_DATA commsData;



// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter by reading characters from a specified pipe.  The pipeRead function is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.  Typically, the pipe is connected to the application's
    USART receive function, but could be any other Harmony module which supports the pipe interface. 
*/
//static void USART_Task (void)
//{
//    switch (commsData.uart.BMState)
//    {
//        default:
//        case USART_BM_INIT:
//        {
//            commsData.uart.tx.count = 0;
//            commsData.uart.rx.count = 0;
//            commsData.uart.BMState = USART_BM_WORKING;
//            break;
//        }
//        case USART_BM_WORKING:
//        {
//            if (commsData.uart.tx.count < sizeof(commsData.uart.tx.buffer)) 
//            {
//                if(!DRV_USART_TransmitBufferIsFull(commsData.uart.handle))
//                {
//                    DRV_USART_WriteByte(commsData.uart.handle, commsData.uart.tx.buffer[commsData.uart.tx.count]);
//                    commsData.uart.tx.count++;
//                }
//            }
//
//            if (commsData.uart.rx.count < sizeof(commsData.uart.handle)) 
//            {
//                if(!DRV_USART_ReceiverBufferIsEmpty(commsData.uart.handle))
//                {
//                    commsData.uart.rx.buffer[commsData.uart.rx.count] = DRV_USART_ReadByte(commsData.uart.handle);
//                    commsData.uart.rx.count++;
//                }
//            }
//
//            /* Have we finished? */
//            if (commsData.uart.tx.count == sizeof(commsData.uart.tx.buffer))// && commsData.uart.rx.count == sizeof(commsData.uart.tx.buffer))
//            {
//                commsData.uart.BMState = USART_BM_DONE;
//            }
//            break;
//        }
//
//        case USART_BM_DONE:
//        {
//            break;
//        }
//    }
//}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void COMMS_Initialize ( void )

  Remarks:
    See prototype in comms.h.
 */

void COMMS_Initialize ( SYS_MODULE_INDEX UARTIndex )
{
    memset(&commsData,0,sizeof(commsData));
    /* Place the App state machine in its initial state. */
    commsData.state = COMMS_STATE_INIT;
    //commsData.uart.handle = DRV_HANDLE_INVALID;
    //commsData.uart.index = UARTIndex;
}


/******************************************************************************
  Function:
    void COMMS_Tasks ( void )

  Remarks:
    See prototype in comms.h.
 */

void COMMS_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( commsData.state )
    {
        /* Application's initial state. */
        case COMMS_STATE_INIT:
        {
            bool appInitialized = true;
       
//            if (commsData.uart.handle == DRV_HANDLE_INVALID)
//            {
//                commsData.uart.handle = DRV_USART_Open(commsData.uart.index, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
//                appInitialized &= ( DRV_HANDLE_INVALID != commsData.uart.handle );
//            }
        
            if (appInitialized)
            {
                /* initialize the USART state machine */
                //commsData.uart.BMState = USART_BM_INIT;
                //strcpy(commsData.uart.tx.buffer,TX_MESSAGE);
                commsData.state = COMMS_STATE_SERVICE_TASKS;
            }
            break;
        }

        case COMMS_STATE_SERVICE_TASKS:
        {
			//USART_Task();        
            break;
        }

        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
