/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    leddisplay.c

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

#include "leddisplay.h"
#include "driver/pmp/drv_pmp.h"

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

LEDDISPLAY_DATA leddisplayData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void LEDDISPLAY_Initialize ( void )

  Remarks:
    See prototype in leddisplay.h.
 */

void LEDDISPLAY_Initialize ( SYS_MODULE_OBJ sysModuleObj )
{
    DRV_PMP_MODE_CONFIG config = {0};
    /* clear out the entire structure */
    memset(&leddisplayData,0,sizeof(leddisplayData));
    leddisplayData.pmpModule = sysModuleObj;
    leddisplayData.pmpDriverHandle = DRV_PMP_Open(DRV_PMP_INDEX_0,DRV_IO_INTENT_EXCLUSIVE| DRV_IO_INTENT_NONBLOCKING);
    
    /* Place the App state machine in its initial state. */
    leddisplayData.state = LEDDISPLAY_STATE_INIT;
    config.pmpMode= PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT;
	
	/* Interrupt mode */
    #ifdef PMP_INTERRUPT
        config.intMode=PMP_INTERRUPT_EVERY_RW_CYCLE;//PMP_INTERRUPT_NONE;
    #else
        config.intMode=PMP_INTERRUPT_NONE;
    #endif
	/* address/buffer increment mode */
    config.incrementMode= PMP_ADDRESS_AUTO_INCREMENT;
	
	/* Endian modes */
    config.endianMode= LITTLE_ENDIAN;
	
	/* Data Port Size */       
    config.portSize= PMP_DATA_SIZE_8_BITS;
	
	/* Wait states */
    config.waitStates.dataHoldWait = PMP_DATA_HOLD_1;
	config.waitStates.dataWait = PMP_DATA_WAIT_ONE;
    config.waitStates.strobeWait = PMP_STROBE_WAIT_4;
    
	/* PMP chip select pins selection */
	config.chipSelect = PMCS1_AS_ADDRESS_LINE_PMCS2_AS_CHIP_SELECT;
	DRV_PMP_ModeConfig ( leddisplayData.pmpDriverHandle, config );
}


/******************************************************************************
  Function:
    void LEDDISPLAY_Tasks ( void )

  Remarks:
    See prototype in leddisplay.h.
 */

void LEDDISPLAY_Tasks ( void )
{
    switch ( leddisplayData.state )
    {
        case LEDDISPLAY_STATE_INIT:
        {   
            if(leddisplayData.pmpDriverHandle == DRV_HANDLE_INVALID)
            {
                leddisplayData.state = LEDDISPLAY_ERROR;
            }
            else 
            {
                leddisplayData.state = LEDDISPLAY_FILL_ARRAY;
            }
            break;
        }
        case LEDDISPLAY_FILL_ARRAY:
        {
            if(DRV_PMP_ClientStatus(leddisplayData.pmpDriverHandle)==DRV_PMP_CLIENT_STATUS_OPEN)
            {
                uint8_t index;
                for(index=0;index<32;index++)
                {
                    leddisplayData.transferData[index]=index;
                }
                leddisplayData.state = LEDDISPLAY_SEND_ARRAY;
            }
            break;
        }
        case LEDDISPLAY_SEND_ARRAY:
        {
            leddisplayData.pQueue = DRV_PMP_Write(&leddisplayData.pmpDriverHandle,
                                                  0,
                                                  &leddisplayData.transferData[0],
                                                  32,
                                                  0); 
            leddisplayData.state = LEDDISPLAY_WAIT_SEND_ARRAY;
            break;
        }
        case LEDDISPLAY_WAIT_SEND_ARRAY:
        {
            if(DRV_PMP_TransferStatus(leddisplayData.pQueue)==PMP_TRANSFER_FINISHED)
            {
                leddisplayData.state = LEDDISPLAY_SEND_ARRAY;//zx LEDDISPLAY_HALT;
            }
            break;
        }
        case LEDDISPLAY_HALT:
        {
            
            break;
        }
        case LEDDISPLAY_ERROR:
        default:
        {
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
