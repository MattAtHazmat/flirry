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

static void CommsSPIStartedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    if((event == DRV_SPI_BUFFER_EVENT_PENDING)||(event == DRV_SPI_BUFFER_EVENT_PROCESSING))
    {
        commsData.spi.status.running = true;
        commsData.spi.status.started = true;
        if(commsData.spi.status.complete)
        {
            commsData.spi.status.error=true;
            commsData.spi.status.overrunError=true;
        }
        commsData.spi.status.complete = false;
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        commsData.spi.status.error=true;
        commsData.spi.status.unknownError=true;
    }        
}

static void CommsSPICompletedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    if(event == DRV_SPI_BUFFER_EVENT_COMPLETE)
    {
        commsData.spi.status.running = false;
        commsData.spi.status.started = false;
        commsData.spi.status.complete = true;
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        commsData.spi.status.error=true;
    }        
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

bool COMMS_OpenCameraSPI(COMMS_DATA *comms)
{
    if(comms->spi.drvHandle != NULL)
    {
        DRV_SPI_Close(comms->spi.drvHandle);
    }
    comms->spi.status.running = false;
    comms->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
                                        DRV_IO_INTENT_NONBLOCKING|
                                        DRV_IO_INTENT_EXCLUSIVE|
                                        DRV_IO_INTENT_READ);
    if(DRV_HANDLE_INVALID != comms->spi.drvHandle)
    {
        DRV_SPI_CLIENT_DATA clientData;
        clientData.baudRate = 0; /* not overriding the baud rate */
        clientData.operationEnded = (void*)CommsSPICompletedCallback;
        clientData.operationStarting = (void*)CommsSPIStartedCallback;
        if(DRV_SPI_ClientConfigure(comms->spi.drvHandle,&clientData)>=0)
        {
                DRV_SPI_BufferAddRead2(comms->spi.drvHandle,
                                      &comms->RXBuffer.b8,
                                      comms->RXBuffer.size.max.b8,
                                      (void*)CommsSPICompletedCallback,
                                      NULL,
                                      &comms->spi.bufferHandle);            
        }
    }
    comms->spi.status.running = ((DRV_HANDLE_INVALID != comms->spi.drvHandle)&&
                                 (DRV_SPI_BUFFER_HANDLE_INVALID != comms->spi.bufferHandle));
    return comms->spi.status.running;
}

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

void COMMS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    commsData.state = COMMS_STATE_INIT;
    memset(&commsData,0,sizeof(commsData));
    commsData.RXBuffer.size.max.b8 = COMMS_BUFFER_SIZE_8;
    commsData.RXBuffer.size.max.b16 = COMMS_BUFFER_SIZE_16;
    commsData.RXBuffer.size.max.b32 = COMMS_BUFFER_SIZE_32;
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
            if(!commsData.spi.status.configured)
            {
                commsData.spi.status.configured = COMMS_OpenCameraSPI(&commsData);
            }            
            if(commsData.spi.status.configured)
            {                
                commsData.status.initialized = true;
            }        
            if (commsData.status.initialized)
            {            
                commsData.state = COMMS_STATE_SERVICE_TASKS;
            }
            break;
        }
        case COMMS_STATE_INCOMING_CAMERA_CLEAN_UP:
        {
            commsData.state = COMMS_STATE_SERVICE_TASKS;
            //break;
        }
        case COMMS_STATE_SERVICE_TASKS:
        {
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(commsData.spi.bufferHandle)) //if(commsData.spi.status.complete)
            {
                commsData.state = COMMS_STATE_INCOMING_CAMERA_DATA;
                memcpy(commsData.RXBuffer.b8,
                       commsData.workingBuffer.b8,
                       commsData.RXBuffer.size.max.b8);
                #ifdef __DEBUG
                    /* if we're debugging, clear out this memory*/
                    memset(commsData.RXBuffer.b8,
                           0,
                           commsData.RXBuffer.size.max.b8);
                #endif
                commsData.spi.status.complete = false;
            }
            else
            {
                break;
            }
        }
        case COMMS_STATE_INCOMING_CAMERA_DATA:
        {
            switch(commsData.workingBuffer.b8[ID_LOCATION])
            {
                case IMAGE_INFO_ID:
                {
                    commsData.state = COMMS_STATE_INCOMING_CAMERA_HEADER;
                    break;
                }
                case IMAGE_LINE_ID:
                {
                    if(commsData.status.imageStartReceived)
                    {
                        commsData.state = COMMS_STATE_INCOMING_CAMERA_LINE;
                    }
                    else
                    {
                        commsData.state = COMMS_STATE_INCOMING_CAMERA_CLEAN_UP;
                    }
                    break;
                }
                case IMAGE_DONE_ID:
                {
                    if(commsData.status.imageStartReceived)
                    {
                        commsData.state = COMMS_STATE_INCOMING_CAMERA_IMAGE_COMPLETE;
                    }
                    else
                    {
                        commsData.state = COMMS_STATE_INCOMING_CAMERA_CLEAN_UP;
                    }                    
                    break;
                }
                default:
                {
                    /* don't know what it is */
                    commsData.state = COMMS_STATE_INCOMING_CAMERA_CLEAN_UP;
                    break;
                }   
            }
            if(commsData.state == COMMS_STATE_INCOMING_CAMERA_CLEAN_UP)
            {
                break;
            }        
            commsData.workingBuffer.line = (uint16_t)commsData.workingBuffer.b8[LINE_LOCATION];
            commsData.workingBuffer.length = (uint16_t)commsData.workingBuffer.b8[LENGTH_LOCATION];        
        }
        case COMMS_STATE_INCOMING_CAMERA_HEADER:
        {
            /* check to make sure the length & line are valid */
            if((commsData.workingBuffer.length == sizeof(IMAGE_INFO_TYPE))&&
               (commsData.workingBuffer.line == IMAGE_INFO_LINE_VALUE))
            {
                commsData.workingBuffer.currentLine = 0;
                commsData.status.imageStartReceived = true;
                memcpy(&commsData.image.properties,&commsData.workingBuffer.b8[DATA_START_LOCATION],sizeof(IMAGE_INFO_TYPE));
            }
            commsData.state = COMMS_STATE_INCOMING_CAMERA_CLEAN_UP;
            break;
        }
        case COMMS_STATE_INCOMING_CAMERA_LINE:
        {
            /* check to make sure the length & line are valid */
            uint16_t lineLengthBytes = commsData.image.properties.dimensions.horizontal*sizeof(FLIR_PIXEL_TYPE);
            if((commsData.workingBuffer.length == lineLengthBytes)&&
                (commsData.workingBuffer.line < commsData.image.properties.dimensions.vertical))
            {
                commsData.workingBuffer.currentLine++;
                memcpy(&commsData.image.buffer.pixel[0][commsData.workingBuffer.line],&commsData.workingBuffer.b8[DATA_START_LOCATION],lineLengthBytes);
            }
            commsData.state = COMMS_STATE_INCOMING_CAMERA_CLEAN_UP;
            break;
        }
        case COMMS_STATE_INCOMING_CAMERA_IMAGE_COMPLETE:
        {
            /* check to make sure the length & line are valid */
            if((commsData.workingBuffer.length == 0)&&
               (commsData.workingBuffer.line == IMAGE_DONE_LINE_VALUE))
            {
                if(commsData.workingBuffer.currentLine == commsData.image.properties.dimensions.vertical)
                {
                    commsData.status.imageComplete = true;
                }
                /* the image transfer is complete! */
            }
            break;
        }
        
        case COMMS_STATE_ERROR:
        default:
        {
            COMMS_Initialize();
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
