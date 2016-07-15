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

//#define TP_OUT
#ifdef TP_OUT
#define mTP37_CONFIG()  LATDCLR=1<<8;TRISDCLR=1<<8
#define mTP37_TOGGLE()  LATDINV=1<<8
#define mTP37_SET()     LATDSET=1<<8
#define mTP37_CLEAR()   LATDCLR=1<<8
#else
#define mTP37_CONFIG()
#define mTP37_TOGGLE()
#define mTP37_SET()  
#define mTP37_CLEAR()
#endif
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

static void CommsSPIStartedCallback(DRV_SPI_BUFFER_EVENT event, 
                                    DRV_SPI_BUFFER_HANDLE handle, 
                                    COMMS_DATA* context)
{
    mTP37_SET();
    if((event == DRV_SPI_BUFFER_EVENT_PENDING)||(event == DRV_SPI_BUFFER_EVENT_PROCESSING))
    {
        commsData.spi.status.running = true;
        if(commsData.spi.status.dataReady)
        {
            commsData.spi.status.error=true;
            commsData.spi.status.overrunError=true;
        }
        commsData.spi.status.dataReady = false;
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        commsData.spi.status.error=true;
        commsData.spi.status.unknownError=true;
    }        
}

/******************************************************************************/

static void CommsSPICompletedCallback(DRV_SPI_BUFFER_EVENT event, 
                                      DRV_SPI_BUFFER_HANDLE handle, 
                                      COMMS_DATA* context)
{
    mTP37_CLEAR();
    mToggleJ10p35();
    context->spi.status.dataReady = true;
    context->spi.status.readStarted = false;
    if(event == DRV_SPI_BUFFER_EVENT_COMPLETE)
    {
        context->spi.status.running = false;     
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        context->spi.status.error=true;
    }        
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


bool COMMS_StartSPIRead(COMMS_DATA *comms)
{
    comms->bufferIndex.filled = comms->bufferIndex.filling;
    comms->bufferIndex.filling++;
    comms->bufferIndex.filling &= WORKING_BUFFERS_MASK;
    DRV_SPI_BufferAddRead2(comms->spi.drvHandle,
                           comms->workingBuffer[comms->bufferIndex.filling].incoming.b32,
                           comms->bufferSize.max.b8,
                          (void*)CommsSPICompletedCallback,
                           comms,
                          &comms->spi.bufferHandle);
    comms->spi.status.readStarted = true;
    return comms->spi.status.readStarted;
}

/******************************************************************************/

bool COMMS_OpenCameraSPI(COMMS_DATA *comms)
{
    if(comms->spi.drvHandle != NULL)
    {
        DRV_SPI_Close(comms->spi.drvHandle);
    }
    comms->spi.status.running = false;
    comms->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
                                        /*DRV_IO_INTENT_NONBLOCKING|*/
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
            COMMS_StartSPIRead(comms);   
        }
    }
    comms->spi.status.running = (DRV_HANDLE_INVALID != comms->spi.drvHandle);
    return comms->spi.status.running & comms->spi.status.readStarted;
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
    mTP37_CONFIG();
    mTP37_TOGGLE();
    /* Place the App state machine in its initial state. */
    commsData.state = COMMS_STATE_INIT;
    memset(&commsData,0,sizeof(commsData));
    commsData.bufferSize.max.b8 = COMMS_BUFFER_SIZE_8;
    commsData.bufferSize.max.b16 = COMMS_BUFFER_SIZE_16;
    commsData.bufferSize.max.b32 = COMMS_BUFFER_SIZE_32;
    commsData.bufferSize.dataStructureRaw = 
        sizeof(commsData.workingBuffer[0].incoming.packet.dataStructure.raw);
    mTP37_TOGGLE();
}

/******************************************************************************/

            
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
                commsData.status.flags.initialized = true;
            }        
            if (commsData.status.flags.initialized & commsData.spi.status.readStarted)
            {
                commsData.state = COMMS_STATE_SERVICE_TASKS;
                /* otherwise, drop through */
            }
            else
            {
                break;                
            }            
        }        
        case COMMS_STATE_SERVICE_TASKS:
        {
            if(commsData.spi.status.dataReady)
            {
                commsData.state = COMMS_STATE_INCOMING_CAMERA_DATA;
                commsData.spi.status.dataReady = false;
                /* toggle LED2 when we get an incoming packet */
                BSP_LEDToggle(BSP_LED_2);
                COMMS_StartSPIRead(&commsData);
            }
            else
            {
                break;
            }
        }
        case COMMS_STATE_INCOMING_CAMERA_DATA:
        {
            switch(commsData.workingBuffer[commsData.bufferIndex.filled].incoming.packet.header.ID)
            {
                case IMAGE_INFO_ID:
                {
                    commsData.state = COMMS_STATE_INCOMING_CAMERA_HEADER;
                    break;
                }
                case IMAGE_LINE_ID:
                {
                    if(commsData.status.flags.imageStartReceived)
                    {
                        commsData.state = COMMS_STATE_INCOMING_CAMERA_LINE;
                    }
                    else
                    {
                        commsData.state = COMMS_STATE_SERVICE_TASKS;
                    }
                    break;
                }
                case IMAGE_DONE_ID:
                {
                    if(commsData.status.flags.imageStartReceived)
                    {
                        commsData.state = COMMS_STATE_INCOMING_CAMERA_IMAGE_COMPLETE;
                    }
                    else
                    {
                        commsData.state = COMMS_STATE_SERVICE_TASKS;
                    }                    
                    break;
                }
                default:
                {
                    /* don't know what it is */
                    commsData.state = COMMS_STATE_SERVICE_TASKS;
                    break;
                }   
            }
            if(commsData.state == COMMS_STATE_SERVICE_TASKS)
            {
                break;
            }        
            commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.line = 
                commsData.workingBuffer[commsData.bufferIndex.filled].incoming.packet.header.line;
            commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.length = 
                commsData.workingBuffer[commsData.bufferIndex.filled].incoming.packet.header.length;  
        }
        case COMMS_STATE_INCOMING_CAMERA_HEADER:
        {
            if(commsData.state == COMMS_STATE_INCOMING_CAMERA_HEADER)
            {
                /* check to make sure the length & line are valid                 */
                if((commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.length == 
                                                            sizeof(IMAGE_INFO_TYPE))&&
                   (commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.line == 
                                                              IMAGE_INFO_LINE_VALUE))
                {
                    commsData.status.flags.imageComplete = false;
                    commsData.status.flags.imageStartReceived = true;
                    commsData.status.linesReceived = 0;                
                    memcpy(&commsData.image.properties,
                           &commsData.workingBuffer[commsData.bufferIndex.filled].incoming.packet.dataStructure.imageInfo,
                           sizeof(IMAGE_INFO_TYPE));
                    commsData.bufferSize.lineLength = commsData.image.properties.dimensions.horizontal * sizeof(FLIR_PIXEL_TYPE);
                    memset(commsData.status.lineList,0,80);
                }
                commsData.state = COMMS_STATE_SERVICE_TASKS;
                break;
            }
        }
        case COMMS_STATE_INCOMING_CAMERA_LINE:
        {
            if(commsData.state == COMMS_STATE_INCOMING_CAMERA_LINE)
            {
                /* check to make sure the length & line are valid.                */
                if(commsData.status.flags.imageStartReceived)
                {
                   
                    if((commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.length == 
                                                                  commsData.bufferSize.lineLength)&&
                        (commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.line < 
                                    commsData.image.properties.dimensions.vertical))
                    {
                        commsData.status.lineList[commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.line] = 0xFF;
                        commsData.status.linesReceived++;
                        memcpy(&commsData.image.buffer.pixel[0][commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.line],
                                commsData.workingBuffer[commsData.bufferIndex.filled].incoming.b8,
                                commsData.bufferSize.lineLength);
                    }
                }
                commsData.state = COMMS_STATE_SERVICE_TASKS;
                break;
            }
        }
        case COMMS_STATE_INCOMING_CAMERA_IMAGE_COMPLETE:
        {
            if(commsData.state == COMMS_STATE_INCOMING_CAMERA_IMAGE_COMPLETE)
            {
                /* image complete message- lower the imageStartReceived flag to   */
                /* indicate that no more lines of data will go into this image    */
                commsData.status.flags.imageStartReceived = false;
                /* check to make sure the length & line are valid                 */
                if((commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.length == 0) &&
                   (commsData.workingBuffer[commsData.bufferIndex.filled].fromPacket.line == 
                                                             IMAGE_DONE_LINE_VALUE))
                {
                    if(commsData.status.linesReceived == commsData.image.properties.dimensions.vertical)
                    {
                        /* if we got all the lines, it is complete. */
                        commsData.status.flags.imageComplete = true;
                        BSP_LEDToggle(BSP_LED_3);
                        commsData.state = COMMS_STATE_PROCESS_IMAGE;
                    }
                    /* the image transfer is complete!                            */
                }
                if(commsData.state != COMMS_STATE_PROCESS_IMAGE)
                {
                    commsData.state = COMMS_STATE_SERVICE_TASKS;
                    break;
                }
                /* otherwise, want to drop through and process the image.         */
            }
            else
            {
                /* not sure what it could be. */
                commsData.status.flags.mysteryState= true;
                commsData.state = COMMS_STATE_SERVICE_TASKS;
                break;
            }
        }
        case COMMS_STATE_PROCESS_IMAGE:
        {
            commsData.state = COMMS_STATE_DELIVER_IMAGE;
            if(commsData.state != COMMS_STATE_DELIVER_IMAGE)
            {
                break;
            }
        }
        case COMMS_STATE_DELIVER_IMAGE:
        {            
            commsData.state = COMMS_STATE_SERVICE_TASKS;
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
