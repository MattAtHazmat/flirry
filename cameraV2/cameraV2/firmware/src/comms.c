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
#include "flir.h"
extern FLIR_DATA flirData;
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
TaskHandle_t commsHandle;
extern TaskHandle_t FLIRHandle;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

static void CommsSPIStartedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    //CommsSPISlaveInvert();// CommsSPISlaveDeselect();
    if((event == DRV_SPI_BUFFER_EVENT_PENDING)||(event == DRV_SPI_BUFFER_EVENT_PROCESSING))
    {
        CommsSPISlaveSelect();
        commsData.spi.status.running = true;
        commsData.spi.status.started = true;
        commsData.spi.status.complete = false;
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        commsData.spi.status.error=true;
    }        
}

static void CommsSPICompletedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    //CommsSPISlaveInvert();// CommsSPISlaveDeselect();
    if(event == DRV_SPI_BUFFER_EVENT_COMPLETE)
    {
        CommsSPISlaveDeselect();
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


bool OpenDisplaySPI(COMMS_DATA *comms)
{
    comms->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_1,
                                        DRV_IO_INTENT_BLOCKING|DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_WRITE);
    if(DRV_HANDLE_INVALID != comms->spi.drvHandle)
    {
        DRV_SPI_CLIENT_DATA clientData;
        clientData.baudRate = 0; /* not overriding the baud rate */
        clientData.operationEnded = (void*)CommsSPICompletedCallback;
        clientData.operationStarting = (void*)CommsSPIStartedCallback;
        if(DRV_SPI_ClientConfigure(comms->spi.drvHandle,&clientData)>=0)
        {
            /* success! */
        }
    }
    return(DRV_HANDLE_INVALID != comms->spi.drvHandle);
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
    CommsSPISlaveDeselect();
    memset(&commsData,0,sizeof(commsData));
    commsData.state = COMMS_STATE_INIT;
    commsData.image.properties.dimensions.horizontal = HORIZONTAL_SIZE;
    commsData.image.properties.dimensions.vertical = VERTICAL_SIZE;
    commsData.image.properties.size.pixels = commsData.image.properties.dimensions.horizontal * 
                                             commsData.image.properties.dimensions.vertical;
    commsData.image.properties.size.bytes =sizeof(IMAGE_BUFFER_TYPE);
    commsData.buffer.number = IMAGE_BUFFERS;
    commsData.TXBuffer.size.max.b8  = COMMS_BUFFER_SIZE_8;
    commsData.TXBuffer.size.max.b16 = COMMS_BUFFER_SIZE_16;
    commsData.TXBuffer.size.max.b32 = COMMS_BUFFER_SIZE_32;
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
            commsData.RTOS.myHandle = xTaskGetCurrentTaskHandle();
            commsHandle = commsData.RTOS.myHandle;
            if(commsData.status.initialized)
            {
                commsData.state = COMMS_STATE_SERVICE_TASKS;
                COMMS_NotifyReady(&commsData);
            }
            else
            {
                if(!commsData.status.SPIInitialized)
                {
                    commsData.state = COMMS_STATE_INITIALIZE_SPI;
                    break;
                }
                
                /* make it here, everything is initialized. */
                commsData.status.initialized = true;                
            }
            break;
        }
        case COMMS_STATE_INITIALIZE_SPI:
        {
            commsData.status.SPIInitialized = OpenDisplaySPI(&commsData);
            commsData.state = COMMS_STATE_INIT;
            break;
        }
        case COMMS_STATE_SERVICE_TASKS:
        {
            uint32_t toTransmit;
            toTransmit = COMMS_WaitForImageReady();
            memcpy(&commsData.image.buffer[commsData.buffer.receive],
                   &flirData.image.buffer[toTransmit].vector,
                   commsData.image.properties.size.bytes);
            commsData.buffer.transmit = commsData.buffer.receive;
            commsData.buffer.receive++;
            commsData.buffer.receive &= (IMAGE_BUFFERS-1);
            commsData.counters.imagesReceived++;
            commsData.state = COMMS_TRANSMIT_IMAGE;            
        }
        case COMMS_TRANSMIT_IMAGE:
        case COMMS_TRANSMIT_IMAGE_HEADER:
        {
            if(COMMS_TransmitImageHeader(&commsData))
            {
                commsData.state = COMMS_TRANSMIT_LINE;
                commsData.transmitLine = 0;
            }
            else
            {
                commsData.counters.failure.header++;
            }
            break;
        }
        case COMMS_TRANSMIT_LINE:            
        {
            if(COMMS_TransmitImageLine(&commsData))
            {
                commsData.transmitLine++;
                if(commsData.transmitLine >= commsData.image.properties.dimensions.vertical)
                {
                    commsData.state = COMMS_STATE_IMAGE_DONE;
                }
            }
            else
            {
                commsData.counters.failure.line++;
            }
            break;
        }
        case COMMS_STATE_IMAGE_DONE:
        {
            if(COMMS_TransmitImageDone(&commsData))
            {
                COMMS_NotifyReady(&commsData);
                commsData.counters.imagesTransmitted++;
                commsData.state = COMMS_STATE_SERVICE_TASKS;                
            }
            else
            {
                commsData.counters.failure.done++;
            }
            break;
        }
        case COMMS_STATE_ERROR:
        default:
        {
            /* not sure what to do. Initialize comms. */
            COMMS_Initialize();
            break;
        }
    }
    /* not sure if this is the right way to get this information- but it seems*/
    /* like a chicken/egg situation. Get it this way for now.                 */
    if(commsData.RTOS.FLIRHandle == NULL)
    {
        commsData.RTOS.FLIRHandle = FLIRHandle;
    }
}

/******************************************************************************/

bool COMMS_TransmitImageHeader(COMMS_DATA *comms)
{
    bool success = false;

    memset(comms->TXBuffer.b8,0xFF,DATA_START_LOCATION);
    comms->TXBuffer.b8[ID_LOCATION] = IMAGE_INFO_ID;
    comms->TXBuffer.b8[LENGTH_MSB_LOCATION] = (0xFF&(sizeof(IMAGE_INFO_TYPE)>>8));   
    comms->TXBuffer.b8[LENGTH_LSB_LOCATION] = (0xff&(sizeof(IMAGE_INFO_TYPE)));
    memcpy(&comms->TXBuffer.b8[DATA_START_LOCATION],&comms->image.properties,sizeof(IMAGE_INFO_TYPE));
    success = COMMS_SPIWrite(comms,MESSAGE_HEADER_LENGTH + sizeof(IMAGE_INFO_TYPE));
    return success;
}

/******************************************************************************/

bool COMMS_TransmitImageLine(COMMS_DATA *comms)
{
    bool success = false;
    uint32_t lineLength;
    memset(comms->TXBuffer.b8,0xFF,DATA_START_LOCATION);
    comms->TXBuffer.b8[ID_LOCATION] = IMAGE_LINE_ID;
    comms->TXBuffer.b8[LINE_LSB_LOCATION] = 0xff & comms->transmitLine;
    comms->TXBuffer.b8[LINE_MSB_LOCATION] = 0xff & (comms->transmitLine>>8);    
    lineLength = comms->image.properties.dimensions.horizontal * sizeof(PIXEL_TYPE);
    memcpy(&comms->TXBuffer.b8[DATA_START_LOCATION],&comms->image.buffer[comms->buffer.transmit].pixel[comms->transmitLine][0],lineLength);
    comms->TXBuffer.b8[LENGTH_LSB_LOCATION] = (0xFF&(lineLength));
    comms->TXBuffer.b8[LENGTH_MSB_LOCATION] = (0xFF&(lineLength)>>8); 
    success = COMMS_SPIWrite(comms,MESSAGE_HEADER_LENGTH + lineLength);
    return success;
}

/******************************************************************************/

bool COMMS_TransmitImageDone(COMMS_DATA *comms)
{
    bool success = false;
    memset(comms->TXBuffer.b8,0xFF,DATA_START_LOCATION);
    comms->TXBuffer.b8[ID_LOCATION] = IMAGE_DONE_ID;
    comms->TXBuffer.b8[LENGTH_MSB_LOCATION] = 0;   
    comms->TXBuffer.b8[LENGTH_LSB_LOCATION] = 0;
    success = COMMS_SPIWrite(comms,MESSAGE_HEADER_LENGTH);
    return success;
}

/******************************************************************************/

bool COMMS_SPIWrite(COMMS_DATA *comms,uint32_t TXSize)
{
    bool success = false;
    if(comms->spi.status.running == false)
    {
        if(TXSize < comms->TXBuffer.size.max.b8)
        {
            //CommsSPISlaveSelect();
            if(DRV_SPI_BUFFER_HANDLE_INVALID != 
               DRV_SPI_BufferAddWrite(comms->spi.drvHandle,
                                      comms->TXBuffer.b8,
                                      TXSize,
                                      (void*)CommsSPICompletedCallback,
                                      NULL))
            {
                comms->TXBuffer.size.transfer.b8  = TXSize;
                comms->TXBuffer.size.transfer.b16 = TXSize>>1;
                comms->TXBuffer.size.transfer.b32 = TXSize>>2;
                comms->spi.status.started=true;
                do {
                    taskYIELD();
                }while(!comms->spi.status.complete);
                success = true;
            }            
        }    
    }   
    return success;
} 

bool COMMS_NotifyReady(COMMS_DATA *comms)
{
    if(comms->RTOS.FLIRHandle)
    {
        return (pdPASS == xTaskNotify(comms->RTOS.FLIRHandle,NULL,eIncrement));
    }
    return false;
}

uint32_t COMMS_WaitForImageReady(void)
{
    return ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
}
/******************************************************************************/
/* End of File*/
/******************************************************************************/
