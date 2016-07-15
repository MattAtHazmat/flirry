// <editor-fold defaultstate="collapsed" desc="...">
/******************************************************************************/
/*MPLAB Harmony Application Source File                                       */
/*                                                                            */
/*Company:                                                                    */
/*  Microchip Technology Inc.                                                 */
/*                                                                            */
/*File Name:                                                                  */
/*  flir.c                                                                    */
/*                                                                            */
/*Summary:                                                                    */
/*  This file contains the source code for the MPLAB Harmony application.     */
/*                                                                            */
/*Description:                                                                */
/*This file contains the source code for the MPLAB Harmony application.  It   */
/*implements the logic of the application's state machine and it may call     */
/*API routines of other MPLAB Harmony modules in the system, such as drivers, */
/*system services, and middleware.  However, it does not call any of the      */
/*system interfaces (such as the "Initialize" and "Tasks" functions) of any of*/
/*the modules in the system or make any assumptions about when those functions*/
/*are called.  That is the responsibility of the configuration-specific system*/
/*files.                                                                      */
/******************************************************************************/

/*******************************************************************************
/*Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights      */
/* reserved.                                                                  */
/*                                                                            */
/*Microchip licenses to you the right to use, modify, copy and distribute     */
/*Software only when embedded on a Microchip microcontroller or digital signal*/
/*controller that is integrated into your product or third party product      */
/*(pursuant to the sublicense terms in the accompanying license agreement).   */
/*                                                                            */
/*You should refer to the license agreement accompanying this Software for    */
/*additional information regarding your rights and obligations.               */
/*                                                                            */
/*SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY     */
/*KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY */
/*OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR    */
/*PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED*/
/*UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF       */
/*WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR */
/*EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT,    */
/*PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF       */
/*PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY     */
/*THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER 
/* SIMILAR COSTS.                                                             */
/*******************************************************************************/
// </editor-fold>

/******************************************************************************/
/******************************************************************************/
/* Section: Included Files                                                    */
/******************************************************************************/
/******************************************************************************/

#include "flir.h"
#include "bsp_config.h"

/******************************************************************************/
/******************************************************************************/
/* Section: Global Data Definitions                                           */
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/* Application Data                                                           */
/*                                                                            */
/*  Summary:                                                                  */
/*    Holds application data                                                  */
/*                                                                            */
/*  Description:                                                              */
/*    This structure holds the application's data.                            */
/*                                                                            */
/*  Remarks:                                                                  */
/*    This structure should be initialized by the APP_Initialize function.    */
/*                                                                            */
/*    Application strings and buffers are be defined outside this structure.  */
/******************************************************************************/

FLIR_DATA flirData;
extern TaskHandle_t commsHandle;
TaskHandle_t FLIRHandle;
extern COMMS_DATA commsData;

inline bool FLIR_ImageTimerTriggered(FLIR_DATA *flir)
{
    bool complete;
    __builtin_disable_interrupts();
    complete = flir->status.getImage;
    __builtin_enable_interrupts();
    return complete;
}
inline void FLIR_ImageTimerReset(FLIR_DATA *flir)
{
    __builtin_disable_interrupts();
    flir->status.getImage = false;
    __builtin_enable_interrupts();
}

/******************************************************************************/
/******************************************************************************/
/* Section: Application Callback Functions                                    */
/******************************************************************************/
/******************************************************************************/

/* Application's Timer Callback Function                                      */

static void FLIR_TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    if(flirData.status.getImage)
    {
        flirData.status.getImageMissed = true;
        flirData.counters.failure.getImageMissed++;
    }
    flirData.status.getImage = true;
}

/******************************************************************************/

static void FLIR_SPIStartedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    
    if(/*(event == DRV_SPI_BUFFER_EVENT_PENDING)||*/(event == DRV_SPI_BUFFER_EVENT_PROCESSING))
    {        
        FLIR_SPISlaveSelect();
        flirData.spi.status.running = true;
    }
    else /*if (event == DRV_SPI_BUFFER_EVENT_ERROR)*/
    {
        flirData.spi.status.error = true;
        //FLIR_SPISlaveDeselect();
    }        
}

static void FLIR_SPICompletedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    FLIR_SPISlaveDeselect();
    flirData.spi.status.running = false;
    flirData.spi.status.started = false;
    if(event == DRV_SPI_BUFFER_EVENT_COMPLETE)
    {   
        flirData.spi.status.complete = true;
    }
    else /*if (event == DRV_SPI_BUFFER_EVENT_ERROR)*/
    {
        flirData.spi.status.error = true;
    }        
}

/******************************************************************************/
/******************************************************************************/
/* Section: Application Local Functions                                       */
/******************************************************************************/
/******************************************************************************/
#define LED_FRAME_INTERVAL  50

/* Application's LED Task Function                                            */
static void FLIR_LedTask( void )
{
    static int frame = 0;
    if((frame++)==LED_FRAME_INTERVAL)
    {
        BSP_LEDToggle(BSP_LED_2);
        frame = 0;
    }        
}

/******************************************************************************/
/* Application's Timer Setup Function                                         */
static bool FLIR_TimerSetup( void )
{
    if(flirData.status.timerRunning==false)
    {
        DRV_TMR_Alarm16BitRegister(flirData.timer.drvHandle, 
                               FLIR_TMR_DRV_PERIOD, 
                               FLIR_TMR_DRV_IS_PERIODIC,
                               (uintptr_t)NULL, 
                               FLIR_TimerCallback);
        return DRV_TMR_Start(flirData.timer.drvHandle);
    }
    return false;
}

/******************************************************************************/
/******************************************************************************/
/* Section: Application Initialization and State Machine Functions            */
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*  Function:                                                                 */
/*    void FLIR_Initialize ( void )                                           */
/*                                                                            */
/*  Remarks:                                                                  */
/*    See prototype in flir.h.                                                */
/******************************************************************************/

void FLIR_Initialize ( void )
{
    memset(&flirData,0,sizeof(flirData));
    /* Place the App state machine in its initial state.                      */
    flirData.state = FLIR_STATE_INIT;
    flirData.timer.drvHandle = DRV_HANDLE_INVALID; 
    flirData.RXBuffer.size.max.b8 = BUFFER_SIZE_8;
    flirData.RXBuffer.size.max.b16 = BUFFER_SIZE_16;
    flirData.RXBuffer.size.max.b32 = BUFFER_SIZE_32;
    flirData.lepton.result = LEP_ERROR;
    flirData.image.properties.dimensions.horizontal = HORIZONTAL_SIZE;
    flirData.image.properties.dimensions.vertical = VERTICAL_SIZE;
    flirData.image.properties.size.pixels = flirData.image.properties.dimensions.horizontal *
                                            flirData.image.properties.dimensions.vertical;
    flirData.image.properties.size.bytes = sizeof(FLIR_IMAGE_TYPE);
    //flirData.frame.discardCountLimit = 100;    
}


/******************************************************************************/
/*  Function:                                                                 */
/*    void FLIR_Tasks ( void )                                                */
/*                                                                            */
/*  Remarks:                                                                  */
/*    See prototype in flir.h.                                                */
/******************************************************************************/

void FLIR_Tasks ( void )
{
    switch ( flirData.state )
    {
        case FLIR_STATE_INIT:
        {
            if(!flirData.status.RTOSConfigured)
            {
                flirData.RTOS.myHandle = xTaskGetCurrentTaskHandle();
                FLIRHandle = flirData.RTOS.myHandle;
                flirData.status.RTOSConfigured = true;
            }
            if(!flirData.status.timerConfigured)
            {
                if (flirData.timer.drvHandle == DRV_HANDLE_INVALID)
                {
                    flirData.timer.drvHandle = DRV_TMR_Open(FLIR_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                }        
                flirData.status.timerConfigured = ( DRV_HANDLE_INVALID != flirData.timer.drvHandle );
            }
            //if(!flirData.status.timerRunning)
            //{
            //    flirData.status.timerRunning = FLIR_TimerSetup();                
            //}
            if(!flirData.status.SPIConfigured)
            {
                flirData.state = FLIR_OPEN_SPI_PORT;
                break;
            }
            flirData.state = FLIR_START;
            break;
        }
            // <editor-fold defaultstate="collapsed" desc="I2C Configuration">
            //        case FLIR_OPEN_I2C_PORT:
            //        {
            //            flirData.status.I2CConfigureAttempted = true;
            //            if(flirData.lepton.result == LEP_OK)
            //            {
            //                //flirData.status.I2CConfigured = true;
            //                //flirData.status.I2CRunning = true;
            //            }
            //            if(flirData.status.SPIConfigureAttempted == false)
            //            {
            //                flirData.state= FLIR_OPEN_SPI_PORT;
            //            }
            //            else
            //            {
            //                flirData.state = FLIR_START;
            //            }
            //            break;
            //        }// </editor-fold>
        case FLIR_OPEN_SPI_PORT:
        {
            flirData.status.SPIConfigured = FLIR_OpenSPI(&flirData);
            if(flirData.status.SPIConfigured)
            {
                flirData.status.SPIRunning = true;
                flirData.state = FLIR_STATE_INIT;                
            }            
            break;
        }
        case FLIR_START:
        {
            if(flirData.status.SPIRunning)
            {
                //flirData.frame.building = 0;
                //flirData.frame.transmitting = -1;
                /* want to set a delay for the first image transmission*/
                if(FLIR_TimerSetup())
                {
                    flirData.status.timerRunning = true;
                    flirData.status.getImage = false;
                    flirData.state = FLIR_STATE_SERVICE_TASKS;
                }
            }
            else
            {
                flirData.state = FLIR_ERROR;
                break;
            }
            if(flirData.state != FLIR_STATE_SERVICE_TASKS)
            {
                break;
            }
            /* drop through */
        }
        case FLIR_STATE_SERVICE_TASKS:
        {          
            if(FLIR_ImageTimerTriggered(&flirData))
            {          
                flirData.state = FLIR_STATE_START_IMAGE;   
            }
            break;            
        }      
        case FLIR_STATE_START_IMAGE:
        {
            flirData.status.imageStarted = false;
            flirData.counters.imagesStarted++;
            flirData.counters.discarded = 0;
            flirData.state = FLIR_STATE_START_GET_LINE;
        }
        case FLIR_STATE_START_GET_LINE:
        {
            if(FLIR_StartGetVoSPI(&flirData))
            {
                flirData.state = FLIR_STATE_WAIT_FOR_LINE;
            }
            else
            {
                flirData.counters.failure.getLine++;
            }
            break;
        }        
        case FLIR_STATE_WAIT_FOR_LINE:
        {
            if(FLIR_GetVoSPI(&flirData))
            {
                if(!FLIR_DiscardLine(&flirData))
                {
                    FLIR_PopulateLine(&flirData);
                    /* did we get the last line of an image? */
                    if((flirData.status.imageStarted)&&
                       (flirData.VoSPI.ID.line == (flirData.image.properties.dimensions.vertical-1)))
                    {
                        /* got the last line of a full image */                        
                        flirData.state = FLIR_STATE_WAIT_TO_TRANSMIT_IMAGE;
                        break;
                    }
                    if (FLIR_ImageTimerTriggered(&flirData))
                    {
                        if(flirData.VoSPI.ID.line == 0)
                        {                            
                            flirData.status.imageStarted = true;
                        }  
                        if((flirData.status.imageStarted)&&
                           (flirData.VoSPI.ID.line > 0))
                        {
                            FLIR_ImageTimerReset(&flirData);
                        }
                    }
                }
                else
                {
                    flirData.counters.discarded++;                      
                }
                /* get the next line */
                flirData.state = FLIR_STATE_START_GET_LINE;
            }
            else
            {
                if(flirData.status.getImageMissed)
                {
                    Nop();
                }
            }
            break;
        }    
        case FLIR_STATE_WAIT_TO_TRANSMIT_IMAGE:
        {
            if(commsData.status.readyForImage)
            {
                commsData.status.readyForImage = false;
                flirData.state = FLIR_STATE_TRANSMIT_IMAGE; 
            }
            else
            {
                break;
            }
        }
        case FLIR_STATE_TRANSMIT_IMAGE: 
        {
            FLIR_TransmitImage(&flirData);
            flirData.state = FLIR_STATE_COPY_IMAGE_WAIT;
            break;            
        }
        case FLIR_STATE_COPY_IMAGE_WAIT:
        {
            if(commsData.status.imageCopied == true)
            {
                commsData.status.imageCopied = false;
                flirData.state = FLIR_STATE_START_IMAGE;//FLIR_STATE_SERVICE_TASKS;
            }
            break;
        }
        case FLIR_ERROR:
        default:
        {
            BSP_LEDOn(BSP_LED_3);
            BSP_LEDOn(BSP_LED_4);
            break;
        }
    }
    /* not sure if this is the right way to get this information- but it seems*/
    /* like a chicken/egg situation. Get it this way for now.                 */
//    if(flirData.RTOS.commsHandle == NULL)
//    {
//        flirData.RTOS.commsHandle = commsHandle;
//    }
    FLIR_LedTask();            
}

/******************************************************************************/

inline bool FLIR_DiscardLine(FLIR_DATA *flir)
{
    return (flir->VoSPI.ID.discard == DISCARD);
}

/******************************************************************************/

bool FLIR_TransmitImage(FLIR_DATA *flir)
{
    flir->status.imageReady = true;
    flir->counters.imagesTransmitted++;
    BSP_LEDToggle(BSP_LED_3);
    return true;
}

/******************************************************************************/

bool FLIR_PopulateLine(FLIR_DATA *flir)
{
    if(flir->VoSPI.ID.discard == DISCARD)
    {
        return false;
    }
    if(flir->VoSPI.ID.line < flir->image.properties.dimensions.vertical)
    {
        memcpy(&(flir->image.buffer.pixel[flir->VoSPI.ID.line][0]),
               flir->VoSPI.payload.b8,
               sizeof(flir->VoSPI.payload.b8));
        return true;
    }
    return false;
}

/******************************************************************************/

bool FLIR_OpenSPI(FLIR_DATA *flir)
{
    bool openSuccess=false;
   flir->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
                                      /*DRV_IO_INTENT_BLOCKING|*/DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READ);
    if(DRV_HANDLE_INVALID != flir->spi.drvHandle)
    {
        DRV_SPI_CLIENT_DATA clientData;
        clientData.baudRate = 0; /* not overriding the baud rate */
        clientData.operationEnded = (void*)FLIR_SPICompletedCallback;
        clientData.operationStarting = (void*)FLIR_SPIStartedCallback;
        if(DRV_SPI_ClientConfigure(flir->spi.drvHandle,&clientData)>=0)
        {
            /* success! */
            openSuccess = true;
            flir->spi.status.running=false;
            flir->spi.status.complete=false;
            flir->spi.status.started=false;
        }
    }
    return (openSuccess)&&(DRV_HANDLE_INVALID != flir->spi.drvHandle);
}

inline bool FLIR_SPIComplete(FLIR_DATA *flir)
{
    bool complete;
    __builtin_disable_interrupts();
    complete = flir->spi.status.complete;
    __builtin_enable_interrupts();
    return complete;
}
/******************************************************************************/

bool FLIR_StartSPIRead(FLIR_DATA *flir,int32_t RXSize)
{

    bool success = false;
    if((!flir->spi.status.started)&&
       (!flir->spi.status.running)&&
       (RXSize < flir->RXBuffer.size.max.b8))
    {
        
        flir->spi.status.complete = false;
        flir->spi.RXBytesExpected = RXSize;
        if(DRV_SPI_BUFFER_HANDLE_INVALID != DRV_SPI_BufferAddRead2(flir->spi.drvHandle,
                                                       flir->RXBuffer.b8,
                                                       RXSize,
                                                       (void*)FLIR_SPICompletedCallback,
                                                       NULL,
                                                       &flir->spi.bufferHandle) )
        {           
            flir->spi.status.started = true;
            /* running gets set in the callback */
            success = true;
        }     
    }
    else
    {
        flir->counters.failure.readStart++;
    }
    return success;
}

bool FLIR_CheckSPIReadDone(FLIR_DATA *flir)
{
    if(FLIR_SPIComplete(flir))
    {
        flir->RXBuffer.size.transfer.b8 = flir->spi.RXBytesExpected;
        flir->RXBuffer.size.transfer.b16 = flir->spi.RXBytesExpected>>1;
        flir->RXBuffer.size.transfer.b32 = flir->spi.RXBytesExpected>>2;
        return true;
    }
    return false;
}

/******************************************************************************/

bool FLIR_GetVoSPI(FLIR_DATA *flir)
{
    if(FLIR_CheckSPIReadDone(flir))
    {
        uint8_t temp;
        uint32_t byteIndex;
        memcpy((void*)flir->VoSPI.b8,(void*)flir->RXBuffer.b8,sizeof(VOSPI_TYPE));                
        /* fix endianness */
        for (byteIndex = 0;byteIndex<sizeof(VOSPI_TYPE);byteIndex=byteIndex+2)
        {
            temp = flir->VoSPI.b8[byteIndex];
            flir->VoSPI.b8[byteIndex] = flir->VoSPI.b8[byteIndex+1];
            flir->VoSPI.b8[byteIndex+1] = temp;
        }
        return true;
    }    
    return false;
}


bool FLIR_StartGetVoSPI(FLIR_DATA *flir)
{
    return FLIR_StartSPIRead(flir,sizeof(VOSPI_TYPE));
}
/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/