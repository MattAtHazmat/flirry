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
/******************************************************************************/
/******************************************************************************/
/* Section: Application Callback Functions                                    */
/******************************************************************************/
/******************************************************************************/

/* Application's Timer Callback Function                                      */

static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    if(flirData.status.getImage)
    {
        flirData.counters.timerMissed++;
    }
    flirData.status.getImage=true;
    flirData.status.timerRunning=false;
}

/******************************************************************************/

//static void SPICallback(uintptr_t context)
//{
//    FLIRSPISlaveDeselect();
//    flirData.spi.status.started = false;
//    flirData.spi.status.complete = true;
//}

static void FLIR_SPIStartedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    if((event == DRV_SPI_BUFFER_EVENT_PENDING)||(event == DRV_SPI_BUFFER_EVENT_PROCESSING))
    {
        FLIRSPISlaveSelect();
        flirData.spi.status.running = true;
        flirData.spi.status.started = true;
        flirData.spi.status.complete = false;
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        flirData.spi.status.error = true;
    }        
}

static void FLIR_SPICompletedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    if(event == DRV_SPI_BUFFER_EVENT_COMPLETE)
    {
        FLIRSPISlaveDeselect();
        flirData.spi.status.running = false;
        flirData.spi.status.started = false;
        flirData.spi.status.complete = true;
    }
    else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
    {
        flirData.spi.status.error = true;
    }        
}

/******************************************************************************/
/******************************************************************************/
/* Section: Application Local Functions                                       */
/******************************************************************************/
/******************************************************************************/
#define LED_FRAME_INTERVAL  5

/* Application's LED Task Function                                            */
static void LedTask( void )
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
static bool TimerSetup( void )
{
    DRV_TMR_Alarm16BitRegister(flirData.timer.drvHandle, 
                               FLIR_TMR_DRV_PERIOD, 
                               FLIR_TMR_DRV_IS_PERIODIC,
                               (uintptr_t)NULL, 
                               TimerCallback);
    flirData.status.getImage = false;
    return DRV_TMR_Start(flirData.timer.drvHandle);
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
//    flirData.TXBuffer.size.max.b8 = BUFFER_SIZE_8;
//    flirData.TXBuffer.size.max.b16 = BUFFER_SIZE_16;
//    flirData.TXBuffer.size.max.b32 = BUFFER_SIZE_32;
    flirData.RXBuffer.size.max.b8 = BUFFER_SIZE_8;
    flirData.RXBuffer.size.max.b16 = BUFFER_SIZE_16;
    flirData.RXBuffer.size.max.b32 = BUFFER_SIZE_32;
    flirData.lepton.result = LEP_ERROR;
    flirData.image.properties.dimensions.horizontal = HORIZONTAL_SIZE;
    flirData.image.properties.dimensions.vertical = VERTICAL_SIZE;
    flirData.image.properties.size.pixels = flirData.image.properties.dimensions.horizontal *
                                            flirData.image.properties.dimensions.vertical;
    flirData.image.properties.size.bytes = sizeof(FLIR_IMAGE_TYPE);
    flirData.frame.discardCountLimit = 100;
    
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
            flirData.RTOS.myHandle = xTaskGetCurrentTaskHandle();
            FLIRHandle = flirData.RTOS.myHandle;
            if (flirData.timer.drvHandle == DRV_HANDLE_INVALID)
            {
                flirData.timer.drvHandle = DRV_TMR_Open(FLIR_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                flirData.status.timerConfigured = ( DRV_HANDLE_INVALID != flirData.timer.drvHandle );
            }        
            
            if (flirData.status.timerConfigured)
            {
                //flirData.status.timerRunning = TimerSetup();
                //if(flirData.status.timerRunning)
                //{
                    flirData.state = FLIR_OPEN_SPI_PORT;
                //}
                //else
                //{
                //    flirData.state = FLIR_ERROR;
                //}
            }
            break;
        }
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
//        }
        case FLIR_OPEN_SPI_PORT:
        {
            flirData.status.SPIConfigured = OpenFLIRSPI(&flirData);
            flirData.status.SPIConfigureAttempted = true;
            if(flirData.status.SPIConfigured)
            {
                flirData.status.SPIRunning = true;
            }
//            if(flirData.status.I2CConfigureAttempted == false)
//            {
//                flirData.state= FLIR_OPEN_I2C_PORT;
//            }
//            else
            {
                flirData.state = FLIR_START;
            }
            break;
        }
        case FLIR_START:
        {
            if(flirData.status.SPIRunning|flirData.status.I2CRunning)
            {
                if(flirData.status.SPIRunning)
                {
                    flirData.status.usingSPI = true;
                }
                if(flirData.status.I2CRunning)
                {
                    flirData.status.usingI2C = true;
                }
                flirData.state = FLIR_STATE_SERVICE_TASKS;
            }
            else
            {
                flirData.state = FLIR_ERROR;
                break;
            }
            /* drop through */
        }
        case FLIR_STATE_SERVICE_TASKS:
        {
            if(flirData.status.usingSPI)
            {
                flirData.state = FLIR_STATE_START_FRAME;
                flirData.frame.building = 0;
                flirData.frame.transmitting = -1;          
                if(!flirData.status.timerRunning)
                {
                    flirData.status.timerRunning = TimerSetup();
                }
            }
            break;
        }      
        case FLIR_STATE_START_FRAME:
        {
            if(FLIRGetVoSPI(&flirData))
            {              
                /* we have data. is it valid? */
                if(FLIRDiscardLine(&flirData))
                {
                    /* just discard the line- nothing to do. */
                    break;
                }
                else
                {
                    /* wait for the start of the frame */
                    if((flirData.status.getImage==true)&&(flirData.VoSPI.ID.line == 0))
                    {
                        
                        /* we are at the start of the frame */
                        flirData.status.timerRunning = TimerSetup();
                        flirData.status.getImage=false;
                        flirData.status.buildingFrame = true;
                        flirData.state = FLIR_STATE_GET_LINE;
                        flirData.status.lineReady = true;
                        flirData.status.imageReady = false;
                        flirData.status.restartFrame = false;
                        flirData.frame.buildingLine = 0;
                                                
                        /* drop through to next state. */
                    }
                    else
                    {
                        /* not the start line, nothing to do. */
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        }
        case FLIR_STATE_GET_LINE:
        {
            while((flirData.status.imageReady==false)&&
                  (flirData.status.lineReady==true)&&
                  (flirData.status.restartFrame==false))
            {                         
                if(FLIRDiscardLine(&flirData))
                {
                    if(++flirData.frame.discardCount > flirData.frame.discardCountLimit)
                    {
                        flirData.frame.discardCount = 0;
                        flirData.status.discardLimit = true;
                        flirData.status.restartFrame = true;
                        flirData.state = FLIR_STATE_START_FRAME;
                        break;
                    }
                }                   
                else if(flirData.frame.buildingLine == flirData.VoSPI.ID.line)
                {
                    FLIRPopulateLine(&flirData);
                    flirData.frame.discardCount = 0;                    
                    flirData.frame.buildingLine++;
                    if(flirData.frame.buildingLine == flirData.image.properties.dimensions.vertical)
                    {
                        flirData.status.imageReady = true;
                        flirData.state = FLIR_STATE_IMAGE_COMPLETE;
                    }
                }
                if(flirData.status.imageReady == false)
                {
                    flirData.status.lineReady = FLIRGetVoSPI(&flirData);
                }
            }   
            if(flirData.state != FLIR_STATE_IMAGE_COMPLETE)
            {
                break;
            }
            /* drop through */
        }
        case FLIR_STATE_IMAGE_COMPLETE:
        {
            /* image is complete- so get ready for the next frame and start */
            /* transmitting the current it */
            flirData.frame.transmitting = flirData.frame.building;
            flirData.frame.building++;
            flirData.frame.building &= (IMAGE_BUFFERS-1);
            flirData.state = FLIR_STATE_TRANSMIT_IMAGE;
        }
        case FLIR_STATE_TRANSMIT_IMAGE: 
        {
            //if(flirData.status.sendImage)
            //{
            //    flirData.status.sendImage = false;
                if(FLIRTransmitImage(&flirData))
                {
                    BSP_LEDToggle(BSP_LED_3);
                }
                
            //}
            //else
            //{
            //    flirData.counters.imagesDiscarded++;
            //}
            //if (DRV_TMR_Status(flirData.timer.drvHandle)==DRV_TMR_CLIENT_STATUS_READY)
            //{
            //    TimerSetup();
            //}
            flirData.state = FLIR_STATE_TRANSMIT_IMAGE_WAIT;
            break;            
        }
        case FLIR_STATE_TRANSMIT_IMAGE_WAIT:
        {
            if(commsData.status.copied == true)
            {
                flirData.state = FLIR_STATE_START_FRAME;
            }
//            else if(flirData.status.sendImage)
//            {
//                /* something went wrong. this should not go true at this */
//                /* point*/
//                flirData.state = FLIR_STATE_START_FRAME;
//                flirData.counters.failure.sendImageTimeout++;
//            }
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
    if(flirData.RTOS.commsHandle == NULL)
    {
        flirData.RTOS.commsHandle = commsHandle;
    }
    LedTask();            
}

/******************************************************************************/

inline bool FLIRDiscardLine(FLIR_DATA *flir)
{
    return (flir->VoSPI.ID.discard == DISCARD);
}

/******************************************************************************/

bool FLIRTransmitImage(FLIR_DATA *flir)
{
    bool imageTransmitted = false;
    if(commsData.status.readyForImage)
    {
        if(flir->RTOS.commsHandle)
        {
            if(pdPASS == xTaskNotify(flir->RTOS.commsHandle, NULL, eNoAction))
            {
                flir->counters.imagesTransmitted++;
                flir->status.imageReady = false;
                imageTransmitted = true;
            }
        }
    }    
    return imageTransmitted;
}

/******************************************************************************/

bool FLIRPopulateLine(FLIR_DATA *flir)
{
    if(flir->VoSPI.ID.discard == DISCARD)
    {
        return false;
    }
    if(flir->VoSPI.ID.line < flir->image.properties.dimensions.vertical)
    {
        memcpy(&(flir->image.buffer[flir->frame.building].pixel[flir->VoSPI.ID.line][0]),
               flir->VoSPI.payload.b8,
               sizeof(flir->VoSPI.payload.b8));
        return true;
    }
    return false;
}

/******************************************************************************/

//bool OpenFLIRSPI(FLIR_DATA *flir)
//{
//    bool SPIReady = false;
//    FLIRSPISlaveDeselect();
//    flir->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
//                                       DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READ);
//    if(DRV_HANDLE_INVALID != flir->spi.drvHandle)
//    {
//        SPIReady = true;
//    }
//    return SPIReady;
//}

bool OpenFLIRSPI(FLIR_DATA *flir)
{
   flir->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
                                      DRV_IO_INTENT_BLOCKING|DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READ);
    if(DRV_HANDLE_INVALID != flir->spi.drvHandle)
    {
        DRV_SPI_CLIENT_DATA clientData;
        clientData.baudRate = 0; /* not overriding the baud rate */
        clientData.operationEnded = (void*)FLIR_SPICompletedCallback;
        clientData.operationStarting = (void*)FLIR_SPIStartedCallback;
        if(DRV_SPI_ClientConfigure(flir->spi.drvHandle,&clientData)>=0)
        {
            /* success! */
        }
    }
    return(DRV_HANDLE_INVALID != flir->spi.drvHandle);
}


/******************************************************************************/

//bool StartFLIRSPIReading(FLIR_DATA *flir,uint32_t size)
//{
//    bool SPIReadingStarted=false;
//    FLIRSPISlaveDeselect();
//    if(size < flir->RXBuffer.size.max.b8)
//    {
//        if(DRV_SPI_BUFFER_HANDLE_INVALID != 
//           DRV_SPI_BufferAddRead2(flir->spi.drvHandle,
//                                  flir->RXBuffer.b8,
//                                  size,
//                                  (void*)FLIR_SPICompletedCallback,
//                                  NULL,
//                                 &flir->spi.bufferHandle))            
//        {
//            flir->RXBuffer.size.transfer.b8 = size;
//            flir->RXBuffer.size.transfer.b16 = size>>1;
//            flir->RXBuffer.size.transfer.b32 = size>>2;
//            flir->spi.status.started=true;
//            SPIReadingStarted=true;
//        }        
//    }
//    return SPIReadingStarted;    
//}

/******************************************************************************/

bool GetFLIRSPIReading(FLIR_DATA *flir)
{
    bool SPIReadingReady=false;
    if(flir->spi.status.complete)
    {
        flir->spi.status.complete = false;
        flir->spi.status.started = false;
        SPIReadingReady = true;
    }
    return SPIReadingReady;    
}

/******************************************************************************/

//bool FLIRSPIWriteRead(FLIR_DATA *flir,uint32_t TXSize,int32_t RXSize)
//{
//    bool success = false;
//    FLIRSPISlaveDeselect();
//    /* blocking */
//    if((TXSize < flir->TXBuffer.size.max.b8)&&
//       (RXSize < flir->RXBuffer.size.max.b8))
//    {
//        FLIRSPISlaveSelect();
//        if(DRV_SPI_BUFFER_HANDLE_INVALID != 
//           DRV_SPI_BufferAddWriteRead(flir->spi.drvHandle,
//                                      flir->TXBuffer.b8,TXSize,
//                                      flir->RXBuffer.b8,RXSize,
//                                      (void*)SPICallback,NULL))
//        {
//            flir->TXBuffer.size.transfer.b8 = TXSize;
//            flir->TXBuffer.size.transfer.b16 = TXSize>>1;
//            flir->TXBuffer.size.transfer.b32 = TXSize>>2;
//            flir->RXBuffer.size.transfer.b8 = RXSize;
//            flir->RXBuffer.size.transfer.b16 = RXSize>>1;
//            flir->RXBuffer.size.transfer.b32 = RXSize>>2;
//            flir->spi.status.started=true;
//            do {
//                taskYIELD();
//            }while(!flir->spi.status.complete);
//            success = true;
//        }
//        else
//        {
//            FLIRSPISlaveDeselect();
//        }
//    }
//    return success;
//}

/******************************************************************************/

bool FLIRSPIRead(FLIR_DATA *flir,int32_t RXSize)
{
    bool success = false;
    /* blocking */
    if(RXSize < flir->RXBuffer.size.max.b8)
    {
        flir->spi.status.complete = false;
        flir->spi.status.started = false;
        flir->spi.bufferHandle = DRV_SPI_BufferAddRead(flir->spi.drvHandle,
                                                       flir->RXBuffer.b8,
                                                       RXSize,
                                                       (void*)FLIR_SPICompletedCallback,
                                                       NULL);
        if(DRV_SPI_BUFFER_HANDLE_INVALID != flir->spi.bufferHandle )
        {            
            flir->RXBuffer.size.transfer.b8 = RXSize;
            flir->RXBuffer.size.transfer.b16 = RXSize>>1;
            flir->RXBuffer.size.transfer.b32 = RXSize>>2;
            flir->spi.status.started=true;
            do{ 
                taskYIELD();
            }while(!flir->spi.status.complete);
            success = true;
        }        
    }
    return success;
}
/******************************************************************************/
bool FLIRGetVoSPI(FLIR_DATA *flir)
{
    bool success = false;
    if(FLIRSPIRead(flir,sizeof(VOSPI_TYPE)))
    {
        if((NULL != flir->spi.bufferHandle)&&
           (DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(flir->spi.bufferHandle)))
        {
            memcpy((void*)flir->VoSPI.b8,(void*)flir->RXBuffer.b8,sizeof(VOSPI_TYPE));
            /* fix the endianness */
            {
                uint8_t temp;
                uint32_t byteIndex;
                for (byteIndex = 0;byteIndex<sizeof(VOSPI_TYPE);byteIndex=byteIndex+2)
                {
                    temp = flir->VoSPI.b8[byteIndex];
                    flir->VoSPI.b8[byteIndex] = flir->VoSPI.b8[byteIndex+1];
                    flir->VoSPI.b8[byteIndex+1] = temp;
                }
            }
            success = true;
        }
    }
    return success;
}
/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/