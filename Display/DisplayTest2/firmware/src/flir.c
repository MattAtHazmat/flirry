/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    flir.c

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

#include "flir.h"
#include "bsp_config.h"
#include "commonHeader.h"
#include "disp.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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
extern DISP_DATA dispData;

/******************************************************************************/
/* Section: Application Callback Functions                                    */
/******************************************************************************/

/* Timer Callback                                                             */

static void FLIR_TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    flirData.counters.getImage++;
    if(flirData.status.flags.getImage)
    {
        flirData.status.flags.getImageMissed = true;
        flirData.counters.failure.getImageMissed++;
    }
    else
    {
        flirData.status.flags.getImage = true;
    }
}

static void FLIR_ResyncCallback ( uintptr_t context, uint32_t alarmCount )
{
    flirData.status.flags.timerRunning=false;
    flirData.status.flags.resyncComplete=true;
}
/******************************************************************************/
/* SPI Callbacks                                                              */
static void FLIR_SPIStartedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    
    if(event == DRV_SPI_BUFFER_EVENT_PROCESSING)
    {        
        FLIR_SPISlaveSelect();
        flirData.spi.status.running = true;
    }
    else /*if (event == DRV_SPI_BUFFER_EVENT_ERROR)*/
    {
        flirData.spi.status.error = true;
        flirData.counters.failure.SPIStart++;
    }        
}

/******************************************************************************/

static void FLIR_SPICompletedCallback(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE handle)
{
    FLIR_SPISlaveDeselect();
    flirData.spi.status.running = false;
    flirData.spi.status.started = false;
    if(event == DRV_SPI_BUFFER_EVENT_COMPLETE)
    {   
        flirData.spi.status.complete = true;
    }
    else 
    {
        flirData.spi.status.error = true;
        flirData.counters.failure.SPIEnd++;
    }        
}

/******************************************************************************/
/* Section: Application Local Functions                                       */
/******************************************************************************/

static inline bool FLIR_ImageTimerTriggered(void)
{
    __builtin_disable_interrupts();
    if(flirData.status.flags.getImage)
    {
        flirData.status.flags.getImage = false;
        __builtin_enable_interrupts();
        return true;
    }
    __builtin_enable_interrupts();
    return false;
}

/******************************************************************************/

static inline bool FLIR_ResyncComplete(void)
{
    __builtin_disable_interrupts();
    if(flirData.status.flags.resyncComplete)
    {
        __builtin_enable_interrupts();
        return true;
    }
    __builtin_enable_interrupts();
    return false;
}

/******************************************************************************/

static inline bool FLIR_ImageTimerMissed(void)
{
    __builtin_disable_interrupts();
    if(flirData.status.flags.getImageMissed)
    {
        flirData.status.flags.getImageMissed = false;
        flirData.status.flags.getImage = false;
        __builtin_enable_interrupts();
        return true;
    }
    __builtin_enable_interrupts();
    return false;
}

/******************************************************************************/
/* Application's Timer Setup Function                                         */
//static bool COMMS_TimerSetup( COMMS_DATA* comms, uint32_t periodUS)

static bool FLIR_TimerSetup( FLIR_DATA* flir, uint32_t periodMS )
{
    uint32_t period;
    if(flir->status.flags.timerRunning)
    {
        DRV_TMR_Stop(flir->timer.drvHandle);
        flir->status.flags.timerRunning = false;
    }       
    TMR4=0;
    TMR5=0;
    period = (periodMS * DRV_TMR_CounterFrequencyGet(flir->timer.drvHandle)/1000);
    DRV_TMR_Alarm32BitRegister(flir->timer.drvHandle, 
                              period, 
                              true,
                              (uintptr_t)NULL, 
                              FLIR_TimerCallback);
    flir->status.flags.timerRunning = DRV_TMR_Start(flir->timer.drvHandle);    
    if(flir->status.flags.timerRunning)
    {
        flir->status.flags.getImage = false;
        return true;
    }   
    return false;
}

/******************************************************************************/

static bool FLIR_StartResync( FLIR_DATA* flir, uint32_t periodMS )
{
    uint32_t period;
    FLIR_SPISlaveDeselect();
    if(flir->status.flags.timerRunning)
    {
        DRV_TMR_Stop(flir->timer.drvHandle);
        flir->status.flags.timerRunning = false;
    }    
    TMR4=0;
    TMR5=0;
    period = (periodMS * DRV_TMR_CounterFrequencyGet(flir->timer.drvHandle)/1000);
    DRV_TMR_Alarm32BitRegister(flir->timer.drvHandle, 
                              period, 
                              false,
                              (uintptr_t)NULL, 
                              FLIR_ResyncCallback);
    flir->status.flags.timerRunning = DRV_TMR_Start(flir->timer.drvHandle);    
    if(flir->status.flags.timerRunning)
    {
        flir->status.flags.resyncComplete = false;
        return true;
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

void FLIR_Initialize ( SYS_MODULE_INDEX timerIndex, SYS_MODULE_INDEX I2CIndex, SYS_MODULE_INDEX SPIIndex )
{
    FLIR_SPISlaveDeselect();
    memset(&flirData,0,sizeof(flirData));
    /* Place the App state machine in its initial state.                      */
    flirData.state = FLIR_STATE_INIT;
    flirData.timer.index = timerIndex;
    flirData.i2c.index = I2CIndex;
    flirData.i2c.pFlirPort = &flirData.i2c.FlirPort;
    flirData.spi.index = SPIIndex;
    flirData.timer.drvHandle = DRV_HANDLE_INVALID; 
    flirData.RXBuffer.size.max.b8 = BUFFER_SIZE_8;
    flirData.RXBuffer.size.max.b16 = BUFFER_SIZE_16;
    flirData.RXBuffer.size.max.b32 = BUFFER_SIZE_32;
    flirData.image.properties.dimensions.horizontal = HORIZONTAL_SIZE;
    flirData.image.properties.dimensions.vertical = VERTICAL_SIZE;
    flirData.image.properties.size.pixels = flirData.image.properties.dimensions.horizontal *
                                            flirData.image.properties.dimensions.vertical;
    flirData.image.properties.size.bytes = sizeof(FLIR_IMAGE_TYPE);
    FLIR_MakeIntensityMap(&flirData);
    flirData.colorMap.maximum = 0;
    flirData.colorMap.minimum = 0x7fffffff;
    flirData.colorMap.size = FLIR_LUT_SIZE;
}

/******************************************************************************/
/*  Function:                                                                 */
/*    void FLIR_Tasks ( void )                                                */
/*                                                                            */
/*  Remarks:                                                                  */
/******************************************************************************/

void FLIR_Tasks ( void )
{
    switch ( flirData.state )
    {
        // <editor-fold defaultstate="collapsed" desc="Initialization">
        case FLIR_STATE_INIT:
        {
            if (!flirData.status.flags.timerConfigured)
            {
                flirData.state = FLIR_OPEN_TIMER;
                break;
            }
            //if (!flirData.status.flags.timerRunning)
            //{
            //    flirData.state = FLIR_START_TIMER;
            //    break;
            //}
            if ((!flirData.status.flags.I2CConfigured)&&
                (!flirData.status.flags.I2CConfigureAttempted))
            {
                flirData.state = FLIR_OPEN_I2C;
                break;
            }
            if (!flirData.status.flags.SPIConfigured)
            {
                flirData.state = FLIR_OPEN_SPI_PORT;
                break;
            }
            flirData.state = FLIR_START;
            break;
        }
        case FLIR_OPEN_TIMER:
        {
            flirData.status.flags.timerConfigured = FLIR_OpenTimer(&flirData);
            flirData.state = FLIR_STATE_INIT;
            break;
        }
        case FLIR_START_TIMER:
        {
            flirData.status.flags.timerRunning = FLIR_TimerSetup(&flirData, FLIR_TIMER_PERIOD_MS);
            flirData.state = FLIR_STATE_INIT;
            break;
        }
        case FLIR_OPEN_I2C:
        {
            flirData.status.flags.I2CConfigureAttempted = true;
            flirData.status.flags.I2CConfigured = FLIR_OpenI2C(&flirData);
            flirData.state = FLIR_STATE_INIT;
            break;
        }
        case FLIR_OPEN_SPI_PORT:
        {
            flirData.status.flags.SPIConfigured = FLIR_OpenSPI(&flirData);
            flirData.state = FLIR_STATE_INIT;
            break;
        }// </editor-fold>
        case FLIR_START:
        {
            flirData.state = FLIR_STATE_START_RESYNC;//FLIR_STATE_START_NEW_IMAGE;
            break;
        }        
        case FLIR_STATE_START_NEW_IMAGE:
        {
            flirData.status.flags.receivedFirstLine = false;
            flirData.counters.imagesStarted++;
            flirData.status.lastLine = -1;
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
            if(FLIR_CheckSPIReadDone(&flirData))
            {
                flirData.state = FLIR_STATE_GET_LINE;
            }
            else
            {
                break;
            }                
        }
        case FLIR_STATE_GET_LINE:
        {
            flirData.state = FLIR_STATE_START_GET_LINE;
            /* we have a line of data from the camera. Check to see if    */
            /* it is valid data (not something to be discarded) */
            if(FLIR_GetVoSPI(&flirData))
            {
                /* it's a valid line, part of the image to build */
                //FLIR_PopulateLine(&flirData);
                /* did we get the first line of an image? */
                if(flirData.VoSPI.ID.line == 0)
                {          
                    flirData.status.flags.receivedFirstLine = true;
                }   /* did we get the last line of an image? */  
                else if((flirData.status.flags.receivedFirstLine)&&
                        (flirData.VoSPI.ID.line == (flirData.image.properties.dimensions.vertical-1)))
                {
                    /* got the last line of a full image */   
                    flirData.state = FLIR_STATE_COPY_IMAGE;
                }                            
            }      
            else if(flirData.status.flags.mysteryLine)
            {
                /* give up on this round, wait for next image */                
                flirData.status.flags.mysteryLine = false;
                flirData.counters.failure.mysteryLine++;
                flirData.state = FLIR_STATE_START_RESYNC;
            }            
            if(flirData.state != FLIR_STATE_COPY_IMAGE)
            {                
                break;
            }
            /* else drop through to copying the image. */
        }   
        case FLIR_STATE_COPY_IMAGE: 
        {               
            FLIR_CopyImage(&flirData);
            if((flirData.counters.recalibrateCount++)==50)
            {
                flirData.counters.recalibrateCount = 0;
                FLIR_ReMakeIntensityMap(&flirData);
            }
            flirData.status.flags.imageCopied = true;
            flirData.state = FLIR_STATE_PULL_DUMMY_LINE;            
            break;            
        }        
        case FLIR_STATE_PULL_DUMMY_LINE:
        {
            if(FLIR_StartGetVoSPI(&flirData))
            {
                flirData.state = FLIR_STATE_WAIT_FOR_DUMMY_LINE;
            }            
            break;
        }
        case FLIR_STATE_WAIT_FOR_DUMMY_LINE:
        {
            if(FLIR_CheckSPIReadDone(&flirData))
            {
                if(FLIR_ImageTimerTriggered())
                {
                    flirData.state = FLIR_STATE_START_NEW_IMAGE;
                }
                else
                {
                    flirData.state = FLIR_STATE_PULL_DUMMY_LINE;
                }
            }            
            break;
        }
        case FLIR_STATE_START_RESYNC:
        {
            flirData.counters.resync++;
            if(FLIR_StartResync(&flirData,FLIR_RESYNC_TIME))
            {
                flirData.state = FLIR_STATE_RESYNC_WAIT;
            }
            break;
        }
        case FLIR_STATE_RESYNC_WAIT:
        {
            if(FLIR_ResyncComplete())
            {
                if(FLIR_TimerSetup(&flirData, FLIR_TIMER_PERIOD_MS))
                {
                    flirData.state = FLIR_STATE_START_NEW_IMAGE;
                }
            }
            break;
        }
        case FLIR_ERROR:
        default:
        {
            BSP_LEDOn(BSP_LED_1);
            BSP_LEDOn(BSP_LED_2);
            BSP_LEDOn(BSP_LED_3);
            break;
        }
    }
}

/******************************************************************************/

bool FLIR_OpenI2C(FLIR_DATA *flir)
{
    LEP_RESULT result;
    result = LEP_OpenPort(flir->i2c.index,LEP_CCI_TWI,FLIR_I2C_SPEED/1000,flir->i2c.pFlirPort);
    
    return (result==LEP_OK);
}

/******************************************************************************/

bool FLIR_CopyImage(FLIR_DATA *flir)
{
    int32_t x,y;
    for (x=0;x<flir->image.properties.dimensions.horizontal;x++)
    {
        for (y=0;y<flir->image.properties.dimensions.vertical;y++)
        {
            uint32_t index = 0;
            if(flir->image.buffer.pixel[y][x]<flir->colorMap.building.minimum)
            {
                flir->colorMap.building.minimum = flir->image.buffer.pixel[y][x];
            }
            else if (flir->image.buffer.pixel[y][x]>flir->colorMap.building.maximum)
            {
                flir->colorMap.building.maximum = flir->image.buffer.pixel[y][x];
            }
            if(flir->status.flags.remadeMap)
            {
                index = flir->image.buffer.pixel[y][x] - flir->colorMap.minimum;
                if(index>=flir->colorMap.size)
                {
                    index = flir->colorMap.size-1;
                }
                dispData.display[dispData.displayInfo.buffer.filling][y+dispData.displayInfo.offset.vertical][x+dispData.displayInfo.offset.horizontal].w = 
                    flir->colorMap.LUT[index].w;
            }
            else
            {   
                /* not the remade map */
                dispData.display[dispData.displayInfo.buffer.filling][y+dispData.displayInfo.offset.vertical][x+dispData.displayInfo.offset.horizontal].w = 
                    flir->colorMap.LUT[(flir->image.buffer.pixel[y][x])&(FLIR_LUT_SIZE-1)].w;
            }
        }
    }
    BSP_LEDToggle(BSP_LED_3);
    return true;
}

/******************************************************************************/

//void FLIR_PopulateLine(FLIR_DATA *flir)
//{
//    memcpy(&(flir->image.buffer.pixel[flir->VoSPI.ID.line][0]),
//                  flir->VoSPI.payload.b8,
//                  sizeof(flir->VoSPI.payload.b8));
//    flir->status.lastLine = flir->VoSPI.ID.line;
//}

/******************************************************************************/

bool FLIR_OpenTimer(FLIR_DATA *flir)
{            
    if (flir->timer.drvHandle == DRV_HANDLE_INVALID)
    {
        flir->timer.drvHandle = DRV_TMR_Open(flir->timer.index, DRV_IO_INTENT_EXCLUSIVE);
    }        
    return ( DRV_HANDLE_INVALID != flir->timer.drvHandle );            
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

/******************************************************************************/

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

/******************************************************************************/

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
    uint32_t byteIndex;
    uint32_t line;
    FLIR_PIXEL_TYPE pixel;
    uint32_t pixelIndex=0;
    /* fix the endianness on the first word to do a bit of decoding.      */
    flir->VoSPI.b8[1]=flir->RXBuffer.b8[0];
    flir->VoSPI.b8[0]=flir->RXBuffer.b8[1];
    if(DISCARD == flir->VoSPI.ID.discard)
    {
        flir->counters.discardLine++;
        /* it will be discarded once we return*/            
        return false;
    } 
    line = flir->VoSPI.ID.line;
    if(line == flir->status.lastLine)
    {
        /* this will also be discarded, since it is the same as a line    */
        /*  already received.       */
        flir->counters.duplicateLine++;         
        return false;
    }
    flir->status.lastLine = line; 
    if(flir->VoSPI.ID.line >= flir->image.properties.dimensions.vertical)
    {
        flir->status.flags.mysteryLine=true;
        return false;
    }
    /* copy the rest, fixing the endianness */        
    byteIndex=4;
    while (pixelIndex<flir->image.properties.dimensions.horizontal)
    {
        flir->image.buffer.pixel[line][pixelIndex] = 
           ((flir->RXBuffer.b8[byteIndex]<<8)|(flir->RXBuffer.b8[byteIndex+1]));
        byteIndex = byteIndex + 2;
        pixelIndex++;
    }
    return true;
}

/******************************************************************************/

bool FLIR_StartGetVoSPI(FLIR_DATA *flir)
{
    return FLIR_StartSPIRead(flir,sizeof(VOSPI_TYPE));
}

/******************************************************************************/

bool FLIR_MakeIntensityMap(FLIR_DATA *flir)
{
    uint32_t index;
    int32_t red,green,blue;
    for(index=0;index<(FLIR_LUT_SIZE>>1);index++)
    {
        /* calculate red negative slope */
        red = ((-2 * FLIR_PEAK_INTENSITY * index)/(FLIR_LUT_SIZE)) +FLIR_PEAK_INTENSITY;       
        if(red<0)
        {
            red=0;
        }
        else if (red>FLIR_PEAK_INTENSITY)
        {
            red = FLIR_PEAK_INTENSITY;
        }
        flir->colorMap.LUT[index].blue = red;
        /* calculate green positive slope */
        green = ((2*FLIR_PEAK_INTENSITY*index)/(FLIR_LUT_SIZE));
        if(green<0)
        {
            green = 0;
        }
        else if (green>FLIR_PEAK_INTENSITY)
        {
            green=FLIR_PEAK_INTENSITY;
        }        
        flir->colorMap.LUT[index].green = green;        
    }
    for(;index<FLIR_LUT_SIZE;index++)
    {
        /* calculate green negative slope */
        green = ((-2 * FLIR_PEAK_INTENSITY * index)/(FLIR_LUT_SIZE))+ (2*FLIR_PEAK_INTENSITY);
        if(green<0)
        {
            green = 0;
        }
        else if (green>FLIR_PEAK_INTENSITY)
        {
            green=FLIR_PEAK_INTENSITY;
        }
        flir->colorMap.LUT[index].green = green;
        /* calculate blue positive slope */
        blue = ((2*FLIR_PEAK_INTENSITY*index)/(FLIR_LUT_SIZE)) - FLIR_PEAK_INTENSITY;
        if(blue<0)
        {
            blue = 0;
        }
        else if (blue>FLIR_PEAK_INTENSITY)
        {
            blue = FLIR_PEAK_INTENSITY;
        }
        flir->colorMap.LUT[index].red = blue;        
    }
    return true;
}

/******************************************************************************/

bool FLIR_ReMakeIntensityMap(FLIR_DATA *flir)
{
    uint32_t index;
    int32_t red,green,blue;
    
    flir->colorMap.size = flir->colorMap.building.maximum - flir->colorMap.building.minimum;
    flir->colorMap.maximum = flir->colorMap.building.maximum;
    flir->colorMap.minimum = flir->colorMap.building.minimum;
    flir->colorMap.building.maximum= 0;
    flir->colorMap.building.minimum = 0x7fffffff;
    for(index=0;index<(flir->colorMap.size>>1);index++)
    {
        /* calculate red negative slope */
        red = ((-2*(FLIR_PEAK_INTENSITY) * index)/(flir->colorMap.size)) +FLIR_PEAK_INTENSITY;       
        if(red<0)
        {
            red=0;
        }
        else if (red>FLIR_PEAK_INTENSITY)
        {
            red = FLIR_PEAK_INTENSITY;
        }
        flir->colorMap.LUT[index].blue = red;
        /* calculate green positive slope */
        green = ((2*FLIR_PEAK_INTENSITY*index)/(flir->colorMap.size));
        if(green<0)
        {
            green = 0;
        }
        else if (green>FLIR_PEAK_INTENSITY)
        {
            green=FLIR_PEAK_INTENSITY;
        }        
        flir->colorMap.LUT[index].green = green;        
    }
    for(;index<flir->colorMap.size;index++)
    {
        /* calculate green negative slope */
        green = ((-2 * FLIR_PEAK_INTENSITY * index)/(flir->colorMap.size)) + (2*FLIR_PEAK_INTENSITY);
        if(green<0)
        {
            green = 0;
        }
        else if (green>FLIR_PEAK_INTENSITY)
        {
            green=FLIR_PEAK_INTENSITY;
        }
        flir->colorMap.LUT[index].green = green;
        /* calculate blue positive slope */
        blue = ((FLIR_PEAK_INTENSITY*index)/(flir->colorMap.size)) - FLIR_PEAK_INTENSITY;
        if(blue<0)
        {
            blue = 0;
        }
        else if (blue>FLIR_PEAK_INTENSITY)
        {
            blue = FLIR_PEAK_INTENSITY;
        }
        flir->colorMap.LUT[index].red = blue;        
    }
    flir->status.flags.remadeMap = true;
    return true;

}
/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/