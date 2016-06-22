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
FLIR_IMAGE_TYPE image;
/******************************************************************************/
/******************************************************************************/
/* Section: Application Callback Functions                                    */
/******************************************************************************/
/******************************************************************************/

/* Application's Timer Callback Function                                      */

static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
    BSP_LEDToggle(BSP_LED_2);
}

/******************************************************************************/

static void SPICallback(uintptr_t context)
{
    FLIRSPISlaveDeselect();
    flirData.spi.status.flags.started = false;
    flirData.spi.status.flags.complete = true;
}

/******************************************************************************/
/******************************************************************************/
/* Section: Application Local Functions                                       */
/******************************************************************************/
/******************************************************************************/


/* Application's LED Task Function                                            */
static void LedTask( void )
{
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
    flirData.TXBuffer.size.max.b8 = BUFFER_SIZE_8;
    flirData.TXBuffer.size.max.b16 = BUFFER_SIZE_16;
    flirData.TXBuffer.size.max.b32 = BUFFER_SIZE_32;
    flirData.RXBuffer.size.max.b8 = BUFFER_SIZE_8;
    flirData.RXBuffer.size.max.b16 = BUFFER_SIZE_16;
    flirData.RXBuffer.size.max.b32 = BUFFER_SIZE_32;
    flirData.lepton.result = LEP_ERROR;
    flirData.image.properties.horizontal = HORIZONTAL_SIZE;
    flirData.image.properties.vertical = VERTICAL_SIZE;
    flirData.image.properties.size.pixels = flirData.image.properties.horizontal * flirData.image.properties.vertical;
    flirData.image.properties.size.bytes =sizeof(flirData.image.image);
    
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
            if (flirData.timer.drvHandle == DRV_HANDLE_INVALID)
            {
                flirData.timer.drvHandle = DRV_TMR_Open(FLIR_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                flirData.status.flags.timerConfigured = ( DRV_HANDLE_INVALID != flirData.timer.drvHandle );
            }        
            if (flirData.status.flags.timerConfigured)
            {
                flirData.status.flags.timerRunning = TimerSetup();
                if(flirData.status.flags.timerRunning)
                {
                    flirData.state = FLIR_OPEN_SPI_PORT;
                }
                else
                {
                    flirData.state = FLIR_ERROR;
                }
            }
            break;
        }
        case FLIR_OPEN_I2C_PORT:
        {
            BSP_LEDOn(BSP_LED_2);
//            flirData.lepton.result = LEP_OpenPort(0,
//                                        LEP_CCI_TWI,
//                                          #ifdef BB_ENABLED
//                                              DRV_I2C_BIT_BANG_BAUD_RATE_IDX0
//                                          #else
//                                              DRV_I2C_BAUD_RATE_IDX0
//                                          #endif
//                                        /1000,&flirData.lepton.cameraPort);
            flirData.status.flags.I2CConfigureAttempted = true;
            if(flirData.lepton.result == LEP_OK)
            {
                //flirData.status.flags.I2CConfigured = true;
                //flirData.status.flags.I2CRunning = true;
            }
            if(flirData.status.flags.SPIConfigureAttempted == false)
            {
                flirData.state= FLIR_OPEN_SPI_PORT;
            }
            else
            {
                flirData.state = FLIR_START;
            }
            break;
        }
        case FLIR_OPEN_SPI_PORT:
        {
            flirData.status.flags.SPIConfigured = OpenFLIRSPI(&flirData);
            flirData.status.flags.SPIConfigureAttempted = true;
            if(flirData.status.flags.SPIConfigured)
            {
                flirData.status.flags.SPIRunning = true;
            }
            if(flirData.status.flags.I2CConfigureAttempted == false)
            {
                flirData.state= FLIR_OPEN_I2C_PORT;
            }
            else
            {
                flirData.state = FLIR_START;
            }
            break;
        }
        case FLIR_START:
        {
            if(flirData.status.flags.SPIRunning||flirData.status.flags.I2CRunning)
            {
                if(flirData.status.flags.SPIRunning)
                {
                    flirData.status.flags.usingSPI = true;
                }
                if(flirData.status.flags.I2CRunning)
                {
                    flirData.status.flags.usingI2C = true;
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
            LedTask();
            if(flirData.status.flags.usingSPI)
            {
                if(FLIRGetVoSPI(&flirData))
                {
                    
                    if(flirData.VoSPI.ID.discard != DISCARD)
                    {
                        FLIRPopulateLine(&flirData);
                        BSP_LEDToggle(BSP_LED_3);
                        //flirData.RXBuffer.CRC.packet = flirData.VoSPI.CRC.b16;
                        /* the CRC is calculated with some parts set to zero */
                        //flirData.VoSPI.CRC.b16 = 0;
                        //flirData.VoSPI.ID.MSBits = 0;
                        //flirData.RXBuffer.CRC.calculated = 
                        //    CalcCRC16Words(flirData.RXBuffer.size.transfer.b16,
                        //                   flirData.VoSPI.b16);
                        //if()
                        //if(flirData.RXBuffer.CRC.calculated == flirData.RXBuffer.CRC.packet)
                        //{
                            //flirData.state = FLIR_STATE_SUCCESS;
                            //FLIRPopulateLine(&flirData);
                        //}
                        
                    }
                }
            }
            break;
        }      
        case FLIR_STATE_SUCCESS:
        {
            LedTask();
            //BSP_LEDOn(BSP_LED_1);
            //BSP_LEDOff(BSP_LED_2);
            BSP_LEDOn(BSP_LED_3);
            BSP_LEDOff(BSP_LED_4);
            break;
        }
        case FLIR_ERROR:
        default:
        {
            //BSP_LEDOn(BSP_LED_1);
            //BSP_LEDOn(BSP_LED_2);
            BSP_LEDOn(BSP_LED_3);
            BSP_LEDOn(BSP_LED_4);
            break;
        }
    }
}

/******************************************************************************/

bool FLIRPopulateLine(FLIR_DATA *flir)
{
    if(flir->VoSPI.ID.line < flir->image.properties.vertical)
    {
        memcpy(&(flir->image.image.pixel[flir->VoSPI.ID.line][0]),
               flir->VoSPI.payload.b8,
               sizeof(flir->VoSPI.payload.b8));
        return true;
    }
    return false;
}

/******************************************************************************/

bool OpenFLIRSPI(FLIR_DATA *flir)
{
    bool SPIReady = false;
    FLIRSPISlaveDeselect();
    flir->spi.drvHandle = DRV_SPI_Open(DRV_SPI_INDEX_0,
                                       DRV_IO_INTENT_EXCLUSIVE|DRV_IO_INTENT_READWRITE);
    if(DRV_HANDLE_INVALID != flir->spi.drvHandle)
    {
        SPIReady = true;
    }
    return SPIReady;
}

/******************************************************************************/

bool StartFLIRSPIReading(FLIR_DATA *flir,uint32_t size)
{
    bool SPIReadingStarted=false;
    FLIRSPISlaveDeselect();
    if(size < flir->TXBuffer.size.max.b8)
    {
        FLIRSPISlaveSelect();
        if(DRV_SPI_BUFFER_HANDLE_INVALID != 
           DRV_SPI_BufferAddRead(flir->spi.drvHandle,flir->TXBuffer.b8,size,
                                 (void*)SPICallback,NULL))
        {
            flir->TXBuffer.size.transfer.b8 = size;
            flir->TXBuffer.size.transfer.b16 = size>>1;
            flir->TXBuffer.size.transfer.b32 = size>>2;
            flir->spi.status.flags.started=true;
            SPIReadingStarted=true;
        }
        else
        {
            //FLIRSPISlaveDeselect();
        }
    }
    return SPIReadingStarted;    
}

/******************************************************************************/

bool GetFLIRSPIReading(FLIR_DATA *flir)
{
    bool SPIReadingReady=false;
    if(flir->spi.status.flags.complete)
    {
        flir->spi.status.flags.complete = false;
        flir->spi.status.flags.started = false;
        SPIReadingReady = true;
    }
    return SPIReadingReady;    
}

bool FLIRSPIWriteRead(FLIR_DATA *flir,uint32_t TXSize,int32_t RXSize)
{
    bool returnValue = false;
    FLIRSPISlaveDeselect();
    /* blocking */
    if((TXSize < flir->TXBuffer.size.max.b8)&&
       (RXSize < flir->RXBuffer.size.max.b8))
    {
        FLIRSPISlaveSelect();
        if(DRV_SPI_BUFFER_HANDLE_INVALID != 
           DRV_SPI_BufferAddWriteRead(flir->spi.drvHandle,
                                      flir->TXBuffer.b8,TXSize,
                                      flir->RXBuffer.b8,RXSize,
                                      (void*)SPICallback,NULL))
        {
            flir->TXBuffer.size.transfer.b8 = TXSize;
            flir->TXBuffer.size.transfer.b16 = TXSize>>1;
            flir->TXBuffer.size.transfer.b32 = TXSize>>2;
            flir->RXBuffer.size.transfer.b8 = RXSize;
            flir->RXBuffer.size.transfer.b16 = RXSize>>1;
            flir->RXBuffer.size.transfer.b32 = RXSize>>2;
            flir->spi.status.flags.started=true;
            do {
                taskYIELD();
            }while(!flir->spi.status.flags.complete);
            returnValue = true;
        }
        else
        {
            FLIRSPISlaveDeselect();
        }
    }
}
bool FLIRSPIRead(FLIR_DATA *flir,int32_t RXSize)
{
    bool returnValue = false;
    FLIRSPISlaveDeselect();
    /* blocking */
    if(RXSize < flir->RXBuffer.size.max.b8)
    {
        FLIRSPISlaveSelect();
        flir->spi.status.flags.complete=false;
        flir->spi.status.flags.started = false;
        flir->spi.bufferHandle = DRV_SPI_BufferAddRead(flir->spi.drvHandle,
                                                       flir->RXBuffer.b8,RXSize,
                                                       (void*)SPICallback,NULL);
        if(DRV_SPI_BUFFER_HANDLE_INVALID != flir->spi.bufferHandle )
        {
            FLIRSPISlaveSelect();
            flir->RXBuffer.size.transfer.b8 = RXSize;
            flir->RXBuffer.size.transfer.b16 = RXSize>>1;
            flir->RXBuffer.size.transfer.b32 = RXSize>>2;
            flir->spi.status.flags.started=true;
            do{ 
                taskYIELD();
            }while(!flir->spi.status.flags.complete);
            returnValue = true;
        }
        else
        {
            FLIRSPISlaveDeselect();
        }
    }
}

bool FLIRGetVoSPI(FLIR_DATA *flir)
{
    if(FLIRSPIRead(flir,sizeof(VOSPI_TYPE)))
    {
        if(NULL == flir->spi.bufferHandle)
        {
            return false;
        }
        if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(flir->spi.bufferHandle))
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
            return true;
        }
    }
    return false;
}
/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/