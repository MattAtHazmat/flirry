/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    disp.c

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
#include "bsp_config.h"
#include "flir.h"
#include "disp.h"



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

DISP_DATA dispData;
extern FLIR_DATA flirData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void DISP_TimerAlarmCallback(uintptr_t context, uint32_t alarmCount)
{
    if(dispData.status.nextSlice==true)
    {
        dispData.status.timerOverrun = true;
    }
    else
    {
        dispData.status.nextSlice = true;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

/******************************************************************************/

inline bool SendSlice(void)
{
    if(dispData.status.nextSlice)
    {        
        dispData.status.nextSlice=false;
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
/*    void DISP_Initialize ( void )                                           */
/*                                                                            */
/*  Remarks:                                                                  */
/*    See prototype in disp.h.                                                */
/*                                                                            */
/******************************************************************************/

bool DISP_Initialize ( SYS_MODULE_OBJ pmpModuleObj, DRV_PMP_INDEX pmpIndex,
                       SYS_MODULE_OBJ tmrModuleObj, SYS_MODULE_INDEX tmrIndex )
{
    memset(&dispData,0,sizeof(dispData));
    dispData.timer.moduleObject = tmrModuleObj;
    dispData.timer.index = tmrIndex;
    dispData.pmp.moduleObject = pmpModuleObj;
    dispData.pmp.index = pmpIndex;
    dispData.state = DISP_STATE_INIT;
    dispData.displayInfo.rows.value = DISPLAY_ROWS;
    dispData.displayInfo.columns.value = DISPLAY_COLUMNS;
    dispData.displayInfo.PWMLevel=0;
    dispData.displayInfo.PWMIncrement = DISP_PWM_INCREMENT;
    dispData.address = 0;
    dispData.timer.driverHandle = DRV_HANDLE_INVALID;
    dispData.pmp.driverHandle = DRV_HANDLE_INVALID;
    dispData.displayInfo.buffer.displaying = 0;
    dispData.displayInfo.buffer.filling = 1;
    ClearSTB();
    return true;
}

/******************************************************************************/

bool DISP_InitializeTimer(DISP_DATA* disp)
{
    if(disp->timer.driverHandle == DRV_HANDLE_INVALID)
    {    
        disp->timer.driverHandle = DRV_TMR_Open(disp->timer.index,
                                                DRV_IO_INTENT_EXCLUSIVE|
                                                DRV_IO_INTENT_NONBLOCKING);
    }        
    return (disp->timer.driverHandle != DRV_HANDLE_INVALID);
}

/******************************************************************************/

bool DISP_InitializePMP(DISP_DATA* disp)
{
    if(disp->pmp.driverHandle == DRV_HANDLE_INVALID)
    {
        DRV_PMP_MODE_CONFIG pmpConfig;
        disp->pmp.driverHandle = DRV_PMP_Open(disp->pmp.index,
                                              DRV_IO_INTENT_EXCLUSIVE|
                                              DRV_IO_INTENT_NONBLOCKING);
        pmpConfig.chipSelect = PMCS1_AS_ADDRESS_LINE_PMCS2_AS_CHIP_SELECT;
        pmpConfig.endianMode=LITTLE; 
        pmpConfig.incrementMode = PMP_ADDRESS_AUTO_INCREMENT;
        pmpConfig.intMode = PMP_INTERRUPT_NONE;
        pmpConfig.pmpMode = PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT; 
        pmpConfig.portSize = PMP_DATA_SIZE_16_BITS;
        pmpConfig.waitStates.dataHoldWait = DISP_DATA_HOLD_WAIT_STATES;
        pmpConfig.waitStates.dataWait = DISP_DATA_SETUP_WAIT;
        pmpConfig.waitStates.strobeWait = DISP_STROBE_WAIT_STATES;
        pmpConfig.pmpMode = PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT;
        DRV_PMP_ModeConfig ( dispData.pmp.driverHandle, pmpConfig );
        PLIB_PMP_AddressPortEnable(dispData.pmp.index, PMP_PMA8_PORT|PMP_PMA9_PORT|PMP_PMA10_PORT|PMP_PMA11_PORT);
        PMCONbits.WRSP = true;       
        PLIB_PMP_Enable(disp->pmp.index);
    }
    if(disp->pmp.driverHandle != DRV_HANDLE_INVALID)
    {
        int x,y;
        uint8_t intensity=0x3f;
        for(y=0;y<disp->displayInfo.rows.value;y++)
        {
            for(x=0;x<disp->displayInfo.columns.value;x++)
            {
                disp->display[0][y][x].w = 0;
                disp->display[1][y][x].w = 0;                          
            }
        }
        dispData.status.displayArrayFilled = true;
    }
    return (disp->pmp.driverHandle != DRV_HANDLE_INVALID);
}

/******************************************************************************/

bool DISP_SetTimerAlarm(DISP_DATA* disp)
{
    uint32_t divider;
    if(disp->timer.driverHandle == DRV_HANDLE_INVALID)
    {    
        return false;
    }
    divider = DRV_TMR_CounterFrequencyGet(dispData.timer.driverHandle)/(DISP_SLICE_UPDATE_RATE*DISP_NUMBER_SLICES);
    return DRV_TMR_AlarmRegister(disp->timer.driverHandle,
                                divider,
                                true,
                                0,
                                DISP_TimerAlarmCallback);    
}

/******************************************************************************/

bool DISP_StartTimer(DISP_DATA* disp)
{
    return DRV_TMR_Start(disp->timer.driverHandle);
}

/******************************************************************************/
/*  Function:                                                                 */
/*    void DISP_Tasks ( void )                                                */
/*                                                                            */
/*  Remarks:                                                                  */
/*   See prototype in disp.h.                                                 */
/******************************************************************************/

void DISP_Tasks ( void )
{
    if(dispData.status.firstSliceSent)
    {
        /* appears to break if the tasks gets called before anything has      */
        /* been put in the queue                                              */
        DRV_PMP_Tasks(dispData.pmp.moduleObject);
    }
    switch ( dispData.state )
    {        
        // <editor-fold defaultstate="collapsed" desc="Initialization Cases">
        case DISP_STATE_INIT:
        {   
            if(!dispData.status.timerInitialized)
            {
                dispData.state = DISP_STATE_INITIALIZE_TIMER;
                break;
            }
            if(!dispData.status.timerAlarmSet)
            {
                dispData.state = DISP_STATE_SET_TIMER_ALARM;
                break;
            }
            if(!dispData.status.PMPInitialized)
            {
                dispData.state = DISP_STATE_INITIALIZE_PMP;
                break;
            }
            if(!dispData.status.timerStarted)
            {
                dispData.state = DISP_STATE_START_TIMER;
                break;
            }
            dispData.state = DISP_STATE_WAIT_FOR_IMAGE;
            break;
        }
        case DISP_STATE_INITIALIZE_TIMER:
        {
            dispData.status.timerInitialized = DISP_InitializeTimer(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }
        case DISP_STATE_INITIALIZE_PMP:
        {
            dispData.status.PMPInitialized = DISP_InitializePMP(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }
        case DISP_STATE_START_TIMER:
        {
            dispData.status.timerStarted = DISP_StartTimer(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }
        case DISP_STATE_SET_TIMER_ALARM:
        {
            dispData.status.timerAlarmSet = DISP_SetTimerAlarm(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }// </editor-fold>
        case DISP_STATE_WAIT_FOR_IMAGE:
        {
            if(dispData.status.displayArrayFilled)
            {
                dispData.state = DISP_FILL_FIRST_SLICE;                
            }
            break;
        }
        case DISP_FILL_FIRST_SLICE:
        {
            if(DRV_PMP_ClientStatus(dispData.pmp.driverHandle)==DRV_PMP_CLIENT_STATUS_OPEN)
            {
                DISP_FillSlice(&dispData);
                dispData.state = DISP_FIRST_SEND_SLICE;
            }
            break;
        }        
        case DISP_WAIT_SEND_SLICE:
        {            
            dispData.state = DISP_WAIT_SEND_SLICE_NO_STROBE;
        }
        case DISP_WAIT_SEND_SLICE_NO_STROBE:
        {
            /* is a new image slice ready? */
            if(SendSlice())
            {
                dispData.state = DISP_FIRST_SEND_SLICE;
            }
            else
            {   /* not time to send the next display slice */
                break;
            }  
            /* and drop through                                        */
        }
        case DISP_FIRST_SEND_SLICE:
        {
            PLIB_PMP_AddressSet(dispData.pmp.index,dispData.address);
            dispData.pmp.pQueue = DRV_PMP_Write(&dispData.pmp.driverHandle,
                                                0,
                                                (uint32_t*)&dispData.sliceBuffer[dispData.status.bufferFilling],
                                                sizeof(dispData.sliceBuffer[0]), 
                                                0);             
            dispData.status.bufferFilling ^= 1; /* switch the filling buffer  */
            dispData.status.firstSliceSent = true;
            dispData.state = DISP_WAIT_FILL_NEXT_SLICE;
            break;
        }
        case DISP_WAIT_FILL_NEXT_SLICE:
        {
            /* fill up the next slice while the current is being sent */
            dispData.status.imageSent = DISP_FillSlice(&dispData);
            if(dispData.status.imageSent)
            {
                dispData.counters.imageSentFlag++;
            }
            dispData.state = DISP_SENDING_SLICE;
            dispData.counters.sliceSend++;
            break;
        }
        case DISP_SENDING_SLICE:
        {
            if(DRV_PMP_TransferStatus(dispData.pmp.pQueue)==PMP_TRANSFER_FINISHED)
            {         
                dispData.state = DISP_DONE_SLICE;
            }
            break;
        }
        case DISP_DONE_SLICE:
        {
            SetSTB();            
            dispData.state = DISP_CHECK_FOR_NEW_IMAGE;
            break;
        }
        case DISP_CHECK_FOR_NEW_IMAGE:
        {
            dispData.counters.imageCheck++;
            if(dispData.status.imageSent && flirData.status.flags.imageCopied)
            {
                dispData.counters.imagesCopied++;
                dispData.status.imageSent = false;
                /* tell the future state that the latest image was copied */
                flirData.status.flags.imageCopied = false;
                dispData.displayInfo.buffer.displaying = dispData.displayInfo.buffer.filling;
                dispData.displayInfo.buffer.filling ^= 1; 
            }            
            ClearSTB();
            dispData.state = DISP_WAIT_SEND_SLICE;
            break;
        }
        case DISP_HALT:
        {
            
            break;
        }
        case TIMER_ERROR:
        case DISP_ERROR:
        default:
        {
            break;
        }
    }
}
 
/******************************************************************************/

bool DISP_FillSlice(DISP_DATA *disp)
{
    bool returnValue=false;
    uint32_t row;
    uint32_t column=0;   
    uint32_t txColumn;
    DISPLAY_PIXEL_TYPE displayPixel;
//    /* increment the slice.                                 */
//    if((disp->slice) == (NUMBER_SLICES-1))
//    {
//        uint32_t tempLevel;
//        disp->slice = 0;
//        /* reached the last slice. time to increment the pwm reference        */
//        tempLevel = disp->displayInfo.PWMLevel + disp->displayInfo.PWMIncrement;
//        if(tempLevel>0xFF)
//        {
//            disp->displayInfo.PWMLevel = 0;
//        }
//        else
//        {
//            disp->displayInfo.PWMLevel = tempLevel;
//        }
//    }
//    else
//    {
//        disp->slice++;
//    }
    txColumn = disp->displayInfo.columns.value;
    do {
        /* start filling in from the end, since the first data shifted in */
        /* will end up at the highest number column. */
        txColumn--;
        row = disp->slice;
        displayPixel.w=0;
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel))
        {
            displayPixel.red0=true;            
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel))
        {
            displayPixel.green0=true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel))
        {
            displayPixel.blue0=true;
        }
        row += DISP_NUMBER_SLICES;
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel))
        {
            displayPixel.red1 = true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel))
        {
            displayPixel.green1 = true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel))
        {
            displayPixel.blue1 = true;
        }
        row += DISP_NUMBER_SLICES;
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel))
        {
            displayPixel.red2 = true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel))
        {
            displayPixel.green2 = true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel))
        {
            displayPixel.blue2 = true;
        }
        row += DISP_NUMBER_SLICES;
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel))
        {
            displayPixel.red3=true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel))
        {
            displayPixel.green3=true;
        }
        if(disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel))
        {
            displayPixel.blue3=true;            
        }
        disp->sliceBuffer[disp->status.bufferFilling].pixel[txColumn] = displayPixel;
        column++;
    } while (txColumn != 0); /* when it's zero, stop */
    /* increment the slice.                                 */
    if((disp->slice) == (DISP_NUMBER_SLICES-1))
    {
        uint32_t tempLevel;
        disp->slice = 0;
        /* reached the last slice. time to increment the pwm reference        */
        tempLevel = disp->displayInfo.PWMLevel + disp->displayInfo.PWMIncrement;
        if(tempLevel>DISP_PEAK_INTENSITY)
        {
            disp->displayInfo.PWMLevel = 0;
            returnValue = true;
        }
        else
        {
            disp->displayInfo.PWMLevel = tempLevel;
        }
    }
    else
    {
        disp->slice++;
    }
    return returnValue;
}

/*******************************************************************************
 End of File
 */
