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
    uint32_t* sliceToSend;   
    SetSTB();
    dispData.counters.timerCallback++;
    if(dispData.status.flags.firstSliceSent &&
       (DRV_PMP_TransferStatus(dispData.pmp.pQueue)!=PMP_TRANSFER_FINISHED))
    {
        dispData.counters.timerOverrun++;
    }
    else
    {
        if(dispData.status.flags.sliceReady&(!dispData.status.flags.forceBlankSlice))
        {
            dispData.slice.displaying = dispData.slice.filling;
            sliceToSend = (uint32_t*)&dispData.slice.buffer[dispData.slice.displaying];
            dispData.status.flags.sliceReady = false;   
            dispData.counters.sliceSent++;     
        }
        else
        {
            /* not ready or forcing a blank slice- send a blank line so we */
            /* don't  get a bright streak */
            sliceToSend = (uint32_t*)&dispData.slice.buffer[BLANK_SLICE];    
            dispData.counters.blankSliceSent++;
        }
        ClearSTB();
        PLIB_PMP_AddressSet(dispData.pmp.index,dispData.address.w);
        dispData.pmp.pQueue = DRV_PMP_Write(&dispData.pmp.driverHandle,
                                            0,
                                            sliceToSend,
                                            sizeof(dispData.slice.buffer[0].b8), 
                                            0);
        
        dispData.status.flags.sliceSent = true;
        dispData.status.flags.firstSliceSent = true;   
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static inline bool DISP_SliceSent(void)
{
    __builtin_disable_interrupts();
    if(dispData.status.flags.sliceSent)
    {
        dispData.status.flags.sliceSent = false;
        __builtin_enable_interrupts();
        return true;
    }
    __builtin_enable_interrupts();
    return false;
}

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
    dispData.displayInfo.rows = DISPLAY_ROWS;
    dispData.displayInfo.columns = DISPLAY_COLUMNS;
    dispData.displayInfo.PWMLevel=0;
    dispData.displayInfo.PWMIncrement = DISP_PWM_INCREMENT;
    dispData.address.w = 0; /* the address we start sending from the PMP each time */
    dispData.timer.driverHandle = DRV_HANDLE_INVALID;
    dispData.pmp.driverHandle = DRV_HANDLE_INVALID;
    dispData.displayInfo.buffer.displaying = 0;
    dispData.displayInfo.buffer.filling = 1;
    dispData.slice.filling = 0;
    dispData.slice.displaying = 1;
    dispData.displayInfo.offset.horizontal = DISP_HORIZONTAL_OFFSET;
    dispData.displayInfo.offset.vertical = DISP_VERTICAL_OFFSET;
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
        pmpConfig.endianMode = LITTLE; 
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
        /* filled with empty */
        dispData.status.flags.displayArrayFilled = true;
    }
    return (disp->pmp.driverHandle != DRV_HANDLE_INVALID);
}

/******************************************************************************/

bool DISP_SetTimerAlarm(DISP_DATA* disp)
{
    if(disp->timer.driverHandle == DRV_HANDLE_INVALID)
    {    
        return false;
    }
    disp->timer.divider = DRV_TMR_CounterFrequencyGet(disp->timer.driverHandle)/(DISP_SLICE_UPDATE_RATE);
    if((DRV_TMR_OPERATION_MODE_16_BIT ==
       DRV_TMR_OperationModeGet(disp->timer.driverHandle))&&(disp->timer.divider>0xFFFF))
    {
        /* the divider is not gonna work for a 16 bit timer */
        return false;
    }
    return DRV_TMR_AlarmRegister(disp->timer.driverHandle,
                                 disp->timer.divider,
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

void DISP_StopTimer(DISP_DATA* disp)
{
    DRV_TMR_Stop(disp->timer.driverHandle);
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
    if(dispData.status.flags.firstSliceSent)
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
            if(!dispData.status.flags.timerInitialized)
            {
                dispData.state = DISP_STATE_INITIALIZE_TIMER;
                break;
            }
            if(!dispData.status.flags.timerAlarmSet)
            {
                dispData.state = DISP_STATE_SET_TIMER_ALARM;
                break;
            }
            if(!dispData.status.flags.PMPInitialized)
            {
                dispData.state = DISP_STATE_INITIALIZE_PMP;
                break;
            }
            if(!dispData.status.flags.timerStarted)
            {
                dispData.state = DISP_STATE_START_TIMER;
                break;
            }
            dispData.state = DISP_STATE_WAIT_FOR_IMAGE;
            break;
        }
        case DISP_STATE_INITIALIZE_TIMER:
        {
            dispData.status.flags.timerInitialized = DISP_InitializeTimer(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }
        case DISP_STATE_INITIALIZE_PMP:
        {
            dispData.status.flags.PMPInitialized = DISP_InitializePMP(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }
        case DISP_STATE_START_TIMER:
        {
            dispData.status.flags.timerStarted = DISP_StartTimer(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }
        case DISP_STATE_SET_TIMER_ALARM:
        {
            dispData.status.flags.timerAlarmSet = DISP_SetTimerAlarm(&dispData);
            dispData.state = DISP_STATE_INIT;
            break;
        }// </editor-fold>
        case DISP_STATE_WAIT_FOR_IMAGE:
        {
            if(dispData.status.flags.displayArrayFilled)
            {
                dispData.state = DISP_STATE_FILL_FIRST_SLICE;                
            }
            break;
        }
        case DISP_STATE_FILL_FIRST_SLICE:
        {
            if(DRV_PMP_CLIENT_STATUS_OPEN == DRV_PMP_ClientStatus(dispData.pmp.driverHandle))
            {
                /* fill the first slice with display data */
                dispData.status.flags.pwmCycleComplete = DISP_FillSlice(&dispData);
                dispData.state = DISP_STATE_WAIT_SLICE_SEND_START;
            }
            else
            {
                break;
            }
        }        
        case DISP_STATE_WAIT_SLICE_SEND_START:        
        {
            /* has the current slice been sent? */
            if(DISP_SliceSent())
            {
                /* once the current slice has started to be sent, fill the    */
                /* next slice */
                dispData.state = DISP_STATE_FILL_SLICE;
                /* and drop through                                        */
            }
            else
            {   
                break;
            }              
        }
        case DISP_STATE_FILL_SLICE:
        {   
            /* fill up the next slice while the current is being sent */            
            dispData.status.flags.pwmCycleComplete = DISP_FillSlice(&dispData);
            if(dispData.status.flags.pwmCycleComplete)
            {
                dispData.counters.imageSent++;
            }            
            
            if(dispData.status.flags.pwmCycleComplete)
            {
                /* check to see if there is a new image only when we have     */
                /* completed sending an image- the full PWM cycle.            */
                dispData.state = DISP_STATE_CHECK_FOR_NEW_IMAGE;
            }
            else
            {
                dispData.state = DISP_STATE_WAIT_SLICE_SEND_START;
            }
            break;
        }        
        case DISP_STATE_CHECK_FOR_NEW_IMAGE:
        {
            dispData.counters.imageCheck++;
            if(flirData.status.flags.imageCopied)
            {
                dispData.counters.imagesCopied++;
                /* tell the future state that the latest image was copied     */
                flirData.status.flags.imageCopied = false;
                /* make the displaying image the one we were filling          */
                dispData.displayInfo.buffer.displaying = dispData.displayInfo.buffer.filling;
                if(dispData.displayInfo.buffer.displaying == 0)
                {
                    dispData.displayInfo.buffer.filling = 1;
                }
                else
                {
                    dispData.displayInfo.buffer.filling = 0;
                }
            }            
            /* no matter what, just wait for the current slice to be sent */
            dispData.state = DISP_STATE_WAIT_SLICE_SEND_START;
            break;
        }
        case DISP_STATE_HALT:
        {        
            DISP_StopTimer(&dispData);
            break;
        }
        case DISP_STATE_TIMER_ERROR:
        case DISP_STATE_ERROR:
        default:
        {
            break;
        }
    }
}
 
/******************************************************************************/

bool DISP_FillSlice(DISP_DATA *disp)
{
    bool pwmIntervalEnd = false;
    uint32_t row;
    uint32_t column;   
    uint32_t txColumn;
    DISPLAY_PIXEL_TYPE displayPixel;
    if(disp->slice.displaying == 0)
    {
        disp->slice.filling = 1;
    }
    else
    {
        disp->slice.filling = 0;
    }
    /* start filling in from the end, since the first data shifted in will    */
    /* end up at the highest number column.                                   */
    txColumn = disp->displayInfo.columns;
    column = 0;
    do {        
        txColumn--;
        row = disp->address.slice;
        displayPixel.w=0;
        displayPixel.red0   = (disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel));
        displayPixel.green0 = (disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel));
        displayPixel.blue0  = (disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel));
        row += DISP_NUMBER_SLICES;
        displayPixel.red1   = (disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel));
        displayPixel.green1 = (disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel));
        displayPixel.blue1  = (disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel));
        row += DISP_NUMBER_SLICES;
        displayPixel.red2   = (disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel));
        displayPixel.green2 = (disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel));
        displayPixel.blue2  = (disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel));
        row += DISP_NUMBER_SLICES;
        displayPixel.red3   = (disp->display[disp->displayInfo.buffer.displaying][row][column].red>(disp->displayInfo.PWMLevel));
        displayPixel.green3 = (disp->display[disp->displayInfo.buffer.displaying][row][column].green>(disp->displayInfo.PWMLevel));
        displayPixel.blue3  = (disp->display[disp->displayInfo.buffer.displaying][row][column].blue>(disp->displayInfo.PWMLevel));
        disp->slice.buffer[disp->slice.filling].pixel[txColumn] = displayPixel;
        column++;
    } while (txColumn != 0); /* when it's zero, stop                          */
    /* increment the slice.                                                   */
    disp->address.slice++;
    if(disp->address.slice == 0) /* it is only 4 bits, so it rolls over       */
    {
        disp->address.w = 0;
        /* reached the last slice. time to increment the pwm reference        */
        disp->displayInfo.PWMLevel += disp->displayInfo.PWMIncrement;
        if(disp->displayInfo.PWMLevel>DISP_PEAK_INTENSITY)
        {
            disp->displayInfo.PWMLevel = 0;
            pwmIntervalEnd = true;
        }
    }    
    disp->status.flags.sliceReady = true;
    return pwmIntervalEnd;
}

/*******************************************************************************
 End of File
 */
