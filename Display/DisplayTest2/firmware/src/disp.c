// <editor-fold defaultstate="collapsed" desc="Header">
/******************************************************************************/
/*  MPLAB Harmony Application Source File                                     */
/*                                                                            */
/*  Company:                                                                  */
/*    Microchip Technology Inc.                                               */
/*                                                                            */
/*  File Name:                                                                */
/*    disp.c                                                                  */
/*                                                                            */
/*  Summary:                                                                  */
/*    This file contains the source code for the MPLAB Harmony application.   */
/*                                                                            */
/*  Description:                                                              */
/*This file contains the source code for the MPLAB Harmony application.  It   */
/*implements the logic of the application's state machine and it may call     */
/*API routines of other MPLAB Harmony modules in the system, such as drivers, */
/*system services, and middleware.  However, it does not call any of the      */
/*stem interfaces (such as the "Initialize" and "Tasks" functions) of any of  */
/*the modules in the system or make any assumptions about when those functions*/
/*are called.  That is the responsibility of the configuration-specific system*/
/*files.                                                                      */
/******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
/* Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights     */
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
/* OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR   */
/*PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED*/
/*UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF       */
/*WARRANTY, OROTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR  */
/*EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT,    */
/*PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF       */
/*PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY     */
/*THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER  */
/* SIMILAR COSTS.                                                             */
/******************************************************************************/
// DOM-IGNORE-END


/******************************************************************************/ 
/******************************************************************************/ 
/* Section: Included Files                                                    */
/******************************************************************************/ 
/******************************************************************************/ 
//</editor-fold>

#include "disp.h"
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
/*                                                                            */
/******************************************************************************/

DISP_DATA dispData;


/******************************************************************************/
/******************************************************************************/
/* Section: Application Callback Functions                                    */
/******************************************************************************/
/******************************************************************************/

void TimerAlarmCallback(uintptr_t context, uint32_t alarmCount)
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
    DRV_PMP_MODE_CONFIG pmpConfig = {0};
    uint32_t divider;
    /* clear out the entire structure                                         */
    memset(&dispData,0,sizeof(dispData));
    /* set up the timer driver                                                */
    /* save the timer driver object and module index in the app's structure   */
    dispData.timerModuleObject = tmrModuleObj;
    dispData.timerIndex = tmrIndex;
    if(DRV_TMR_Status(dispData.timerModuleObject)!=SYS_STATUS_READY)
    {
        return false;
    }
    dispData.timerDriverHandle = DRV_TMR_Open(dispData.timerIndex,DRV_IO_INTENT_EXCLUSIVE| DRV_IO_INTENT_NONBLOCKING);
    if (DRV_TMR_ClientStatus(dispData.timerDriverHandle) != DRV_TMR_CLIENT_STATUS_READY)
    {
        return false;
    }
    divider = DRV_TMR_CounterFrequencyGet(dispData.timerDriverHandle)/(DISPLAY_UPDATE*NUMBER_SLICES);    
    if(!DRV_TMR_AlarmRegister(dispData.timerDriverHandle,divider,true,0,TimerAlarmCallback))
    {
        return false;
    }
    /* set up the parallel master port (PMP)                                  */
    /* save the pmp driver object and module index in the app's structure     */
    dispData.pmpModuleObject = pmpModuleObj;
    dispData.pmpIndex = pmpIndex;
    
    dispData.pmpDriverHandle = DRV_PMP_Open(dispData.pmpIndex,DRV_IO_INTENT_EXCLUSIVE| DRV_IO_INTENT_NONBLOCKING);
    if(dispData.pmpDriverHandle == DRV_HANDLE_INVALID)
    {
        return false;
    }
    pmpConfig.pmpMode=  PMP_MASTER_READ_WRITE_STROBES_INDEPENDENT;
	/* Interrupt mode                                                         */
    pmpConfig.intMode=PMP_INTERRUPT_NONE;
	/* address/buffer increment mode                                          */
    pmpConfig.incrementMode= PMP_ADDRESS_AUTO_INCREMENT;
	/* Endian modes                                                           */
    pmpConfig.endianMode= LITTLE_ENDIAN;
	/* Data Port Size                                                         */       
    pmpConfig.portSize= PMP_DATA_SIZE_16_BITS;
	/* Wait states                                                            */
    pmpConfig.waitStates.dataWait =  PMP_DATA_WAIT_ONE;  /* WAITB             */
    pmpConfig.waitStates.strobeWait = PMP_STROBE_WAIT_1; /* WAITM             */
    pmpConfig.waitStates.dataHoldWait = PMP_DATA_HOLD_1; /* WAITE             */
    /* PMP chip select pins selection                                         */
	pmpConfig.chipSelect = PMCS1_PMCS2_AS_ADDRESS_LINES;
	DRV_PMP_ModeConfig ( dispData.pmpDriverHandle, pmpConfig );
    PMAEN = _PMAEN_PTEN14_MASK | 0xFFC;
    /* Place the App state machine in its initial state.                      */
    dispData.state = DISP_STATE_INIT;
    dispData.displayInfo.rows.value = DISPLAY_ROWS;
    dispData.displayInfo.columns.value = DISPLAY_COLUMNS;
    dispData.displayInfo.PWMLevel=0;
    dispData.displayInfo.PWMIncrement = PWM_INCREMENT;
    dispData.displayInfo.numberSprites = NUMBER_SPRITES;
    dispData.sprite[0].position.row.value=0;
    dispData.sprite[0].position.column.value=0;
    dispData.sprite[0].color.red=0x7f;
    dispData.sprite[1].position.row.value=2;
    dispData.sprite[1].position.column.value=2;
    dispData.sprite[1].color.green=0x7f;
    dispData.sprite[2].position.row.value=4;
    dispData.sprite[2].position.column.value=4;
    dispData.sprite[2].color.blue=0x7f;
    //uint32_t sprite = 0;
    
//    for(sprite=0;sprite<dispData.displayInfo.numberSprites;sprite++)
//    {
//        dispData.sprite[sprite].position.row.value=(2*sprite)+10;
//        dispData.sprite[sprite].position.column.value=(2*sprite)+10;
//    }
//    dispData.sprite[0].color.blue=0x7f;
//    dispData.sprite[1].color.green=0x7f;
//    dispData.sprite[2].color.red=0x7f;
//    dispData.sprite[0].velocity.column.w =-(1<<8);
//    dispData.sprite[0].velocity.row.w = 1<<8;
//    dispData.sprite[1].velocity.row.w= 0b11<<7;
//    dispData.sprite[2].velocity.row.w = 0b1<<4;
//    dispData.sprite[2].velocity.column.w = 0b1<<7;

    ClearOE();
    ClearSTB();
    return true;
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
        DRV_PMP_Tasks(dispData.pmpModuleObject);
    }
    switch ( dispData.state )
    {
        case DISP_STATE_INIT:
        {   
            if(dispData.pmpDriverHandle == DRV_HANDLE_INVALID)
            {
                dispData.state = DISP_ERROR;
            }
            else if(!dispData.status.displayArrayFilled)
            {
                memset(dispData.display,0,sizeof(dispData.display));
                dispData.status.displayArrayFilled = true;
                dispData.state = DISP_FILL_FIRST_SLICE;
            }
            if(dispData.timerDriverHandle == DRV_HANDLE_INVALID)
            {
                dispData.state = TIMER_ERROR;
            }
            else
            {
                /* start the timer                                            */
                if(!dispData.status.timerStarted && DRV_TMR_Start(dispData.timerDriverHandle))
                {
                    dispData.status.timerStarted=true;
                }
            }
            if(dispData.status.displayArrayFilled && dispData.status.timerStarted)
            {
                dispData.state = DISP_FILL_FIRST_SLICE;
            }
            break;
        }
        case DISP_FILL_FIRST_SLICE:
        {
            if(DRV_PMP_ClientStatus(dispData.pmpDriverHandle)==DRV_PMP_CLIENT_STATUS_OPEN)
            {
                DISP_FillSlice(&dispData);
                dispData.state = DISP_FIRST_SEND_SLICE;
            }
            break;
        }        
        case DISP_WAIT_SEND_SLICE:
        {
            ClearSTB();
            if(!SendSlice())
            {       
                uint8_t index;
                memset(dispData.display,0,sizeof(dispData.display));
                for(index=0;index<dispData.displayInfo.numberSprites;index++)
                {
//                    dispData.sprite[index].position.row.w += dispData.sprite[index].velocity.row.w;
//                    if(dispData.sprite[index].position.row.value < 0)
//                    {
//                        dispData.sprite[index].position.row.w *= -1;
//                        dispData.sprite[index].velocity.row.w *= -1;
//                    }
//                    else if(((dispData.sprite[index].position.row.value))>=dispData.displayInfo.rows.value)
//                    {
//                        dispData.sprite[index].position.row.w -= (dispData.displayInfo.columns.w);
//                        dispData.sprite[index].velocity.row.w *= -1;
//                    }
//                    dispData.sprite[index].position.column.w += dispData.sprite[index].velocity.column.w;
//                    if(dispData.sprite[index].position.column.value<0)
//                    {
//                        dispData.sprite[index].position.column.w *= -1;
//                        dispData.sprite[index].velocity.column.w *= -1;
//                    }
//                    else if(((dispData.sprite[index].position.column.value))>= dispData.displayInfo.columns.value)
//                    {
//                        dispData.sprite[index].position.column.w -= (dispData.displayInfo.columns.w);
//                        dispData.sprite[index].velocity.column.w *= -1;
//                    }
//                    dispData.sprite[index].position.row.value &= dispData.displayInfo.rows.value -1;
//                    dispData.sprite[index].position.row.w &= 0x00ff0000;                    
//                    dispData.sprite[index].position.column.value &= dispData.displayInfo.columns.value-1;
//                    dispData.sprite[index].position.column.w &= 0x00ff0000;
                    dispData.display[dispData.sprite[index].position.row.value]
                                    [dispData.sprite[index].position.column.value].w = 
                                        dispData.sprite[index].color.w;
                }
                break;
            }       
            /* otherwise, drop through                                        */
        }
        case DISP_FIRST_SEND_SLICE:
        {
            PLIB_PMP_AddressSet(dispData.pmpIndex,((uint16_t)dispData.status.slice)<<8);
            dispData.pQueue = DRV_PMP_Write(&dispData.pmpDriverHandle,
                                            0,
                                            (uint32_t*)&dispData.sliceBuffer[dispData.status.bufferFilling],
                                            DISPLAY_BUFFER_SIZE+1,
                                            0);             
            dispData.status.bufferFilling ^= 1; /* switch the filling buffer  */
            dispData.status.firstSliceSent = true;
            dispData.state = DISP_WAIT_FILL_NEXT_SLICE;
            break;
        }
        case DISP_WAIT_FILL_NEXT_SLICE:
        {
            /* fill up the next slice while the current is being sent */
            DISP_FillSlice(&dispData);
            dispData.state = DISP_SENDING_SLICE;
            break;
        }
        case DISP_SENDING_SLICE:
        {
            if(DRV_PMP_TransferStatus(dispData.pQueue)==PMP_TRANSFER_FINISHED)
            {
                SetSTB();
                dispData.state = DISP_WAIT_SEND_SLICE;
            }
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

void DISP_FillSlice(DISP_DATA *displayData)
{
    volatile uint32_t row;
    volatile uint32_t column;   
    /* start out with the entire pixel row as zero                            */
    memset(&displayData->sliceBuffer[displayData->status.bufferFilling],0,sizeof(displayData->sliceBuffer[0]));
    for(column=0;column<(displayData->displayInfo.columns.value);column++)
    {
        row = displayData->status.slice;
        if(displayData->display[row][column].red>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= RED_0_MASK;
        }
        if(displayData->display[row][column].green>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= GREEN_0_MASK;
        }
        if(displayData->display[row][column].blue>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= BLUE_0_MASK;
        }
        row += NUMBER_SLICES;//(displayData->displayInfo.rows.upper>>2);
        if(displayData->display[row][column].red>(displayData->displayInfo.PWMLevel))
        {
           displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= RED_1_MASK;
        }
        if(displayData->display[row][column].green>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= GREEN_1_MASK;
        }
        if(displayData->display[row][column].blue>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= BLUE_1_MASK;
        }
        row += NUMBER_SLICES;//(displayData->displayInfo.rows>>2);
        if(displayData->display[row][column].red>(displayData->displayInfo.PWMLevel))
        {
           displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= RED_2_MASK;
        }
        if(displayData->display[row][column].green>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= GREEN_2_MASK;
        }
        if(displayData->display[row][column].blue>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= BLUE_2_MASK;
        }
        row += NUMBER_SLICES;//(displayData->displayInfo.rows>>2);
        if(displayData->display[row][column].red>(displayData->displayInfo.PWMLevel))
        {
           displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= RED_3_MASK;
        }
        if(displayData->display[row][column].green>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= GREEN_3_MASK;
        }
        if(displayData->display[row][column].blue>(displayData->displayInfo.PWMLevel))
        {
            displayData->sliceBuffer[displayData->status.bufferFilling].pixel[column].w |= BLUE_3_MASK;
        }
    }
    /* point to the next slice for next time.                                 */
    if((displayData->status.slice) == (NUMBER_SLICES-1))
    {
        uint32_t tempLevel;
        displayData->status.slice = 0;
        /* reached the last slice. time to increment the pwm reference        */
        tempLevel = displayData->displayInfo.PWMLevel + displayData->displayInfo.PWMIncrement;
        if(tempLevel>0xFF)
        {
            displayData->displayInfo.PWMLevel = 0;
        }
        else
        {
            displayData->displayInfo.PWMLevel = tempLevel;
        }
    }
    else
    {
        displayData->status.slice++;
    }
}

/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/
