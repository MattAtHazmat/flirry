// <editor-fold defaultstate="collapsed" desc="Header">
/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    disp.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END
// </editor-fold>

#ifndef _DISP_H
#define _DISP_H

/******************************************************************************/
/******************************************************************************/
/* Section: Included Files                                                    */
/******************************************************************************/
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#define PWM_INCREMENT       (0x3F)
#define DISPLAY_UPDATE      (500) /*Hz                                        */
#define NUMBER_SLICES       (16)
#define DISPLAY_QUANTA      (32)
#define DISPLAY_ROWS        (DISPLAY_QUANTA*2)
#define DISPLAY_COLUMNS     (DISPLAY_QUANTA*3)
#define RED_0_MASK          (0b00000001)
#define GREEN_0_MASK        (0b00000010)
#define BLUE_0_MASK         (0b00000100)
#define RED_1_MASK          (RED_0_MASK<<3)
#define GREEN_1_MASK        (GREEN_0_MASK<<3)
#define BLUE_1_MASK         (BLUE_0_MASK<<3)
#define RED_2_MASK          (RED_0_MASK<<6)
#define GREEN_2_MASK        (GREEN_0_MASK<<6)
#define BLUE_2_MASK         (BLUE_0_MASK<<6)
#define RED_3_MASK          (RED_0_MASK<<9)
#define GREEN_3_MASK        (GREEN_0_MASK<<9)
#define BLUE_3_MASK         (BLUE_0_MASK<<9)
#define DISPLAY_BUFFER_SIZE (DISPLAY_COLUMNS)
#define SetSTB()            LATDSET = _LATD_LATD10_MASK
#define ClearSTB()          LATDCLR = _LATD_LATD10_MASK
#define SetOE()             LATCSET = _LATC_LATC1_MASK 
#define ClearOE()           LATCCLR = _LATC_LATC1_MASK 

/******************************************************************************/
/******************************************************************************/
/* Section: Type Definitions                                                  */
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*  Application states                                                        */
/*                                                                            */
/*  Summary:                                                                  */
/*    Application states enumeration                                          */
/*                                                                            */
/*  Description:                                                              */
/*    This enumeration defines the valid application states.  These states    */
/*    determine the behavior of the application at various times.             */
/******************************************************************************/

typedef enum
{
	/* Application's state machine's initial state.                              */
	DISP_STATE_INIT=0,
    DISP_FILL_FIRST_SLICE,
    DISP_FIRST_SEND_SLICE,
    DISP_SEND_ARRAY,
    DISP_WAIT_FILL_NEXT_SLICE,
    DISP_WAIT_SEND_SLICE,
    DISP_SENDING_SLICE,
    DISP_HALT,
    TIMER_ERROR,
    DISP_ERROR
} DISP_STATES;

typedef union {
    uint32_t w;
    struct __attribute__ ((packed)){
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        unsigned:8;
    };
} PIXEL_TYPE;

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
/*    Application strings and buffers are be defined outside this structure.  */
/******************************************************************************/

typedef union {
    uint16_t w;
    struct __attribute__ ((packed)){
        unsigned red0:1;
        unsigned green0:1;
        unsigned blue0:1;
        unsigned red1:1;
        unsigned green1:1;
        unsigned blue1:1;
        unsigned red2:1;
        unsigned green2:1;
        unsigned blue2:1;
        unsigned red3:1;
        unsigned green3:1;
        unsigned blue3:1;
        unsigned :4;
    };
} DISPLAY_PIXEL_TYPE;

typedef struct
{
    union{
        uint32_t w;
        struct __attribute__ ((packed)) {
            unsigned nextSlice:1;
            unsigned timerOverrun:1;
            unsigned displayArrayFilled:1;
            unsigned timerStarted:1;
            unsigned firstSliceSent:1;
            unsigned bufferFilling:1;
            unsigned slice:4;
        }status;
    };        
    struct {
        uint8_t PWMIncrement;
        uint8_t PWMLevel;
        uint32_t rows;
        uint32_t columns;
    } displayInfo;
    DISP_STATES state;
    SYS_MODULE_INDEX timerIndex; 
    DRV_PMP_INDEX pmpIndex;
    DRV_HANDLE pmpDriverHandle;
    DRV_HANDLE timerDriverHandle;
    SYS_MODULE_OBJ timerModuleObject;
    SYS_MODULE_OBJ pmpModuleObject;
    union __attribute__ ((packed)){
        DISPLAY_PIXEL_TYPE pixel[DISPLAY_BUFFER_SIZE+1];
        uint8_t  b8[2*(DISPLAY_BUFFER_SIZE+1)];
    } sliceBuffer[2];
    PMP_QUEUE_ELEMENT_OBJECT* pQueue;
    PIXEL_TYPE display[DISPLAY_ROWS][DISPLAY_COLUMNS];    
} DISP_DATA;


/******************************************************************************/
/******************************************************************************/
/* Section: Application Callback Routines                                     */
/******************************************************************************/
/******************************************************************************/
/* These routines are called by drivers when certain events occur.            */	
/******************************************************************************/
/******************************************************************************/
/* Section: Application Initialization and State Machine Functions            */
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*  Function:                                                                 */
/*    void DISP_Initialize ( void )                                           */
/*                                                                            */
/*  Summary:                                                                  */
/*     MPLAB Harmony application initialization routine.                      */
/*                                                                            */
/*  Description:                                                              */
/*    This function initializes the Harmony application.  It places the       */
/*    application in its initial state and prepares it to run so that its     */
/*    APP_Tasks function can be called.                                       */
/*                                                                            */
/*  Precondition:                                                             */
/*    All other system initialization routines should be called before calling*/
/*    this routine (in "SYS_Initialize").                                     */
/*                                                                            */
/*  Parameters:                                                               */
/*    None.                                                                   */
/*                                                                            */
/*  Returns:                                                                  */
/*    None.                                                                   */
/*                                                                            */
/*  Example:                                                                  */
/*    <code>                                                                  */
/*    DISP_Initialize();                                                      */
/*    </code>                                                                 */
/*                                                                            */
/*  Remarks:                                                                  */
/*    This routine must be called from the SYS_Initialize function.           */
/******************************************************************************/

bool DISP_Initialize ( SYS_MODULE_OBJ, DRV_PMP_INDEX, SYS_MODULE_OBJ, SYS_MODULE_INDEX );


/*******************************************************************************
/*  Function:                                                                 */
/*    void DISP_Tasks ( void )                                                */
/*                                                                            */
/*  Summary:                                                                  */
/*    MPLAB Harmony Demo application tasks function                           */
/*                                                                            */
/*  Description:                                                              */
/*    This routine is the Harmony Demo application's tasks function.  It      */
/*    defines the application's state machine and core logic.                 */
/*                                                                            */
/*  Precondition:                                                             */
/*    The system and application initialization ("SYS_Initialize") should be  */
/*    called before calling this.                                             */
/*                                                                            */
/*  Parameters:                                                               */
/*    None.                                                                   */
/*                                                                            */
/*  Returns:                                                                  */
/*    None.                                                                   */
/*                                                                            */
/*  Example:                                                                  */
/*    <code>                                                                  */
/*    DISP_Tasks();                                                           */
/*    </code>                                                                 */
/*                                                                            */
/*  Remarks:                                                                  */
/*    This routine must be called from SYS_Tasks() routine.                   */
/*                                                                            */
/******************************************************************************/

void DISP_Tasks( void );

void DISP_FillSlice(DISP_DATA *displayData);

#endif /* _DISP_H                                                             */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/