/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    leds.c

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

#include "leds.h"

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

LEDS_DATA ledsData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void LEDS_Initialize ( void )

  Remarks:
    See prototype in leds.h.
                                                                              */

void LEDS_Initialize ( void )
{
    memset(&ledsData,0,sizeof(ledsData));
    ledsData.state = LEDS_STATE_INIT;
    ledsData.leds.LD1 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_1));
    ledsData.leds.LD1New = ledsData.leds.LD1;
    ledsData.leds.LD2 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_2));
    ledsData.leds.LD2New = ledsData.leds.LD2;
    ledsData.leds.LD3 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_3));
    ledsData.leds.LD3New = ledsData.leds.LD3;
    ledsData.leds.LD4 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_4));
    ledsData.leds.LD4New = ledsData.leds.LD4;
}


/******************************************************************************/
/*  Function:                                                                 */
/*    void LEDS_Tasks ( void )                                                */
/*                                                                            */
/*  Remarks:                                                                  */
/*    See prototype in leds.h.                                                */
/******************************************************************************/

void LEDS_Tasks ( void )
{

    switch ( ledsData.state )
    {

        case LEDS_STATE_INIT:
        {
            ledsData.state = LEDS_STATE_READ;
            break;
        }
        
        case LEDS_STATE_CHANGE:
        {
            if(!ledsData.flags.change)
            {
                break;                
            }
            ledsData.flags.change = false;
            ledsData.state = LEDS_STATE_UPDATE;
        }
        case LEDS_STATE_UPDATE:
        {
            BSP_LEDStateSet(BSP_LED_1,ledsData.leds.LD1New);
            BSP_LEDStateSet(BSP_LED_2,ledsData.leds.LD2New);
            BSP_LEDStateSet(BSP_LED_3,ledsData.leds.LD3New);
            BSP_LEDStateSet(BSP_LED_4,ledsData.leds.LD4New);
            ledsData.state = LEDS_STATE_READ;            
        }
        case LEDS_STATE_READ:
        {
            ledsData.leds.LD1 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_1));
            ledsData.leds.LD1New = ledsData.leds.LD1;
            ledsData.leds.LD2 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_2));
            ledsData.leds.LD2New = ledsData.leds.LD2;
            ledsData.leds.LD3 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_3));
            ledsData.leds.LD3New = ledsData.leds.LD3;
            ledsData.leds.LD4 = (BSP_LED_STATE_ON ==  BSP_LEDStateGet(BSP_LED_4));
            ledsData.leds.LD4New = ledsData.leds.LD4;
            ledsData.state = LEDS_STATE_CHANGE;
            break;
        }
        case LEDS_STATE_ERROR:
        default:
        {
            LEDS_Initialize();
            break;
        }
    }
}
 
/******************************************************************************/

bool GetLED(LEDS_TYPE led)
{
    bool LEDOn = false;
    switch (led)
    {
        case LD1:
        {
            LEDOn = ledsData.leds.LD1;
            break;
        }
        case LD2:
        {
            LEDOn = ledsData.leds.LD2;
            break;
        }
        case LD3:
        {
            LEDOn = ledsData.leds.LD3;
            break;
        }
        case LD4:
        {
            LEDOn = ledsData.leds.LD4;
            break;
        }
        default:
        {
            break;
        }
    }
    return LEDOn;
}

/******************************************************************************/

bool SetLED(LEDS_TYPE led, bool on)
{
    switch (led)
    {
        case LD1:
        {
            ledsData.leds.LD1New = on;
            ledsData.flags.change |= (ledsData.leds.LD1New != ledsData.leds.LD1);
            break;
        }
        case LD2:
        {
            ledsData.leds.LD2New = on;
            ledsData.flags.change |= (ledsData.leds.LD2New != ledsData.leds.LD2);
            break;
        }
        case LD3:
        {
            ledsData.leds.LD3New = on;
            ledsData.flags.change |= (ledsData.leds.LD3New != ledsData.leds.LD3);
            break;
        }
        case LD4:
        {
            ledsData.leds.LD4New = on;
            ledsData.flags.change |= (ledsData.leds.LD4New != ledsData.leds.LD4);
            break;
        }
        default:
        {
            break;
        }
    }
    return ledsData.flags.change;
}

/******************************************************************************/

bool SetLEDs(uint8_t on)
{
    uint8_t count = 0;
    on &= 0b1111;
    do {
        if(on & 0x01)
        {
            SetLED(count,true);
        }
        else
        {
            SetLED(count,false);
        }  
        on >>= 1;
    }while(++count<4);    
    return ledsData.flags.change;
}

/******************************************************************************/

uint8_t GetLEDs(void)
{
    return (ledsData.leds.w & 0xf);
}

/******************************************************************************/
/* End of File                                                                */
/******************************************************************************/
