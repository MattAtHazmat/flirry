/*******************************************************************************
  Board Support Package Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    bsp_sys_init.c

  Summary:
    Board Support Package implementation for PIC32MZ Embedded Connectivity (EC)
    Starter Kit.

  Description:
    This file contains routines that implement the board support package for
    PIC32MZ Embedded Connectivity (EC) Starter Kit.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

//#############################################################################
//------------------- ChipKIT Wi-Fire Pic32Mz --------------------------------
// Written by QuantumHawk - 10 Feb 2015

//#############################################################################


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "bsp_config.h"

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function: 
    void BSP_Initialize(void)

  Summary:
    Performs the necessary actions to initialize a board
  
  Description:
    This function initializes the LED, Switch and other ports on the board.
    This function must be called by the user before using any APIs present in
    this BSP.  

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_Initialize(void )
{
//    PLIB_PORTS_PinModeSelect(PORTS_ID_0, BSP_LED_1_AN, PORTS_PIN_MODE_DIGITAL);
//    PLIB_PORTS_PinModeSelect(PORTS_ID_0, BSP_LED_3_AN, PORTS_PIN_MODE_DIGITAL);
//    PLIB_PORTS_PinModeSelect(PORTS_ID_0, BSP_LED_4_AN, PORTS_PIN_MODE_DIGITAL);
    /* set pins as output */
//    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, BSP_LED_1_PORT, BSP_LED_1 );
//    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, BSP_LED_2_PORT, BSP_LED_2 );
//    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, BSP_LED_3_PORT, BSP_LED_3 );
//    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, BSP_LED_4_PORT, BSP_LED_3 );
    /* Switch off LEDs */
    PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_1_PORT, BSP_LED_1 );
    PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_2_PORT, BSP_LED_2 );
    PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_3_PORT, BSP_LED_3 );
    PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_4_PORT, BSP_LED_3 );
}

// *****************************************************************************
/* Function: 
    void BSP_LEDStateSet(BSP_LED led, BSP_LED_STATE state);

  Summary:
    Controls the state of the LED.
  
  Description:
    This function allows the application to specify the state of the LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDStateSet(BSP_LED led, BSP_LED_STATE state)
{
    /* Set the state of the LED */
    //PLIB_PORTS_PinWrite (PORTS_ID_0 , BSP_LED_PORT , led, state );

     switch (led) {
        case BSP_LED_1 :
            PLIB_PORTS_PinWrite (PORTS_ID_0 , BSP_LED_1_PORT , led, state );
            break;
        case BSP_LED_2 :
            PLIB_PORTS_PinWrite (PORTS_ID_0 , BSP_LED_2_PORT , led, state );
            break;
        case BSP_LED_3 :
            PLIB_PORTS_PinWrite (PORTS_ID_0 , BSP_LED_3_PORT , led, state );
            break;
        case BSP_LED_4 :
            PLIB_PORTS_PinWrite (PORTS_ID_0 , BSP_LED_4_PORT , led, state );
            break;
    }


}

// *****************************************************************************
/* Function: 
    void BSP_LEDToggle(BSP_LED led);

  Summary:
    Toggles the state of the LED between BSP_LED_STATE_ON and BSP_LED_STATE_OFF.
  
  Description:
    This function toggles the state of the LED between BSP_LED_STATE_ON and
    BSP_LED_STATE_OFF.

  Remarks:
    Refer to bsp_config.h for usage information.
*/    

void BSP_LEDToggle(BSP_LED led)
{
    //PLIB_PORTS_PinToggle(PORTS_ID_0, BSP_LED_PORT, led );

    switch (led) {
        case BSP_LED_1 :
            PLIB_PORTS_PinToggle(PORTS_ID_0,BSP_LED_1_PORT, led );
            break;
        case BSP_LED_2 :
            PLIB_PORTS_PinToggle(PORTS_ID_0,BSP_LED_2_PORT, led );
            break;
        case BSP_LED_3 :
            PLIB_PORTS_PinToggle(PORTS_ID_0,BSP_LED_3_PORT, led );
            break;
        case BSP_LED_4 :
            PLIB_PORTS_PinToggle(PORTS_ID_0,BSP_LED_4_PORT, led );
            break;
    }

}

// *****************************************************************************
/* Function: 
    BSP_LED_STATE BSP_LEDStateGet(BSP_LED led);

  Summary:
    Returns the present state of the LED.
  
  Description:
    This function returns the present state of the LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/    

BSP_LED_STATE BSP_LEDStateGet (BSP_LED led)
{
    /* Get LED Status */
    //return PLIB_PORTS_PinGetLatched (PORTS_ID_0, BSP_LED_PORT, led);

    switch (led) {
        case BSP_LED_1 :
            return PLIB_PORTS_PinGetLatched (PORTS_ID_0, BSP_LED_1_PORT, led);
        case BSP_LED_2 :
            return PLIB_PORTS_PinGetLatched (PORTS_ID_0, BSP_LED_2_PORT, led);
        case BSP_LED_3 :
            return PLIB_PORTS_PinGetLatched (PORTS_ID_0, BSP_LED_3_PORT, led);
        case BSP_LED_4 :
            return PLIB_PORTS_PinGetLatched (PORTS_ID_0, BSP_LED_4_PORT, led);
        default :
            return BSP_LED_STATE_OFF;
    }
}

// *****************************************************************************
/* Function: 
    void BSP_LEDOn(BSP_LED led);

  Summary:
    Switches ON the specified LED.
  
  Description:
    This function switches ON the specified LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDOn(BSP_LED led)
{
    //PLIB_PORTS_PinSet( PORTS_ID_0, BSP_LED_PORT, led );

    switch (led) {
        case BSP_LED_1 :
            PLIB_PORTS_PinSet( PORTS_ID_0, BSP_LED_1_PORT, led );
            break;
        case BSP_LED_2 :
            PLIB_PORTS_PinSet( PORTS_ID_0, BSP_LED_2_PORT, led );
            break;
        case BSP_LED_3 :
            PLIB_PORTS_PinSet( PORTS_ID_0, BSP_LED_3_PORT, led );
            break;
        case BSP_LED_4 :
            PLIB_PORTS_PinSet( PORTS_ID_0, BSP_LED_4_PORT, led );
            break;
    }
}

// *****************************************************************************
/* Function: 
    void BSP_LEDOff(BSP_LED led);

  Summary:
    Switches Off the specified LED.
  
  Description:
    This function switches Off the specified LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDOff(BSP_LED led)
{
    //PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_PORT, led );

    switch (led) {
        case BSP_LED_1 :
            PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_1_PORT, led );
            break;
        case BSP_LED_2 :
            PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_2_PORT, led );
            break;
        case BSP_LED_3 :
            PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_3_PORT, led );
            break;
        case BSP_LED_4 :
            PLIB_PORTS_PinClear( PORTS_ID_0, BSP_LED_4_PORT, led );
            break;
    }
}

// *****************************************************************************
/* Function: 
    void BSP_SwitchStateGet(BSP_SWITCH switch);

  Summary:
    Returns the present state (pressed or not pressed) of the specified switch.
  
  Description:
    This function returns the present state (pressed or not pressed) of the
    specified switch.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

BSP_SWITCH_STATE BSP_SwitchStateGet( BSP_SWITCH bspSwitch )
{
    return ( PLIB_PORTS_PinGet(PORTS_ID_0, BSP_BTN_PORT, bspSwitch) );
}

// *****************************************************************************
/* Function: 
    void BSP_USBVBUSSwitchStateSet(BSP_USB_VBUS_SWITCH_STATE state);

  Summary:
    This function enables or disables the USB VBUS switch on the board.
  
  Description:
    This function enables or disables the VBUS switch on the board.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_USBVBUSSwitchStateSet(BSP_USB_VBUS_SWITCH_STATE state)
{
    /* Enable the VBUS switch */

   // PLIB_PORTS_PinWrite( PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5, state );
}


/*******************************************************************************
 End of File
*/
