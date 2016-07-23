/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    flir.h

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

#ifndef _FLIR_H
#define _FLIR_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

typedef enum
{
	/* Application's state machine's initial state. */
	FLIR_STATE_INIT=0,
    //FLIR_OPEN_I2C_PORT,
            FLIR_OPEN_TIMER,
            FLIR_START_TIMER,
    FLIR_OPEN_SPI_PORT,
    FLIR_START,
	FLIR_STATE_WAIT_TO_GET_IMAGE,
    FLIR_STATE_START_READING_IMAGE,
    FLIR_STATE_START_GET_LINE,
    FLIR_STATE_WAIT_FOR_LINE,
    FLIR_STATE_GET_LINE,
    FLIR_STATE_COPY_IMAGE,
    FLIR_ERROR,
} FLIR_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */



/******************************************************************************/

typedef struct __attribute__((packed)) {
    FLIR_STATES state;
    struct {
        SYS_MODULE_INDEX index;
        DRV_HANDLE drvHandle;
    } timer;
    struct {
        SYS_MODULE_INDEX index;
        DRV_HANDLE drvHandle;
        DRV_I2C_BUFFER_HANDLE bufferHandle;        
    } i2c;
    struct {
        SYS_MODULE_INDEX index;
        DRV_HANDLE drvHandle;
        DRV_SPI_BUFFER_HANDLE bufferHandle;
        struct{
            unsigned complete:1;
            unsigned started:1;
            unsigned error:1;
            unsigned running:1;
        } status;
        uint32_t RXBytesExpected;
    } spi;
    struct __attribute__((aligned(4))){
        union{
            uint8_t b8[BUFFER_SIZE_8];
            uint16_t b16[BUFFER_SIZE_16];
            uint32_t b32[BUFFER_SIZE_32];
        };  
        struct{
            BUFFER_SIZE_TYPE max;
            BUFFER_SIZE_TYPE transfer;
        }size;
        struct {
            uint16_t calculated;
            uint16_t packet;
        }CRC;
    }RXBuffer;
    struct {
        struct {
            unsigned timerConfigured:1;
            unsigned SPIConfigured:1;
            unsigned timerRunning:1;
            unsigned imageStarted:1;
            unsigned getImage:1;
            unsigned getImageMissed:1;
            unsigned imageCopied:1;
        } flags;
        int32_t lastLine;
    }status;
    VOSPI_TYPE VoSPI;
    FLIR_IMAGE_TYPE image;
    struct {
        uint32_t imagesStarted;
        uint32_t imagesCopied;
        uint32_t discardLine;
        uint32_t duplicateLine;
        struct {
            uint32_t sendImageTimeout;
            uint32_t getLine;
            uint32_t timerSetup;
            uint32_t getImageMissed;
            uint32_t readStart;
        }failure;
    }counters;
    //uint32_t debug;
} FLIR_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void FLIR_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    FLIR_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void FLIR_Initialize ( SYS_MODULE_INDEX timerIndex, SYS_MODULE_INDEX I2CIndex, SYS_MODULE_INDEX SPIIndex );


/*******************************************************************************
  Function:
    void FLIR_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    FLIR_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void FLIR_Tasks( void );

bool FLIR_OpenSPI(FLIR_DATA *flir);
bool FLIR_GetVoSPI(FLIR_DATA *flir);
bool FLIR_StartGetVoSPI(FLIR_DATA *flir);
#define FLIR_SPISlaveSelect()       LATECLR = 1<<9
#define FLIR_SPISlaveDeselect()     LATESET = 1<<9
bool FLIR_PopulateLine(FLIR_DATA *flir);
bool FLIR_CopyImage(FLIR_DATA *flir);
inline bool FLIR_DiscardLine(FLIR_DATA *flir);
bool FLIR_OpenTimer(FLIR_DATA *flir);
static bool FLIR_TimerSetup( FLIR_DATA* flir, uint32_t periodMS );
#endif /* _FLIR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

