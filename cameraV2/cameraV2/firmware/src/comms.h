/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    comms.h

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

#ifndef _COMMS_H
#define _COMMS_H

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
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	COMMS_STATE_INIT=0,
    COMMS_STATE_INITIALIZE_SPI,
	COMMS_STATE_SERVICE_TASKS,
    COMMS_TRANSMIT_IMAGE,
    COMMS_TRANSMIT_IMAGE_HEADER,
    COMMS_TRANSMIT_LINE,
    COMMS_STATE_IMAGE_DONE,
    COMMS_STATE_ERROR
} COMMS_STATES;

#define COMMS_BUFFER_SIZE       (0x100)
#define COMMS_BUFFER_SIZE_8     COMMS_BUFFER_SIZE
#define COMMS_BUFFER_SIZE_16    (COMMS_BUFFER_SIZE_8>>1)
#define COMMS_BUFFER_SIZE_32    (COMMS_BUFFER_SIZE_8>>2)

#define IMAGE_INFO_ID   (0xA5)
#define IMAGE_LINE_ID   (0x5A)
#define IMAGE_DONE_ID   (0x55)

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    COMMS_STATES state;
    struct {
        DRV_HANDLE drvHandle;
        DRV_SPI_BUFFER_HANDLE bufferHandle;
        struct{
            unsigned complete:1;
            unsigned started:1;
        }status;            
    } spi;
    struct __attribute__((packed)) {
        union{
            uint8_t   b8[COMMS_BUFFER_SIZE_8];
            uint16_t b16[COMMS_BUFFER_SIZE_16];
            uint32_t b32[COMMS_BUFFER_SIZE_32];
        };  
        struct{
            BUFFER_SIZE_TYPE max;
            BUFFER_SIZE_TYPE transfer;
        }size;
    }TXBuffer;    
    struct {
        unsigned initialized:1;
        unsigned SPIInitialized:1;
    }status;
    struct {
        TaskHandle_t myHandle;
        TaskHandle_t FLIRHandle;
    }RTOS;
    struct {
        uint32_t receive;
        uint32_t transmit;
        uint32_t number;
    }buffer;
    FLIR_IMAGE_TYPE image;
    uint32_t transmitLine;
    struct {
        uint32_t imagesReceived;
        uint32_t imagesTransmitted;
        uint32_t spiFailures;
    }counters;
} COMMS_DATA;


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
    void COMMS_Initialize ( void )

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
    COMMS_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void COMMS_Initialize ( void );


/*******************************************************************************
  Function:
    void COMMS_Tasks ( void )

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
    COMMS_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void COMMS_Tasks( void );
bool COMMS_TransmitImageHeader(COMMS_DATA *comms);
bool COMMS_TransmitImageLine(COMMS_DATA *comms);
bool COMMS_TransmitImageDone(COMMS_DATA *comms);
bool COMMS_SPIWrite(COMMS_DATA *comms,uint32_t TXSize);
bool COMMS_NotifyReady(COMMS_DATA *comms);
uint32_t COMMS_WaitForImageReady(void);

#define CommsSPISlaveSelect()       LATECLR = 1<<9
#define CommsSPISlaveDeselect()     LATESET = 1<<9

#endif /* _COMMS_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

