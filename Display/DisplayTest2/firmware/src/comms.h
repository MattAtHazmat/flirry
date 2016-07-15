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
#define COMMS_BUFFER_SIZE       (0x100)
#define COMMS_BUFFER_SIZE_8     COMMS_BUFFER_SIZE
#define COMMS_BUFFER_SIZE_16    (COMMS_BUFFER_SIZE_8>>1)
#define COMMS_BUFFER_SIZE_32    (COMMS_BUFFER_SIZE_8>>2)
#define IMAGE_INFO_ID           (0xA5)
#define IMAGE_LINE_ID           (0x5A)
#define IMAGE_DONE_ID           (0x55)

#define IMAGE_INFO_LINE_VALUE   (0xFFFF)
#define IMAGE_DONE_LINE_VALUE   IMAGE_INFO_LINE_VALUE
    
#define ID_LOCATION             (0)
#define LENGTH_LSB_LOCATION     (ID_LOCATION+1)
#define LENGTH_MSB_LOCATION     (LENGTH_LSB_LOCATION+1)
#define LENGTH_LOCATION         (LENGTH_LSB_LOCATION)
#define LINE_LSB_LOCATION       (LENGTH_MSB_LOCATION+1)
#define LINE_MSB_LOCATION       (LINE_LSB_LOCATION+1)
#define LINE_LOCATION           (LINE_LSB_LOCATION)
#define DATA_START_LOCATION     (LINE_MSB_LOCATION+1)
#define MESSAGE_HEADER_LENGTH   DATA_START_LOCATION
#define HORIZONTAL_SIZE         (80)
#define VERTICAL_SIZE           (60)

#define IMAGE_INFO_LENGTH       (MESSAGE_HEADER_LENGTH + sizeof(IMAGE_INFO_TYPE))
#define IMAGE_LINE_LENGTH       (MESSAGE_HEADER_LENGTH + (HORIZONTAL_SIZE * sizeof(FLIR_PIXEL_TYPE)))
#define IMAGE_DONE_LENGTH       MESSAGE_HEADER_LENGTH
#define NUMBER_WORKING_BUFFERS  2
#define WORKING_BUFFERS_MASK    (NUMBER_WORKING_BUFFERS-1)    
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
    
    typedef uint16_t FLIR_PIXEL_TYPE;
typedef struct __attribute__((packed)) {
    uint32_t b8;
    uint32_t b16;
    uint32_t b32;
} BUFFER_SIZE_TYPE;

typedef struct __attribute__((packed))  {
    struct __attribute__((packed)) {
        uint16_t horizontal;
        uint16_t vertical;
    } dimensions;
    struct __attribute__((packed)) {
        uint16_t pixels;
        uint16_t bytes;
    }size;
}IMAGE_INFO_TYPE;

typedef union __attribute__((packed)) {
    FLIR_PIXEL_TYPE vector[HORIZONTAL_SIZE*VERTICAL_SIZE];
    FLIR_PIXEL_TYPE pixel[VERTICAL_SIZE][HORIZONTAL_SIZE];
} IMAGE_BUFFER_TYPE;

typedef struct __attribute__((packed)) {
    IMAGE_INFO_TYPE properties;
    IMAGE_BUFFER_TYPE buffer;
}FLIR_IMAGE_TYPE;

typedef struct __attribute__((packed)){
    uint8_t ID;
    uint16_t length;
    uint16_t line;
}PACKET_HEADER_TYPE;

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
    //COMMS_STATE_INCOMING_CAMERA_START_SPI_RX,
    //COMMS_STATE_INCOMING_CAMERA_CLEAN_UP,
	COMMS_STATE_SERVICE_TASKS,
    COMMS_STATE_INCOMING_CAMERA_DATA,
    COMMS_STATE_INCOMING_CAMERA_HEADER,
    COMMS_STATE_INCOMING_CAMERA_LINE,
    COMMS_STATE_INCOMING_CAMERA_IMAGE_COMPLETE,    
    COMMS_STATE_PROCESS_IMAGE,
    COMMS_STATE_DELIVER_IMAGE,
    COMMS_STATE_ERROR
} COMMS_STATES;



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
    struct __attribute__((packed)) {
        DRV_HANDLE drvHandle;
        DRV_SPI_BUFFER_HANDLE bufferHandle;
        struct __attribute__((packed)) {
            unsigned dataReady:1;
            unsigned readStarted:1;            
            unsigned running:1;
            unsigned configured:1;
            unsigned error:1;
            unsigned overrunError:1;
            unsigned unknownError:1;
            unsigned :25;
        }status;            
    } spi;
    struct __attribute__((aligned(4))) {
        union {
            uint8_t   b8[COMMS_BUFFER_SIZE_8];
            uint16_t b16[COMMS_BUFFER_SIZE_16];
            uint32_t b32[COMMS_BUFFER_SIZE_32];            
            struct __attribute__((packed)) {
                PACKET_HEADER_TYPE header;
                union {
                    IMAGE_INFO_TYPE imageInfo;
                    FLIR_PIXEL_TYPE pixel[(COMMS_BUFFER_SIZE_8-sizeof(PACKET_HEADER_TYPE))/sizeof(FLIR_PIXEL_TYPE)];
                    uint8_t raw[COMMS_BUFFER_SIZE_8-sizeof(PACKET_HEADER_TYPE)];
                } dataStructure;
            }packet;
        } incoming;
        struct {
            uint32_t line;
            uint32_t length;
        }fromPacket;        
    } workingBuffer[NUMBER_WORKING_BUFFERS];
    struct __attribute__((packed)) {
        BUFFER_SIZE_TYPE max;
        BUFFER_SIZE_TYPE transfer;
        uint32_t dataStructureRaw;
        uint32_t lineLength;
    }bufferSize;
    struct {
        uint32_t filled;
        uint32_t filling;
    } bufferIndex;
    struct __attribute__((packed)) {
        uint32_t linesReceived;
        struct {
            unsigned initialized:1;
            unsigned imageStartReceived:1;
            unsigned imageComplete:1;
            unsigned dataReadStarted:1;
            unsigned mysteryState:1;
            unsigned :27;
        } flags;
        uint8_t lineList[80];
    }status;
    FLIR_IMAGE_TYPE image;    
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

#define mBitClear(a,b)              (a ## CLR = 1<<b)
#define mBitSet(a,b)                (a ## SET = 1<<b)
#define mBitToggle(a,b)             (a ## INV = 1<<b)

#define mToggleJ10p35()             mBitToggle(LATA,14)

#endif /* _COMMS_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

