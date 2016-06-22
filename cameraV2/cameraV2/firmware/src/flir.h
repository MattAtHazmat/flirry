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
#include "LEPTON_SDK.h"

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
	FLIR_STATE_INIT=0,
    FLIR_OPEN_I2C_PORT,
    FLIR_OPEN_SPI_PORT,
            FLIR_START,
	FLIR_STATE_SERVICE_TASKS,
            FLIR_STATE_SUCCESS,
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
#define VOSPI_HEADER_LENGTH     (4)
#define VOSPI_PAYLOAD_LENGTH    (160)
#define VOSPI_LENGTH            (VOSPI_HEADER_LENGTH+VOSPI_PAYLOAD_LENGTH)
#define DISCARD                 (0x0f)
typedef uint16_t PIXEL_TYPE;
#define HORIZONTAL_SIZE         (VOSPI_PAYLOAD_LENGTH>>1)
#define VERTICAL_SIZE           (60)

typedef struct {
    struct {
        uint16_t horizontal;
        uint16_t vertical;
        struct {
            uint16_t pixels;
            uint16_t bytes;
        }size;
    }properties;
    union{
        PIXEL_TYPE vector[HORIZONTAL_SIZE*VERTICAL_SIZE];
        PIXEL_TYPE pixel[VERTICAL_SIZE][HORIZONTAL_SIZE];
    }image;
}FLIR_IMAGE_TYPE;

typedef struct __attribute__((packed)){
    uint16_t telemetryRevision;
    uint16_t uptime[2];
    uint16_t status[2];
    uint16_t serial[8];
    uint16_t RESERVED1[4];
    uint16_t frameCounter[2];
    uint16_t frameMean;
    uint16_t FPATemp;
    uint16_t housingTempCounts;
    uint16_t housingTempK;
    uint16_t RESERVED2[2];
    struct __attribute__((packed)){
        uint16_t FFATemp;
        uint16_t timeCounter[2];
        uint16_t housingTempK;
    }lastFFC;
    uint16_t RESERVED3;
    struct __attribute__((packed)){
        struct __attribute__((packed)){
            uint16_t top;
            uint16_t left;
            uint16_t bottom;
            uint16_t right;
        }ROI;
        struct __attribute__((packed)){
            uint16_t high;
            uint16_t low;
        }clipLimit;
    }AGC;
    uint16_t RESERVED4[34];
    uint16_t FFCFramesLog2;
    uint16_t RESERVED5[5];
}TELEMETRY_A;

typedef union __attribute__((packed)){
    uint8_t b8[VOSPI_LENGTH];
    uint16_t b16[VOSPI_LENGTH>>1];
    struct __attribute__((packed)){
        union __attribute__((packed)){
            uint16_t b16;
            uint8_t b8[2];
            struct __attribute__((packed)){
                unsigned:8;
                unsigned discard:4;
                unsigned MSBits:4;
            };
            struct __attribute__((packed)){
                unsigned line:12;
                unsigned:4;
            };
        }ID;
        union __attribute__((packed)){
            uint16_t b16;
            uint8_t b8[2];
        }CRC;
        union __attribute__((packed)){ 
            uint8_t b8[VOSPI_PAYLOAD_LENGTH];
            uint16_t b16[VOSPI_PAYLOAD_LENGTH>>1];
            TELEMETRY_A telemetry;
        }payload;
    };
}VOSPI_TYPE;

#define BUFFER_SIZE_32  (256)
#define BUFFER_SIZE_16  (BUFFER_SIZE_32*2)
#define BUFFER_SIZE_8   (BUFFER_SIZE_32*4)

typedef struct
{
    FLIR_STATES state;
    struct {
        DRV_HANDLE drvHandle;
    } timer;
    struct {
        DRV_HANDLE drvHandle;
        DRV_I2C_BUFFER_HANDLE bufferHandle;        
    } i2c;
    struct {
        DRV_HANDLE drvHandle;
        DRV_SPI_BUFFER_HANDLE bufferHandle;
        union{
            uint32_t w;
            struct{
                unsigned complete:1;
                unsigned started:1;
            }flags;
        }status;
    } spi;
    struct {
        LEP_RESULT result;
        LEP_CAMERA_PORT_DESC_T cameraPort;
    }lepton;
    struct{
        union{
            uint8_t b8[BUFFER_SIZE_8];
            uint16_t b16[BUFFER_SIZE_16];
            uint32_t b32[BUFFER_SIZE_32];
        };  
        struct{
            struct {
                uint32_t b8;
                uint32_t b16;
                uint32_t b32;
            } max;
            struct {
                uint32_t b8;
                uint32_t b16;
                uint32_t b32;
            } transfer;
        }size;
    }TXBuffer;
    struct{
        union{
            uint8_t b8[BUFFER_SIZE_8];
            uint16_t b16[BUFFER_SIZE_16];
            uint32_t b32[BUFFER_SIZE_32];
        };  
        struct{
            struct {
                uint32_t b8;
                uint32_t b16;
                uint32_t b32;
            } max;
            struct {
                uint32_t b8;
                uint32_t b16;
                uint32_t b32;
            } transfer;
        }size;
        struct {
            uint16_t calculated;
            uint16_t packet;
        }CRC;
    }RXBuffer;
    union {
        uint32_t w;
        struct {
            unsigned timerConfigured:1;
            unsigned timerRunning:1;
            unsigned I2CConfigured:1;
            unsigned I2CRunning:1;
            unsigned I2CConfigureAttempted:1;
            unsigned usingI2C:1;
            unsigned SPIConfigured:1;
            unsigned SPIRunning:1;
            unsigned SPIConfigureAttempted:1;
            unsigned usingSPI:1;
        }flags;
    }status;
    VOSPI_TYPE VoSPI;
    FLIR_IMAGE_TYPE image;
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

void FLIR_Initialize ( void );


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

bool OpenFLIRSPI(FLIR_DATA *flir);
bool StartFLIRSPIReading(FLIR_DATA *flir,uint32_t size);
bool GetFLIRSPIReading(FLIR_DATA *flir);
bool FLIRSPIWriteRead(FLIR_DATA *flir,uint32_t TXSize,int32_t RXSize);
bool FLIRGetVoSPI(FLIR_DATA *flir);
#define FLIRSPISlaveSelect()       LATGCLR = 1<<9
#define FLIRSPISlaveDeselect()     LATGSET = 1<<9
bool FLIRPopulateLine(FLIR_DATA *flir);

#endif /* _FLIR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

