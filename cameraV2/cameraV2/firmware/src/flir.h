// <editor-fold defaultstate="collapsed" desc="SLA">
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
 *******************************************************************************/// </editor-fold>


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

typedef enum
{
	/* Application's state machine's initial state. */
	FLIR_STATE_INIT=0,
    //FLIR_OPEN_I2C_PORT,
    FLIR_OPEN_SPI_PORT,
    FLIR_START,
	FLIR_STATE_SERVICE_TASKS,
    FLIR_STATE_START_FRAME,
    FLIR_STATE_GET_LINE,
            FLIR_STATE_IMAGE_COMPLETE,
            FLIR_STATE_TRANSMIT_IMAGE,
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
        DRV_HANDLE drvHandle;
    } timer;
    struct {
        DRV_HANDLE drvHandle;
        DRV_I2C_BUFFER_HANDLE bufferHandle;        
    } i2c;
    struct {
        DRV_HANDLE drvHandle;
        DRV_SPI_BUFFER_HANDLE bufferHandle;
        struct{
            unsigned complete:1;
            unsigned started:1;
        }status;
    } spi;
    struct {
        LEP_RESULT result;
        LEP_CAMERA_PORT_DESC_T cameraPort;
    }lepton;
    struct __attribute__((packed)){
        union{
            uint8_t b8[BUFFER_SIZE_8];
            uint16_t b16[BUFFER_SIZE_16];
            uint32_t b32[BUFFER_SIZE_32];
        };  
        struct{
            BUFFER_SIZE_TYPE max;
            BUFFER_SIZE_TYPE transfer;
        }size;
    }TXBuffer;
    struct __attribute__((packed)){
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
        unsigned buildingFrame:1;
        unsigned transmittingFrame:1;
        unsigned lineReady:1;
        unsigned imageReady:1;
        unsigned discardLimit:1;
        unsigned restartFrame:1;
        unsigned frameRestarted:1;
    }status;
    VOSPI_TYPE VoSPI;
    struct {
        int32_t building;
        int32_t transmitting;
        int32_t buildingLine;
        uint32_t discardCount;
        uint32_t discardCountLimit;
    } frame;
    FLIR_IMAGE_TYPE image;
    struct {
        TaskHandle_t myHandle;
        TaskHandle_t commsHandle;
    }RTOS;  
    struct {
        uint32_t imagesTransmitted;
    }counters;
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
bool FLIRTransmitImage(FLIR_DATA *flir);
inline bool FLIRDiscardLine(FLIR_DATA *flir);
#endif /* _FLIR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

