
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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


#include <xc.h>                 /* Defines special funciton registers, CP0 regs  */
#include "system_config.h"
#include "system_definitions.h"
#include "system/debug/sys_debug.h"
#include "bsp_config.h"
#include "lepton.h"

LEP_RESULT LEP_OpenPort(LEP_UINT16 portID,
    LEP_CAMERA_PORT_E portType,
LEP_UINT16 portBaudRate,
LEP_CAMERA_PORT_DESC_T_PTR portDescPtr )
/* portID -
User defined value to identify a specific comm port.
Useful when multiple cameras are attached to a single Host. */
/* portBaudRate ? Port-specific Units: kHz.
 Supported TWI: 400
Supported SPI: 20000 max (20 MHz) */
/* Lepton physical transport interfaces */
typedef enum LEP_CAMERA_PORT_E_TAG {
    LEP_CCI_TWI=0,
    LEP_CCI_SPI,
    LEP_END_CCI_PORTS
}LEP_CAMERA_PORT_E, *LEP_CAMERA_PORT_E_PTR;
/* Communications Port Descriptor Type */
typedef struct LEP_CAMERA_PORT_DESC_T_TAG
{
    LEP_UINT16 portID;
    LEP_CAMERA_PORT_E portType;
    LEP_UINT16 portBaudRate;
}LEP_CAMERA_PORT_DESC_T, *LEP_CAMERA_PORT_DESC_T_PTR;

