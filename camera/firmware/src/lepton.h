/* 
 * File:   lepton.h
 * Author: mjb
 *
 * Created on May 7, 2016, 6:18 PM
 */

#ifndef LEPTON_H
#define	LEPTON_H

typedef enum {
    GET = 0b00, /* Get a module property or attribute value */
    SET = 0b01, /* Set a module property or attribute value */
    RUN = 0b10  /* Run ? execute a camera operation exposed by that module */
} LEPTON_COMMAND_TYPE;

typedef enum {
    AGC = 0b0001,
    SYS = 0b0010,
    VID = 0b0011
} LEPTON_MODULE_ID_TYPE;

typedef union {
    uint16_t w;
    struct{
        LEPTON_COMMAND_TYPE commandType:2;
        unsigned commandID:6;
        LEPTON_MODULE_ID_TYPE moduleID:4;
        unsigned :4;
        
    };
} LEPTON_COMMAND_WORD_TYPE;

typedef enum /*Result */
{
    FLR_OK = 0,                             /* Camera ok                      */
    FLR_COMM_OK = FLR_OK,                   /* Camera comm ok (same as FLR_OK)*/
    FLR_ERROR = -1,                         /* Camera general error           */
    FLR_NOT_READY = -2,                     /* Camera not ready error         */
    FLR_RANGE_ERROR = -3,                   /* Camera range error             */
    FLR_CHECKSUM_ERROR = -4,                /* Camera checksum error          */
    FLR_BAD_ARG_POINTER_ERROR = -5,         /* Camera Bad argument error      */
    FLR_DATA_SIZE_ERROR = -6,               /* Camera byte count error        */
    FLR_UNDEFINED_FUNCTION_ERROR = -7,      /* Camera undefined function error*/
    FLR_FUNCTION_NOT_SUPPORTED_ERROR = -8,  /* Camera function not supported error*/
    FLR_DATA_OUT_OF_RANGE_ERROR = -9,       /* Camera Data out of range error */
    /* OTP access errors                                                      */
    FLR_OTP_WRITE_ERROR = -15,              /* Camera OTP write error         */
    FLR_OTP_SEC_READ_ERROR = -16,           /* single bit error detected (correctable)*/
    FLR_OTP_DED_READ_ERROR = -17,           /* single bit error detected (correctable)*/
    FLR_OTP_NOT_PROGRAMMED_ERROR = -18,     /* Flag read as non-zero          */
    /* Operation Errors                                                       */
    FLR_DIV_ZERO_ERROR= -80,                /* Attempted div by zero          */
    /* Communication Errors                                                   */
    FLR_COMM_PORT_NOT_OPEN = -101,          /* Comm port not open             */
    FLR_COMM_RANGE_ERROR= -102,             /* Comm port range error          */
    FLR_ERROR_CREATING_COMM= -103,          /* Error creating comm            */
    FLR_ERROR_STARTING_COMM = -104,         /* Error starting comm            */
    FLR_ERROR_CLOSING_COMM= -105,           /* Error closing comm             */
    FLR_COMM_CHECKSUM_ERROR = -106,         /* Comm checksum error            */
    FLR_COMM_NO_DEV = -107,                 /* No comm device                 */
    FLR_COMM_TIMEOUT_ERROR = -108,          /* Comm timeout error             */
    FLR_COMM_ERROR_WRITING_COMM = -109,     /* Error writing comm             */
    FLR_COMM_ERROR_READING_COMM = -110,     /* Error reading comm             */
    FLR_COMM_COUNT_ERROR = -111,            /* Comm byte count error          */
    /* Other Errors                                                           */
    FLR_OPERATION_CANCELED = -126,          /* Camera operation canceled      */
    FLR_UNDEFINED_ERROR_CODE = -127,        /* Undefined error                */
    FLR_END_ERROR_CODES
} FLR_RESULT_TYPE;

typedef union {
    uint16_t w;
    struct {
        unsigned busy:1;
        unsigned bootMode:1;
        unsigned bootStatus:1;
        unsigned :5; /* reserved area */
        union {
           int8_t b; 
           FLR_RESULT_TYPE responseErrorCode;
        };        
    };
} CCI_TWI_STATUS_TYPE;

#endif	/* LEPTON_H */

