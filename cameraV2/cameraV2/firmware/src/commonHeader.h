/******************************************************************************/
/** commonHeader.h

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/******************************************************************************/

#ifndef COMMON_HEADERS_H    /* Guard against multiple inclusion */
#define COMMON_HEADERS_H

#define IMAGE_BUFFERS           (2) /* must be a power of 2 */
#define BUFFER_SIZE_32          (256)
#define BUFFER_SIZE_16          (BUFFER_SIZE_32*2)
#define BUFFER_SIZE_8           (BUFFER_SIZE_32*4)
#define VOSPI_HEADER_LENGTH     (4)
#define VOSPI_PAYLOAD_LENGTH    (160)
#define VOSPI_LENGTH            (VOSPI_HEADER_LENGTH+VOSPI_PAYLOAD_LENGTH)
#define DISCARD                 (0x0f)

#define HORIZONTAL_SIZE         (VOSPI_PAYLOAD_LENGTH>>1)
#define VERTICAL_SIZE           (60)


/******************************************************************************/

typedef uint16_t PIXEL_TYPE;

/******************************************************************************/

typedef union {
    PIXEL_TYPE vector[HORIZONTAL_SIZE*VERTICAL_SIZE];
    PIXEL_TYPE pixel[VERTICAL_SIZE][HORIZONTAL_SIZE];
} IMAGE_BUFFER_TYPE;

/******************************************************************************/

typedef struct {
    struct {
        uint16_t horizontal;
        uint16_t vertical;
    } dimensions;
    struct {
        uint16_t pixels;
        uint16_t bytes;
    }size;
}IMAGE_INFO_TYPE;

/******************************************************************************/

typedef struct {
    IMAGE_INFO_TYPE properties;
    IMAGE_BUFFER_TYPE buffer;
}FLIR_IMAGE_TYPE;

/******************************************************************************/

typedef struct __attribute__((packed)) {
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

/******************************************************************************/

typedef union __attribute__((packed)) {
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

typedef struct {
    uint32_t b8;
    uint32_t b16;
    uint32_t b32;
} BUFFER_SIZE_TYPE;


#endif /* COMMON_HEADERS_H */

/******************************************************************************/
/* End of File */
/******************************************************************************/
