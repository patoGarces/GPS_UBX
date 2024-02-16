#ifndef __GPS_UBX_H__
#define __GPS_UBX_H__

#include "stdio.h"

enum{
    NO_FIX,
    DEAD_RECKONING_ONLY,
    TWO_D_FIX,
    THREE_D_FIX,
    GPS_AND_DEAD_RECKONING,
    TIME_ONLY_FIX
};

typedef struct{
    uint32_t timeOfWeek;
    int32_t timeFractional;
    int16_t weekNumber;
    uint8_t gpsFix;
    uint8_t statusFlag;
    int32_t ecefPosX;
    int32_t ecefPosY;
    int32_t ecefPosZ;
    uint32_t posAccuracy;
    int32_t ecefVelX;
    int32_t ecefVelY;
    int32_t ecefVelZ;
    uint32_t speedAccuracy;
    uint16_t posDOP;
    uint8_t reserved1;
    uint8_t numSV;
    uint32_t reserved2;
}nav_sol_data_t;

typedef struct{
    uint32_t timeOfWeek;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed3D;
    uint32_t speed2D;
    int32_t heading;
    uint32_t speedAccuracy;
    uint32_t courseHeadingAccuracy;
}nav_velned_data_t;

typedef struct{
    uint32_t timeOfWeek;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t heightAtSeaLevel;
    uint32_t horizontalAccurary;
    uint32_t VerticalAccuracy;
}nav_posllh_data_t;

void gpsUbxInit(uint8_t _uartPort,uint32_t _baudrate,uint8_t _rxPin,uint8_t _txPin);
#endif
