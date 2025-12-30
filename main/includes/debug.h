#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif




enum DebugFileLevel{
    DEBUG_MAIN      = 0x1,
    DEBUG_BMP       = 0x2,
    DEBUG_RADIO     = 0x4,
    DEBUG_DISPLAY   = 0x8,
    DEBUG_MPU6050   = 0x10,
    DEBUG_I2C       = 0x20,
    DEBUG_TELEMETRY = 0x40,
    DEBUG_DSHOT     = 0x80,
    DEBUG_DRONE     = 0x100,
    DEBUG_DJIO4PRO  = 0x200,
};

enum DebugPrio{
    DEBUG_LOWLEVEL = 0x1,
    DEBUG_LOGIC    = 0x2,
    DEBUG_ARGS     = 0x4,
    DEBUG_DATA     = 0x8
};

void print_debug(enum DebugFileLevel a_file, enum DebugPrio a_prio, const char *format,  ...);

void set_loglevel(uint32_t a_file, uint32_t a_prio);

#ifdef __cplusplus
}
#endif

#endif
