#pragma once

#include "common_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


enum MpuRotation{
	ROTATE_0,
	ROTATE_90,
	ROTATE_180,
	ROTATE_270
};

enum YawPitchRollEnum{
	YAW,
	PITCH,
	ROLL,
};

class ImuIf{

    protected:
    virtual esp_err_t set_chip_rotation(enum MpuRotation rotation) = 0;
    virtual esp_err_t send_ypr_data(YawPitchRoll ypr) = 0;

    public:
    ImuIf() = default;
    virtual RingbufHandle_t get_queue_handle() = 0;
    virtual void calibrate_mpu() = 0;
    virtual esp_err_t set_dmp_enabled(bool enable);
    virtual void dmp_task(void* args);

};