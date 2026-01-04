#pragma once

#include "common_data.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"




class VtxIf{

    protected:
   

    public:
    VtxIf() = default;

    virtual esp_err_t send_data(uint8_t* buf, size_t len) = 0;
    virtual esp_err_t get_data(uint8_t* buf, size_t len) = 0;
    virtual void vtx_task(void* args) = 0;
};