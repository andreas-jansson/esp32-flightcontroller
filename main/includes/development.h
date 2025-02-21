#pragma once 


#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

void telemetry_task(void* args);

void init_telemetry_buffer();

RingbufHandle_t get_telemetry_handle();