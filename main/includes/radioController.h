#pragma once

#include "esp_check.h"
#include "driver/uart.h"

#include "common_data.h"

#include "ringbuffer.h"



class RadioController{
     static RadioController* radio;
     //RingbufHandle_t radio_queue_handle;
     CircularHandle_t radio_queue_handle; 
     uart_port_t uartNum{2};
     QueueHandle_t uart_queue{};
     Channel channel{};
     int id{0x69};

     SemaphoreHandle_t channelSem;


     RadioController();

     esp_err_t uart_write(uint8_t* data, uint8_t dataLength);
     esp_err_t uart_read(uint8_t* data, size_t& dataLength);

     esp_err_t set_channel_data(void* data);
     esp_err_t get_channel_data(void* data);

     esp_err_t send_channel_to_queue(void* newChannel);
     bool isNewData(Channel* newChannel);

     public:
     RadioController(RadioController &other) = delete;
     void operator=(const RadioController &) = delete;
     static RadioController *GetInstance();
     void radio_task(void* args);
     double get_altitude();

     CircularHandle_t get_queue_handle(){return radio_queue_handle;}
     esp_err_t get_channel_data(Channel& data);
     esp_err_t get_pitch(uint16_t& data);
     esp_err_t get_roll(uint16_t& data);
     esp_err_t get_speed(uint16_t& data);
     esp_err_t get_yaw(uint16_t& data);
     esp_err_t get_mode(uint16_t& data);


};