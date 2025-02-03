#pragma once

#include "esp_check.h"
#include "driver/uart.h"



typedef struct __attribute__((packed)){
    unsigned ch1  : 11;
    unsigned ch2  : 11;
    unsigned ch3  : 11;
    unsigned ch4  : 11;
    unsigned ch5  : 11;
    unsigned ch6  : 11;
    unsigned ch7  : 11;
    unsigned ch8  : 11;
    unsigned ch9  : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    unsigned ch16 : 11;
} Channel;



class RadioController{
     uart_port_t uartNum{UART_NUM_2};
     QueueHandle_t* uart_queue{};
     Channel channel{};
     int id{0x69};



     esp_err_t uart_write(uint8_t* data, uint8_t dataLength);
     esp_err_t uart_read(uint8_t* data, size_t& dataLength);

     esp_err_t get_channel_data(uint16_t& data);

     public:
     RadioController();
     double get_altitude();
     void test();

};