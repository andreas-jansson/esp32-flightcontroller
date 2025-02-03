#pragma once

#include "esp_check.h"
#include "driver/uart.h"


class RadioController{
     uart_port_t uartNum{UART_NUM_2};
     int id{0x69};


     esp_err_t uart_write(uint8_t* data, uint8_t dataLength);
     esp_err_t uart_read(uint8_t* data, size_t& dataLength);

     public:
     RadioController();
     double get_altitude();
     void test();

};