#pragma once


#include <iostream>


#include "esp_check.h"



class I2cHandler{
    uint8_t sdaPin{};
    uint8_t sclPin{};
    uint32_t freq{};
    bool initiated{};

    public:
    I2cHandler(uint8_t sdaPin, uint8_t sclPin, uint32_t freq);

    esp_err_t init();
    void i2c_scanner();
    bool is_initiated(){return this->initiated;};


    esp_err_t write(uint8_t devAddress, uint8_t startRegisterAddress, uint8_t* data, uint8_t dataLength);
    esp_err_t read(uint8_t devAddress, uint8_t startRegisterAddress, uint8_t* data, uint8_t dataLength);
    esp_err_t write_bit(uint8_t devAddress, uint8_t startRegisterAddress, uint8_t data, uint8_t mask);

};






