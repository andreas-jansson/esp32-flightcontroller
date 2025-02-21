
#include <iostream>
#include <chrono>
#include <cstring>

#include "i2c.h"
#include "debug.h"
#include "common_data.h"


#include "driver/i2c.h"
#include "esp_check.h"

#define log_tag "i2c"


#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number (I2C_NUM_0 or I2C_NUM_1)
#define I2C_MASTER_FREQ_HZ 400000 // Frequency of the I2C bus
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

I2cHandler::I2cHandler(uint8_t sdaPin, uint8_t sclPin, uint32_t freq){
    this->sdaPin = sdaPin;
    this->sclPin = sclPin;
    this->freq = freq;
}


esp_err_t I2cHandler::init(){
    esp_err_t status = 0;
    i2c_config_t conf = {};

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = this->sdaPin;
    conf.scl_io_num = this->sclPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = this->freq;
    
    //     uart_config.source_clk = UART_SCLK_APB;  

    // Apply the configuration
    status = i2c_param_config(I2C_MASTER_NUM, &conf);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to configure I2C");

    // Install the I2C driver
    status = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    ESP_RETURN_ON_ERROR(status, log_tag, "FFailed to install I2C driver");

    initiated = true;

    i2c_scanner();

    return status;
}


void I2cHandler::i2c_scanner() {
    uint64_t elapsed{};
    for (uint8_t addr = 1; addr < 127; addr++) {
        auto start = std::chrono::steady_clock::now();
        esp_err_t status = i2c_master_write_to_device(I2C_MASTER_NUM, addr, &addr, 1, 1000);
        auto end = std::chrono::steady_clock::now();
        elapsed += std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        if (status == ESP_OK) {
            print_debug(DEBUG_I2C, DEBUG_DATA, "I2C device found at address 0x%02X\n", addr);
        }
    }
    printf("i2c avg write bytes too %llu us\n", elapsed / 126);
}



esp_err_t I2cHandler::write(uint8_t devAddress, uint8_t startRegisterAddress, uint8_t* data, uint8_t dataLength){


    ESP_RETURN_ON_ERROR((dataLength>20), log_tag, "Failed i2c write data: addr 0x%x reg 0x%x len %u", devAddress, startRegisterAddress, dataLength);
    esp_err_t status = 0;
    uint8_t buffer[20] = {};

    buffer[0] = startRegisterAddress;
    std::memcpy(&buffer[1], data, (size_t)dataLength);

    if(dataLength == 0)
        print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, "%-33s: addr 0x%.2x starAddr 0x%.2x %-25s dataLength %-2u\n", __func__, devAddress, startRegisterAddress, regMapX[(enum MpuReg)startRegisterAddress].c_str(), dataLength);
    else
        print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, "%-33s: addr 0x%.2x starAddr 0x%.2x %-25s dataLength %-2u data:", __func__, devAddress, startRegisterAddress, regMapX[(enum MpuReg)startRegisterAddress].c_str(), dataLength);
    
    for(int i=0;i<dataLength;i++){
        print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, " 0x%x", data[i]);
    }
    print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, "\n");

    //swap_endian(data, dataLength);

    for(int i=0;i<3;i++){
        status = i2c_master_write_to_device(I2C_MASTER_NUM, devAddress, buffer, dataLength + 1, 1000);
        if (status == ESP_OK)
            break;
    }
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed i2c write data: addr 0x%x reg 0x%x len %u", devAddress, startRegisterAddress, dataLength);
    
    return status;

}



esp_err_t I2cHandler::read(uint8_t devAddress, uint8_t startRegisterAddress, uint8_t* data, uint8_t dataLength){
    print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, "%-33s: addr 0x%x starAddr 0x%x %-25s dataLength %-2u data:", __func__, devAddress, startRegisterAddress, regMapX[(enum MpuReg)startRegisterAddress].c_str(), dataLength);

    esp_err_t status = i2c_master_write_to_device(I2C_MASTER_NUM, devAddress, &startRegisterAddress, 1, 1000);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed i2c write addr 0x%.2x reg 0x%.2x len %-2u", devAddress, startRegisterAddress, dataLength);


    for(int i=0;i<3;i++){
        status = i2c_master_read_from_device(I2C_MASTER_NUM, devAddress, data, dataLength, 1000);
        if (status == ESP_OK)
            break;
    }
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed i2c read data: addr 0x%.2x reg 0x%.2x len %-2u", devAddress, startRegisterAddress, dataLength);


    for(int i=0;i<dataLength;i++){
        print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, " 0x%x", data[i]);
    }
    print_debug(DEBUG_I2C, DEBUG_LOWLEVEL, "\n");

    return status;
}

esp_err_t I2cHandler::write_bit(uint8_t devAddress, uint8_t startRegisterAddress, uint8_t data, uint8_t mask){
    print_debug(DEBUG_I2C, DEBUG_ARGS, "%-33s: reg  0x%x data 0x%x mask 0x%x\n", __func__, startRegisterAddress, data, mask);
    esp_err_t status = 0;
    uint8_t buffer = 0;

    status = read(devAddress, startRegisterAddress, &buffer, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", devAddress, startRegisterAddress);

    //shift data by index of lowest 1 value in mask 
    uint8_t shiftAmount = __builtin_ctz(mask);
    data = (data << shiftAmount) & mask;

    buffer &= ~mask; 
    data &= mask; 
    data |= buffer; 
    status = write(devAddress, startRegisterAddress, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to write to addr 0x%x reg 0x%x data 0x%x", devAddress, startRegisterAddress , data);

    return status;
}

