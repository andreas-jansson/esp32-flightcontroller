#include "radiocontroller_stub.h"   

#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "debug.h"
#include "display.h"

#define log_tag "radio"

RadioController_stub* RadioController_stub::radio = nullptr;


RadioController_stub::RadioController_stub(){

   
}   

RadioController_stub* RadioController_stub::GetInstance(){
    if(radio==nullptr){
        radio = new RadioController_stub();
    }
    return radio;
}

esp_err_t RadioController_stub::uart_write(uint8_t* data, uint8_t dataLength){
    esp_err_t status = 0;

    return status;
}

esp_err_t RadioController_stub::uart_read(uint8_t* data, size_t& dataLength){
    esp_err_t status = 0;

    return status;
}

uint8_t reverseBits(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4; // Swap nibbles
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2; // Swap pairs
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1; // Swap individual bits
    return byte;
}

void RadioController_stub::radio_task(void* args){

    esp_err_t status = 0;

    while(true){

        uint8_t data[64]{};

        data[3] = 200;
        data[4] = 300;
        data[5] = 400;
        data[6] = 500;
        data[7] = 600;
        data[8] = 700;

        Display::set_radio_status(true);
        send_channel_to_queue((Channel*)data);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

bool RadioController_stub::isNewData(Channel* newChannel){

    return true;
}

esp_err_t RadioController_stub::send_channel_to_queue(void* newChannel){
    esp_err_t status = ESP_OK;

    // If data is the same, no need to update
    if (!isNewData(static_cast<Channel*>(newChannel+3))){
        return ESP_OK;
    }

    CircularBufEnqueue(radio_queue_handle, newChannel + 3);
    
    memcpy(&this->channel, newChannel + 3, 22);

    return ESP_OK;

}

esp_err_t RadioController_stub::set_channel_data(void* data) {
    esp_err_t status = ESP_OK;

    // If data is the same, no need to update
    if (memcmp(&this->channel, data + 3, 22) == 0)
        return ESP_OK;

    memcpy(&this->channel, data + 3, 22);

    return ESP_OK;
}

esp_err_t RadioController_stub::get_channel_data(Channel& data){
    esp_err_t status = ESP_OK;

    data = this->channel;

    return ESP_OK;
}

esp_err_t RadioController_stub::get_pitch(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch1;

    return ESP_OK;
}

esp_err_t RadioController_stub::get_roll(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch2;

    return ESP_OK;
}

esp_err_t RadioController_stub::get_speed(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch3;

    return ESP_OK;
}

esp_err_t RadioController_stub::get_yaw(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch4;

    return ESP_OK;
}