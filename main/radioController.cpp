#include <radioController.h>
#include <cstring>

#include "esp_check.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "debug.h"


RadioController::RadioController(){

    constexpr int txPin = 32;
    constexpr int rxPin = 33;
    constexpr int rtsPin = UART_PIN_NO_CHANGE;
    constexpr int ctsPin = UART_PIN_NO_CHANGE;
    constexpr int uart_buffer_size = (512);
    uart_config_t uart_config;

    uart_config.baud_rate = 420000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    this->uartNum = UART_NUM_1;

    ESP_ERROR_CHECK(uart_param_config(this->uartNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(this->uartNum, txPin, rxPin, rtsPin, ctsPin));
    ESP_ERROR_CHECK(uart_driver_install(this->uartNum, uart_buffer_size, uart_buffer_size, 10, this->uart_queue, 0));

}   


esp_err_t RadioController::uart_write(uint8_t* data, uint8_t dataLength){
    esp_err_t status = 0;

    uart_write_bytes(this->uartNum, data, dataLength);

    return status;
}


esp_err_t RadioController::uart_read(uint8_t* data, size_t& dataLength){
    esp_err_t status = 0;
    print_debug(DEBUG_RADIO, DEBUG_ARGS, "%-33s: uartNum: %d dataLength: %u id: %d\n", __func__, this->uartNum, dataLength, this->id);

    ESP_ERROR_CHECK(uart_get_buffered_data_len(this->uartNum, &dataLength));

    dataLength = uart_read_bytes(this->uartNum, data, dataLength, 100);

    return status;
}


uint8_t reverseBits(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4; // Swap nibbles
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2; // Swap pairs
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1; // Swap individual bits
    return byte;
}


void RadioController::test(){

    int loops = 0;
    while(true){
        uint8_t data[512]{};
        size_t dataLength{}; 

        uart_read(data, dataLength);

        if(!dataLength){
            vTaskDelay(100/portTICK_PERIOD_MS);
            continue;
        }


        for(int i=0;i<dataLength;i++){

            if(data[i] == 0xc8){
                uint8_t frameLen = data[i+1];
                uint8_t type = data[i+2];
                //printf("[%d]sync: 0x%x  type: 0x%x frameLen: %u\n", i, data[i], type, frameLen);

                if(type == 0x16){

                    memcpy(&(this->channel), (void*)&data[i+3], 22);
                    printf("ch1: %-4u ch2: %-4u ch3: %-4u ch4: %-4u ch5: %-4u ch6: %-4u\n", channel.ch1, channel.ch2, channel.ch3, channel.ch4, channel.ch5, channel.ch6);
                    
                    
                    int16_t ch1 = (data[i+4] << 8 | data[i+3]) & 0b0000'0111'1111'1111;
                    int16_t ch2 = (data[i+6] << 8 | data[i+5]) & 0b0000'0111'1111'1111;
                    int16_t ch3 = (data[i+8] << 8 | data[i+7]) & 0b0000'0111'1111'1111;
                    int16_t ch4 = (data[i+10] << 8 | data[i+9]) & 0b0000'0111'1111'1111;
                    int16_t ch5 = (data[i+12] << 8 | data[i+11]) & 0b0000'0111'1111'1111;
                    int16_t ch6 = (data[i+14] << 8 | data[i+13]) & 0b0000'0111'1111'1111;

                    
                    printf("ch1: %-4u ch2: %-4u ch3: %-4u ch4: %-4u ch5: %-4u ch6: %-4u\n\n", ch1, ch2, ch3, ch4, ch5, ch6);

                    //printf("uart data[%d] startIdx[%d] stopIdx[%d] ", i, i+3, (i+frameLen-4));
                    //for(int j=(i+3); j<(i+frameLen-4); j++){
                    //    printf("0x%x ", data[j]);
                    //}   
                    //printf("\n");
                    //int16_t ch1 = (data[i+4] << 8 | data[i+3]) & 0b0000'0111'1111'1111;
                    //printf("channel1: %d\n", ch1);

                }

                i+=frameLen;
            }
            vTaskDelay(10/portTICK_PERIOD_MS);
        }

        vTaskDelay(100/portTICK_PERIOD_MS);

        loops++;
    }
}


double RadioController::get_altitude(){

    return 5.0;
}
