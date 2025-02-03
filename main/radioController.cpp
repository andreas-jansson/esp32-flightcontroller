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

                if(type == 0x16){
                    uint8_t tmp[22]{};
                    memcpy(&this->channel, (void*)&data[i+3], 22);
    
                    printf("ch1: %-4d ch2: %-4d ch3: %-4d ch4: %-4d ch5: %-4d ch6: %-4d\n", 
                    this->channel.ch1, this->channel.ch2, this->channel.ch3, this->channel.ch4, this->channel.ch5, this->channel.ch6);
                    
                }

                i+=frameLen;
            }
            vTaskDelay(2/portTICK_PERIOD_MS);
        }

        vTaskDelay(2/portTICK_PERIOD_MS);

        loops++;
    }
}


double RadioController::get_altitude(){

    return 5.0;
}
