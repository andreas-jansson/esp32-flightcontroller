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
    QueueHandle_t uart_queue;
    uart_config_t uart_config;

    uart_config.baud_rate = 420000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
    uart_config.rx_flow_ctrl_thresh = 32;

    this->uartNum = UART_NUM_2;

    ESP_ERROR_CHECK(uart_param_config(this->uartNum, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(this->uartNum, txPin, rxPin, rtsPin, ctsPin));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(this->uartNum, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

}   


esp_err_t RadioController::uart_write(uint8_t* data, uint8_t dataLength){
    esp_err_t status = 0;

    uart_write_bytes(this->uartNum, data, dataLength);

    return status;
}


esp_err_t RadioController::uart_read(uint8_t* data, size_t& dataLength){
    esp_err_t status = 0;
    print_debug(DEBUG_RADIO, DEBUG_ARGS, "%-33s: uartNum: %d dataLength: %u id: %d\n", __func__, this->uartNum, dataLength, this->id);

    ESP_ERROR_CHECK(uart_get_buffered_data_len(2, &dataLength));
    print_debug(DEBUG_RADIO, DEBUG_ARGS, "%-33s: uart buffer len: %u\n", __func__, dataLength);

    dataLength = uart_read_bytes(2, data, dataLength, 100);

    return status;
}

void RadioController::test(){

    //while(true){
    //    printf(" %d\n", gpio_get_level(static_cast<gpio_num_t>(33)));
    //    vTaskDelay(1/portTICK_PERIOD_MS);
    //}

    while(true){
        uint8_t data[512]{};
        size_t dataLength{}; 

        uart_read(data, dataLength);

        if(!dataLength)
            continue;

        printf("uart data:");
        for(int i=0;i<dataLength;i++){
            for(int j=1;j<=16;j++){

                printf("channel %-2d 0x%x\n",j, data[i]);
            }
            printf("[16A");
        }
        vTaskDelay(10/portTICK_PERIOD_MS);


    }
}


double RadioController::get_altitude(){

    return 5.0;
}
