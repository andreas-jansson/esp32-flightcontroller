#include <radioController.h>
#include <cstring>



#include "esp_check.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "debug.h"
#include "display.h"

#define log_tag "radio"



RadioController* RadioController::radio = nullptr;
uart_port_t RadioController::s_uartNum{2};






RadioController::RadioController(){

    constexpr int txPin = 32; // white
    constexpr int rxPin = 33; // green
    constexpr int rtsPin = UART_PIN_NO_CHANGE;
    constexpr int ctsPin = UART_PIN_NO_CHANGE;
    constexpr int uart_buffer_size = (512);
    uart_config_t uart_config;

    uart_config.baud_rate = 420000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;  

    s_uartNum = UART_NUM_2;

    ESP_ERROR_CHECK(uart_param_config(s_uartNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_uartNum, txPin, rxPin, rtsPin, ctsPin));
    ESP_ERROR_CHECK(uart_driver_install(s_uartNum, uart_buffer_size, uart_buffer_size, 10, &this->uart_queue, 0));

    channelSem = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((channelSem==nullptr));

    this->radio_queue_handle = CircularBufCreate(10, sizeof(channel), "radio");
    if (this->radio_queue_handle == NULL) {
        printf("Failed to create radio channel ring buffer\n");
    }

    this->radio_statistics_queue_handle = CircularBufCreate(10, sizeof(RadioStatistics), "radio_statistics");
    if (this->radio_statistics_queue_handle == NULL) {
        printf("Failed to create radio statistics ring buffer\n");
    }


    channel.ch1 = -1;
    channel.ch2 = -1;
    channel.ch3 = -1;
    channel.ch4 = -1;
    channel.ch5 = -1;
    channel.ch6 = -1;
    channel.ch7 = -1;
    channel.ch8 = -1;
    channel.ch9 = -1;
    channel.ch10 = -1;
    channel.ch11 = -1;
    channel.ch12 = -1;
    channel.ch13 = -1;
    channel.ch14 = -1;
    channel.ch15 = -1;
    channel.ch16 = -1;
}   

RadioController* RadioController::GetInstance(){
    if(radio==nullptr){
        radio = new RadioController();
    }
    return radio;
}

esp_err_t RadioController::uart_write(uint8_t* data, uint8_t dataLength){
    esp_err_t status = 0;

    uart_write_bytes(s_uartNum, data, dataLength);

    return status;
}

esp_err_t RadioController::uart_read(uint8_t* data, size_t& dataLength){
    esp_err_t status = 0;
    print_debug(DEBUG_RADIO, DEBUG_ARGS, "%-33s: uartNum: %d dataLength: %u id: %d\n", __func__, s_uartNum, dataLength, this->id);

    ESP_ERROR_CHECK(uart_get_buffered_data_len(s_uartNum, &dataLength));

    dataLength = uart_read_bytes(s_uartNum, data, dataLength, 100);

    return status;
}

uint8_t reverseBits(uint8_t byte) {
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4; // Swap nibbles
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2; // Swap pairs
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1; // Swap individual bits
    return byte;
}

void RadioController::radio_task(void* args){

    using namespace radio;

    esp_err_t status = 0;

    constexpr uint8_t flightControllerAddr{0xC8};
    constexpr uint8_t c_addrIdx{0};
    constexpr uint8_t c_frameLenIdx{1};
    constexpr uint8_t c_dataIdx{2};

    constexpr uint8_t c_frameTypeChannels{0x16};
    constexpr uint8_t c_frameTypeLinkStatistics{0x14};
    bool alertDisplayActive{};


    while(true){

        uint8_t data[512]{};
        size_t dataLength{};
        TickType_t lastSleep = xTaskGetTickCount();
        TickType_t sleepThld = pdMS_TO_TICKS(100);
        TickType_t sleepDur = pdMS_TO_TICKS(1); 

        uart_event_t event;
        if (xQueueReceive(uart_queue, (void *)&event, pdMS_TO_TICKS(100))){
            if (event.type == UART_DATA) {
                memset(data, 0, 512);
                dataLength = 0;
                status = uart_read(data, dataLength);

                if(!dataLength){
                    continue;
                }

    

                if(data[c_addrIdx] == flightControllerAddr){
                    uint8_t frameLen = data[c_frameLenIdx];
                    uint8_t type = data[c_dataIdx];

                    if(!alertDisplayActive){
                        alertDisplayActive = true;
                        Display::set_radio_status(alertDisplayActive);
                    }

                    if(type == c_frameTypeChannels){

                        send_channel_to_queue((Channel*)data);

                    }
                    else if(type == c_frameTypeLinkStatistics){
                        /*
                            printf("stats: up_RSSI_1: %-2udb up_RSSI_2: %-2udb up_quality: %u %% up_SNR: %-2ddb" \
                               "active_antenna_nr: %-2u rf_Mode: %-2x up_TX_Power: %-2x down_rssi: %-2udb down_quality: %-2udb down_snr: %-2ddb\n",
                            data[UPPLINK_RSSI_1], 
                            data[UPPLINK_RSSI_2], 
                            data[UPPLINK_QUALITY], 
                            data[UPPLINK_SNR], 
                            data[UPPLINK_ACTIVE_ANTENNA], 
                            data[RF_MODE], 
                            data[UPPLINK_TX_POWER],
                            data[DOWNLINK_RSSI],
                            data[DOWNLINK_QUALITY],
                            data[DOWNLINK_SNR]);
                        printf("sync[0x%x] len[%u] type[0x%x] data[0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x] crc[0x%x]", 
                            data[0], data[1], data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11],data[12],data[13]);
                        
                        printf("****************************************************\n");
                        */

                        send_statistics(data);
                    
   
 
                        // TODO send a specific channel message so that drone attempts to land safely.
                        // uplink_Link_quality: 0x0, detect loss of controller
                    }
                    else{
                        // only ever seen 0x16 & 0x14 messages
                        //printf("Frame type: 0x%x len: %u\n", type, frameLen);
                    }
                }
            }
        }
        else{
            if(alertDisplayActive){
                alertDisplayActive = false;
                Display::set_radio_status(alertDisplayActive);
            }
        }
    }
}

bool RadioController::isNewData(Channel* newChannel){

    if(newChannel->ch1 != this->channel.ch1){
        return true;
    }
    else if(newChannel->ch2 != this->channel.ch2){
        return true;
    }
    else if(newChannel->ch3 != this->channel.ch3){
        return true;
    }
    else if(newChannel->ch4 != this->channel.ch4){
        return true;
    }
    else if(newChannel->ch5 != this->channel.ch5){
        return true;
    }
    else if(newChannel->ch6 != this->channel.ch6){
        return true;
    }
    else if(newChannel->ch7 != this->channel.ch7){
        return true;
    }
    else if(newChannel->ch8 != this->channel.ch8){
        return true;
    }
    else if(newChannel->ch9 != this->channel.ch9){
        return true;
    }
    else if(newChannel->ch10 != this->channel.ch10){
        return true;
    }
    else if(newChannel->ch11 != this->channel.ch11){
        return true;
    }
    else if(newChannel->ch12 != this->channel.ch12){
        return false;   // not supported
    }
    else if(newChannel->ch13 != this->channel.ch13){
        return false;   // not supported
    }
    else if(newChannel->ch14 != this->channel.ch14){
        return false;   // not supported
    }
    else if(newChannel->ch15 != this->channel.ch15){
        return false;   // not supported
    }
    else if(newChannel->ch16 != this->channel.ch16){
        return false;   // not supported
    }   
    
    return false;
}

esp_err_t RadioController::send_channel_to_queue(void* newChannel){
    esp_err_t status = ESP_OK;
    static uint32_t bootCounter{};

    // send data regardless to allow hw check in drone task
    if(bootCounter < 10000){
        bootCounter++;
    }

    // comment back to only send new data - removed due to drone logic thinking it looses contact with radio
    //// If data is the same, no need to update
    //if (!isNewData(static_cast<Channel*>(newChannel+3)) && bootCounter >= 10000){
    //    return ESP_OK;
    //}

    CircularBufEnqueue(radio_queue_handle, newChannel + 3);
    memcpy(&this->channel, newChannel + 3, 22);


    return ESP_OK;

}

esp_err_t RadioController::send_statistics(void* data){
    esp_err_t status = ESP_OK;

    CircularBufEnqueue(radio_statistics_queue_handle, data + 3);

    return ESP_OK;

}


esp_err_t RadioController::set_channel_data(void* data) {
    esp_err_t status = ESP_OK;

    // If data is the same, no need to update
    if (memcmp(&this->channel, data + 3, 22) == 0)
        return ESP_OK;

    memcpy(&this->channel, data + 3, 22);

    return ESP_OK;
}

esp_err_t RadioController::get_channel_data(Channel& data){
    esp_err_t status = ESP_OK;

    data = this->channel;

    return ESP_OK;
}

esp_err_t RadioController::get_pitch(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch1;

    return ESP_OK;
}

esp_err_t RadioController::get_roll(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch2;

    return ESP_OK;
}

esp_err_t RadioController::get_speed(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch3;

    return ESP_OK;
}

esp_err_t RadioController::get_yaw(uint16_t& data){
    esp_err_t status = ESP_OK;

    data = channel.ch4;

    return ESP_OK;
}



esp_err_t RadioController::send_attitude(YawPitchRoll& ypr){
    esp_err_t status{};
    radio::AttitudeMsg msg{};

    msg.pitch = static_cast<int16_t>(ypr.pitch * 10000);
    msg.roll = static_cast<int16_t>(ypr.roll * 10000);
    msg.yaw = static_cast<int16_t>(ypr.yaw * 10000);

    uint8_t* crsf_frame = nullptr;
    ESP_ERROR_CHECK(set_telemetry_data(radio::CRSF_FRAMETYPE_ATTITUDE, reinterpret_cast<uint8_t*>(&msg), &crsf_frame));
    uart_write_bytes(s_uartNum, crsf_frame, sizeof(msg) + 4);

    free(crsf_frame);
    return 0;
}

int16_t swap_endian(int16_t val)
{
    return (int16_t)(((val & 0xFF00) >> 8) |
                     ((val & 0x00FF) << 8));
}

esp_err_t RadioController::send_battery_data(float voltage, float current, float used){
    esp_err_t status{};
    radio::BatteryMsg msg{};

    int16_t voltage_dv = static_cast<int16_t>(voltage * 10);
    int16_t current_da = static_cast<int16_t>(current * 10);
    uint8_t used_mah = used;
    int8_t percent_left = 75;

    msg.voltage_dV = swap_endian(voltage_dv);
    msg.current_dA = swap_endian(current_da);
    msg.used_mAh[2] = used;
    msg.battery_percent_left = percent_left;

    uint8_t* crsf_frame = nullptr;
    ESP_ERROR_CHECK(set_telemetry_data(radio::CRSF_FRAMETYPE_BATTERY_SENSOR, reinterpret_cast<uint8_t*>(&msg), &crsf_frame));
    uart_write_bytes(s_uartNum, crsf_frame, sizeof(msg) + 4);
    free(crsf_frame);
    return 0;
}

esp_err_t RadioController::set_telemetry_data(enum radio::crsfFrameType type, uint8_t *data, uint8_t **r_crsf_frame){

    if(!radio::crsfMsgSize.at(type)){
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t frame_size = radio::crsfMsgSize.at(type) + 4;
    uint8_t nr_bytes_crc = radio::crsfMsgSize.at(type) + 1;

    *r_crsf_frame = static_cast<uint8_t*>(calloc(1, frame_size));
    (*r_crsf_frame)[0] = 0xC8;
    (*r_crsf_frame)[1] = radio::crsfMsgSize.at(type) + 2;
    (*r_crsf_frame)[2] = type;

    for(uint8_t i=0;i<radio::crsfMsgSize[type];i++){
        (*r_crsf_frame)[i+3] = data[i];
    }

    uint8_t len = (*r_crsf_frame)[1]; // total = type + payload + crc
    uint8_t crc = crc8_dvb_s2(&(*r_crsf_frame)[2], len - 1);
    (*r_crsf_frame)[2 + len - 1] = crc;

    printf("sync[0x%x] len[0x%x] type[0x%x] data[", (*r_crsf_frame)[0], (*r_crsf_frame)[1], (*r_crsf_frame)[2]);
    for(int i=0;i<radio::crsfMsgSize[type];i++){
        printf("0x%x ", (*r_crsf_frame)[i+3]);
    }
    printf("] crc[0x%x]\n", (*r_crsf_frame)[frame_size - 1]);

    return 0;
}


uint8_t RadioController::crc8_dvb_s2(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    const uint8_t poly = 0xD5;

    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}