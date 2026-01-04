#pragma once

#include "esp_check.h"
#include "driver/uart.h"

#include "common_data.h"

#include "ringbuffer.h"
#include "mspData.h"

#include "vtxIf.h"
#include  <atomic>


/*
 *  MSP data format
 *
 *      0            1            2           3         4             5           5 + size  
 * ----------------------------------------------------------------------------------------
 * |            |           |  < to fc  |          |           |               |          |          
 * |     $      |    M/X    |  > to vtx |   size   |    cmd    |   payload..n  |    CRC   |          
 * | start sym  |   v1/v2   |  ! error  |          |           |               |          |          
 * ----------------------------------------------------------------------------------------
 * 
 * */


class DjiO4Pro : public VtxIf{


    CircularHandle_t m_ringBufHandle; 
    uart_port_t s_uartNum{UART_NUM_1};
    QueueHandle_t uart_queue{};
    std::atomic<bool> msp_recieved{}; 

    esp_err_t uart_write(uint8_t* data, uint8_t dataLength);
    esp_err_t uart_read(uint8_t* data, size_t& dataLength);
    esp_err_t msp_write(Msp::cmd_e cmd, uint8_t* data, uint8_t dataLength);
    esp_err_t msp_read(uint8_t* data, size_t& dataLength);
    uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
    uint8_t crc8_v1(uint8_t cmd, uint8_t* payload, uint8_t len);
    esp_err_t extract_payload(const uint8_t* data, uint8_t len, uint8_t** payload, uint8_t& pLen);

    void vtx_task(void* args);

    void parse_data(const uint8_t* buf, size_t len);
    void parse_msg(Msp::cmd_e cmd, const uint8_t* buf, uint8_t len);


    esp_err_t respond_status();
    esp_err_t respond_status_ex();
    esp_err_t respond_rc_channel();
    esp_err_t respond_analog();
    esp_err_t respond_rc_tuning();
    esp_err_t respond_pid();
    esp_err_t respond_battery_status();
    esp_err_t respond_fc_version();

    esp_err_t respond_name();
    esp_err_t respond_filter_config();
    esp_err_t respond_pid_advanced();

    esp_err_t send_dp_heartbeat();
    


    esp_err_t set_canvas(const uint8_t* buf, size_t len);
    esp_err_t init_displayport();

    public:
    
    DjiO4Pro();

    esp_err_t send_data(uint8_t* msg, size_t len);
    esp_err_t get_data(uint8_t* msg, size_t len);
};