#pragma once

#include "esp_check.h"
#include "driver/uart.h"

#include "common_data.h"

#include "ringbuffer.h"

/* https://github.com/crsf-wg/crsf/wiki/Packet-Types */




namespace radio{
    enum statistics{
        UPPLINK_RSSI_1 = 3,
        UPPLINK_RSSI_2,
        UPPLINK_QUALITY,
        UPPLINK_SNR,
        UPPLINK_ACTIVE_ANTENNA,
        RF_MODE,
        UPPLINK_TX_POWER,
        DOWNLINK_RSSI ,
        DOWNLINK_QUALITY,
        DOWNLINK_SNR
    };

    enum crsfFrameType{
        CRSF_FRAMETYPE_GPS = 0x02,
        CRSF_FRAMETYPE_VARIO = 0x07,
        CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
        CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
        CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
        CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
        CRSF_FRAMETYPE_RADIO_ID = 0x3A,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
        CRSF_FRAMETYPE_LINK_RX_ID = 0x1C,
        CRSF_FRAMETYPE_LINK_TX_ID = 0x1D,
        CRSF_FRAMETYPE_ATTITUDE = 0x1E,
        CRSF_FRAMETYPE_FLIGHT_MODE = 0x21, // Extended Header Frames, range: 0x28 to 0x96
        CRSF_FRAMETYPE_DEVICE_PING = 0x28,
        CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
        CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
        CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
        CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
        CRSF_FRAMETYPE_COMMAND = 0x32, // KISS frames
        CRSF_FRAMETYPE_KISS_REQ = 0x78,
        CRSF_FRAMETYPE_KISS_RESP = 0x79, // MSP commands
        CRSF_FRAMETYPE_MSP_REQ = 0x7A,
        CRSF_FRAMETYPE_MSP_RESP = 0x7B,
        CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // Ardupilot frames
        CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
    };

    enum crsfPayloadSize{
        CRSF_SIZE_GPS = 15,
        CRSF_SIZE_BATTERY_SENSOR = 8,
        CRSF_SIZE_BARO_ALTITUDE = 4,
        CRSF_SIZE_LINK_RX_ID = 5,
        CRSF_SIZE_LINK_TX_ID = 6,
        CRSF_SIZE_ATTITUDE = 6,
        CRSF_SIZE_FLIGHT_MODE = 14, //null nerminated string, max strlen 13
    };

    inline std::map<uint8_t, uint8_t> crsfMsgSize{
        {CRSF_FRAMETYPE_GPS, 15},
        {CRSF_FRAMETYPE_BATTERY_SENSOR, 8},
        {CRSF_FRAMETYPE_BARO_ALTITUDE, 4},
        {CRSF_FRAMETYPE_LINK_RX_ID, 5},
        {CRSF_FRAMETYPE_LINK_TX_ID, 6},
        {CRSF_FRAMETYPE_ATTITUDE, 6},
        {CRSF_FRAMETYPE_FLIGHT_MODE, 14} //null nerminated string, max strlen 13
    };

/* [sync] [len] [type] [payload] [crc8] */

    struct __attribute__((packed))CrsfFrame{
        uint8_t sync;
        uint8_t len;
        uint8_t type;
        void* payload;
        uint8_t crc;
    };

    struct __attribute__((packed))AttitudeMsg{
        uint16_t pitch;
        uint16_t roll;
        uint16_t yaw;
    };

    struct __attribute__((packed))BatteryMsg{
        int16_t voltage_dV;  //big endian e.g. 25.2V sent as 0x00FC / 252
        int16_t current_dA;  //big endian 18.9A sent as 0x00BD / 189
        uint8_t used_mAh[3]; //int24_t e.g. 2199mAh used sent as 0x0897 / 2199
        int8_t battery_percent_left; // e.g. 100% full battery sent as 0x64 / 100
    };

    struct __attribute__((packed))BaroMsg{
        uint16_t altitude; // If the high bit is not set, altitude value is in decimeters + 10000, allowing altitudes of -1000.0m to 2276.7m
                           // 10m sent as (100 + 10000) = 0x2774 / 10100
                           // -10m sent as (-100 + 10000) = 0x26AC / 9900
                           // If the high bit is set, altitude value is in meters, allowing altitudes of 0m - 32767m
                           // 10m sent as (10 | 0x8000) = 0x800A / 32768
                           
        int16_t vertical_speed; // (optional) in cm/s (e.g. 1.5m/s sent as 150)
    };

    struct __attribute__((packed))LinkRxIdMsg{
        uint8_t downlink_rssi_dbm; // (positivized?)
        uint8_t downlink_rssi_percent; 
        uint8_t downlink_link_quality;
        int8_t downlink_snr;
        uint8_t uplink; // (sic) RF power index
    };

    struct flightModeMsg{
        char mode[14];
    };
}




class RadioController{
     static RadioController* radio;
     CircularHandle_t radio_queue_handle; 
     CircularHandle_t radio_statistics_queue_handle; 

     static uart_port_t s_uartNum;
     QueueHandle_t uart_queue{};
     Channel channel{};
     int id{0x69};

     SemaphoreHandle_t channelSem;


     RadioController();

     esp_err_t uart_write(uint8_t* data, uint8_t dataLength);
     esp_err_t uart_read(uint8_t* data, size_t& dataLength);

     esp_err_t set_channel_data(void* data);
     esp_err_t get_channel_data(void* data);

     esp_err_t send_channel_to_queue(void* newChannel);
     esp_err_t send_statistics(void* data);

     esp_err_t send_data_controller(uint16_t& data);


     bool isNewData(Channel* newChannel);

     static uint8_t crc8_dvb_s2(const uint8_t *data, size_t len);
     static esp_err_t set_telemetry_data(enum radio::crsfFrameType type, uint8_t *data, uint8_t **r_crsf_frame);

     static int16_t swap_endian(int16_t val)
     {
         return (int16_t)(((val & 0xFF00) >> 8) |
                          ((val & 0x00FF) << 8));
     }

     static uint16_t swap_endian(uint16_t val)
     {
         return (uint16_t)(((val & 0xFF00) >> 8) |
                          ((val & 0x00FF) << 8));
     }


     public:
     RadioController(RadioController &other) = delete;
     void operator=(const RadioController &) = delete;
     static RadioController *GetInstance();
     void radio_task(void* args);
     double get_altitude();

     CircularHandle_t get_queue_handle(){return radio_queue_handle;}
     CircularHandle_t get_statistics_queue_handle(){return radio_statistics_queue_handle;}

     esp_err_t get_channel_data(Channel& data);
     esp_err_t get_pitch(uint16_t& data);
     esp_err_t get_roll(uint16_t& data);
     esp_err_t get_speed(uint16_t& data);
     esp_err_t get_yaw(uint16_t& data);
     esp_err_t get_mode(uint16_t& data);

     static esp_err_t send_attitude(YawPitchRoll& ypr);
     static esp_err_t send_battery_data(float voltage, float current, float used);

};