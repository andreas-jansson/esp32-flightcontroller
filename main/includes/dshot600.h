#pragma once

#include <map>
#include <string>
#include <memory>

#include "esp_check.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "driver/rmt_tx.h"

#include "common_data.h"
#include "i2c.h"
 


typedef struct {
    uint32_t resolution;    /*!< Encoder resolution, in Hz */
    uint32_t baud_rate;     /*!< Dshot protocol runs at several different baud rates, e.g. DSHOT300 = 300k baud rate */
    uint32_t post_delay_us; /*!< Delay time after one Dshot frame, in microseconds */
} dshot_esc_encoder_config_t;




typedef struct {
    uint16_t throttle;  /*!< Throttle value */
    bool telemetry_req; /*!< Telemetry request */
} dshot_esc_throttle_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t dshot_delay_symbol;
    int state;
} rmt_dshot_esc_encoder_t;

typedef union {
    struct {
        uint16_t crc: 4;       /*!< CRC checksum */
        uint16_t telemetry: 1; /*!< Telemetry request */
        uint16_t throttle: 11; /*!< Throttle value */
    };
    uint16_t val;
} dshot_esc_frame_t;

class Dshot600{


    rmt_sync_manager_handle_t synchro{};
    rmt_channel_handle_t esc_motor_chan[4]{};


    rmt_encoder_handle_t dshot_encoder{};

    
    esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

    esp_err_t write(struct DshotMessage& msg, bool sendCommand);
    esp_err_t read();

    esp_err_t parse_dshot_message(struct DshotMessage& msg);


    public:
    Dshot600(gpio_num_t motorPin[Dshot::maxChannels]);

    esp_err_t set_speed(struct DshotMessage& msg);


    // Commands
    esp_err_t beep_cmd(enum BeepNum num);

    esp_err_t set_motor_spin_cmd(enum Motor motorNum, enum MotorDirection direction);

    esp_err_t set_3d_mode_cmd(bool on);
    esp_err_t save_settings_cmd();

    esp_err_t set_motor_direction_cmd(enum Motor motorNum, enum MotorDirection direction);



};