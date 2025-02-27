#pragma once

#include <map>
#include <string>
#include <memory>
#include <unordered_map>

#include "esp_check.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "driver/rmt_tx.h"

#include "common_data.h"
#include "i2c.h"
 


namespace Dshot{

    static constexpr uint8_t maxChannels{4};
    static constexpr uint16_t throttleOffset{48}; 

    static constexpr uint16_t minThrottleValue{48}; 
    static constexpr uint16_t maxThrottleValue{2047}; 

    typedef enum BeepNum{
        BEEP1,
        BEEP2,
        BEEP3,
        BEEP4,
        BEEP5,
    };

    struct DshotMessage{
    
        uint16_t speed[Dshot::maxChannels];
        int32_t loops[Dshot::maxChannels]{};
        uint8_t cmd[Dshot::maxChannels]{};
    
        bool telemetryReq[Dshot::maxChannels]{};
        bool writeTo[Dshot::maxChannels]{};
    };



    /* Data types for borrowed logic */

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

}

/* https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#special-commands */
// "Need 6x means send the command 6 times, security mechanism"
enum DSHOTCODES{
    DSHOT_CMD_MOTOR_STOP,                                  // Currently not implemented
    DSHOT_CMD_BEEP1,                                       // Wait at least length of beep (260ms) before next command                
    DSHOT_CMD_BEEP2,                                       // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP3,                                       // Wait at least length of beep (260ms) before next command                                        
    DSHOT_CMD_BEEP4,                                       // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP5,                                       // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_ESC_INFO,                                    // Wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_1,                            // Need 6x
    DSHOT_CMD_SPIN_DIRECTION_2,                            // Need 6x
    DSHOT_CMD_3D_MODE_OFF,                                 // Need 6x              
    DSHOT_CMD_3D_MODE_ON,                                  // Need 6x
    DSHOT_CMD_SETTINGS_REQUEST,                            // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,                               // Need 6x, wait at least 35ms before next command
    DSHOT_EXTENDED_TELEMETRY_ENABLE,                       // Need 6x (only on EDT enabed firmware)
    DSHOT_EXTENDED_TELEMETRY_DISABLE,                      // Need 6x (only on EDT enabed firmware)
    NOT_KNOWN1,
    NOT_KNOWN2,
    NOT_KNOWN3,
    NOT_KNOWN4,
    NOT_KNOWN5,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL,                       // Need 6x
    DSHOT_CMD_SPIN_DIRECTION_REVERSED,                     // Need 6x
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    AUDIO_STREAM_MODE,                                      // Currently not implemented
    SILENT_MODE,                                            // Currently not implemented
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE,                // Need 6x. Disables commands 42 to 47
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE,                 // Need 6x. Enables commands 42 to 47
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY,        // Need 6x. Enables commands 42 to 47 and sends erpm if normal Dshot frame
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY, // Need 6x. Enables commands 42 to 47 and sends erpm period if normal Dshot frame
    NOT_KNOWN6,
    NOT_KNOWN7,
    NOT_KNOWN8,
    NOT_KNOWN9,
    NOT_KNOWN10,
    NOT_KNOWN11,
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY,            // 1°C per LSB
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,                // 10mV per LSB, 40.95V max
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,                // 100mA per LSB, 409.5A max
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,            // 10mAh per LSB, 40.95Ah max
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,                   // 100erpm per LSB, 409500erpm max
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY             // 16us per LSB, 65520us max TBD
};



class Dshot600{

    RingbufHandle_t m_dshot_queue_handle{};
    static Dshot600* dshot;

    rmt_sync_manager_handle_t synchro{};
    rmt_channel_handle_t esc_motor_chan[4]{};
    rmt_encoder_handle_t dshot_encoder{};

    Dshot600(gpio_num_t motorPin[Dshot::maxChannels]);

    esp_err_t rmt_new_dshot_esc_encoder(const Dshot::dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

    esp_err_t write_speed(struct Dshot::DshotMessage& msg);
    esp_err_t write_command(struct Dshot::DshotMessage& msg);
    esp_err_t read();

    /* helper */
    esp_err_t parse_dshot_message(struct Dshot::DshotMessage& msg);


    esp_err_t delay(struct Dshot::DshotMessage& msg);


    esp_err_t get_message(struct Dshot::DshotMessage& msg, TickType_t ticks);
    bool is_command(struct Dshot::DshotMessage& msg, int i);
    bool is_speed(struct Dshot::DshotMessage& msg, int i);

    public:

    Dshot600(Dshot600 &other) = delete;
    void operator=(const Dshot600 &) = delete;
    static Dshot600 *GetInstance(gpio_num_t motorPin[Dshot::maxChannels]);
    static Dshot600 *GetInstance();

    void dshot_task_old(void* args);
    void dshot_task(void* args);

    RingbufHandle_t get_queue_handle(){return m_dshot_queue_handle;}

    esp_err_t set_speed(struct Dshot::DshotMessage& msg);


    // Commands
    esp_err_t beep_cmd(enum Dshot::BeepNum num);
    esp_err_t blink_led_cmd();
    esp_err_t arm_esc_cmd();

    // not implemented 
    esp_err_t set_motor_spin_cmd(enum Motor motorNum, enum MotorDirection direction);
    esp_err_t set_motor_direction_cmd(enum Motor motorNum, enum MotorDirection direction);

    esp_err_t set_3d_mode_cmd(bool on);
    esp_err_t save_settings_cmd();




};