#pragma once

#include <map>
#include <string>
#include <memory>
#include <unordered_map>

#include "esp_check.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/uart.h"

#include "dshot600.h"
#include "common_data.h"
#include "i2c.h"
#include "ringbuffer.h"



namespace Dshot{

    static constexpr uint8_t maxChannels{4};
    static constexpr uint16_t throttleOffset{48}; 

    static constexpr uint16_t minThrottleValue{48}; 
    static constexpr uint16_t maxThrottleValue{2047}; 

    static constexpr uint16_t invalidThrottle{0xFFFF}; 

    typedef enum BeepNum{
        BEEP1,
        BEEP2,
        BEEP3,
        BEEP4,
        BEEP5,
    };

    typedef enum DshotMessageType{
        COMMAND,
        THROTTLE,
    }dshot_msg_type_t;

    struct DshotMessage{
        dshot_msg_type_t msgType{};
        uint16_t speed[Dshot::maxChannels]{};
        int32_t loops[Dshot::maxChannels]{};
        uint8_t cmd[Dshot::maxChannels]{};
    
        bool telemetryReq[Dshot::maxChannels]{};
        //bool writeTo[Dshot::maxChannels]{};
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

   

    struct rmt_dshot_esc_encoder_t {
    rmt_encoder_t base;
    rmt_encoder_handle_t bytes_encoder;
    rmt_encoder_handle_t copy_encoder;
    rmt_symbol_word_t dshot_delay_symbol;
    int state;
    gpio_num_t pin;    // <-- add pin here
    };
    
    typedef union {
        struct {
            uint16_t crc: 4;       /*!< CRC checksum */
            uint16_t telemetry: 1; /*!< Telemetry request */
            uint16_t throttle: 11; /*!< Throttle value */
        };
        uint16_t val;
    } dshot_esc_frame_t;

    typedef enum{
        RPM = 0x1,
        TEMP = 0x2,
        VOLT = 0x4,
        AMP = 0x6,
        DEBUG1 = 0x8,
        DEBUG2 = 0xA,
        DEBUG3 = 0xC,
        EVENT = 0xE,
    } bidi_telemetry_type_t;

    typedef struct {
        gpio_num_t motor_pin{};
        uint64_t sym[32] {};
        uint8_t num_symbols{};
        uint8_t total_us{};
        struct {
            float period_us{};
            int sym_lvl{};
        } bit_info[32];

        uint32_t gcr_data{};   // symbols decodede to gcr bits
        uint16_t data_bits{};  // data bits eeemmmmmmmmmcrc4
        bidi_telemetry_type_t msgType;
        uint16_t data{};
  
    } rx_symbol_t;

    inline std::map<uint16_t, uint16_t> unmapGcr = {
    {0x19,0}, 
    {0x1B,1}, 
    {0x12,2}, 
    {0x13,3},
    {0x1D,4},
    {0x15,5},
    {0x16,6},
    {0x17,7},
    {0x1A,8},
    {0x09,9},
    {0x0A,10},
    {0x0B,11},
    {0x1E,12},
    {0x0D,13},
    {0x0E,14},
    {0x0F,15}};

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


struct cb_dshot_ctx{
    rmt_channel_handle_t tx_handle;
    rmt_channel_handle_t rx_handle;
    gpio_num_t pin;
    motor_type_t motor;
    rmt_receive_config_t rx_chan_config;
};


class Dshot600{

    //CircularHandle_t m_dshot_queue_handle{};
    static Dshot600* dshot;
    static bool m_isBidi;

    rmt_sync_manager_handle_t synchro{};
    rmt_channel_handle_t esc_motor_chan[Dshot::maxChannels]{};
    rmt_channel_handle_t rmt_rx_handle[Dshot::maxChannels]{};
    rmt_encoder_handle_t dshot_encoder{};

    cb_dshot_ctx* m_ctx[Dshot::maxChannels]{};

    gpio_num_t m_gpioMotorPin[4]{};
    
    int c_cpuFreq{};
    float c_cycleToUs{};


    Dshot600(gpio_num_t motorPin[Dshot::maxChannels], bool isBidi);

    void read_dshot_telemetry();

    esp_err_t write_speed(struct Dshot::DshotMessage& msg);
    esp_err_t write_command(struct Dshot::DshotMessage& msg);
    esp_err_t read();

    /* helper */
    esp_err_t parse_dshot_message(struct Dshot::DshotMessage& msg);

    esp_err_t delay(struct Dshot::DshotMessage& msg);

    esp_err_t get_message(struct Dshot::DshotMessage& msg, TickType_t ticks);


    static size_t rmt_encode_dshot_esc(
        rmt_encoder_t *encoder, 
        rmt_channel_handle_t channel, 
        const void *primary_data, 
        size_t data_size, 
        rmt_encode_state_t *ret_state);

    esp_err_t rmt_new_dshot_esc_encoder(const Dshot::dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);
    static esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder);
    static esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder);
    static void make_dshot_frame(Dshot::dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry);
    static void make_bidi_dshot_frame(Dshot::dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry);

    int decode_timings_to_gcr(Dshot::rx_symbol_t& rx_sym);
    int extract_nibbles(Dshot::rx_symbol_t& rx_sym);
    int crc_4(uint16_t& word16);
    int decode_msg(Dshot::rx_symbol_t& rx_sym);
    int process_erpm_data(Dshot::rx_symbol_t& rx_sym);

    void attach_my_rmt_tx_isr();

    public:

    Dshot600(Dshot600 &other) = delete;
    void operator=(const Dshot600 &) = delete;
    static Dshot600 *GetInstance(gpio_num_t motorPin[Dshot::maxChannels], bool isBidi);
    static Dshot600 *GetInstance();

    esp_err_t set_speed(struct Dshot::DshotMessage& msg);

    void dshot_task(void* args);


    // Commands
    esp_err_t beep_cmd(enum Dshot::BeepNum num);
    esp_err_t blink_led_cmd();

    // not implemented 
    esp_err_t set_motor_spin_cmd(enum Motor motorNum, enum MotorDirection direction);
    esp_err_t set_motor_direction_cmd(enum Motor motorNum, enum MotorDirection direction);

    esp_err_t set_3d_mode_cmd(bool on);
    esp_err_t save_settings_cmd();

    esp_err_t set_extended_telemetry(bool enable);

};