#pragma once

#include <atomic>
#include <memory>
#include <mutex>

#include "esp_check.h"
#include "radioController.h"
#include "dshot600.h"
#include "imuIf.h"


typedef enum ToggleState{
    TOGGLE_LOW,
    TOGGLE_HIGH,
    TOGGLE_UNCHANGED,
}toggleState_t;


typedef enum MotorPosition{
    REAR_LEFT,
    REAR_RIGHT,
    FRONT_LEFT,
    FRONT_RIGHT,
}motorPos_t;

struct MotorLaneMapping{
    
    motor_type_t rearLeftlane{MOTOR3};
    motor_type_t rearRightlane{MOTOR1};
    motor_type_t frontLeftlane{MOTOR4};
    motor_type_t frontRightlane{MOTOR2};

};

struct FwVersion{
    uint8_t major{};
    uint8_t minor{};
    uint8_t patch{};
};

struct BatteryInfo{
    uint8_t cellCount;
    uint16_t capacityMah;
    // uint8_t voltage; legacy
    uint16_t mahDrawn;
    int16_t current;
    //uint8_t alerts;
    uint16_t voltage;
};

class Drone{

    static Drone* drone;
    static Channel channel;
    static Pid m_pid[3];
    static char* s_name;

    static DroneState m_state;
    MotorLaneMapping m_motorLaneMapping{};

    YawPitchRoll m_ypr1{};
    static RadioStatistics radio_statistics;

    Dshot600* m_dshotObj;
    std::vector<ImuIf*> m_mpuObj1;

    Dshot::DshotMessage m_prevSpeedMsg{};
    bool m_newImuData{};

    static FwVersion m_fwVersion;
    static BatteryInfo m_batInfo;

    const float m_maxPitch{45};
    const float m_minPitch{-45};
    const float m_maxYaw{45};
    const float m_minYaw{-45};
    const float m_maxRoll{45};
    const float m_minRoll{-45};
    const float m_deadzone{295 * 1.05}; // min throttle raw value: 295

    const uint16_t c_minThrottleValue{48}; // lowest value for dshot protocol
    const bool c_saftyParams{true}; //limits various values while testing


    std::mutex m_escTelemetryLock;
    std::atomic<uint8_t> m_telemetryReqIdx{};
    bool m_motorTelemetryRequest[Dshot::maxChannels];

    std::vector<RingbufHandle_t> m_dmp_queue_handle{};
    RingbufHandle_t m_dmp_queue_handle2{};
    CircularHandle_t m_radio_queue_handle{};
    CircularHandle_t m_radio_statistics_queue_handle{};
    CircularHandle_t m_telemetry_queue_handle{};
    CircularHandle_t m_dshot_queue_handle{};

    static QueueHandle_t m_pid_conf_sem;

    uint32_t m_dmpFreq{};
    uint32_t m_radioFreq{};
    uint32_t m_loopFreq{};

    bidi_telemetry_t m_bidiTelemetry{};

    uart_port_t uartNum{UART_NUM_1};
    QueueHandle_t uart_queue{};

    // delete this probably
    float m_battert_max_voltage{};
    float m_battert_max_mah{};
    float m_battert_max_wh{};

    Drone(
        Dshot600* dshot_obj, 
        std::vector<ImuIf*> mpu,
        CircularHandle_t radio_queue_handle,
        CircularHandle_t radio_statistics_queue_handle);

    static void pid_configure_task(void* args);

    esp_err_t parse_channel_state(const Channel& channel);

    esp_err_t start_arm_process();
    esp_err_t start_dissarm_process();

    esp_err_t send_dshot_message(Dshot::DshotMessage& msg);

    esp_err_t measure_current();
    esp_err_t send_telemetry_message(TelemetryData& msg);
    esp_err_t send_telemetry();
    esp_err_t signal_telemetry_request(Dshot::DshotMessage& msg, bool& newTelemetryReq);
    void set_esc_telemetry_data(uint8_t temperature, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t rpm, uint8_t motorPos);


    esp_err_t set_flight_mode(int value);
    esp_err_t set_mpu_calibration_mode(int value);

    esp_err_t verify_components_process();
    esp_err_t arming_process();

    esp_err_t calibrate_gyro();
    bool verify_controller_data(Channel channel);
    bool verify_imu_data(YawPitchRoll ypr);

    esp_err_t send_beep();
    esp_err_t toggle_motor_direction(); //not working 
    esp_err_t blink_led(uint16_t newChannelValue); //not working, no led?

    esp_err_t get_imu_data(int imuNr, YawPitchRoll& newData, TickType_t ticks);
    esp_err_t get_radio_data(Channel& newData, TickType_t ticks);
    esp_err_t get_radio_statistics(RadioStatistics &newData, TickType_t ticks);

    ToggleState did_channel_state_switch(uint16_t newChannelValue, uint8_t channelNr);

    uint16_t mapValue(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
    int mapValue(int x, int in_min, int in_max, int out_min, int out_max);
    float mapValue(int x, int in_min, int in_max, float out_min, float out_max);
    float mapValue2(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min, float out_max);

    void pid(float target, float current, Pid& pid);
    void get_new_speed(Dshot::DshotMessage& msg);
    void write_speed(Dshot::DshotMessage &msg);
    void get_speed(Dshot::DshotMessage &msg);

    uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed);
    uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);

    bool configure_pid();

    esp_err_t arm_drone(uint32_t value);   


    public:

    Drone(Drone &other) = delete;
    void operator=(const Drone &) = delete;
    static Drone *GetInstance(
        Dshot600* dshot_obj, 
        std::vector<ImuIf*> mpu,
        CircularHandle_t radio_queue_handle,
        CircularHandle_t radio_statistics_queue_handle
    );
    static Drone *GetInstance();

    esp_err_t set_motor_lane_mapping(const MotorLaneMapping motorMapping);

    void drone_task(void *args);

    void esc_telemetry_task(void* args);

    esp_err_t init_esc_telemetry_uart(int rxPin, int txPin, int baudrate);

    CircularHandle_t get_telemetry_handle(){return m_telemetry_queue_handle;}

    void set_battery_data(float max_mah, float max_voltage, float max_wh){
        m_battert_max_voltage = max_voltage;
        m_battert_max_mah = max_mah;
        m_battert_max_wh = max_wh;
    };

    static const char* get_name(){
        return s_name;
    }

    esp_err_t set_motor_direction(enum Motor motorNum, enum MotorDirection direction);  // TODO
    esp_err_t get_motor_direction(enum Motor motorNum, enum MotorDirection& direction); // TODO


    static void set_fw_version(uint8_t major, uint8_t minor, uint8_t patch);
    static void get_fw_version(uint8_t& major, uint8_t& minor, uint8_t& patch);
    static void get_osd_status();
    static void get_channel_values(Channel& ch);
    static void set_battery_status(uint8_t cellCount, uint16_t capacityMah);
    static void get_battery_status(BatteryInfo& info);
    static esp_err_t get_pid(Pid* pid, uint8_t size_t);
    static void get_radio_status(RadioStatistics& radio);

};
