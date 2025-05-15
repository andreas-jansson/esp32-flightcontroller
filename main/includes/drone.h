#pragma once

#include <atomic>
#include <memory>
#include <mutex>


#include "esp_check.h"


#include "radioController.h"
#include "mpu6050.h"
#include "dshot600.h"


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
    
    motor_type_t rearLeftlane{MOTOR1};
    motor_type_t rearRightlane{MOTOR2};
    motor_type_t frontLeftlane{MOTOR3};
    motor_type_t frontRightlane{MOTOR4};

};




class Drone{

    static Drone* drone;

    const float m_maxPitch{45};
    const float m_minPitch{-45};
    const float m_maxYaw{45};
    const float m_minYaw{-45};
    const float m_maxRoll{45};
    const float m_minRoll{-45};

    const uint16_t c_minThrottleValue{100}; //sending lower may cause strange motor behaviour
    const bool c_saftyParams{true}; //limists various values while testing

    DroneState m_state{};
    MotorLaneMapping m_motorLaneMapping{};

    std::mutex m_escTelemetryLock;

    std::atomic<uint8_t> m_telemetryReqIdx{};
   

    Channel channel{};
    YawPitchRoll ypr{};

    Pid m_pid[3]{};

    RingbufHandle_t dmp_queue_handle{};
    CircularHandle_t radio_queue_handle{};
    CircularHandle_t telemetry_queue_handle{};
    RingbufHandle_t m_dshot_queue_handle{};

    bool motor1DirectionNormal{true};
    bool motor2DirectionNormal{true};
    bool motor3DirectionNormal{true};
    bool motor4DirectionNormal{true};

    bool motorTelemetryRequest[Dshot::maxChannels];

    float m_amperageLevel{};
    float m_radioSignalStr{};

    uint32_t dmpFreq{};
    uint32_t radioFreq{};
    uint32_t loopFreq{};

    uart_port_t uartNum{1};
    QueueHandle_t uart_queue{};


    Drone(
        RingbufHandle_t dshot, 
        RingbufHandle_t dmp_queue_handle, 
        CircularHandle_t radio_queue_handle);


    esp_err_t parse_channel_state(const Channel& channel);

    esp_err_t start_arm_process();
    esp_err_t start_dissarm_process();

    esp_err_t send_dshot_message(Dshot::DshotMessage& msg);
    esp_err_t send_telemetry_message(TelemetryData& msg);
    esp_err_t send_telemetry();
    esp_err_t signal_telemetry_request(Dshot::DshotMessage& msg, bool& newTelemetryReq);



    esp_err_t set_flight_mode(int value);

    void set_esc_telemetry_data(uint8_t temperature, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t rpm, uint8_t motorPos);

    esp_err_t verify_components_process();
    esp_err_t arming_process();

    bool verify_controller_data(Channel channel);
    bool verify_imu_data(YawPitchRoll ypr);

    esp_err_t send_beep();
    esp_err_t toggle_motor_direction(); //not working 

    esp_err_t blink_led(uint16_t newChannelValue); //not working, no led?

    esp_err_t get_imu_data(YawPitchRoll& newData, TickType_t ticks);
    esp_err_t get_radio_data(Channel& newData, TickType_t ticks);

    esp_err_t measure_current();

    ToggleState did_channel_state_switch(uint16_t newChannelValue, uint8_t channelNr);

    uint16_t mapValue(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
    int mapValue(int x, int in_min, int in_max, int out_min, int out_max);
    float mapValue(int x, int in_min, int in_max, float out_min, float out_max);
    float mapValue2(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min, float out_max);

    void pid(float target, float current, Pid& pid);
    void get_new_speed(Dshot::DshotMessage& msg, bool& didChange);

    esp_err_t ramp_up_motors();

    uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed);
    uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen);


    public:

    Drone(Drone &other) = delete;
    void operator=(const Drone &) = delete;
    static Drone *GetInstance(
        RingbufHandle_t dshot, 
        RingbufHandle_t dmp_queue_handle, 
        CircularHandle_t radio_queue_handle);
    static Drone *GetInstance();

    esp_err_t set_motor_lane_mapping(const MotorLaneMapping motorMapping);

    void drone_task(void *args);
    void esc_telemetry_task(void* args);

    esp_err_t init_uart(int rxPin, int txPin, int baudrate);


    CircularHandle_t get_telemetry_handle(){return telemetry_queue_handle;}

    //manage state
    esp_err_t set_armed(uint32_t value);    // works


    // readings
    esp_err_t get_radio_pairing();          // TODO       
    esp_err_t get_battery_voltage();        // TODO
    esp_err_t get_battery_amperage();       // TODO

    esp_err_t set_motor_direction(enum Motor motorNum, enum MotorDirection direction);  // TODO
    esp_err_t get_motor_direction(enum Motor motorNum, enum MotorDirection& direction); // TODO


};
