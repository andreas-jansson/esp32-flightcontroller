#pragma once


#include "esp_check.h"


#include "radioController.h"
#include "mpu6050.h"
#include "dshot600.h"


typedef enum ToggleState{
    TOGGLE_LOW,
    TOGGLE_HIGH,
    TOGGLE_UNCHANGED,
}toggleState_t;


class Drone{

    static Drone* drone;

    // states
    bool isPaired{};
    bool isArmed{};
    bool isGrounded{};
    enum FlightMode mode{};

    Channel channel{};
    YawPitchRoll ypr{};

    RingbufHandle_t dmp_queue_handle{};
    RingbufHandle_t radio_queue_handle{};
    RingbufHandle_t telemetry_queue_handle{};
    RingbufHandle_t m_dshot_queue_handle{};

    bool motor1DirectionNormal{true};
    bool motor2DirectionNormal{true};
    bool motor3DirectionNormal{true};
    bool motor4DirectionNormal{true};


    float voltageLevel{};
    float amperageLevel{};
    float radioSignalStr{};
    float escTemperature{};
    uint32_t dmpFreq{};
    uint32_t radioFreq{};
    uint32_t loopFreq{};

    Drone(RingbufHandle_t dshot, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle);

    esp_err_t parse_channel_state(const Channel& channel);

    esp_err_t start_arm_process();
    esp_err_t start_dissarm_process();

    esp_err_t send_dshot_message(Dshot::DshotMessage& msg);
    esp_err_t send_telemetry_message(TelemetryData& msg);


    esp_err_t set_flight_mode(int value);

    esp_err_t verify_components_process();
    esp_err_t arming_process();

    bool verify_controller_data(Channel channel);
    bool verify_imu_data(YawPitchRoll ypr);

    esp_err_t send_beep();

    esp_err_t blink_led(uint16_t newChannelValue);

    esp_err_t get_imu_data(YawPitchRoll& newData, TickType_t ticks);
    esp_err_t get_radio_data(Channel& newData, TickType_t ticks);

    ToggleState did_channel_state_switch(uint16_t newChannelValue, uint8_t channelNr);

    int mapValue(int x, int in_min, int in_max, int out_min, int out_max);


    public:

    Drone(Drone &other) = delete;
    void operator=(const Drone &) = delete;
    static Drone *GetInstance(RingbufHandle_t dshot, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle);
    static Drone *GetInstance();

    void drone_task(void* args);
    RingbufHandle_t get_queue_handle(){return telemetry_queue_handle;}

    //manage state
    esp_err_t set_armed(int value);         // works
    esp_err_t get_armed(bool& armed);       // TODO
    esp_err_t set_grounded(bool grounded);  // TODO
    esp_err_t get_grounded(bool& grounded); // TODO

    // readings
    esp_err_t get_radio_pairing();     // TODO       
    esp_err_t get_battery_voltage();   // TODO
    esp_err_t get_battery_amperage();  // TODO

    esp_err_t set_motor_direction(enum Motor motorNum, enum MotorDirection direction);  // TODO
    esp_err_t get_motor_direction(enum Motor motorNum, enum MotorDirection& direction); // TODO

    
};
