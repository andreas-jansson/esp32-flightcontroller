#pragma once


#include "esp_check.h"


#include "radioController.h"
#include "mpu6050.h"
#include "dshot600.h"





class Drone{

    static Drone* drone;
    Dshot600* dshot;

    // states
    bool isPaired{};
    bool isArmed{};
    bool isGrounded{};
    enum FlightMode mode{};

    Channel channel{};
    YawPitchRoll ypr{};

    RingbufHandle_t dmp_queue_handle;
    RingbufHandle_t radio_queue_handle;
    RingbufHandle_t telemetry_queue_handle;

    bool motor1DirectionNormal{true};
    bool motor2DirectionNormal{true};
    bool motor3DirectionNormal{true};
    bool motor4DirectionNormal{true};

    const int maxRadioValue{1811};
    const int minRadioValue{174};
    const int neutralRadioValue{997};

    float voltageLevel{};
    float amperageLevel{};
    float radioSignalStr{};
    float escTemperature{};
    uint32_t dmpFreq{};
    uint32_t radioFreq{};

    Drone(Dshot600* dshot, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle);

    esp_err_t parse_channel_state(const Channel& channel);

    esp_err_t start_arm_process();
    esp_err_t start_dissarm_process();

    esp_err_t set_flight_mode(int value);

    esp_err_t verify_components();

    bool verify_controller_data(Channel channel);

    bool verify_imu_data(YawPitchRoll ypr);


    public:

    Drone(RadioController &other) = delete;
    void operator=(const Drone &) = delete;
    static Drone *GetInstance(Dshot600* dshot, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle);
    static Drone *GetInstance();

    void drone_task(void* args);
    RingbufHandle_t get_queue_handle(){return telemetry_queue_handle;}

    //manage state
    esp_err_t set_armed(int value);
    esp_err_t get_armed(bool& armed);
    esp_err_t set_grounded(bool grounded);
    esp_err_t get_grounded(bool& grounded);

    // readings
    esp_err_t get_radio_pairing();
    esp_err_t get_battery_voltage();
    esp_err_t get_battery_amperage();

    esp_err_t set_motor_direction(enum Motor motorNum, enum MotorDirection direction);
    esp_err_t get_motor_direction(enum Motor motorNum, enum MotorDirection& direction);

    
};
