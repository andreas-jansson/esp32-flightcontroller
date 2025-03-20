#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <chrono>
#include <atomic>
#include <chrono>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "radioController.h"
#include "debug.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "dshot600.h"
#include "i2c.h"
#include "quaternion.h"
#include "esp_crsf.h"
#include "webClient.h"
#include "development.h"
#include "drone.h"


// uncommen to add back tasks
#define WEB_TASK
#define TELEMETRY_TASK


// FIXME remove when not needed
#define I2C_MASTER_SCL_IO 22      // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 21      // GPIO number for I2C SDA
#define I2C_MASTER_FREQ_HZ 400000 // Frequency of the I2C bus
#define I2C_DMP_DATA_PIN 27

#define UART_ESC_RX_IO 37
#define UART_ESC_TX_IO 17   // not used
#define UART_ESC_BAUDRATE 115200

#define log_tag "main"

SemaphoreHandle_t cfg_btn_sem = nullptr;

extern "C"
{
    void app_main(void);
}

static void IRAM_ATTR cfg_btn_handler(void *args)
{
    xSemaphoreGive(cfg_btn_sem);
}



void web_task(void *args)
{

    RingbufHandle_t ringBuffer_dmp{};
    constexpr TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    esp_err_t status = 0;

    std::string ssid{"Ubiquity 2"};
    std::string wpa{"#SupderDuper66!"};
    std::string ip{"192.168.1.246"};

    Client *client = Client::GetInstance(ssid, wpa, ip, 6669);
    client->init(ringBuffer_dmp, nullptr);

    while (true)
    {

        vTaskDelay(xDelay);
    }
}

void altitude_task(void *args){
    constexpr TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    constexpr TickType_t xDelayShort = 10 / portTICK_PERIOD_MS;
    float altitudeHypsometricSum = 0;
    float altitudeHypsometric = 0;
    float altitudeHypsometricDefault = 0;
    constexpr int nrSamples = 100;
    esp_err_t status = 0;
    sensors_t sensorsData = {};

    uint8_t deviceAddress = bmp280_init();
    if (deviceAddress == NONE_DETECTED)
    {
        printf("NONE_DETECTED\n");
        for (;;)
        {
            vTaskDelay(xDelay);
        }
    }

    status = bmp280_set_mode(deviceAddress, MODE_NORMAL);
    vTaskDelay(xDelayShort * 10);

    for (int i = 0; i < nrSamples; i++)
    {
        sensorsData = {};
        altitudeHypsometric = 0;

        status = bmp280_get_all(deviceAddress, sensorsData);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        status = bmp280_calculate_altitude_hypsometric(sensorsData.pressure, sensorsData.temperature, altitudeHypsometric);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);
        print_debug(DEBUG_MAIN, DEBUG_DATA, "default[%i] = %f m\n", i, altitudeHypsometric);
        altitudeHypsometricSum += altitudeHypsometric;
    }
    altitudeHypsometricDefault = altitudeHypsometricSum / nrSamples;
    print_debug(DEBUG_MAIN, DEBUG_DATA, "altitudeHypsometricDefault = %f m\n\n", altitudeHypsometricDefault);
    vTaskDelay(xDelay * 5);

    for (;;)
    {

        ////////////// Altitude measurements //////////////
        sensorsData = {};
        altitudeHypsometricSum = 0;

        for (int i = 0; i < nrSamples; i++)
        {
            sensorsData = {};
            altitudeHypsometric = 0;

            status = bmp280_get_all(deviceAddress, sensorsData);
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);

            status = bmp280_calculate_altitude_hypsometric(sensorsData.pressure, sensorsData.temperature, altitudeHypsometric);
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);

            altitudeHypsometricSum += altitudeHypsometric;
        }
        altitudeHypsometric = altitudeHypsometricSum / nrSamples;

        print_debug(DEBUG_MAIN, DEBUG_DATA, "Temperature                          = %f °C\n", sensorsData.temperature);
        print_debug(DEBUG_MAIN, DEBUG_DATA, "Pressure                             = %u Pa\n", sensorsData.pressure);
        print_debug(DEBUG_MAIN, DEBUG_DATA, "Altitude(altitudeHypsometric)        = %f m\n", altitudeHypsometric);
        print_debug(DEBUG_MAIN, DEBUG_DATA, "Altitude(altitudeHypsometricDefault) = %f m\n", altitudeHypsometricDefault);
        print_debug(DEBUG_MAIN, DEBUG_DATA, "Altitude(relative)                   = %.2f m\n", altitudeHypsometric - altitudeHypsometricDefault);

        vTaskDelay(xDelay);
    }
}

void crsf_task(void *args){
    crsf_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 32,
        .rx_pin = 33};
    CRSF_init(&config);

    crsf_channels_t channels = {0};
    crsf_battery_t battery = {0};
    crsf_gps_t gps = {0};
    while (1)
    {

        CRSF_receive_channels(&channels);
        printf("Channel 1: %d Channel 2: %d Channel 3: %d Channel 4: %d Channel 5: %d Channel 6: %d\n",
               channels.ch1, channels.ch2, channels.ch3, channels.ch4, channels.ch5, channels.ch6);

        battery.voltage = 120;   // voltage 10*V
        battery.current = 100;   // current 10*A
        battery.capacity = 1000; // capacity
        battery.remaining = 50;  // remaining % of battery

        CRSF_send_battery_data(CRSF_DEST_FC, &battery);

        gps.latitude = 42.4242 * 10000000;  // 42.4242 deg
        gps.longitude = 56.5656 * 10000000; // 56.5656 deg
        gps.altitude = 5 + 1000;            // 5m
        gps.groundspeed = 420;              // 42km/h
        gps.heading = 90 * 100;             // 90 deg NOT WORKING WELL NOW
        gps.satellites = 8;                 // 8 satellites

        CRSF_send_gps_data(CRSF_DEST_FC, &gps);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void dispatch_radio(void *args)
{
    RadioController *radio = RadioController::GetInstance();
    radio->radio_task(nullptr);
}

void dispatch_dmp(void *args)
{
    Mpu6050 *mpu = Mpu6050::GetInstance();
    mpu->dmp_task(nullptr);
}

void dispatch_raw(void *args)
{
    Mpu6050 *mpu = Mpu6050::GetInstance();
    mpu->raw_task(nullptr);
}

void dispatch_webClient(void *args)
{
    Client *client = Client::GetInstance();
    client->web_task2(nullptr);
}

void dispatch_drone(void *args)
{
    Drone *drone = Drone::GetInstance();
    drone->drone_task(nullptr);
}

void dispatch_dshot(void *args)
{
    Dshot600 *dshot = Dshot600::GetInstance();
    dshot->dshot_task(nullptr);
}

void dispatch_esc_telemetry(void *args){
    Drone *drone = Drone::GetInstance();
    drone->esc_telemetry_task(nullptr);
}

void main_task(void *args)
{
    TaskHandle_t altitude_handle{};
    TaskHandle_t dmp_handle{}; 
    TaskHandle_t raw_handle{}; 
    TaskHandle_t radio_handle{}; 
    TaskHandle_t web_handle{}; 
    TaskHandle_t telemetry_handle{}; 
    TaskHandle_t drone_handle{};
    TaskHandle_t esc_telemetry_handle{};

    RingbufHandle_t ringBuffer_dmp{};
    RingbufHandle_t ringBuffer_radio{};
    RingbufHandle_t ringBuffer_web{};
    RingbufHandle_t ringBuffer_telemetry{};
    RingbufHandle_t ringBuffer_dshot{};



    /******* radio *******/
    RadioController* radio = RadioController::GetInstance();
    ringBuffer_radio = radio->get_queue_handle();



    /******* I2C setup *******/
    I2cHandler *i2c = new I2cHandler(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    i2c->init();



    /******* MPU setup *******/
    Mpu6050 *mpu = Mpu6050::GetInstance(ADDR_68, i2c);
    ringBuffer_dmp = mpu->get_queue_handle();



    /******* wifi *******/
    #ifdef WEB_TASK
    std::string ssid{"Ubiquity 2"};
    std::string wpa{"#SupderDuper66!"};
    std::string ip{"192.168.1.247"};

    init_telemetry_buffer();
    ringBuffer_web = get_telemetry_handle();
    Client *client = Client::GetInstance(ssid, wpa, ip, 6669);
    client->init(ringBuffer_dmp, ringBuffer_web);
    #endif



    /******* Dshot600 *******/
    gpio_num_t motorPin[Radio::maxChannels]{};
    motorPin[MOTOR1] = static_cast<gpio_num_t>(15);
    motorPin[MOTOR2] = static_cast<gpio_num_t>(2);
    motorPin[MOTOR3] = static_cast<gpio_num_t>(12);
    motorPin[MOTOR4] = static_cast<gpio_num_t>(13);

    Dshot600* dshot = Dshot600::GetInstance(motorPin);
    ringBuffer_dshot = dshot->get_queue_handle();




    /*******  Drone *******/
    MotorLaneMapping motorLanes{
        .rearLeftlane = MOTOR1,
        .rearRightlane = MOTOR2,
        .frontLeftlane = MOTOR3,
        .frontRightlane = MOTOR4
    };

    Drone* drone = Drone::GetInstance(ringBuffer_dshot, ringBuffer_dmp, ringBuffer_radio);
    drone->init_uart(UART_ESC_RX_IO, UART_ESC_TX_IO, UART_ESC_BAUDRATE);
    drone->set_motor_lane_mapping(motorLanes);
    ringBuffer_telemetry = drone->get_queue_handle();
    


    /* start tasks */
    print_debug(DEBUG_MAIN, DEBUG_LOGIC, "starting tasks\n");
    #ifdef WEB_TASK
    xTaskCreatePinnedToCore(dispatch_webClient, "web_task", 4048, nullptr, 23, &web_handle, 1);
    #endif
    #ifdef TELEMETRY_TASK
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 4048, nullptr, 23, &telemetry_handle, 1);
    #endif
    xTaskCreatePinnedToCore(dispatch_drone, "dispatch_drone", 4048, nullptr,  23, &drone_handle, 1);
    xTaskCreatePinnedToCore(dispatch_radio, "radio_task", 4048, nullptr,  23, &radio_handle, 0);
    xTaskCreatePinnedToCore(dispatch_dmp, "dmp_task", 4048, nullptr,  23, &dmp_handle, 1);
    xTaskCreatePinnedToCore(dispatch_dshot, "dshot_task", 4048, nullptr,  23, &dmp_handle, 1);
    xTaskCreatePinnedToCore(dispatch_esc_telemetry, "esc_telemetry_task", 4048, nullptr,  23, &esc_telemetry_handle, 1);
    //xTaskCreatePinnedToCore(dispatch_raw, "raw_task", 4048, nullptr,  23, &dmp_handle, 1);

    while (true)
    {
        vTaskDelay(10000000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    TaskHandle_t main_handle{};
    uint32_t files = DEBUG_MAIN | DEBUG_TELEMETRY;// | DEBUG_TELEMETRY;// | DEBUG_DSHOT // | DEBUG_TELEMETRY;// | DEBUG_DSHOT | DEBUG_DSHOT | DEBUG_RADIO | DEBUG_DRONE | DEBUG_TELEMETRY | DEBUG_MPU6050 | DEBUG_I2C; | DEBUG_BMP ;
    uint32_t prio = DEBUG_DATA;// | DEBUG_LOGIC;  // | DEBUG_ARGS;// | DEBUG_LOGIC;

    set_loglevel(files, prio);

    vSemaphoreCreateBinary(cfg_btn_sem);
    ESP_ERROR_CHECK_WITHOUT_ABORT((cfg_btn_sem == nullptr));

    printf("*****************************************************************\n");
    printf("*****************************************************************\n");
    printf("******                                                     ******\n");
    printf("******                  Flight Controller                  ******\n");
    printf("******                      Alpha 1                        ******\n");
    printf("*****************************************************************\n");
    printf("*****************************************************************\n\n\n");


    xTaskCreatePinnedToCore(main_task, "main_task", 4048, nullptr, configMAX_PRIORITIES - 3, &main_handle, 0);

    
}
