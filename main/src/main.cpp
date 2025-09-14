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

#include "mpu6050_stub.h"
#include "radiocontroller_stub.h"

#include "debug.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "mpu6050_stub.h"
#include "dshot600.h"
#include "i2c.h"
#include "quaternion.h"
#include "esp_crsf.h"
#include "webClient.h"
#include "development.h"
#include "drone.h"
#include "display.h"
#include "ringbuffer.h"
#include "radioController.h"



// uncomment to add back tasks
#define WEB_TASK
#define TELEMETRY_TASK
//#define STUB_RADIOCONTROLLER
//#define STUB_MPU6050

// FIXME remove when not needed
#define I2C_MASTER_SCL_IO 22      // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 21      // GPIO number for I2C SDA
#define I2C_MASTER_FREQ_HZ 400000 // Frequency of the I2C bus
#define I2C_DMP_DATA_PIN 27

#define UART_ESC_RX_IO 37   
#define UART_ESC_BAUDRATE 115200

#define ESC_CURRENT_PIN 17   // FIXME current reading adc not supported! resolder tx and rx and maybe remove from uart setup

#define HIGH_PRIO 15
#define LOW_PRIO  14

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

    //std::string ssid{"Wavy"};
    //std::string wpa{"##SnabbtSkit555!!"};
    //std::string ip{"192.168.104.183"};

    WebClient *client = WebClient::GetInstance(ssid, wpa, ip, 6669);
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
    #ifdef STUB_RADIOCONTROLLER
    RadioController_stub *radio = RadioController_stub::GetInstance();
    #else
    RadioController *radio = RadioController::GetInstance();
    #endif
    radio->radio_task(nullptr);
}

void dispatch_dmp(void *args)
{
    
    #ifdef STUB_MPU6050
    //Mpu6050_stub *mpu = Mpu6050_stub::GetInstance();
    #else
    //Mpu6050 *mpu = Mpu6050::GetInstance();

    #endif
    Mpu6050* mpu = reinterpret_cast<Mpu6050*>(args);
    mpu->dmp_task(nullptr);
}

void dispatch_webClient(void *args)
{
    WebClient *client = WebClient::GetInstance();
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

void dispatch_display(void *args)
{
    Display *display = Display::GetInstance();
    display->display_task(nullptr);
}

void dispatch_esc_telemetry(void *args){
    Drone *drone = Drone::GetInstance();
    drone->esc_telemetry_task(nullptr);
}

void main_task(void *args)
{
    TaskHandle_t altitude_handle{};
    TaskHandle_t dshot_handle{};
    TaskHandle_t dmp_handle1{}; 
    TaskHandle_t dmp_handle2{}; 
    TaskHandle_t raw_handle{}; 
    TaskHandle_t radio_handle{}; 
    TaskHandle_t web_handle{}; 
    TaskHandle_t telemetry_handle{}; 
    TaskHandle_t drone_handle{};
    TaskHandle_t esc_telemetry_handle{};
    TaskHandle_t display_handle{};

    RingbufHandle_t  ringBuffer_dmp1{};
    RingbufHandle_t  ringBuffer_dmp2{};
    CircularHandle_t ringBuffer_radio{};
    CircularHandle_t ringBuffer_radio_statistics{};
    RingbufHandle_t  ringBuffer_web{};

    /******* I2C setup *******/
    I2cHandler *i2c = new I2cHandler(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    i2c->init();

    /******* radio *******/
    #ifdef STUB_RADIOCONTROLLER
    RadioController_stub* radio = RadioController_stub::GetInstance();
    #else
    RadioController* radio = RadioController::GetInstance();
    #endif
    ringBuffer_radio = radio->get_queue_handle();
    ringBuffer_radio_statistics = radio->get_statistics_queue_handle();

    /******* MPU 1 setup *******/
    #ifdef STUB_MPU6050
    Mpu6050_stub *mpu = Mpu6050_stub::GetInstance(ADDR_68, i2c);
    #else
    //Mpu6050 *mpu1 = Mpu6050::GetInstance(ADDR_68, i2c);
    Mpu6050 *mpu1 = new Mpu6050(ADDR_68, i2c, 27);
    #endif
    ringBuffer_dmp1 = mpu1->get_queue_handle();

   /******* MPU 2 setup *******/
   #ifdef STUB_MPU6050
   Mpu6050_stub *mpu = Mpu6050_stub::GetInstance(ADDR_68, i2c);
   #else
   Mpu6050 *mpu2 = new Mpu6050(ADDR_69, i2c, 26);
   #endif
   ringBuffer_dmp2 = mpu2->get_queue_handle();



    /******* wifi *******/
    #ifdef WEB_TASK
    std::string ssid{"Ubiquity 2"};
    std::string wpa{"#SuperDuper66!"};
    std::string ip{"192.168.1.245"};

    init_telemetry_buffer();
    ringBuffer_web = get_telemetry_handle();
    WebClient *client = WebClient::GetInstance(ssid, wpa, ip, 6669);
    client->init(ringBuffer_dmp1, ringBuffer_web);
    #endif


    /******* Dshot600 *******/
    gpio_num_t motorPin[Radio::maxChannels]{};
    motorPin[MOTOR1] = static_cast<gpio_num_t>(12); // rear right   0
    motorPin[MOTOR2] = static_cast<gpio_num_t>(13); // front right  1
    motorPin[MOTOR3] = static_cast<gpio_num_t>(15); // rear left    2
    motorPin[MOTOR4] = static_cast<gpio_num_t>(2);  // front left   3

    Dshot600* dshot = Dshot600::GetInstance(motorPin);


    /*******  Drone *******/
    MotorLaneMapping motorLanes{
        .rearLeftlane   = MOTOR3,
        .rearRightlane  = MOTOR1,
        .frontLeftlane  = MOTOR4,    
        .frontRightlane = MOTOR2
    };


    Drone* drone = Drone::GetInstance(dshot, ringBuffer_dmp1, ringBuffer_dmp2, ringBuffer_radio, ringBuffer_radio_statistics);
    drone->init_uart(UART_ESC_RX_IO, ESC_CURRENT_PIN, UART_ESC_BAUDRATE);
    drone->set_motor_lane_mapping(motorLanes);

    esp_err_t status = gpio_install_isr_service(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);


    /* start tasks */
    print_debug(DEBUG_MAIN, DEBUG_LOGIC, "starting tasks\n");
    xTaskCreatePinnedToCore(dispatch_display, "display_task", 6144, nullptr,  LOW_PRIO, &display_handle, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreatePinnedToCore(dispatch_drone, "drone_task", 4048, nullptr,  LOW_PRIO, &drone_handle, 1);

    #ifdef WEB_TASK
    xTaskCreatePinnedToCore(dispatch_webClient, "web_task", 4048, nullptr, LOW_PRIO, &web_handle, 0);
    #endif
    #ifdef TELEMETRY_TASK
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 4048, nullptr, LOW_PRIO, &telemetry_handle, 0);
    #endif

    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreatePinnedToCore(dispatch_radio, "radio_task", 4048, nullptr,  HIGH_PRIO, &radio_handle, 1);
    xTaskCreatePinnedToCore(dispatch_dmp, "dmp_task1", 4048, mpu1,  HIGH_PRIO, &dmp_handle1, 1);
    xTaskCreatePinnedToCore(dispatch_dmp, "dmp_task2", 4048, mpu2,  HIGH_PRIO, &dmp_handle2, 1);
    xTaskCreatePinnedToCore(dispatch_esc_telemetry, "esc_telemetry_task", 4048, nullptr,  LOW_PRIO, &esc_telemetry_handle, 0);

    char buffer[500]{};
    while (true)
    {
        // enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
        vTaskDelay(pdMS_TO_TICKS(5000));
        //vTaskGetRunTimeStats(buffer);
        printf("%s", buffer);
    }
}

void app_main(void)
{
    TaskHandle_t main_handle{};
    uint32_t files = DEBUG_MAIN ; //  | DEBUG_RADIO | DEBUG_DRONE | DEBUG_MPU6050 | DEBUG_I2C; | DEBUG_BMP ;
    uint32_t prio = DEBUG_DATA; // | DEBUG_ARGS; // DEBUG_LOGIC | DEBUG_LOWLEVEL |

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
 