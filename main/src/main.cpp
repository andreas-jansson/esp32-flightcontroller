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

#include "blackbox.h"
#include "debug.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "mpu6050_stub.h"
#include "dshot600.h"
#include "i2c.h"
#include "imuIf.h"
#include "quaternion.h"
#include "webClient.h"
#include "development.h"
#include "drone.h"
#include "display.h"
#include "ringbuffer.h"
#include "radioController.h"
#include "djiO4Pro.h"
#include "vtxIf.h"
#include "mspData.h"

// battery info
// battery 4000 mah
// 22.2v
// 60c / 120c burst
// 88.8 WH

#define BATTERY_MAX_V   22.2
#define BATTERY_MAX_MAH 4000
#define BATTERY_MAX_WH  88.8

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

#define ESC_CURRENT_PIN 17 // FIXME current reading adc not supported! resolder tx and rx and maybe remove from uart setup

#define log_tag "main"

enum{
    PRIO_BG       = 3,   // display / web / telemetry
    PRIO_SENSORS  = 7,   // radio + DMP readers (paced with vTaskDelayUntil)
    PRIO_CONTROL  = 10,  // drone PID + RMT (deadline-driven)
};

extern "C"
{
    void app_main(void);
}

void web_task(void *args){
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

void dispatch_radio(void *args){
#ifdef STUB_RADIOCONTROLLER
    RadioController_stub *radio = RadioController_stub::GetInstance();
#else
    RadioController *radio = RadioController::GetInstance();
#endif
    radio->radio_task(nullptr);
}

void dispatch_dmp(void *args){

#ifdef STUB_MPU6050
//Mpu6050_stub *mpu = Mpu6050_stub::GetInstance();
#else
    //Mpu6050 *mpu = Mpu6050::GetInstance();

#endif
    ImuIf* mpu = reinterpret_cast<ImuIf *>(args);
    mpu->dmp_task(nullptr);
}

void dispatch_webClient(void* args){
    WebClient *client = WebClient::GetInstance();
    client->web_task2(nullptr);
}

void dispatch_drone(void* args){
    Drone *drone = Drone::GetInstance();
    drone->drone_task(nullptr);
}

void dispatch_dshot(void* args){
    Dshot600 *dshot = Dshot600::GetInstance();
    dshot->dshot_task(nullptr);
}

void dispatch_display(void *args){
    Display *display = Display::GetInstance();
    display->display_task(nullptr);
}

void dispatch_esc_telemetry(void *args){
    Drone *drone = Drone::GetInstance();
    drone->esc_telemetry_task(nullptr);
}

void dispatch_vtx(void *args){
    VtxIf *vtx = reinterpret_cast<VtxIf *>(args);
    vtx->vtx_task(nullptr);
}

void dispatch_blackbox(void *args){
     Blackbox* bb = reinterpret_cast<Blackbox*>(args);
     bb->log_task(nullptr);
}

void main_task(void *args){
    // TaskHandle_t altitude_handle{};
    // TaskHandle_t dshot_handle{};
    TaskHandle_t dmp_handle1{};
    // TaskHandle_t dmp_handle2{};
    // TaskHandle_t raw_handle{};
    TaskHandle_t radio_handle{};
    TaskHandle_t web_handle{};
    TaskHandle_t telemetry_handle{};
    TaskHandle_t drone_handle{};
    TaskHandle_t esc_telemetry_handle{};
    TaskHandle_t display_handle{};
    TaskHandle_t vtx_handle{};
    TaskHandle_t blackbox_handle{};

    //std::map<std::string, TaskHandle_t> taskHandles{};

    RingbufHandle_t ringBuffer_dmp1{};
    RingbufHandle_t ringBuffer_dmp2{};
    CircularHandle_t ringBuffer_radio{};
    CircularHandle_t ringBuffer_radio_statistics{};
    CircularHandle_t blackbox_telemetry_handle{};
    RingbufHandle_t ringBuffer_web{};

    constexpr bool isBidiDshot{true};
    std::vector<ImuIf*> mpuVector;

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
    ImuIf *mpu1 = new Mpu6050(ADDR_68, i2c, 27);
    mpuVector.emplace_back(mpu1);
#endif

/******* MPU 2 setup *******/
#ifdef STUB_MPU6050
    Mpu6050_stub *mpu = Mpu6050_stub::GetInstance(ADDR_68, i2c);
#else
    //Mpu6050 *mpu2 = new Mpu6050(ADDR_69, i2c, 26);
    //mpuVector.emplace_back(mpu1);

#endif

/******* wifi *******/
#ifdef WEB_TASK
    std::string ssid{"wajfaj 2"};
    std::string wpa{"##FortSomFan666!!"};
    std::string ip{"192.168.1.130"};

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

    Dshot600* dshot = Dshot600::GetInstance(motorPin, isBidiDshot);

    /*******  Drone *******/
    MotorLaneMapping motorLanes{
        .rearLeftlane   = MOTOR3,
        .rearRightlane  = MOTOR1,
        .frontLeftlane  = MOTOR4,
        .frontRightlane = MOTOR2};

    Drone* drone = Drone::GetInstance(dshot, mpuVector, ringBuffer_radio, ringBuffer_radio_statistics);
    drone->init_esc_telemetry_uart(UART_ESC_RX_IO, 25, UART_ESC_BAUDRATE);
    drone->set_motor_lane_mapping(motorLanes);
    drone->set_battery_data(BATTERY_MAX_MAH, BATTERY_MAX_V, BATTERY_MAX_WH);

    esp_err_t status = gpio_install_isr_service(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    Drone::set_fw_version(1, 1, 0);
    Drone::set_battery_status(6u, 3500u);

    blackbox_telemetry_handle = drone->get_blackbox_telemetry_handle();

    /* Display */
    Display::GetInstance();  //allow constructor to run

    /* Dji o4 Pro */
    VtxIf* vtx = new DjiO4Pro();

    /* Blackbox */
    Blackbox* bb = new Blackbox(blackbox_telemetry_handle);


    /* start tasks */
    print_debug(DEBUG_MAIN, DEBUG_LOGIC, "starting tasks\n");
    // xTaskCreatePinnedToCore(dispatch_blackbox, "blackbox_task", 8096, bb,  PRIO_BG, &blackbox_handle, 0);
    xTaskCreatePinnedToCore(dispatch_display, "display_task", 8096, nullptr, PRIO_BG, &display_handle, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    Display::set_display_state(BOOTING);

    xTaskCreatePinnedToCore(dispatch_drone, "drone_task", 4048, nullptr, PRIO_CONTROL, &drone_handle, 1);

#ifdef WEB_TASK
    xTaskCreatePinnedToCore(dispatch_webClient, "web_task", 8096, nullptr, PRIO_BG, &web_handle, 0);
#endif
#ifdef TELEMETRY_TASK
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 4048, nullptr, PRIO_BG, &telemetry_handle, 0);
#endif

    vTaskDelay(pdMS_TO_TICKS(200));
    xTaskCreatePinnedToCore(dispatch_radio, "radio_task", 4048, nullptr, PRIO_SENSORS, &radio_handle, 0);
    xTaskCreatePinnedToCore(dispatch_dmp, "dmp_task1", 4048, mpu1, PRIO_SENSORS, &dmp_handle1, 0);
    // xTaskCreatePinnedToCore(dispatch_dmp, "dmp_task2", 4048, mpu2,  PRIO_SENSORS, &dmp_handle2, 0);
    xTaskCreatePinnedToCore(dispatch_esc_telemetry, "esc_telemetry_task", 4048, nullptr, PRIO_BG, &esc_telemetry_handle, 0);
    xTaskCreatePinnedToCore(dispatch_vtx, "vtx_task", 4048, vtx, PRIO_BG, &vtx_handle, 0);

    /*
    taskHandles["dmp1"] = dmp_handle1;
    taskHandles["radio_handle"] = radio_handle;
    taskHandles["web_handle"] = web_handle;
    taskHandles["telemetry_handle"] = telemetry_handle;
    taskHandles["drone_handle"] = drone_handle;
    taskHandles["esc_telemetry_handle"] = esc_telemetry_handle;
    taskHandles["display_handle"] = display_handle;
    taskHandles["vtx_handle"] = vtx_handle;
    taskHandles["blackbox_handle"] = blackbox_handle;
    */

    //char buffer[500]{};
    while (true)
    {
        // enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
        //vTaskGetRunTimeStats(buffer);
        //printf("%s", buffer);

        //for(const auto& i : taskHandles){
        //    //printf("%s stack hwmark=%u\n", i.first.c_str() ,uxTaskGetStackHighWaterMark(i.second));
        //}
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}


#include "esp_cpu.h"
#include "esp_debug_helpers.h"

#define log_tag "main"


#define CTOR_PROBE(tag) do { \
  void* ra = __builtin_return_address(0); \
  void* sp = (void*)esp_cpu_get_sp(); \
  printf("CTOR_PROBE: %s ra=%p sp=%p\n", tag, ra, sp); \
} while(0)


__attribute__((constructor))
static void ctor_probe(void) {
    esp_rom_printf("CTOR_PROBE: entered\n");
    CTOR_PROBE(log_tag);
}

void app_main(void)
{

    
    TaskHandle_t h = xTaskGetCurrentTaskHandle();  

    printf("main task configured stack: %u bytes\n", (unsigned)CONFIG_ESP_MAIN_TASK_STACK_SIZE);

    UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(h);
    printf("main task min-free stack (high water mark): %u words = %u bytes\n",
           (unsigned)hwm_words, (unsigned)(hwm_words * sizeof(StackType_t)));
    

    TaskHandle_t main_handle{};
    uint32_t files = DEBUG_MAIN; // | DEBUG_TELEMETRY; //  | DEBUG_RADIO | DEBUG_DRONE | DEBUG_MPU6050 | DEBUG_I2C; | DEBUG_BMP ;
    uint32_t prio = DEBUG_DATA;  // | DEBUG_ARGS; // DEBUG_LOGIC | DEBUG_LOWLEVEL |

    set_loglevel(files, prio);

    printf("*****************************************************************\n");
    printf("*****************************************************************\n");
    printf("******                                                     ******\n");
    printf("******                  Flight Controller                  ******\n");
    printf("******                      Alpha 1                        ******\n");
    printf("*****************************************************************\n");
    printf("*****************************************************************\n\n\n");

    xTaskCreatePinnedToCore(main_task, "main_task", 4048, nullptr, configMAX_PRIORITIES - 3, &main_handle, 0);
}
