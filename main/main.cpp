#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <chrono>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "driver/gpio.h"


#include "radioController.h"
#include "debug.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "i2c.h"
#include "quaternion.h"
#include "esp_crsf.h"

#define log_tag "main"

#define I2C_MASTER_SCL_IO  22      // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO  21      // GPIO number for I2C SDA
#define I2C_MASTER_FREQ_HZ 400000 // Frequency of the I2C bus
#define I2C_DMP_DATA_PIN   27

SemaphoreHandle_t dmp_avail = nullptr;
 

extern "C"{
    void app_main(void);
}




static void IRAM_ATTR dmp_data_handler(void *args){
    xSemaphoreGive(dmp_avail);
}




void dmp_task(void* args){

    constexpr TickType_t xDelay = 100 / portTICK_PERIOD_MS;
    I2cHandler* i2c = new I2cHandler(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    i2c->init();

    Mpu6050 mpu(ADDR_68, i2c);

    esp_err_t status = 0;
    int nrSamples = 100;
    uint8_t dmpData[mpu.dmpPacketSize] = {};
    DmpFifoStatus dmpStatus = DMP_FIFO_OK;

    status = mpu.mpu6050_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    //status = mpu.dump_registers();
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = mpu.dmp_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    uint8_t intStatus = 0;
    status = mpu.set_fifo_enable(true);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);


    status = mpu.set_x_accel_offset(-2889);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = mpu.set_y_accel_offset(-444);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = mpu.set_z_accel_offset(698);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = mpu.set_x_gyro_offset(149);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = mpu.set_y_gyro_offset(27);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = mpu.set_z_gyro_offset(17);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    status = mpu.calibrate_accel(6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = mpu.calibrate_gyro(6);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);


    status = mpu.set_dmp_enabled(true);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    //status = mpu.dump_registers();
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);


    //setup interrupt

    vSemaphoreCreateBinary(dmp_avail);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    
    gpio_config_t ioConf{};

    ioConf.intr_type = GPIO_INTR_ANYEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<<I2C_DMP_DATA_PIN;
    ioConf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&ioConf);

    status = gpio_install_isr_service(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = gpio_isr_handler_add(static_cast<gpio_num_t>(I2C_DMP_DATA_PIN), dmp_data_handler, (void *)I2C_DMP_DATA_PIN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);



    int count_ = 0;
    while (true)
    {
        xSemaphoreTake(dmp_avail, portMAX_DELAY);

        //status = mpu.get_curr_fifo_packet2();
        status = mpu.GetCurrentFIFOPacket(0,0);
        if(status == ESP_ERR_INVALID_SIZE){
            continue;
        }
        else
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);

        Quaternion quaternion;
        VectorFloat vec;
        mpu.get_quaternion(quaternion);
        mpu.get_gravity(vec, quaternion);
        mpu.get_yaw_pitch_roll(quaternion,vec);
        //print_debug(DEBUG_MAIN, DEBUG_DATA, "w: %.3f x: %.3f y: %.3f z: %.3f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);


        vTaskDelay(100/portTICK_PERIOD_MS);

    }

}

void gyro_task(void* args){

    constexpr TickType_t xDelay = 10 / portTICK_PERIOD_MS;
    esp_err_t status = 0;
    int nrSamples = 100;

    I2cHandler* i2c = new I2cHandler(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    Mpu6050 mpu(ADDR_68, i2c);


    status = mpu.mpu6050_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

	while (true)
	{
        int16_t x_acc = 0;
        int16_t y_acc = 0;
        int16_t z_acc = 0;
        int16_t x_gyro = 0;
        int16_t y_gyro = 0;
        int16_t z_gyro = 0;
        int16_t x_accSum = 0;
        int16_t y_accSum = 0;
        int16_t z_accSum = 0;
        int16_t x_gyroSum = 0;
        int16_t y_gyroSum = 0;
        int16_t z_gyroSum = 0;
        int16_t x_accAvg = 0;
        int16_t y_accAvg = 0;
        int16_t z_accAvg = 0;
        int16_t x_gyroAvg = 0;
        int16_t y_gyroAvg = 0;
        int16_t z_gyroAvg = 0;


        for(int i=0;i<nrSamples;i++){
            mpu.get_6axis_motion(x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro);
            x_accSum += x_acc;
            y_accSum += y_acc;
            z_accSum += z_acc;
            x_gyroSum += x_gyro;
            y_gyroSum += y_gyro;
            z_gyroSum += z_gyro;
        }
        x_accAvg =  x_accSum / nrSamples;
        y_accAvg = y_accSum / nrSamples;
        z_accAvg = z_accSum / nrSamples;
        x_gyroAvg = x_gyroSum / nrSamples;
        y_gyroAvg = y_gyroSum / nrSamples;
        z_gyroAvg = z_gyroSum / nrSamples;


        mpu.get_6axis_motion(x_acc, y_acc, z_acc, x_gyro, y_gyro, z_gyro);
        print_debug(DEBUG_MAIN, DEBUG_DATA, "\rx: %4f y: %4f z: %4f\n", x_accSum / 131.072, y_accSum / 131.072, z_accSum /131.072);

        vTaskDelay(xDelay);
	}
}

void altitude_task(void* args){
    constexpr TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    constexpr TickType_t xDelayShort = 10 / portTICK_PERIOD_MS;
    float altitudeHypsometricSum = 0;
    float altitudeHypsometric = 0;
    float altitudeHypsometricDefault = 0;
    constexpr int nrSamples = 100;
    esp_err_t status = 0;
    sensors_t sensorsData = {};

    uint8_t deviceAddress = bmp280_init();      
    if(deviceAddress == NONE_DETECTED){
        printf("NONE_DETECTED\n");
        for(;;){
            vTaskDelay(xDelay);
        }
    }

    status = bmp280_set_mode(deviceAddress, MODE_NORMAL);
    vTaskDelay(xDelayShort*10);

    for(int i=0;i<nrSamples;i++){
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
    vTaskDelay(xDelay*5);


    for(;;){
        
        ////////////// Altitude measurements //////////////
        sensorsData = {};
        altitudeHypsometricSum = 0;

        for(int i=0;i<nrSamples;i++){
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

void radio_task(void* args){
    RadioController* radio = new RadioController();

    radio->test();

}

void crsf_task(void* args){
    crsf_config_t config = {
    .uart_num = UART_NUM_1,
    .tx_pin = 32,
    .rx_pin = 33
    };
    CRSF_init(&config);

    crsf_channels_t channels = {0};
    crsf_battery_t battery = {0};
    crsf_gps_t gps = {0};
    while (1)
    {
        
        CRSF_receive_channels(&channels);
        printf(">Channel 1: %d Channel 2: %d\n", channels.ch1, channels.ch2);

        battery.voltage = 120; //voltage 10*V
        battery.current = 100; //current 10*A
        battery.capacity = 1000; //capacity
        battery.remaining = 50; //remaining % of battery

        CRSF_send_battery_data(CRSF_DEST_FC, &battery);

        gps.latitude = 42.4242 * 10000000; //42.4242 deg
        gps.longitude = 56.5656 * 10000000; //56.5656 deg
        gps.altitude = 5 + 1000; //5m
        gps.groundspeed = 420; //42km/h
        gps.heading = 90 * 100; //90 deg NOT WORKING WELL NOW
        gps.satellites = 8; //8 satellites

        CRSF_send_gps_data(CRSF_DEST_FC, &gps);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    uint32_t files =  DEBUG_MAIN | DEBUG_RADIO; // | DEBUG_MPU6050 | DEBUG_I2C; // | DEBUG_BMP ;
    uint32_t prio  = DEBUG_DATA | DEBUG_LOGIC; // | DEBUG_ARGS;// | DEBUG_LOGIC;

    set_loglevel(files, prio);

    printf("*****************************************************************\n");
    printf("*****************************************************************\n");
    printf("******                                                     ******\n");
    printf("******                  Flight Controller                  ******\n");
    printf("******                      Alpha 1                        ******\n");
    printf("*****************************************************************\n");
    printf("*****************************************************************\n");


    TaskHandle_t altitude_handle, gyro_handle, dmp_handle, radio_handle, crsf_handle;
    //xTaskCreatePinnedToCore(altitude_task, "altitude_task", 4048, nullptr, configMAX_PRIORITIES - 3, &altitude_handle, 0);
    //xTaskCreatePinnedToCore(gyro_task, "gyro_task", 4048, nullptr, configMAX_PRIORITIES - 3, &altitude_handle, 0);
    //xTaskCreatePinnedToCore(dmp_task, "dmp_task", 4048, nullptr, configMAX_PRIORITIES - 3, &dmp_handle, 0);
    xTaskCreatePinnedToCore(radio_task, "radio_task", 16192, nullptr, configMAX_PRIORITIES - 3, &radio_handle, 0);
    //xTaskCreatePinnedToCore(crsf_task, "crsf_task", 16192, nullptr, configMAX_PRIORITIES - 3, &crsf_handle, 0);

}
