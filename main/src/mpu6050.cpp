#include <pgmspace.h>
#include <cstring>
#include <stdlib.h>
#include <memory>
#include <bitset>
#include <iostream>
#include <algorithm> 
#include <cmath> 
#include <chrono> 

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "mpu6050.h"
#include "debug.h"
#include "i2c.h"


#define I2C_DMP_DATA_PIN   27

#define RAD_TO_DEG (180.0/M_PI)

#define log_tag "mpu6050"


Mpu6050* Mpu6050::mpu = nullptr;
SemaphoreHandle_t Mpu6050::dmp_avail_sem = nullptr;  
RingbufHandle_t Mpu6050::dmp_buf_handle = nullptr;
RingbufHandle_t Mpu6050::web_buf_handle = nullptr;



using namespace mpu6050Value;


void IRAM_ATTR Mpu6050::dmp_data_handler(void *args){
    xSemaphoreGive(Mpu6050::dmp_avail_sem);
}

static esp_err_t set_bits_8(uint8_t& data, uint8_t value, uint8_t lsb){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x lsb %u\n", __func__, data, lsb);
    ESP_RETURN_ON_ERROR((lsb > 7), log_tag, "Failed set bits lsb %u value %u ", lsb, value);
    data |= value << lsb;
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "set bits lsb %u value %u data: 0x%x\n", lsb, value, data);
    return ESP_OK;
}

Mpu6050::Mpu6050(enum Mpu6050Addr address, I2cHandler* i2c){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "addr 0x%x\n", this->address);

    if(!i2c->is_initiated()){
        printf("ERROR i2c dependency not initiated before passing to Mpu6050");
    }

    this->address = address;
    // multiple of dmpPacketsize 
    dmp_buf_handle = xRingbufferCreate(sizeof(YawPitchRoll)*5, RINGBUF_TYPE_NOSPLIT);
    if (dmp_buf_handle == NULL) {
        printf("Failed to create DMP ring buffer\n");
    }

    web_buf_handle = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);
    if (web_buf_handle == NULL) {
        printf("Failed to create DMP ring buffer\n");
    }
}

esp_err_t Mpu6050::mpu6050_init(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;

    status = set_clk_source(1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set clock source: 0x%x", 0);

    //status = set_sample_rate(4);
    //ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set sample rate: 0x%x", 0);

    status = set_full_scale_gyro_range(0);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set gyro scale range: 0x%x", 0);

    status = set_full_scale_accel_range(0);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set accel scale range: 0x%x", 0);

    status = set_sleep_mode(false);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set sleep mode: %u", false);

    return status;
}

esp_err_t Mpu6050::dmp_init(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;

    /************** DMP INIT *****************/
    // reset
    status = reset();

    // disable sleep mode
    status = set_sleep_mode(false);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set sleep mode: %u", false);

    /******* get MPU hardware revision *******/

    //     setMemoryBank(0x10, true, true);
    status = set_mem_bank(0x10, true, true);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem bank 0x%x prefetch %u user bank %u", 0x10, true, true);
    //     setMemoryStartAddress(0x06);

    status = set_mem_start_addr(0x6);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem start addr 0x%x", 0x6);

    // 	DEBUG_PRINTLN(readMemoryByte());
    uint8_t rev = 0;
    status = read_mem_byte(rev);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get re bank %u", 0x6);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "rev: 0x%x\n", rev);

    // 	setMemoryBank(0, false, false);
    status = set_mem_bank(0x0, false, false);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem bank 0x%x prefetch %u user bank %u", 0x9, false, false);

	///// check OTP bank valid //////
    // getOTPBankValid()
    uint8_t valid = 0;
    status = get_opt_bank_valid(valid);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to validate opt bank %u", status);
    ESP_RETURN_ON_ERROR(!valid, log_tag, "Failed to validate opt bank %u", valid);

    /******* setup weird slave stuff (?) *******/

    // 	setSlaveAddress(0, 0x7F);
    status = set_slave_addr(0, 0x7F);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set slave num 0x%x addr 0x%x", 0, 0x7F);

    // 	setI2CMasterModeEnabled(false);
    status = setI2CMasterModeEnabled(false);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set i2c master enable: %u", false);

	// setSlaveAddress(0, 0x68);
    status = set_slave_addr(0, 0x68);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set slave num 0x%x addr 0x%x", 0, 0x68);

    // resetI2CMaster();
    status = resetI2CMaster();
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to reset i2c master");

    vTaskDelay(pdMS_TO_TICKS(20));

    // setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    status = set_clk_source(0x3);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set clk source 0x%x", 0x3);

    // setIntEnabled(1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_INTERRUPT_DMP_INT_BIT);
    status = set_int_enabled((1<<INT_ENABLE_FIFO_OFLOW_EN_LSB) | (1<< INT_ENABLE_DMP_INT_EN_LSB));
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set int enable  0x%x", (1<<INT_ENABLE_FIFO_OFLOW_EN_LSB) | (1<< INT_ENABLE_DMP_INT_EN_LSB));

    // 	setRate(4); // 1khz / (1 + 4) = 200 Hz
    status = set_sample_rate(4);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set sample rate  0x%x", 4u);

    //setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);
    status = set_external_frame_sync(1u); //TEMP_OUT_L 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set ext frame sync 0x%x", 1u);

	//setDLPFMode(MPU6050_DLPF_BW_42);
    status = set_dlpf_mode(0x3);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set DLPF 0x%x", 0x3);

    //setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    status = set_full_scale_gyro_range(0x3);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set full scale gyro 0x%x", 0x3);

    // load DMP code into memory banks
	//if (!i2c->writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) return 1; // Failed
    status = writeProgMemoryBlock(dmpMemory, dmp_code_size);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed load DMP fw");

	// Set the FIFO Rate Divisor int the DMP Firmware Memory
	//unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
	//i2c->writeMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16); // Lets i2c->write the dmpUpdate data to the Firmware image, we have 2 bytes to i2c->write in bank 0x02 with the Offset 0x16
   	unsigned char dmpUpdate[] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
    status = writeMemoryBlock(dmpUpdate, 0x02, 0x02, 0x16);

	//i2c->write start address MSB into register
	//setDMPConfig1(0x03);
	//i2c->write start address LSB into register
	//setDMPConfig2(0x00);
    status = set_dmp_config1(0x3);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set DMP config 1");

    status = set_dmp_config2(0x0);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set DMP config 2");

    //setOTPBankValid(false);
    status = set_opt_bank_valid(false);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set OPT bank: %u", false);

	//setMotionDetectionThreshold(2);
    status = set_motion_detection_thld(2);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set OPT bank: %u", false);

	//setZeroMotionDetectionThreshold(156);
    status = set_zero_motion_detection_thld(156);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set zero motion detection thld: %u", 156);

    //setMotionDetectionDuration(80);
    status = set_moion_dection_duration(80);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set motion detecion duration: %u", 80);

    //setZeroMotionDetectionDuration(0);
    status = set_zero_moion_dection_duration(0);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set zero motion detecion duration: %u", 0);

	// setFIFOEnabled(true);
    status = set_fifo_enable(true);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set enabled fifo: %u", true);

    // 	resetDMP();
    status = reset_dmp();
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed reset DMP");

    //setDMPEnabled(false);
    status = set_dmp_enabled(false);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed set enabled DMP: %u", false);

    // 	DEBUG_PRINTLN(F("Setting up internal 42-byte (default) DMP packet buffer..."));
	dmpPacketSize = 42;
    dmpBuffer = std::make_unique<uint8_t[]>(dmpPacketSize);

    // 	resetFIFO();
    status = reset_fifo();
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed reset fifo");

    // 	getIntStatus();
    uint8_t intStatus = 0;
    status = get_int_satus(intStatus);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed get interrupt status");
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "Interrupt status 0x%x\n", intStatus);

    print_debug(DEBUG_MPU6050, DEBUG_DATA, "DMP initiated successfully\n");
    return status;
}

Mpu6050* Mpu6050::GetInstance(enum Mpu6050Addr address, I2cHandler* i2c){
  if(mpu==nullptr){
        mpu = new Mpu6050(address, i2c);
    }
    return mpu;
}

Mpu6050* Mpu6050::GetInstance(){
    return mpu;
}


void Mpu6050::dmp_task(void* args){
    esp_err_t status = 0;
    YawPitchRoll ypr{};
    YawPitchRoll yprPrev{};

    constexpr TickType_t xDelay = 2 / portTICK_PERIOD_MS;

    /////// MPU setup /////////

    status = this->mpu6050_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = this->dmp_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    //status = this->set_fifo_enable(true);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    // old
    // getXAccelOffset: -3881
    // getYAccelOffset: 246
    // getZAccelOffset: 56
    // getXGyroOffset: 13
    // getYGyroOffset: 82
    // getZGyroOffset: 7

    // new
    //getXAccelOffset: -4991
    //getYAccelOffset: 1534
    //getZAccelOffset: 476
    //getXGyroOffset: -61
    //getYGyroOffset: 32
    //getZGyroOffset: 3

    status = set_x_accel_offset(-4991);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_y_accel_offset(1534);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_z_accel_offset(476);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_x_gyro_offset(-61);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    status = set_y_gyro_offset(32);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_z_gyro_offset(3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    //status = mpu.calibrate_accel(6);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	//status = mpu.calibrate_gyro(6);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = this->set_chip_rotation(ROTATE_90);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    
    /////// ISR setup /////////
    vSemaphoreCreateBinary(dmp_avail_sem);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    
    gpio_config_t ioConf{};

    ioConf.intr_type = GPIO_INTR_POSEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<<I2C_DMP_DATA_PIN;
    ioConf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&ioConf);

    status = gpio_install_isr_service(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = gpio_isr_handler_add(static_cast<gpio_num_t>(I2C_DMP_DATA_PIN), dmp_data_handler, (void *)I2C_DMP_DATA_PIN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = this->set_dmp_enabled(true);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    while (true)
    {
        BaseType_t semStatus = xSemaphoreTake(this->dmp_avail_sem, portMAX_DELAY);

        status = get_dmp_packet();
        //status = get_curr_fifo_packet2();
        if(status != ESP_OK){
            continue;
        }

        Quaternion quaternion{};
        VectorFloat vec{};
        YawPitchRoll ypr{};

        this->get_quaternion(quaternion);
        this->get_gravity(vec, quaternion);
        this->get_yaw_pitch_roll(quaternion,vec, ypr);

        if(ypr != yprPrev){ // compare the first 4 bytes of the struct which are the 3 floats
            BaseType_t res = xRingbufferSend(dmp_buf_handle, &ypr, sizeof(YawPitchRoll), 1);
            if (res != pdTRUE) {
                //printf("Failed to send dmp item: handle %p size %u\n", web_buf_handle, sizeof(ypr));
            } 
        }
    }
}

void Mpu6050::raw_task(void* args){
    esp_err_t status = 0;
    YawPitchRoll ypr{};
    YawPitchRoll yprPrev{};

    constexpr TickType_t xDelay = 2 / portTICK_PERIOD_MS;

    /////// MPU setup /////////

    status = this->mpu6050_init();
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    // getXAccelOffset: -3881
    // getYAccelOffset: 246
    // getZAccelOffset: 56
    // getXGyroOffset: 13
    // getYGyroOffset: 82
    // getZGyroOffset: 7
    status = set_x_accel_offset(-3881);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_y_accel_offset(246);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_z_accel_offset(56);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_x_gyro_offset(13);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    status = set_y_gyro_offset(82);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	status = set_z_gyro_offset(7);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    //status = mpu.calibrate_accel(6);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);
	//status = mpu.calibrate_gyro(6);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    // 	setRate(4); // 1khz / (1 + 4) = 200 Hz
    status = set_sample_rate(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = set_dlpf_mode(0x3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    //setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);
    //status = set_external_frame_sync(1u); //TEMP_OUT_L 
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    //setDLPFMode(MPU6050_DLPF_BW_42);
    //status = set_dlpf_mode(0);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    //setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    status = set_full_scale_gyro_range(0x3);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = set_full_scale_accel_range(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = set_int_enabled((1<<INT_ENABLE_FIFO_OFLOW_EN_LSB) | (1<< INT_ENABLE_DATA_RDY_EN_LSB));
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = this->set_chip_rotation(ROTATE_90);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    
    /////// ISR setup /////////
    vSemaphoreCreateBinary(dmp_avail_sem);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);
    
    gpio_config_t ioConf{};

    ioConf.intr_type = GPIO_INTR_POSEDGE;
    ioConf.mode = GPIO_MODE_INPUT;
    ioConf.pin_bit_mask = 1ULL<<I2C_DMP_DATA_PIN;
    ioConf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&ioConf);

    status = gpio_install_isr_service(0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = gpio_isr_handler_add(static_cast<gpio_num_t>(I2C_DMP_DATA_PIN), dmp_data_handler, (void *)I2C_DMP_DATA_PIN);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    status = this->set_fifo_enable(true);
    ESP_ERROR_CHECK_WITHOUT_ABORT(status);

    uint32_t freq = 0;
    uint32_t freqPrev = 0;

    auto start = std::chrono::steady_clock::now();
    auto dtStart = std::chrono::steady_clock::now();
    while (true)
    {

        BaseType_t semStatus = xSemaphoreTake(this->dmp_avail_sem, portMAX_DELAY);
        get_6axis_motion(ypr.x_accel, ypr.y_accel, ypr.z_accel, ypr.x_gyro, ypr.y_gyro, ypr.z_gyro);


        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if(elapsed >= 1000){

            printf("[4A");
            printf("[K  z_accel   x_accel   y_accel   z_gyro   x_gyro   y_gyro freq: %lu Hz\n", freqPrev);
            printf("[K  %6d  %8d  %8d  %8d  %8d  %8d\n", 
                ypr.z_accel, 
                ypr.x_accel, 
                ypr.y_accel,
                ypr.z_gyro, 
                ypr.x_gyro, 
                ypr.y_gyro);  

            freqPrev = freq;
            freq = 0;
            start = std::chrono::steady_clock::now();
        }


        if(ypr != yprPrev){
            //printf("[4A");
            //printf("[K  z_accel   x_accel   y_accel   z_gyro   x_gyro   y_gyro freq: %lu Hz\n", freqPrev);
            //printf("[K  %6d  %8d  %8d  %8d  %8d  %8d\n", 
            //    ypr.z_accel, 
            //    ypr.x_accel, 
            //    ypr.y_accel,
            //    ypr.z_gyro, 
            //    ypr.x_gyro, 
            //    ypr.y_gyro);  

            auto dtEnd = std::chrono::steady_clock::now();
            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(dtEnd - dtStart).count() / 1e6;
            comp_filter(dt, 0.98, ypr);
            dtStart = std::chrono::steady_clock::now();

            //printf("[K  yaw      pitch     roll   s: %fs\n", dt);
            //printf("[K %4.3f    %4.3f     %4.3f\n", 
            //    ypr.yaw, 
            //    ypr.pitch,  
            //    ypr.roll);  

            //BaseType_t res = xRingbufferSend(dmp_buf_handle, &ypr, sizeof(YawPitchRoll), 1);
            //if (res != pdTRUE) {
            //    printf("Failed to send Raw item: handle %p size %u\n", web_buf_handle, sizeof(ypr));
            //} 

            yprPrev = ypr;
            freq++;
        }
    }
}

esp_err_t Mpu6050::comp_filter(double dt, float tau, YawPitchRoll& ypr){
    static Kalman kalmanRoll{}, kalmanPitch{};
    //readCalData();
    // Convert accelerometer values to g's
    ypr.z_accel = ypr.z_accel * 9.82 / 16384.0;    
    ypr.x_accel = ypr.x_accel * 9.82 / 16384.0;        
    ypr.y_accel = ypr.y_accel * 9.82 / 16384.0;    

    // Convert gyro values to degrees per second
    //  0   250 degree/s 131 LSB
    //  1   500          65.5
    //  2   1000         32.8
    //  3   2000         16.4

    ypr.x_gyro /= 16.4; 
    ypr.y_gyro /= 16.4;
    ypr.z_gyro /= 16.4;

    // Complementary filter
    float accelPitch = atan2(ypr.y_accel, ypr.z_accel) * (180 / M_PI);
    float accelRoll = atan2(ypr.x_accel, ypr.z_accel) * (180 / M_PI);
  
    //ypr.roll = (tau)*(ypr.roll + ypr.y_gyro*dt) + (1-tau)*(accelRoll);
    //ypr.pitch = (tau)*(ypr.pitch + ypr.x_gyro*dt) + (1-tau)*(accelPitch);
    ypr.roll = kalmanRoll.update(accelRoll, ypr.y_gyro, dt);
    ypr.pitch = kalmanPitch.update(accelPitch, ypr.x_gyro, dt);
    ypr.yaw += ypr.z_gyro * dt;

    return ESP_OK;
}

esp_err_t Mpu6050::set_sleep_mode(bool sleep){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: sleep: %s\n", __func__, sleep? "true" : "false");
    esp_err_t status = 0;
    uint8_t data = 0;
    status = set_bits_8(data, sleep, PWR_MGMT_1_SLEEP_LSB);
    status = i2c->write(address, PWR_MGMT_1_REG, &data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, PWR_MGMT_1_REG, data);
    return status;
}

esp_err_t Mpu6050::reset(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint8_t data = 0;
    status = set_bits_8(data, 1u, PWR_MGMT_1_DEVICE_RESET_LSB);
    status = i2c->write(address, PWR_MGMT_1_REG, &data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, PWR_MGMT_1_REG, data);

    delay(30);

    return status;
}

esp_err_t Mpu6050::set_mem_bank(uint8_t bank, bool prefetchEnabled, bool userBank){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: bank %0x prefetchEnabled: %s userBank: %s\n", __func__, bank, prefetchEnabled? "true" : "false", userBank? "true" : "false");
    esp_err_t status = 0;
    uint8_t data = 0;

    bank &= 0x1F;       // what is this?
    if (userBank) bank |= 0x20; //why?
    if (prefetchEnabled) bank |= 0x40;  // que?

    status = i2c->write(address, BANK_SEL_REG, &bank, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, BANK_SEL_REG, data);
    return status;
}

esp_err_t Mpu6050::set_mem_start_addr(uint8_t addr){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: addr 0x%x\n", __func__, addr);
    esp_err_t status = 0;

    status = i2c->write(address, MEM_START_ADDR_REG, &addr, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, MEM_START_ADDR_REG, addr);
    return status;
}

esp_err_t Mpu6050::read_mem_byte(uint8_t& data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->read(address, MEM_R_W_REG, &data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", address, MEM_R_W_REG);

    return status;
}

esp_err_t Mpu6050::get_opt_bank_valid(uint8_t& data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->read(address, XG_OFFS_TC_REG, &data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", address, XG_OFFS_TC_REG);

    data &= 1u;

    return status;
}

esp_err_t Mpu6050::set_opt_bank_valid(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write_bit(address, XG_OFFS_TC_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to address 0x%x reg 0x%x data 0x%x", address, XG_OFFS_TC_REG, data);

    return status;
}

esp_err_t Mpu6050::set_slave_addr(uint8_t num, uint8_t addr){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: i2c num: %u addr 0x%x\n", __func__, num, addr);
    esp_err_t status = 0;
    ESP_RETURN_ON_ERROR((num > 3), log_tag, "Bad num: %u", num);

    status = i2c->write(address, I2C_SLV0_ADDR_REG + num *3, &addr, 1); // ??
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, I2C_SLV0_ADDR_REG , addr);

    return status; 
}

esp_err_t Mpu6050::setI2CMasterModeEnabled(bool enable){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: enable: %s\n", __func__, enable? "true" : "false");
    esp_err_t status = 0;
    uint8_t data = enable? 1u : 0u;

    status = i2c->write_bit(address, USER_CTRL_REG, data, USER_CTRL_I2C_MST_EN_LSB); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, USER_CTRL_REG , data);

    return status; 
}

esp_err_t Mpu6050::resetI2CMaster(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;

    status = i2c->write_bit(address, USER_CTRL_REG, 1u, USER_CTRL_I2C_MST_RESET_LSB); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, USER_CTRL_REG , 1u);

    return status; 
}

esp_err_t Mpu6050::set_clk_source(uint8_t src){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: src 0x%x\n", __func__, src);
    esp_err_t status = 0;
    uint8_t mask = 0x7;

    status = i2c->write_bit(address, PWR_MGMT_1_REG, src, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, PWR_MGMT_1_REG , src);

    return status; 
}

esp_err_t Mpu6050::set_int_enabled(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;
    status = i2c->write(address, INT_ENABLE_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, INT_ENABLE_REG , data);

    return status; 
}

esp_err_t Mpu6050::set_sample_rate(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;
    status = i2c->write(address, SMPLRT_DIV_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, SMPLRT_DIV_REG, data);

    return status; 
}

esp_err_t Mpu6050::set_external_frame_sync(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;
    uint8_t mask = 0b0001'1000;

    status = i2c->write_bit(address, CONFIG_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, CONFIG_REG , data);

    return status; 
}

esp_err_t Mpu6050::set_dlpf_mode(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;
    uint8_t mask = 0x7;

    status = i2c->write_bit(address, CONFIG_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, CONFIG_REG , data);

    return status; 

}

esp_err_t Mpu6050::set_full_scale_gyro_range(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;
    uint8_t mask = 0b0001'1000;

    status = i2c->write_bit(address, GYRO_CONFIG_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, GYRO_CONFIG_REG , data);

    return status; 
}

esp_err_t Mpu6050::set_full_scale_accel_range(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;
    uint8_t mask = 0b0001'1000;

    status = i2c->write_bit(address, ACCEL_CONFIG_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, ACCEL_CONFIG_REG , data);

    return status; 
}

esp_err_t Mpu6050:: set_dmp_config1(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write(address, DMP_CFG_1_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, DMP_CFG_1_REG, data);

    return status; 
}

esp_err_t Mpu6050::set_dmp_config2(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write(address, DMP_CFG_2_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, DMP_CFG_2_REG, data);

    return status; 
}

esp_err_t Mpu6050::set_motion_detection_thld(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write(address, MOT_THR_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, MOT_THR_REG, data);

    return status; 
}

esp_err_t Mpu6050::set_zero_motion_detection_thld(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write(address, ZRMOT_THR_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, ZRMOT_THR_REG, data);

    return status; 
}

esp_err_t Mpu6050::set_moion_dection_duration(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write(address, MOT_DUR_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, MOT_DUR_REG, data);

    return status; 
}

esp_err_t Mpu6050::set_zero_moion_dection_duration(uint8_t data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: data 0x%x\n", __func__, data);
    esp_err_t status = 0;

    status = i2c->write(address, ZRMOT_DUR_REG, &data, 1); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, ZRMOT_DUR_REG, data);

    return status; 

}

esp_err_t Mpu6050::set_fifo_enable(bool enable){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: enable: %s\n", __func__, enable? "true" : "false");
    esp_err_t status = 0;
    uint8_t mask = 1<<USER_CTRL_FIFO_EN_LSB;
    uint8_t data = enable? 1u : 0u;

    status = i2c->write_bit(address, USER_CTRL_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, USER_CTRL_REG , data);

    return status; 
}

esp_err_t Mpu6050::set_dmp_enabled(bool enable){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: enable: %s\n", __func__, enable? "true" : "false");
    esp_err_t status = 0;
    uint8_t mask = 1<<USER_CTRL_DMP_EN_LSB;
    uint8_t data = enable? 1u : 0u;

    status = i2c->write_bit(address, USER_CTRL_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, USER_CTRL_REG , data);

    return status; 
}

esp_err_t Mpu6050::reset_dmp(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint8_t mask = 1<< USER_CTRL_DMP_RESET_LSB;
    uint8_t data = 1;

    status = i2c->write_bit(address, USER_CTRL_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, USER_CTRL_REG , data);

    return status; 
}

esp_err_t Mpu6050::reset_fifo(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint8_t mask = 1<< USER_CTRL_FIFO_RESET_LSB;
    uint8_t data = 1;

    status = i2c->write_bit(address, USER_CTRL_REG, data, mask); 
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write to addr 0x%x reg 0x%x data 0x%x", address, USER_CTRL_REG , data);

    return status; 
}

esp_err_t Mpu6050::get_int_satus(uint8_t& data){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;

    status = i2c->read(address, INT_STATUS_REG, &data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", address, INT_STATUS_REG);

    return status; 
}

esp_err_t Mpu6050::calibrate_gyro(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;


    return status;
}

esp_err_t Mpu6050::get_6axis_motion(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint8_t data[14] = {};

    status = i2c->read(address, ACCEL_XOUT_H_REG, data, 14);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", address, ACCEL_XOUT_H_REG);

    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: acc  X H 0x%x L 0x%x\n", __func__, data[0], data[1]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: acc  Y H 0x%x L 0x%x\n", __func__, data[2], data[3]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: acc  Z H 0x%x L 0x%x\n", __func__, data[4], data[5]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: T      H 0x%x L 0x%x\n", __func__, data[6], data[7]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: gyro X H 0x%x L 0x%x\n", __func__, data[8], data[9]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: gyro Y H 0x%x L 0x%x\n", __func__, data[10], data[11]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: gyro Z H 0x%x L 0x%x\n", __func__, data[12], data[13]);

    ax = (((int16_t)data[0]) << 8) | data[1];
    ay = (((int16_t)data[2]) << 8) | data[3];
    az = (((int16_t)data[4]) << 8) | data[5];
    gx = (((int16_t)data[8]) << 8) | data[9];
    gy = (((int16_t)data[10]) << 8) |data[11];
    gz = (((int16_t)data[12]) << 8) |data[13];

    //printf("raw: w 0x.2x x 0x.2x y 0x.2x z 0x.2x", data[0], data[1], data[2], data[3]);

    return status;
}

esp_err_t Mpu6050::get_6axis_motion2(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint8_t data[14] = {};

    status = i2c->read(address, FIFO_R_W_REG, data, 14);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", address, ACCEL_XOUT_H_REG);

    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: acc  X H 0x%x L 0x%x\n", __func__, data[0], data[1]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: acc  Y H 0x%x L 0x%x\n", __func__, data[2], data[3]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: acc  Z H 0x%x L 0x%x\n", __func__, data[4], data[5]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: T      H 0x%x L 0x%x\n", __func__, data[6], data[7]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: gyro X H 0x%x L 0x%x\n", __func__, data[8], data[9]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: gyro Y H 0x%x L 0x%x\n", __func__, data[10], data[11]);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "%s: gyro Z H 0x%x L 0x%x\n", __func__, data[12], data[13]);

    ax = (((int16_t)data[0]) << 8) | data[1];
    ay = (((int16_t)data[2]) << 8) | data[3];
    az = (((int16_t)data[4]) << 8) | data[5];
    gx = (((int16_t)data[8]) << 8) | data[9];
    gy = (((int16_t)data[10]) << 8) |data[11];
    gz = (((int16_t)data[12]) << 8) |data[13];

    //printf("raw: w 0x.2x x 0x.2x y 0x.2x z 0x.2x", data[0], data[1], data[2], data[3]);

    return status;
}


/**
 * 
 * 
 * 
 */
//esp_err_t Mpu6050::get_curr_fifo_packet(uint8_t* buffer, uint8_t length, enum DmpFifoStatus& dmpStatus){
esp_err_t Mpu6050::get_curr_fifo_packet(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint16_t fifoCount = 0;
    bool completePacket = false;
    enum DmpFifoStatus dmpStatus;    
    uint8_t length = dmpPacketSize;

    // get fifo count FIFO_COUNT_H FIFO_COUNT_L
    status = get_fifo_count(fifoCount);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");

    if(!fifoCount){
        print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: no data: %u\n", __func__, __LINE__, fifoCount);
        dmpStatus = DMP_FIFO_OK;
        return ESP_OK;
    }

    // if until count > 200 then reset fifo
    while(!completePacket){
        //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: waiting for completePacket\n", __func__, __LINE__);

        if(fifoCount > dmpPacketSize){
            //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo %u > dmpPacketSize %u\n", __func__, __LINE__, fifoCount, dmpPacketSize);
            if(fifoCount > 200){
                //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo %u over 200 thld\n", __func__, __LINE__, fifoCount);
                status = reset_fifo();
                ESP_RETURN_ON_ERROR(status, log_tag, "Failed to reset fifo");
                fifoCount = 0;
            }
            else{
                //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo %u below 200 thld\n", __func__, __LINE__, fifoCount);
                while(true){

                    status = get_fifo_count(fifoCount);
                    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");

                    while(fifoCount > length){
                        uint8_t trashBuf[300];
                        uint8_t removeSize = fifoCount - length;  // save the latest length bytes
                        //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: clearing out fifo, bytes to clear: %u\n", __func__, __LINE__, removeSize);

                        while(removeSize){ // read until only length bytes left in fifo
                            uint8_t bytesToRemove = fifoCount < 64 ? fifoCount : 64; 
                            status = i2c->read(address, FIFO_R_W_REG, trashBuf, bytesToRemove);
                            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read fifo register");
                            removeSize -= bytesToRemove;
                            //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: clearing out fifo, bytes to clear: %u / %u\n", __func__, __LINE__, bytesToRemove, removeSize);
                        }

                        status = get_fifo_count(fifoCount);
                        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");
                        print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo count post clearing: %u\n", __func__, __LINE__, fifoCount - length);
                    }
                }
            }
        }

        while(!completePacket){
            status = get_fifo_count(fifoCount);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");
            completePacket = fifoCount == dmpPacketSize;
            ESP_RETURN_ON_ERROR((fifoCount>dmpPacketSize), log_tag, "Error fifoCount: %u > %u", fifoCount, dmpPacketSize);
        }
    } 

    //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: reading DMP packet of len %u\n", __func__, __LINE__, dmpPacketSize);
    status = i2c->read(address, FIFO_R_W_REG, dmpBuffer.get(), length);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read fifo register");

    return ESP_OK;
}

esp_err_t Mpu6050::get_curr_fifo_packet2(){
    esp_err_t status = 0;
    uint16_t fifoCount = 0;
    uint16_t fifoCountPrev = 0;
    uint8_t length = dmpPacketSize;

    status = get_fifo_count(fifoCount);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");

    if(!fifoCount){
        return ESP_ERR_INVALID_SIZE;
    }
    else if(fifoCount > 200){
        fifoCountPrev = fifoCount;
        reset_fifo();
    }
    else{

        status = get_fifo_count(fifoCount);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");

        while(fifoCount > length){
            uint8_t trashBuf[128];
            uint8_t removeSize = fifoCount - length;  // save the latest length bytes
            //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: clearing out fifo, bytes to clear: %u\n", __func__, __LINE__, removeSize);

            while(removeSize){ // read until only length bytes left in fifo
                uint8_t bytesToRemove = fifoCount < 128 ? fifoCount : 128; 
                status = i2c->read(address, FIFO_R_W_REG, trashBuf, bytesToRemove);
                ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read fifo register");
                removeSize -= bytesToRemove;
                //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: clearing out fifo, bytes to clear: %u / %u\n", __func__, __LINE__, bytesToRemove, removeSize);
            }

            status = get_fifo_count(fifoCount);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");
            //print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo count post clearing: %u\n", __func__, __LINE__, fifoCount - length);
        }
    }

    status = get_fifo_count(fifoCount);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");
    ESP_RETURN_ON_ERROR((fifoCount > dmpPacketSize), log_tag, "Error fifoCount: %u was larger than dmpPacketsize: %u", fifoCount, dmpPacketSize);

    if(!fifoCount){
        return ESP_ERR_INVALID_SIZE;
    }

    while(fifoCount != dmpPacketSize){
            status = get_fifo_count(fifoCount);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");
    }

    status = i2c->read(address, FIFO_R_W_REG, dmpBuffer.get(), dmpPacketSize);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read fifo register");
    //printf("data: w: 0x%.2x 0x%.2x x: 0x%.2x 0x%.2x y: 0x%.2x 0x%.2x z: 0x%.2x 0x%.2x\n", 
    //dmpBuffer[0], dmpBuffer[1], dmpBuffer[4], dmpBuffer[5], dmpBuffer[8], dmpBuffer[9], dmpBuffer[12], dmpBuffer[13]);
    set_new_data_exists();

    return status;
}

esp_err_t Mpu6050::get_dmp_packet() { // overflow proof    

    esp_err_t status = 0;
    uint16_t count = 0;

    status = i2c->read(this->address, FIFO_R_W_REG, dmpBuffer.get(), (uint8_t)dmpPacketSize);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read DMP data");

    set_new_data_exists();

    return status;
}

esp_err_t Mpu6050::get_fifo_count(uint16_t& count){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    uint8_t data[2] = {};

    status = i2c->read(address, FIFO_COUNTH_REG, data, 2);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read address 0x%x reg 0x%x", address, FIFO_COUNTH_REG);

    count = (uint16_t)data[0] << 8 | data[1];

    return status;
}



/////// borrowed functions ////////////

esp_err_t Mpu6050::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: dataSize %u bank 0x%x address 0x%x verify: %s\n", __func__, dataSize, bank, address, verify? "true" : "false");
    return writeMemoryBlock(data, dataSize, bank, address, verify, true);
}

esp_err_t Mpu6050::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: dataSize %u bank 0x%x address 0x%x verify: %s useProgMem: %s\n", __func__, dataSize, bank, address, verify? "true" : "false", useProgMem? "true" : "false");
    uint8_t chunkSize;
    uint8_t *verifyBuffer=0;
    uint8_t *progBuffer=0;
    uint16_t i;
    uint8_t j;
    constexpr uint8_t dmp_chunk_size = 16u;
    esp_err_t status = 0;

    status = set_mem_bank(bank);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem bank %u\n", bank);
    status = set_mem_start_addr(address);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem start addr 0x%x\n", address);

    if (verify) verifyBuffer = (uint8_t *)malloc(dmp_chunk_size);  
    if (useProgMem) progBuffer = (uint8_t *)malloc(dmp_chunk_size); 
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = dmp_chunk_size;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        if (useProgMem) {
            // i2c->write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // i2c->write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        //I2Cdev::i2c->writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer, wireObj);
        status = i2c->write(this->address, MEM_R_W_REG, progBuffer, chunkSize);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write reg 0x%x size %u\n", MEM_R_W_REG, chunkSize);

        // verify data if needed
        if (verify && verifyBuffer) {
            status = set_mem_bank(bank);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem bank %u\n", bank);
            status = set_mem_start_addr(address);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem start addr 0x%x\n", address);

            //I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer, I2Cdev::readTimeout, wireObj);
            status = i2c->read(this->address, MEM_R_W_REG, verifyBuffer, chunkSize);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read reg 0x%x size %u\n", MEM_R_W_REG, chunkSize);

            //printf("prog:");
            //for(int i=0;i<16;i++){
            //    printf(" 0x%.2x", progBuffer[i]);
            //}
            //printf("\n");
            //printf("veri:");
            //for(int i=0;i<16;i++){
            //    printf(" 0x%.2x", verifyBuffer[i]);
            //}
            //printf("\n");

            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                /*Serial.print("Block i2c->write verification error, bank ");
                Serial.print(bank, DEC);
                Serial.print(", address ");
                Serial.print(address, DEC);
                Serial.print("!\nExpected:");
                for (j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (progBuffer[j] < 16) Serial.print("0");
                    Serial.print(progBuffer[j], HEX);
                }
                Serial.print("\nReceived:");
                for (uint8_t j = 0; j < chunkSize; j++) {
                    Serial.print(" 0x");
                    if (verifyBuffer[i + j] < 16) Serial.print("0");
                    Serial.print(verifyBuffer[i + j], HEX);
                }
                Serial.print("\n");*/
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return ESP_FAIL; // uh oh.
            }

        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            status = set_mem_bank(bank);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem bank: %u\n", bank);

            status = set_mem_start_addr(address);
            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mem start addr: 0x%x\n", address);
        }
        vTaskDelay(10);
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return status;
}


esp_err_t Mpu6050::dump_registers(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "*****************************\n");
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "************ DUMP ***********\n");
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "*****************************\n");
    for (const auto& v: regMapX)
    {
        uint8_t buffer = 0;
        status = i2c->read(address, v.first, &buffer, 1);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read reg 0x%x %-30s\n", v.first, v.second.c_str());
        std::bitset<8> bits{buffer};
        print_debug(DEBUG_MPU6050, DEBUG_DATA, "0x%.2x %-30s: 0x%.2x", v.first, v.second.c_str(), buffer);
        std::cout<<" "<<bits<<"\n";

    }

    print_debug(DEBUG_MPU6050, DEBUG_DATA, "*****************************\n");
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "*****************************\n");
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "*****************************\n");
    return status;
}

esp_err_t Mpu6050::set_x_accel_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, XA_OFFS_H_REG, data, 2);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write x accel offset reg 0x%x data 0x%x 0x%x\n", XA_OFFS_H_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_y_accel_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, YA_OFFS_H_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write y accel offset reg 0x%x data 0x%x 0x%x\n", YA_OFFS_H_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_z_accel_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, ZG_OFFS_USRH_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write z accel offset reg 0x%x data 0x%x 0x%x\n", ZA_OFFS_H_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_x_gyro_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, XG_OFFS_USRH_REG, data, 2);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write x gyro offset reg 0x%x data 0x%x 0x%x\n", XG_OFFS_USRH_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_y_gyro_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, YG_OFFS_USRH_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write y gyro offset reg 0x%x data 0x%x 0x%x\n", YG_OFFS_USRH_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_z_gyro_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, ZG_OFFS_USRH_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write z gyro offset reg 0x%x data 0x%x 0x%x\n", ZG_OFFS_USRH_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::calibrate_accel(uint8_t loops){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: loops %u\n", __func__, loops);
    
    esp_err_t status = 0;

    // why these values?
    float kP = 0.3;
    float kI = 20;
    float in_min = 1;
    float in_max = 5;
    float out_min = 20;
    float out_max = 0;
    float x = loops;

     x = (100 - (long)((x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min)) * 0.1; //why this formula?
     kP *= x;
	 kI *= x;
     print_debug(DEBUG_MPU6050, DEBUG_DATA, "%-33s: x: %f kP: %f kI: %f\n", __func__, x, kP, kI);

    return status;
}

esp_err_t Mpu6050::calibrate_gyro(uint8_t loops){
    esp_err_t status = 0;


    return status;
}

esp_err_t Mpu6050::get_quaternion(Quaternion& quaternion){

    
    ESP_RETURN_ON_ERROR(!new_data_exists(), log_tag, "No new data exists\n");

    int16_t w =(int16_t)((dmpBuffer[0]  << 8) | dmpBuffer[1]);      //
    int16_t x =(int16_t)((dmpBuffer[4]  << 8) | dmpBuffer[5]);      //
    int16_t y =(int16_t)((dmpBuffer[8]  << 8) | dmpBuffer[9]);      //
    int16_t z =(int16_t)((dmpBuffer[12] << 8) | dmpBuffer[13]);     //
	//printf("raw: w 0x%.2x x 0x%.2x y 0x%.2x z 0x%.2x\n", (uint16_t)w, (uint16_t)x, (uint16_t)y, (uint16_t)z);

    quaternion.w = (float)w / 16384.0f;     // 
    quaternion.x = (float)x / 16384.0f;     //
    quaternion.y = (float)y / 16384.0f;     //
    quaternion.z = (float)z / 16384.0f;     //

    return ESP_OK;
}

esp_err_t Mpu6050::get_gravity(VectorFloat& vector, Quaternion quaternion){

    //v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    //v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    // v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;

    vector.x = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
    vector.y = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
    vector.z = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;

    return ESP_OK;
}

esp_err_t Mpu6050::get_yaw_pitch_roll(Quaternion& q, VectorFloat& gravity, YawPitchRoll& r_yawPitchRoll) {

    // yaw: (about Z axis)
    ypr.yaw   = atan2(2*q.x*q.y - 2*q.w*q.z, 2*q.w*q.w + 2*q.x*q.x - 1);
    // pitch: (nose up/down, about Y axis)
    ypr.pitch = atan2(gravity.x , sqrt(gravity.y*gravity.y + gravity.z*gravity.z));
    // roll: (tilt left/right, about X axis)
    ypr.roll  = atan2(gravity.y , gravity.z);

    if (gravity.z < 0) {
        if(ypr.pitch > 0) {
            ypr.pitch = M_PI - ypr.pitch; 
        } else { 
            ypr.pitch = -M_PI - ypr.pitch;
        }
    }

    float tmp{};
    switch(this->rotation){
        case ROTATE_0:
            break;
        case ROTATE_90:
            tmp = ypr.roll;
            ypr.roll = ypr.pitch;
            ypr.pitch = tmp * -1;
            break;
        case ROTATE_180:
            //not testeed
            ypr.roll *= -1;
            ypr.pitch *= -1;
            ypr.yaw *= -1;
            break;
        case ROTATE_270:
            //not implemented
        
            break;
        default:
            printf("ERROR bad rotation: %u", this->rotation);
    }
    //printf("\033[?25l");
    //printf("[2A[2K  Roll    Pitch     Yaw\n");
    //printf("[2K%-5.3f °C %-5.3f °C %-5.3f °C\n", ypr.roll * RAD_TO_DEG, ypr.pitch * RAD_TO_DEG, ypr.yaw * RAD_TO_DEG);

    r_yawPitchRoll = this->ypr;

    return ESP_OK;
}


void Mpu6050::delay(const uint8_t milli){

    TickType_t xDelay  = milli / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);

}

//counter clockwise
esp_err_t Mpu6050::set_chip_rotation(enum MpuRotation rotation){

    this->rotation = rotation;

    return ESP_OK;
}


esp_err_t Mpu6050::pid_calibrate(float target, float curr, float roll, int nrLoops){
  esp_err_t status = 0;

    float kP{0.1};
    float kI{0.1};
    float kD{0.1};
    float c{};
    float prevErr{0};
    float integral{};
    float dt{1};
    float derivative{};

    uint8_t reg{};
    uint8_t data[2]{};
    uint16_t measurement{};
    Pid pidObj{};


    for(int i=0;i<nrLoops;i++){
        
        // read yaw, pitch, roll: eg pitch 5 degrees


        // read offset value: eg 350

        // target degree: 0

        // write new offset = 350 + c

        status = i2c->read(this->address, reg, data, 2u);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write offset 0x%x\n", reg);

        measurement = (data[1] << 8 | data[0]);
        float current = measurement;

        pid(target, measurement, pidObj);

    }
    return status;
}


void Mpu6050::pid(float target, float current, Pid& pid){

    float kPOut{};
    float kIOut{};
    float kDOut{};
    float err{};
    float integral{};
    float derivative{};

    err = target - current;

    kPOut = pid.kP * err;

    integral = err * pid.dt;
    kIOut = pid.kI * integral;

    derivative = (err - pid.prevErr) / pid.dt;
    kDOut = pid.kD * derivative;

    pid.prevErr = err;
    pid.c = kPOut + kIOut + kDOut;

}
