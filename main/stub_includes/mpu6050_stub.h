#pragma once

#include <map>
#include <string>
#include <memory>

#include "esp_check.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "common_data.h"
#include "i2c.h"
#include "quaternion.h"
 
#include "mpu6050.h"



class Mpu6050_stub{

    static Mpu6050_stub* mpu;
	static SemaphoreHandle_t dmp_avail_sem;
	static RingbufHandle_t dmp_buf_handle;
	static RingbufHandle_t web_buf_handle;

    //enum Mpu6050Addr address;
	enum MpuRotation rotation{ROTATE_0};

    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t frequency;
	uint8_t dmpPacketSize;

	std::unique_ptr<uint8_t[]> dmpBuffer{nullptr};

	bool newData{};

    I2cHandler* i2c;
	YawPitchRoll ypr{};

	/*** methods ***/
    esp_err_t i2c_write();
	static void IRAM_ATTR dmp_data_handler(void *args);
    Mpu6050_stub(enum Mpu6050Addr addr, I2cHandler* i2c);


    public:

    enum Mpu6050Addr address;

	Mpu6050_stub(Mpu6050_stub &other) = delete;
    void operator=(const Mpu6050_stub &) = delete;
    static Mpu6050_stub *GetInstance(enum Mpu6050Addr address, I2cHandler* i2c);
	static Mpu6050_stub *GetInstance();

    esp_err_t mpu6050_init();
    esp_err_t dmp_init();
	void dmp_task(void* args);
	void raw_task(void* args);

	RingbufHandle_t get_queue_handle(){
		return dmp_buf_handle;
	}

    esp_err_t get_6axis_motion(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
	esp_err_t get_6axis_motion2(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
    esp_err_t calibrate_gyro();
    esp_err_t set_sleep_mode(bool sleep);

    esp_err_t set_mem_bank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);
    esp_err_t set_mem_start_addr(uint8_t addr);
    esp_err_t read_mem_byte(uint8_t& data);

    esp_err_t get_opt_bank_valid(uint8_t& data);
    esp_err_t set_opt_bank_valid(uint8_t data);

    esp_err_t set_slave_addr(uint8_t num, uint8_t addr);
    esp_err_t setI2CMasterModeEnabled(bool enable);
    esp_err_t set_clk_source(uint8_t src);
    esp_err_t set_sample_rate(uint8_t data);
    esp_err_t set_external_frame_sync(uint8_t data);
    esp_err_t set_dlpf_mode(uint8_t data);

	esp_err_t set_chip_rotation(enum MpuRotation);

    esp_err_t set_full_scale_gyro_range(uint8_t);
    esp_err_t set_full_scale_accel_range(uint8_t data);

    esp_err_t set_dmp_config1(uint8_t data);
    esp_err_t set_dmp_config2(uint8_t data);

    esp_err_t set_motion_detection_thld(uint8_t data);
    esp_err_t set_moion_dection_duration(uint8_t data);

    esp_err_t set_zero_motion_detection_thld(uint8_t data);
    esp_err_t set_zero_moion_dection_duration(uint8_t data);

    esp_err_t set_int_enabled(uint8_t data);
    esp_err_t set_fifo_enable(bool enable);
    esp_err_t set_dmp_enabled(bool enable);

    esp_err_t reset();
    esp_err_t reset_dmp();
    esp_err_t reset_fifo();
    esp_err_t resetI2CMaster();

    esp_err_t get_int_satus(uint8_t& data);

    esp_err_t get_curr_fifo_packet();
	esp_err_t get_curr_fifo_packet2();
	esp_err_t get_dmp_packet();

    esp_err_t get_fifo_count(uint16_t& count);

    esp_err_t dump_registers();
    esp_err_t dump_dmp_fw();

    esp_err_t set_x_accel_offset(int16_t offset);
    esp_err_t set_y_accel_offset(int16_t offset);
    esp_err_t set_z_accel_offset(int16_t offset);
    esp_err_t set_x_gyro_offset(int16_t offset);
    esp_err_t set_y_gyro_offset(int16_t offset);
    esp_err_t set_z_gyro_offset(int16_t offset);
    esp_err_t calibrate_accel(uint8_t loops);
	esp_err_t calibrate_gyro(uint8_t loops);

	esp_err_t comp_filter(double dt, float tau, YawPitchRoll& ypr);
	esp_err_t get_quaternion(Quaternion& quaternion);
	esp_err_t get_gravity(VectorFloat& vector, Quaternion quaternion);
	esp_err_t get_yaw_pitch_roll(Quaternion& q, VectorFloat& gravity, YawPitchRoll& r_YawPitchRoll);

	bool new_data_exists(){
		if(newData){
			newData = false;
			return true;
		}
		return false;
	}

	void set_new_data_exists(){newData = true;};

    void delay(const uint8_t milli);

	esp_err_t pid_calibrate(float yaw, float pitch, float roll, int nrLoops);
	void pid(float target, float current, Pid& pid);


    // borrowed
    esp_err_t writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true);
    esp_err_t writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true, bool useProgMem=false);

};


#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01


