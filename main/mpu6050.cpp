#include <pgmspace.h>
#include <cstring>
#include <stdlib.h>
#include <memory>
#include <bitset>
#include <iostream>
#include <algorithm> 


#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "mpu6050.h"
#include "debug.h"
#include "i2c.h"


#define log_tag "mpu6050"



using namespace mpu6050Value;


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

}

esp_err_t Mpu6050::mpu6050_init(){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s:\n", __func__);
    esp_err_t status = 0;

    status = set_clk_source(0x1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set clock source: 0x%x", 0);

    status = set_sample_rate(0);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set sample rate: 0x%x", 0);

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
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to validate opt bank %u", valid);
    ESP_RETURN_ON_ERROR(!valid, log_tag, "Failed to validate opt bank %u", valid);
    print_debug(DEBUG_MPU6050, DEBUG_DATA, "valid: 0x%x\n", valid);

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

    // setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    status = set_clk_source(0x3);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set clk source 0x%x", 0x3);

    // setIntEnabled(1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT|1<<MPU6050_INTERRUPT_DMP_INT_BIT);
    status = set_int_enabled((1<<INT_ENABLE_FIFO_OFLOW_EN_LSB) | (1<< INT_ENABLE_DMP_INT_EN_LSB));
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set int enable  0x%x", (1<<INT_ENABLE_FIFO_OFLOW_EN_LSB) | (1<< INT_ENABLE_DMP_INT_EN_LSB));

    // 	setRate(4); // 1khz / (1 + 4) = 200 Hz
    status = set_sample_rate(4u);
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

    delay(20);

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
        print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: witing for completePacket\n", __func__, __LINE__);

        if(fifoCount > dmpPacketSize){
            print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo %u > dmpPacketSize %u\n", __func__, __LINE__, fifoCount, dmpPacketSize);
            if(fifoCount > 200){
                print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo %u over 200 thld\n", __func__, __LINE__, fifoCount);
                status = reset_fifo();
                ESP_RETURN_ON_ERROR(status, log_tag, "Failed to reset fifo");
                fifoCount = 0;
            }
            else{
                print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo %u below 200 thld\n", __func__, __LINE__, fifoCount);
                while(true){

                    status = get_fifo_count(fifoCount);
                    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");

                    while(fifoCount > length){
                        uint8_t trashBuf[300];
                        uint8_t removeSize = fifoCount - length;  // save the latest length bytes
                        print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: clearing out fifo, bytes to clear: %u\n", __func__, __LINE__, removeSize);

                        while(removeSize){ // read until only length bytes left in fifo
                            uint8_t bytesToRemove = fifoCount < 64 ? fifoCount : 64; 
                            status = i2c->read(address, FIFO_R_W_REG, trashBuf, bytesToRemove);
                            ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read fifo register");
                            removeSize -= bytesToRemove;
                            print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: clearing out fifo, bytes to clear: %u / %u\n", __func__, __LINE__, bytesToRemove, removeSize);
                        }

                        status = get_fifo_count(fifoCount);
                        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to get fifo count");
                        print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: fifo count post clearing: %u\n", __func__, __LINE__, fifoCount - length);
                    }
                }
            }
        }
        if(!fifoCount){
            print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: no data: %u\n", __func__, __LINE__, fifoCount);
            dmpStatus = DMP_FIFO_OK;
            return ESP_OK;
        }
        completePacket = fifoCount == dmpPacketSize;
    } 
    print_debug(DEBUG_MPU6050, DEBUG_LOGIC, "%-33s %u: reading DMP packet of len %u\n", __func__, __LINE__, dmpPacketSize);
    status = i2c->read(address, FIFO_R_W_REG, dmpBuffer.get(), length);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to read fifo register");

    return ESP_OK;
}

/*
int8_t GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof
     int16_t fifoC;
     // This section of code is for when we allowed more than 1 packet to be acquired
     uint32_t BreakTimer = micros();
     bool packetReceived = false;
     do {
         if ((fifoC = getFIFOCount())  > length) {

             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 resetFIFO(); // Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = getFIFOCount()) && ((micros() - BreakTimer) <= (getFIFOTimeout()))); // Get Next New Packet
            } 
            else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
               
                uint8_t Trash[I2CDEVLIB_WIRE_BUFFER_LENGTH];

                while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                                        //180   < 32? (false) 180 : (true) 32
                         RemoveBytes = (fifoC < I2CDEVLIB_WIRE_BUFFER_LENGTH) ? fifoC : I2CDEVLIB_WIRE_BUFFER_LENGTH; // Buffer Length is different than the packet length this will efficiently clear the buffer
                         getFIFOBytes(Trash, (uint8_t)RemoveBytes);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }

         if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         packetReceived = fifoC == length;
         if (!packetReceived && (micros() - BreakTimer) > (getFIFOTimeout())) return 0;
     } while (!packetReceived);
     getFIFOBytes(data, length); //Get 1 packet
     return 1;
}

*/

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
            status = i2c->read(address, MEM_R_W_REG, verifyBuffer, chunkSize);
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

    status = i2c->write(address, ZA_OFFS_H_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write z accel offset reg 0x%x data 0x%x 0x%x\n", ZA_OFFS_H_REG, data[0], data[1]);

    return status;
}
esp_err_t Mpu6050::set_x_gyro_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, GYRO_CONFIG_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write x gyro offset reg 0x%x data 0x%x 0x%x\n", MEM_R_W_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_y_gyro_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, GYRO_CONFIG_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write y gyro offset reg 0x%x data 0x%x 0x%x\n", MEM_R_W_REG, data[0], data[1]);

    return status;
}

esp_err_t Mpu6050::set_z_gyro_offset(int16_t offset){
    print_debug(DEBUG_MPU6050, DEBUG_ARGS, "%-33s: offset %d\n", __func__, offset);
    esp_err_t status = 0;
    uint8_t data[2] = { (uint8_t)(offset >> 8), (uint8_t)(offset & 0xff)};

    status = i2c->write(address, GYRO_CONFIG_REG, data, 1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to i2c->write z gyro offset reg 0x%x data 0x%x 0x%x\n", MEM_R_W_REG, data[0], data[1]);

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

void Mpu6050::delay(const uint8_t milli){

    TickType_t xDelay  = milli / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);

}