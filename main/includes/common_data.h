#pragma once

#include <iostream>
#include <map>
#include <vector>
#include <string>

#define RAD_TO_DEG (180.0/M_PI)


#define RAD_TO_DEGREE (180.0 / M_PI)

namespace Radio{
    static constexpr int maxChannelValue{1811};
    static constexpr int maxChannelThld{maxChannelValue-100};

    static constexpr int minChannelValue{174};
    static constexpr int minChannelThld{minChannelValue+100};

    static constexpr int neutralChannelValue{997};

    static constexpr uint8_t maxChannels{11};
}
 

typedef enum Motor{
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4
} motor_type_t;


typedef enum MotorDirection{
    NORMAL,
    REVERSED,
};


/* chatgpt created */
struct Kalman { 
    float angle = 0.0; // Estimated angle
    float bias = 0.0;  // Gyro bias
    float rate = 0.0;  // Gyro rate

    float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix
    float Q_angle = 0.001; // Process noise variance for the angle
    float Q_bias = 0.003;  // Process noise variance for the gyro bias
    float R_measure = 0.03; // Measurement noise variance

    float update(float newAngle, float newRate, float dt) {
        // Predict step
        rate = newRate - bias;
        angle += dt * rate;

        // Update error covariance matrix
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Calculate Kalman gain
        float S = P[0][0] + R_measure;
        float K[2] = {P[0][0] / S, P[1][0] / S};

        // Update estimate
        float y = newAngle - angle; // Angle difference
        angle += K[0] * y;
        bias += K[1] * y;

        // Update error covariance matrix
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};


typedef struct __attribute__((packed)){
    unsigned ch1  : 11; // right stick x axis 
    unsigned ch2  : 11; // right stick y axis
    unsigned ch3  : 11; // left  stick y axis
    unsigned ch4  : 11; // left  stick x axis
    unsigned ch5  : 11; // left  upper trigger (arm)
    unsigned ch6  : 11; // left  3-point switch (flight mode?)
    unsigned ch7  : 11; // right 3-point switch
    unsigned ch8  : 11; // right upper trigger
    unsigned ch9  : 11; // left  bottom trigger
    unsigned ch10 : 11; // right bottom scroll   241 - 1792
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    unsigned ch16 : 11;

    std::string to_str(){

        std::string str;

        str += std::to_string(ch1) + ",";
        str += std::to_string(ch2) + ",";
        str += std::to_string(ch3) + ",";
        str += std::to_string(ch4) + ",";
        str += std::to_string(ch5) + ",";
        str += std::to_string(ch6) + ",";
        str += std::to_string(ch7) + ",";
        str += std::to_string(ch8) + ",";
        str += std::to_string(ch9) + ",";
        str += std::to_string(ch10) + ",";
        str += std::to_string(ch11) + ",";
        str += std::to_string(ch12) + ",";
        str += std::to_string(ch13) + ",";
        str += std::to_string(ch14) + ",";
        str += std::to_string(ch15) + ",";
        str += std::to_string(ch16);


        return str;
    }
    
} Channel;


struct RadioStatistics{
    uint8_t upplink_rssi_1;
    uint8_t upplink_rssi_2;
    uint8_t upplink_quality;
    uint8_t upplink_snr;
    uint8_t upplink_active_antenna;
    uint8_t rf_mode;
    uint8_t upplink_tx_power;
};



enum FlightMode{
    ANGLE_MODE,
    ACRO_MODE,
    SELFLEVL_MODE,
};

struct EscTelemetry{
    
    uint8_t temperature{};
    uint16_t voltage{};
    uint16_t current{};
    uint16_t consumption{};
    uint16_t rpm{};

    std::string to_str(){
        return std::to_string(temperature) + "," + 
               std::to_string(voltage * 0.10) + "," + 
               std::to_string(current * 0.10) + "," + 
               std::to_string(consumption * 0.10)  + "," + 
               std::to_string(rpm);
    }

};

struct DroneState{

    bool isPaired{};
    bool isControllerArmed{};
    bool isDroneArmed{};
    bool isGrounded{};
    bool calibrateGyro{};

    float currentDraw{};

    enum FlightMode mode;
    uint32_t dmpFreq{};
    uint32_t radioFreq{};
    uint32_t loopFreq{};

    uint16_t throttle{};
    uint16_t motorRlSpeed{};
    uint16_t motorRrSpeed{};
    uint16_t motorFlSpeed{};
    uint16_t motorFrSpeed{};

    float pidCPitch{};
    float pidCYaw{};

    float targetPitch{};
    float targetYaw{};
    float targetRoll{};
    float currPitch{};
    float currYaw{};
    float currRoll{};

    EscTelemetry escState[4];

    std::string throttle_fl_to_str(){return std::to_string(motorFlSpeed);}

    std::string throttle_fr_to_str(){return std::to_string(motorFrSpeed);}

    std::string throttle_rl_to_str(){return std::to_string(motorRlSpeed);}

    std::string throttle_rr_to_str(){return std::to_string(motorRrSpeed);}

    std::string current_to_str(){return std::to_string(currentDraw);}


};

struct YawPitchRoll{
	float yaw{};
	float pitch{};
	float roll{};

    int16_t z_accel{};
    int16_t x_accel{};
    int16_t y_accel{};
    int16_t z_gyro{};
    int16_t x_gyro{};
    int16_t y_gyro{};

    void operator = (const YawPitchRoll& ypr){
        this->yaw = ypr.yaw;
        this->pitch = ypr.pitch;
        this->roll = ypr.roll;
        this->z_accel = ypr.z_accel;
        this->x_accel = ypr.x_accel;
        this->y_accel = ypr.y_accel;
        this->z_gyro = ypr.z_gyro;
        this->y_gyro = ypr.x_gyro;
        this->x_gyro = ypr.y_gyro;
    }


    bool operator == (const YawPitchRoll& ypr){

        if(
            this->yaw     != ypr.yaw ||
            this->pitch   != ypr.pitch ||
            this->roll    != ypr.roll ||
            this->z_accel != ypr.z_accel ||
            this->x_accel != ypr.x_accel ||
            this->y_accel != ypr.y_accel ||
            this->z_gyro  != ypr.z_gyro ||
            this->x_gyro  != ypr.x_gyro ||
            this->y_gyro  != ypr.y_gyro){
            return false;
        }
        return true;
    }

    bool operator != (const YawPitchRoll& ypr){


        if(
            this->yaw     == ypr.yaw &&
            this->pitch   == ypr.pitch &&
            this->roll    == ypr.roll &&
            this->z_accel == ypr.z_accel &&
            this->x_accel == ypr.x_accel &&
            this->y_accel == ypr.y_accel &&
            this->z_gyro  == ypr.z_gyro &&
            this->x_gyro  == ypr.x_gyro &&
            this->y_gyro  == ypr.y_gyro){
            return false;
        }
        return true;
    }

	std::string to_str(){
		return (std::to_string(yaw) + "," + std::to_string(pitch) + "," + std::to_string(roll));
	}
};

struct TelemetryData{
    Channel channel{};
    RadioStatistics radioStatistics{};
    YawPitchRoll ypr1{};
    YawPitchRoll ypr2{};
    DroneState drone{};
};

struct Pid{
    float kP{.5};
    float kI{0.1};
    float kD{0.1};
	float dt{0.1};
    float c{};
    float prevErr{};
    float integral{};
};


typedef enum Progress{
    NOT_STARTED,
    STARTED,
    PASS,
    FAILED
}progress_t;

enum PidConfigDisplay{
    P,
    I,
    D,
    APPLY,
    CANCEL_PID,
    PID_CONFIG_APPLY,
    PID_CONFIG_CANCEL
};


enum MpuReg{
    XG_OFFS_TC_REG         = 0x00, //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
    YG_OFFS_TC_REG         = 0x01, //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
    ZG_OFFS_TC_REG         = 0x02, //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
    XA_OFFS_H_REG          = 0x06, //[15:0] XA_OFFS
    XA_OFFS_L_TC_REG       = 0x07,
    YA_OFFS_H_REG          = 0x08, //[15:0] YA_OFFS
    YA_OFFS_L_TC_REG       = 0x09, 
    ZA_OFFS_H_REG          = 0x0A, //[15:0] ZA_OFFS
    ZA_OFFS_L_TC_REG       = 0x0B, 
    SELF_TEST_X_REG        = 0x0D,
    SELF_TEST_Y_REG        = 0x0E,
    SELF_TEST_Z_REG        = 0x0F,
    SELF_TEST_A_REG        = 0x10,
    XG_OFFS_USRH_REG       = 0x13, //[15:0] XG_OFFS_USR
    XG_OFFS_USRL_REG       = 0x14,
    YG_OFFS_USRH_REG       = 0x15, //[15:0] YG_OFFS_USR
    YG_OFFS_USRL_REG       = 0x16,
    ZG_OFFS_USRH_REG       = 0x17, //[15:0] ZG_OFFS_USR
    ZG_OFFS_USRL_REG       = 0x18,
    SMPLRT_DIV_REG         = 0x19,
    CONFIG_REG             = 0x1A,
    GYRO_CONFIG_REG        = 0x1B,
    ACCEL_CONFIG_REG       = 0x1C,
    MOT_THR_REG            = 0x1F,
    MOT_DUR_REG            = 0x20, //unknown
    ZRMOT_THR_REG          = 0x21, //unknown
    ZRMOT_DUR_REG          = 0x22,
    FIFO_EN_REG            = 0x23,
    I2C_MST_CTRL_REG       = 0X24,
    I2C_SLV0_ADDR_REG      = 0x25,
    I2C_SLV0_REG_REG       = 0x26,
    I2C_SLV0_CTRL_REG      = 0x27,
    I2C_SLV1_ADDR_REG      = 0x28,
    I2C_SLV1_REG_REG       = 0x29,
    I2C_SLV1_CTRL_REG      = 0x2A,
    I2C_SLV2_ADDR_REG      = 0x2B,
    I2C_SLV2_REG_REG       = 0x2C,
    I2C_SLV2_CTRL_REG      = 0x2D,
    I2C_SLV3_ADD_REG       = 0x2E,
    I2C_SLV3_REG_REG       = 0x2F,
    I2C_SLV3_CTRL_REG      = 0x30,
    I2C_SLV4_ADDR_REG      = 0x31,
    I2C_SLV4_REG_REG       = 0x32,
    I2C_SLV4_DO_REG        = 0x33,
    I2C_SLV4_CTRL_REG      = 0x34,
    I2C_SLV4_DI_REG        = 0x35,
    I2C_MST_STATUS_REG     = 0x36,
    INT_PIN_CFG_REG        = 0x37,
    INT_ENABLE_REG         = 0x38,
    INT_STATUS_REG         = 0x3A,
    ACCEL_XOUT_H_REG       = 0x3B,
    ACCEL_XOUT_L_REG       = 0x3C,
    ACCEL_YOUT_H_REG       = 0x3D,
    ACCEL_YOUT_L_REG       = 0x3E,
    ACCEL_ZOUT_H_REG       = 0x3F,
    ACCEL_ZOUT_L_REG       = 0x40,
    TEMP_OUT_H_REG         = 0x41,
    TEMP_OUT_L_REG         = 0x42,
    GYRO_XOUT_H_REG        = 0x43,
    GYRO_XOUT_L_REG        = 0x44,
    GYRO_YOUT_H_REG        = 0x45,
    GYRO_YOUT_L_REG        = 0x46,
    GYRO_ZOUT_H_REG        = 0x47,
    GYRO_ZOUT_L_REG        = 0x48,
    EXT_SENS_DATA_00_REG   = 0x49,
    EXT_SENS_DATA_01_REG   = 0x4A,
    EXT_SENS_DATA_02_REG   = 0x4B,
    EXT_SENS_DATA_03_REG   = 0x4C,
    EXT_SENS_DATA_04_REG   = 0x4D,
    EXT_SENS_DATA_05_REG   = 0x4E,
    EXT_SENS_DATA_06_REG   = 0x4F,
    EXT_SENS_DATA_07_REG   = 0x50,
    EXT_SENS_DATA_08_REG   = 0x51,
    EXT_SENS_DATA_09_REG   = 0x52,
    EXT_SENS_DATA_10_REG   = 0x53,
    EXT_SENS_DATA_11_REG   = 0x54,
    EXT_SENS_DATA_12_REG   = 0x55,
    EXT_SENS_DATA_13_REG   = 0x56,
    EXT_SENS_DATA_14_REG   = 0x57,
    EXT_SENS_DATA_15_REG   = 0x58,
    EXT_SENS_DATA_16_REG   = 0x59,
    EXT_SENS_DATA_17_REG   = 0x5A,
    EXT_SENS_DATA_18_REG   = 0x5B,
    EXT_SENS_DATA_19_REG   = 0x5C,
    EXT_SENS_DATA_20_REG   = 0x5D,
    EXT_SENS_DATA_21_REG   = 0x5E,
    EXT_SENS_DATA_22_REG   = 0x5F,
    EXT_SENS_DATA_23_REG   = 0x60,
    I2C_SLV0_DO_REG        = 0x63,
    I2C_SLV1_DO_REG        = 0x64,
    I2C_SLV2_DO_REG        = 0x65,
    I2C_SLV3_DO_REG        = 0x66,
    I2C_MST_DELAY_CTRL_REG = 0x67,
    SIGNAL_PATH_RESET_REG  = 0x68,
    MOT_DETECT_CTRL_REG    = 0x69,
    USER_CTRL_REG          = 0x6A,
    PWR_MGMT_1_REG         = 0x6B,
    PWR_MGMT_2_REG         = 0x6C,
    BANK_SEL_REG           = 0x6D, //unknown
    MEM_START_ADDR_REG     = 0x6E, //unknown
    MEM_R_W_REG            = 0x6F, //unknown
    DMP_CFG_1_REG          = 0x70, //unknown
    DMP_CFG_2_REG          = 0x71, //unknown
    FIFO_COUNTH_REG        = 0x72,
    FIFO_COUNTL_REG        = 0x73,
    FIFO_R_W_REG           = 0x74,
    WHO_AM_I_REG           = 0x75,
};

enum MpuRegLsb{
    SELF_TEST_X_LSB        = 0x0D,
    SELF_TEST_Y_LSB        = 0x0E,
    SELF_TEST_Z_LSB        = 0x0F,
    SELF_TEST_A_LSB        = 0x10,
    SMPLRT_DIV_LSB         = 0x0D,
    CONFIG_LSB             = 0x1A,
    GYRO_CONFIG_FS_SEL_LSB = 3u,
    GYRO_CONFIG_ZG_ST_LSB  = 5u,
    GYRO_CONFIG_YG_ST_LSB  = 6u,
    GYRO_CONFIG_XG_ST_LSB  = 7u,
    ACCEL_CONFIG_AFS_SEL_LSB = 3u,
    ACCEL_CONFIG_ZA_ST_LSB   = 5u,
    ACCEL_CONFIG_YA_ST_LSB   = 6u,
    ACCEL_CONFIG_XA_ST_LSB   = 7u,
    MOT_THR_LSB            = 0x1F,
    FIFO_EN_LSB            = 0x23,
    I2C_MST_CTRL_LSB       = 0X24,
    I2C_SLV0_ADDR_LSB      = 0x25,
    I2C_SLV0_REG_LSB       = 0x26,
    I2C_SLV0_CTRL_LSB      = 0x27,
    I2C_SLV1_ADDR_LSB      = 0x28,
    I2C_SLV1_REG_LSB       = 0x29,
    I2C_SLV1_CTRL_LSB      = 0x2A,
    I2C_SLV2_ADDR_LSB      = 0x2B,
    I2C_SLV2_REG_LSB       = 0x2C,
    I2C_SLV2_CTRL_LSB      = 0x2D,
    I2C_SLV3_ADD_LSB       = 0x2E,
    I2C_SLV3_REG_LSB       = 0x2F,
    I2C_SLV3_CTRL_LSB      = 0x30,
    I2C_SLV4_ADDR_LSB      = 0x31,
    I2C_SLV4_REG_LSB       = 0x32,
    I2C_SLV4_DO_LSB        = 0x33,
    I2C_SLV4_CTRL_LSB      = 0x34,
    I2C_SLV4_DI_LSB        = 0x35,
    I2C_MST_STATUS_LSB     = 0x36,
    INT_PIN_CFG_LSB        = 0x37,
    INT_ENABLE_DATA_RDY_EN_LSB       = 0u,
    INT_ENABLE_DMP_INT_EN_LSB        = 1u, //unknown
    INT_ENABLE_I2C_MST_INT_EN_LSB    = 3u,
    INT_ENABLE_FIFO_OFLOW_EN_LSB     = 4u,
    INT_ENABLE_MOT_EN_LSB            = 6u,

    INT_STATUS_LSB         = 0x3A,
    ACCEL_XOUT_H_LSB       = 0x3B,
    ACCEL_XOUT_L_LSB       = 0x3C,
    ACCEL_YOUT_H_LSB       = 0x3D,
    ACCEL_YOUT_L_LSB       = 0x3E,
    ACCEL_ZOUT_H_LSB       = 0x3F,
    ACCEL_ZOUT_L_LSB       = 0x40,
    TEMP_OUT_H_LSB         = 0x41,
    TEMP_OUT_L_LSB         = 0x42,
    GYRO_XOUT_H_LSB        = 0x43,
    GYRO_XOUT_L_LSB        = 0x44,
    GYRO_YOUT_H_LSB        = 0x45,
    GYRO_YOUT_L_LSB        = 0x46,
    GYRO_ZOUT_H_LSB        = 0x47,
    GYRO_ZOUT_L_LSB        = 0x48,
    EXT_SENS_DATA_00_LSB   = 0x49,
    EXT_SENS_DATA_01_LSB   = 0x4A,
    EXT_SENS_DATA_02_LSB   = 0x4B,
    EXT_SENS_DATA_03_LSB   = 0x4C,
    EXT_SENS_DATA_04_LSB   = 0x4D,
    EXT_SENS_DATA_05_LSB   = 0x4E,
    EXT_SENS_DATA_06_LSB   = 0x4F,
    EXT_SENS_DATA_07_LSB   = 0x50,
    EXT_SENS_DATA_08_LSB   = 0x51,
    EXT_SENS_DATA_09_LSB   = 0x52,
    EXT_SENS_DATA_10_LSB   = 0x53,
    EXT_SENS_DATA_11_LSB   = 0x54,
    EXT_SENS_DATA_12_LSB   = 0x55,
    EXT_SENS_DATA_13_LSB   = 0x56,
    EXT_SENS_DATA_14_LSB   = 0x57,
    EXT_SENS_DATA_15_LSB   = 0x58,
    EXT_SENS_DATA_16_LSB   = 0x59,
    EXT_SENS_DATA_17_LSB   = 0x5A,
    EXT_SENS_DATA_18_LSB   = 0x5B,
    EXT_SENS_DATA_19_LSB   = 0x5C,
    EXT_SENS_DATA_20_LSB   = 0x5D,
    EXT_SENS_DATA_21_LSB   = 0x5E,
    EXT_SENS_DATA_22_LSB   = 0x5F,
    EXT_SENS_DATA_23_LSB   = 0x60,
    I2C_SLV0_DO_LSB        = 0x63,
    I2C_SLV1_DO_LSB        = 0x64,
    I2C_SLV2_DO_LSB        = 0x65,
    I2C_SLV3_DO_LSB        = 0x66,
    I2C_MST_DELAY_CTRL_LSB = 0x67,
    SIGNAL_PATH_RESET_LSB  = 0x68,
    MOT_DETECT_CTRL_LSB    = 0x69,
    USER_CTRL_SIG_COND_RESET_LSB   = 0u,
    USER_CTRL_I2C_MST_RESET_LSB    = 1u,
    USER_CTRL_FIFO_RESET_LSB        = 2u,
    USER_CTRL_DMP_RESET_LSB        = 3u,
    USER_CTRL_I2C_IF_DIS_LSB       = 4u,
    USER_CTRL_I2C_MST_EN_LSB       = 5u,
    USER_CTRL_FIFO_EN_LSB          = 6u,
    USER_CTRL_DMP_EN_LSB           = 7u,
    PWR_MGMT_1_CLKSEL_LSB        = 0x0,
    PWR_MGMT_1_TEMP_DIS_LSB      = 0x3,
    PWR_MGMT_1_CYCLE_LSB         = 0x5,
    PWR_MGMT_1_SLEEP_LSB         = 0x6,
    PWR_MGMT_1_DEVICE_RESET_LSB  = 0x7,
    PWR_MGMT_2_LSB         = 0x6C,
    FIFO_COUNTH_LSB        = 0x72,
    FIFO_COUNTL_LSB        = 0x73,
    FIFO_R_W_LSB           = 0x74,
    WHO_AM_I_LSB           = 0x75,
};

enum DmpFifoStatus{
    DMP_FIFO_OK,
    DMP_FIFO_NO_DATA,
    DMP_FIFO_OVERFLOW,
};

inline std::map<enum MpuReg, std::string> regMapX = { 
    {XG_OFFS_TC_REG,"XG_OFFS_TC_REG"}, 
    {YG_OFFS_TC_REG,"YG_OFFS_TC_REG"}, 
    {ZG_OFFS_TC_REG,"ZG_OFFS_TC_REG"}, 
    {XA_OFFS_H_REG,"XA_OFFS_H_REG"}, 
    {XA_OFFS_L_TC_REG,"XA_OFFS_L_TC_REG"}, 
    {YA_OFFS_H_REG,"YA_OFFS_H_REG"}, 
    {ZA_OFFS_H_REG,"ZA_OFFS_H_REG"}, 
    {ZA_OFFS_L_TC_REG,"ZA_OFFS_L_TC_REG"}, 
    {SELF_TEST_X_REG,"SELF_TEST_X_REG"}, 
    {SELF_TEST_Y_REG,"SELF_TEST_Y_REG"}, 
    {SELF_TEST_Z_REG,"SELF_TEST_Z_REG"}, 
    {SELF_TEST_A_REG,"SELF_TEST_A_REG"}, 
    {SMPLRT_DIV_REG,"SMPLRT_DIV_REG"}, 
    {CONFIG_REG,"CONFIG_REG"}, 
    {GYRO_CONFIG_REG,"GYRO_CONFIG_REG"}, 
    {ACCEL_CONFIG_REG,"ACCEL_CONFIG_REG"}, 
    {MOT_THR_REG,"MOT_THR_REG"}, 
    {MOT_DUR_REG,"MOT_DUR_REG"}, 
    {ZRMOT_THR_REG,"ZRMOT_THR_REG"}, 
    {ZRMOT_DUR_REG,"ZRMOT_DUR_REG"}, 
    {FIFO_EN_REG,"FIFO_EN_REG"}, 
    {I2C_MST_CTRL_REG,"I2C_MST_CTRL_REG"}, 
    {I2C_SLV0_ADDR_REG,"I2C_SLV0_ADDR_REG"}, 
    {I2C_SLV0_REG_REG,"I2C_SLV0_REG_REG"}, 
    {I2C_SLV0_CTRL_REG,"I2C_SLV0_CTRL_REG"}, 
    {I2C_SLV1_ADDR_REG,"I2C_SLV1_ADDR_REG"}, 
    {I2C_SLV1_REG_REG,"I2C_SLV1_REG_REG"}, 
    {I2C_SLV1_CTRL_REG,"I2C_SLV1_CTRL_REG"}, 
    {I2C_SLV2_ADDR_REG,"I2C_SLV2_ADDR_REG"}, 
    {I2C_SLV2_REG_REG,"I2C_SLV2_REG_REG"}, 
    {I2C_SLV2_CTRL_REG,"I2C_SLV2_CTRL_REG"}, 
    {I2C_SLV3_ADD_REG,"I2C_SLV3_ADD_REG"}, 
    {I2C_SLV3_REG_REG,"I2C_SLV3_REG_REG"}, 
    {I2C_SLV3_CTRL_REG,"I2C_SLV3_CTRL_REG"}, 
    {I2C_SLV4_ADDR_REG,"I2C_SLV4_ADDR_REG"}, 
    {I2C_SLV4_REG_REG,"I2C_SLV4_REG_REG"}, 
    {I2C_SLV4_DO_REG,"I2C_SLV4_DO_REG"}, 
    {I2C_SLV4_CTRL_REG,"I2C_SLV4_CTRL_REG"}, 
    {I2C_SLV4_DI_REG,"I2C_SLV4_DI_REG"}, 
    {I2C_MST_STATUS_REG,"I2C_MST_STATUS_REG"}, 
    {INT_PIN_CFG_REG,"INT_PIN_CFG_REG"}, 
    {INT_ENABLE_REG,"INT_ENABLE_REG"}, 
    {INT_STATUS_REG,"INT_STATUS_REG"}, 
    {ACCEL_XOUT_H_REG,"ACCEL_XOUT_H_REG"}, 
    {ACCEL_XOUT_L_REG,"ACCEL_XOUT_L_REG"}, 
    {ACCEL_YOUT_H_REG,"ACCEL_YOUT_H_REG"}, 
    {ACCEL_YOUT_L_REG,"ACCEL_YOUT_L_REG"}, 
    {ACCEL_ZOUT_H_REG,"ACCEL_ZOUT_H_REG"}, 
    {ACCEL_ZOUT_L_REG,"ACCEL_ZOUT_L_REG"}, 
    {TEMP_OUT_H_REG,"TEMP_OUT_H_REG"}, 
    {TEMP_OUT_L_REG,"TEMP_OUT_L_REG"}, 
    {GYRO_XOUT_H_REG,"GYRO_XOUT_H_REG"}, 
    {GYRO_XOUT_L_REG,"GYRO_XOUT_L_REG"}, 
    {GYRO_YOUT_H_REG,"GYRO_YOUT_H_REG"}, 
    {GYRO_YOUT_L_REG,"GYRO_YOUT_L_REG"}, 
    {GYRO_ZOUT_H_REG,"GYRO_ZOUT_H_REG"}, 
    {GYRO_ZOUT_L_REG,"GYRO_ZOUT_L_REG"}, 
    {EXT_SENS_DATA_00_REG,"EXT_SENS_DATA_00_REG"}, 
    {EXT_SENS_DATA_01_REG,"EXT_SENS_DATA_01_REG"}, 
    {EXT_SENS_DATA_02_REG,"EXT_SENS_DATA_02_REG"}, 
    {EXT_SENS_DATA_03_REG,"EXT_SENS_DATA_03_REG"}, 
    {EXT_SENS_DATA_04_REG,"EXT_SENS_DATA_04_REG"}, 
    {EXT_SENS_DATA_05_REG,"EXT_SENS_DATA_05_REG"}, 
    {EXT_SENS_DATA_06_REG,"EXT_SENS_DATA_06_REG"}, 
    {EXT_SENS_DATA_07_REG,"EXT_SENS_DATA_07_REG"}, 
    {EXT_SENS_DATA_08_REG,"EXT_SENS_DATA_08_REG"}, 
    {EXT_SENS_DATA_09_REG,"EXT_SENS_DATA_09_REG"}, 
    {EXT_SENS_DATA_10_REG,"EXT_SENS_DATA_10_REG"}, 
    {EXT_SENS_DATA_11_REG,"EXT_SENS_DATA_11_REG"}, 
    {EXT_SENS_DATA_12_REG,"EXT_SENS_DATA_12_REG"}, 
    {EXT_SENS_DATA_13_REG,"EXT_SENS_DATA_13_REG"}, 
    {EXT_SENS_DATA_14_REG,"EXT_SENS_DATA_14_REG"}, 
    {EXT_SENS_DATA_15_REG,"EXT_SENS_DATA_15_REG"}, 
    {EXT_SENS_DATA_16_REG,"EXT_SENS_DATA_16_REG"}, 
    {EXT_SENS_DATA_17_REG,"EXT_SENS_DATA_17_REG"}, 
    {EXT_SENS_DATA_18_REG,"EXT_SENS_DATA_18_REG"}, 
    {EXT_SENS_DATA_19_REG,"EXT_SENS_DATA_19_REG"}, 
    {EXT_SENS_DATA_20_REG,"EXT_SENS_DATA_20_REG"}, 
    {EXT_SENS_DATA_21_REG,"EXT_SENS_DATA_21_REG"}, 
    {EXT_SENS_DATA_22_REG,"EXT_SENS_DATA_22_REG"}, 
    {EXT_SENS_DATA_23_REG,"EXT_SENS_DATA_23_REG"}, 
    {I2C_SLV0_DO_REG,"I2C_SLV0_DO_REG"}, 
    {I2C_SLV1_DO_REG,"I2C_SLV1_DO_REG"}, 
    {I2C_SLV2_DO_REG,"I2C_SLV2_DO_REG"}, 
    {I2C_SLV3_DO_REG,"I2C_SLV3_DO_REG"}, 
    {I2C_MST_DELAY_CTRL_REG,"I2C_MST_DELAY_CTRL_REG"}, 
    {SIGNAL_PATH_RESET_REG,"SIGNAL_PATH_RESET_REG"}, 
    {MOT_DETECT_CTRL_REG,"MOT_DETECT_CTRL_REG"}, 
    {USER_CTRL_REG,"USER_CTRL_REG"}, 
    {PWR_MGMT_1_REG,"PWR_MGMT_1_REG"}, 
    {PWR_MGMT_2_REG,"PWR_MGMT_2_REG"}, 
    {BANK_SEL_REG,"BANK_SEL_REG"}, 
    {MEM_START_ADDR_REG,"MEM_START_ADDR_REG"}, 
    {MEM_R_W_REG,"MEM_R_W_REG"}, 
    {DMP_CFG_1_REG,"DMP_CFG_1_REG"}, 
    {DMP_CFG_2_REG,"DMP_CFG_2_REG"}, 
    {FIFO_COUNTH_REG,"FIFO_COUNTH_REG"}, 
    {FIFO_COUNTL_REG,"FIFO_COUNTL_REG"}, 
    {FIFO_R_W_REG,"FIFO_R_W_REG"}, 
    {WHO_AM_I_REG,"WHO_AM_I_REG"}}; 
