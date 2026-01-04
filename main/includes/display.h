#pragma once

#include <string>
#include <map>

#include "Arduino.h"
#include <TFT_eSPI.h>

#include "freertos/task.h"
#include "freertos/semphr.h"

#include "font_hemi.h"

#include "common_data.h"

enum DisplayState{
    BOOTING,
    DRONE,
    PID_CONFIG,
};


class Display{
    static Display* display;
    static TFT_eSprite s_backgroundSprite;
    static TFT_eSPI m_tft;
    TFT_eSprite m_wifiIcon{TFT_eSprite(&m_tft)};
    TFT_eSprite m_sensorConIcon{TFT_eSprite(&m_tft)};
    TFT_eSprite m_flightIcon{TFT_eSprite(&m_tft)};
    TFT_eSprite m_background{TFT_eSprite(&m_tft)};

    const int32_t c_screenWidth{135};
    const int32_t c_screenHeight{240};

	static SemaphoreHandle_t s_newData;

    static YawPitchRoll m_yprMpu1;
    static YawPitchRoll m_yprMpu2;
    static bool m_isArmed;
    //static bool m_isBooting;
    static enum DisplayState m_state;

    static progress_t m_isCalibrating;
    static bool m_isArmed_bad_state;
    static bool m_radioActive;
    static bool m_wifiConnected;
    static progress_t m_isVerifyingMPU1;
    static progress_t m_isVerifyingMPU2;
    static progress_t m_isVerifyingRadio;
    static enum PidConfigDisplay m_pidSelected;
    static Pid m_pid;

    const int32_t c_screenMidX{c_screenWidth/2};
    const int32_t c_screenMidY{c_screenHeight/2};

    
    Display();
    void init();
    void boot_menu();
    void cleanup();
    void drone_state();
    void pid_menu();

    void draw_btn(
        int32_t x, 
        int32_t y, 
        int32_t w, 
        int32_t h, 
        uint16_t borderColor, 
        uint16_t backgroundColor, 
        const char* str, 
        uint16_t strColor);



public:


    Display(Display &other) = delete;
    void operator=(const Display &) = delete;
    static Display *GetInstance();


    static void set_display_state(enum DisplayState state){m_state = state; xSemaphoreGive(s_newData);};

    static void set_armed_status(bool isArmed);
    //static void set_booting_status(){m_state = BOOTING; xSemaphoreGive(s_newData);}
    static void set_armed_bad_state_status(bool is_bad){m_isArmed_bad_state = is_bad; xSemaphoreGive(s_newData);}
    static void set_bad_arm_status(bool isArmed){m_isArmed = isArmed; xSemaphoreGive(s_newData);}
    static void set_radio_status(bool radioActive){m_radioActive = radioActive; xSemaphoreGive(s_newData); }
    static void set_wifi_status(bool wifiConnected){m_wifiConnected = wifiConnected; xSemaphoreGive(s_newData);}
    static void set_verify_mpu1_status(progress_t status){m_isVerifyingMPU1 = status; xSemaphoreGive(s_newData);}
    static void set_verify_mpu2_status(progress_t status){m_isVerifyingMPU2 = status; xSemaphoreGive(s_newData);}
    static void set_verify_radio_status(progress_t status){m_isVerifyingRadio = status; xSemaphoreGive(s_newData);}
    static void set_mpu1_angle(YawPitchRoll yprMpu1){m_yprMpu1 = yprMpu1; xSemaphoreGive(s_newData);}
    static void set_mpu2_angle(YawPitchRoll yprMpu2){m_yprMpu2 = yprMpu2; xSemaphoreGive(s_newData);}
    static void set_calibration_status(progress_t status){m_isCalibrating = status; xSemaphoreGive(s_newData);}
    static void set_pid_config_data(Pid pid, enum PidConfigDisplay state);

    void display_task(void* args);

    void draw_border(uint16_t color);
    void recolor_icon(unsigned short wifiIcon[], uint16_t color);
    void draw_leveling(int32_t offsetMiddle, int32_t y, float angle, float length, uint16_t color);


};
