#pragma once

#include <string>
#include <map>

//#include "Arduino.h"
#include <TFT_eSPI.h>
//#include <SPI.h>

#include "freertos/task.h"
#include "freertos/semphr.h"

#include "font_hemi.h"

#include "common_data.h"



class Display{


    static Display* display;

    TFT_eSPI m_tft{TFT_eSPI()};
    TFT_eSprite m_wifiIcon{TFT_eSprite(&m_tft)};
    TFT_eSprite m_sensorConIcon{TFT_eSprite(&m_tft)};
    TFT_eSprite m_flightIcon{TFT_eSprite(&m_tft)};
    TFT_eSprite m_background{TFT_eSprite(&m_tft)};

    const uint8_t c_screenWidth{135};
    const uint8_t c_screenLength{240};

	static SemaphoreHandle_t s_newData;

    static bool m_isArmed;
    static bool m_isBooting;
    static bool m_isArmed_bad_state;
    static bool m_radioActive;
    static bool m_wifiConnected;


    YawPitchRoll ypr{};

    Display() = default;


public:


    Display(Display &other) = delete;
    void operator=(const Display &) = delete;
    static Display *GetInstance();


    static void set_armed_status(bool isArmed);
    static void set_booting_status(bool isBooting){m_isBooting = isBooting; xSemaphoreGive(s_newData);}
    static void set_armed_bad_state_status(bool is_bad){m_isArmed_bad_state = is_bad; xSemaphoreGive(s_newData);}
    static void set_bad_arm_status(bool isArmed){m_isArmed = isArmed; xSemaphoreGive(s_newData);}
    static void set_radio_status(bool radioActive){m_radioActive = radioActive; xSemaphoreGive(s_newData); }
    static void set_wifi_status(bool wifiConnected){m_wifiConnected = wifiConnected; xSemaphoreGive(s_newData);}

    void display_task(void* args);

    void draw_border(uint16_t color);
    void recolor_icon(unsigned short wifiIcon[], uint16_t color);


};
