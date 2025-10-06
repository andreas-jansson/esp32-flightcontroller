#include "display.h"
#include "icons.h"


Display *Display::display = nullptr;
TFT_eSPI Display::m_tft{TFT_eSPI()};
TFT_eSprite Display::s_backgroundSprite{TFT_eSprite(&m_tft)};
SemaphoreHandle_t Display::s_newData = nullptr;

YawPitchRoll Display::m_yprMpu1{};
YawPitchRoll Display::m_yprMpu2{};
bool Display::m_isArmed{};
bool Display::m_isBooting{true};
progress_t Display::m_isCalibrating{};
bool Display::m_isArmed_bad_state{};
bool Display::m_radioActive{};
bool Display::m_wifiConnected{};
progress_t Display::m_isVerifyingMPU1{NOT_STARTED};
progress_t Display::m_isVerifyingMPU2{NOT_STARTED};
progress_t Display::m_isVerifyingRadio{NOT_STARTED};

Display *Display::GetInstance(){
    if (display == nullptr)
    {
        vSemaphoreCreateBinary(s_newData)
        display = new Display();
    }
    return display;
}

void Display::draw_leveling(int32_t offsetMiddle, int32_t y, float angle, float length, uint16_t color){

    const int32_t cx = c_screenMidX + offsetMiddle;
    const int32_t cy = y;
    const float half_len = length/2;    // half the drawn length (total = 60 px)
    const float gain = 1.0f;            // scale if you want: theta = gain * roll
    // If roll is in radians (common):
    float theta = gain * angle;

    float cs = cosf(theta);
    float sn = sinf(theta);

    // Screen Y grows downward, so use minus for "mathematical up"
    int32_t xs = cx - (int32_t)lroundf(half_len * cs);
    int32_t ys = cy - (int32_t)lroundf(half_len * sn);
    int32_t xe = cx + (int32_t)lroundf(half_len * cs);
    int32_t ye = cy + (int32_t)lroundf(half_len * sn);
    s_backgroundSprite.drawLine(xs, ys, xe, ye, color);
}


void Display::display_task(void* args){
        
    const uint8_t borderWidth{15};


    m_tft.init();
    m_tft.setSwapBytes(false);
    m_tft.setRotation(0);

    s_backgroundSprite.createSprite(c_screenWidth, c_screenHeight);
    s_backgroundSprite.setSwapBytes(false);
    s_backgroundSprite.setRotation(0);
    s_backgroundSprite.loadFont(fontHemi);

    m_wifiIcon.createSprite(48, 48);
    m_wifiIcon.setSwapBytes(false);

    m_sensorConIcon.createSprite(48, 48);
    m_sensorConIcon.setSwapBytes(false);

    m_flightIcon.createSprite(48, 48);
    m_flightIcon.setSwapBytes(false);

    const uint16_t centerPos = (c_screenWidth / 2) - (48/2); 

    s_backgroundSprite.fillSprite(TFT_BLACK);
    s_backgroundSprite.pushSprite(0, 0);

    /* booting starge */
    while(m_isBooting){

            s_backgroundSprite.fillSprite(TFT_BLACK);
            s_backgroundSprite.loadFont(fontHemi);
            s_backgroundSprite.setTextColor(TFT_WHITE);
            s_backgroundSprite.drawString("Booting...", 10, 30);



            if(m_isCalibrating == STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextSize(2);
                s_backgroundSprite.drawString("Calibrating", 10, 60);
            }
            else if(m_isCalibrating == NOT_STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.setTextSize(2);
                s_backgroundSprite.drawString("Calibrating", 10, 60);
            }
            else if(m_isCalibrating == PASS){
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextSize(2);
                s_backgroundSprite.drawString("Calibrating", 10, 60);
            }


            if(m_isVerifyingMPU1 == NOT_STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }
            else if(m_isVerifyingMPU1 == STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }
            else if(m_isVerifyingMPU1 == PASS){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }
             else if(m_isVerifyingMPU1 == FAILED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }

            if(m_isVerifyingMPU2 == NOT_STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == PASS){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
             else if(m_isVerifyingMPU2 == FAILED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
           

            if(m_isVerifyingMPU2 == NOT_STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == PASS){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
             else if(m_isVerifyingMPU2 == FAILED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }


            if(m_isVerifyingRadio == NOT_STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("radio", 10, 150);
            }
            else if(m_isVerifyingRadio == STARTED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("radio", 10, 150);
            }
            else if(m_isVerifyingRadio == PASS){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("radio", 10, 150);
            }
             else if(m_isVerifyingRadio == FAILED){
                s_backgroundSprite.unloadFont();
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("radio", 10, 150);
            }

            
        if (m_isVerifyingMPU1 != NOT_STARTED) {

            float lineLen = 25;     

            s_backgroundSprite.setTextSize(1);
            draw_leveling(-40, 200, m_yprMpu1.roll, lineLen, TFT_WHITE); 


            draw_leveling(0, 200, -m_yprMpu1.pitch, lineLen, TFT_WHITE); 


            draw_leveling(40, 200, m_yprMpu1.yaw + (M_PI/2), lineLen, TFT_WHITE); 

            s_backgroundSprite.setTextColor(TFT_WHITE);
            s_backgroundSprite.drawString("roll", 10, 225);
            s_backgroundSprite.drawString("pitch", 50, 225);
            s_backgroundSprite.drawString("yaw", 100, 225);

        }

        if (m_isVerifyingMPU2 != NOT_STARTED) {

            float lineLen = 25;     

            s_backgroundSprite.setTextSize(1);
            draw_leveling(-35, 195, m_yprMpu2.roll, lineLen, TFT_BLUE); 


            draw_leveling(5, 195, -m_yprMpu2.pitch, lineLen, TFT_BLUE); 


            draw_leveling(45, 195, m_yprMpu2.yaw + (M_PI/2), lineLen, TFT_BLUE); 

        }


 
            s_backgroundSprite.pushSprite(0, 0);
            xSemaphoreTake(s_newData, pdMS_TO_TICKS(1000));
    }


    vTaskDelay(pdMS_TO_TICKS(1000));
    s_backgroundSprite.fillSprite(TFT_BLACK);
    s_backgroundSprite.pushSprite(0, 0);

    while(true){


        if(m_wifiConnected){
            recolor_icon(icons::wifiIcon, TFT_BLUE);
            m_wifiIcon.pushImage(0, 0, 48, 48, icons::wifiIcon);
            m_wifiIcon.pushSprite(centerPos, 50);
  
        }
        else{
            recolor_icon(icons::wifiIcon, TFT_GREEN);
            m_wifiIcon.pushImage(0, 0, 48, 48, icons::wifiIcon);
            m_wifiIcon.pushSprite(centerPos, 50);
 
        }

        if(m_radioActive){
            recolor_icon(icons::sensorConIcon, TFT_BLUE);
            m_sensorConIcon.pushImage(0, 0, 48, 48, icons::sensorConIcon);
            m_sensorConIcon.pushSprite(centerPos, 90);
        }
        else{
            recolor_icon(icons::sensorConIcon, TFT_GREEN);
            m_sensorConIcon.pushImage(0, 0, 48, 48, icons::sensorConIcon);
            m_sensorConIcon.pushSprite(centerPos, 90);
        }

     
        if(m_isArmed_bad_state){
            draw_border(0xff20);
            recolor_icon(icons::flightIcon, 0x20ff);
            m_flightIcon.pushImage(0, 0, 48, 48, icons::flightIcon);
            m_flightIcon.pushSprite(centerPos, 140);
        }
        else if(m_isArmed){
            draw_border(TFT_GREEN);
            recolor_icon(icons::flightIcon, TFT_BLUE);
            m_flightIcon.pushImage(0, 0, 48, 48, icons::flightIcon);
            m_flightIcon.pushSprite(centerPos, 140);
        }
        else{
            draw_border(TFT_RED);
            recolor_icon(icons::flightIcon, TFT_GREEN);
            m_flightIcon.pushImage(0, 0, 48, 48, icons::flightIcon);
            m_flightIcon.pushSprite(centerPos, 140);
        }


        xSemaphoreTake(s_newData, pdMS_TO_TICKS(1000));
    }

}

void Display::draw_border(uint16_t color){

    constexpr uint8_t borderWidth{15};

    m_tft.fillRect(0, 0, c_screenWidth, borderWidth, color);
    
    m_tft.fillRect(0, 0, borderWidth, c_screenHeight, color);

    m_tft.fillRect(c_screenWidth - borderWidth, 0, borderWidth, c_screenHeight, color);

    m_tft.fillRect(0, c_screenHeight - borderWidth, c_screenWidth, borderWidth, color);
    
}


void Display::recolor_icon(unsigned short icon[], uint16_t color){

    for(int i=0;i<2304;i++){
        if(icon[i] != 0)
            icon[i] = color;
    }

}

void Display::set_armed_status(bool isArmed){
    m_isArmed = isArmed;
    m_isArmed_bad_state = isArmed? false : m_isArmed_bad_state;
    xSemaphoreGive(s_newData);
}

