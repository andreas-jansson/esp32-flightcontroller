#include "display.h"
#include "icons.h"


Display *Display::display = nullptr;
SemaphoreHandle_t Display::s_newData = nullptr;


bool Display::m_isArmed{};
bool Display::m_isArmed_bad_state{};
bool Display::m_radioActive{};
bool Display::m_wifiConnected{};

Display *Display::GetInstance(){
    if (display == nullptr)
    {
        vSemaphoreCreateBinary(s_newData)
        display = new Display();
    }
    return display;
}


void Display::display_task(void* args){
        
    const uint8_t borderWidth{15};

    m_tft.init();
    m_tft.setSwapBytes(false);
    m_tft.setRotation(0);
    m_tft.loadFont(fontHemi);

    m_tft.fillScreen(TFT_BLACK);

    m_wifiIcon.createSprite(48, 48);
    m_wifiIcon.setSwapBytes(false);

    m_sensorConIcon.createSprite(48, 48);
    m_sensorConIcon.setSwapBytes(false);

    m_flightIcon.createSprite(48, 48);
    m_flightIcon.setSwapBytes(false);

    const uint16_t centerPos = (c_screenWidth / 2) - (48/2); 

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



        xSemaphoreTake(s_newData, portMAX_DELAY);
    }

}

void Display::draw_border(uint16_t color){

    constexpr uint8_t borderWidth{15};

    m_tft.fillRect(0, 0, c_screenWidth, borderWidth, color);
    
    m_tft.fillRect(0, 0, borderWidth, c_screenLength, color);

    m_tft.fillRect(c_screenWidth - borderWidth, 0, borderWidth, c_screenLength, color);

    m_tft.fillRect(0, c_screenLength - borderWidth, c_screenWidth, borderWidth, color);
    
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

