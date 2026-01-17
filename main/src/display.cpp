#include "display.h"
#include "icons.h"


Display *Display::display = nullptr;
TFT_eSPI Display::m_tft{TFT_eSPI()};
TFT_eSprite Display::s_backgroundSprite{TFT_eSprite(&m_tft)};
SemaphoreHandle_t Display::s_newData = nullptr;
YawPitchRoll Display::m_yprMpu1{};
YawPitchRoll Display::m_yprMpu2{};
bool Display::m_isArmed{};
bool Display::m_isArmed_bad_state{};
bool Display::m_radioActive{};
bool Display::m_wifiConnected{};
progress_t Display::m_isCalibrating{};
progress_t Display::m_isVerifyingMPU1{NOT_STARTED};
progress_t Display::m_isVerifyingMPU2{NOT_STARTED};
progress_t Display::m_isVerifyingRadio{NOT_STARTED};
enum PidConfigDisplay Display::m_pidSelected;
volatile enum DisplayState Display::m_state{};
Pid Display::m_pid;


Display::Display(){
    vSemaphoreCreateBinary(s_newData);
}

Display *Display::GetInstance(){
    if (display == nullptr)
    {
        display = new Display();
    }
    return display;
}

void Display::draw_leveling(int32_t offsetMiddle, int32_t y, float angle, float length, uint16_t color){

    const int32_t cx = c_screenMidX + offsetMiddle;
    const int32_t cy = y;
    const float half_len = length/2;    
    const float gain = 1.0f;          
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



void Display::init(){
    m_tft.init();
    m_tft.setSwapBytes(false);
    m_tft.setRotation(0);

    s_backgroundSprite.createSprite(c_screenWidth, c_screenHeight);
    s_backgroundSprite.setSwapBytes(false);
    s_backgroundSprite.setRotation(0);
    //s_backgroundSprite.loadFont(fontHemi);

    m_wifiIcon.createSprite(48, 48);
    m_wifiIcon.setSwapBytes(false);

    m_sensorConIcon.createSprite(48, 48);
    m_sensorConIcon.setSwapBytes(false);

    m_flightIcon.createSprite(48, 48);
    m_flightIcon.setSwapBytes(false);

    s_backgroundSprite.fillSprite(TFT_BLACK);
    s_backgroundSprite.pushSprite(0, 0);
}

void Display::boot_menu(){

    while(m_state == BOOTING){

            s_backgroundSprite.fillSprite(TFT_BLACK);
            //s_backgroundSprite.loadFont(fontHemi);
            s_backgroundSprite.setTextColor(TFT_WHITE);
            s_backgroundSprite.drawString("Booting...", 10, 30);
            //s_backgroundSprite.unloadFont();


            if(m_isCalibrating == STARTED){
                s_backgroundSprite.setTextSize(2);
                s_backgroundSprite.drawString("Calibrating", 10, 60);
            }
            else if(m_isCalibrating == NOT_STARTED){
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.setTextSize(2);
                s_backgroundSprite.drawString("Calibrating", 10, 60);
            }
            else if(m_isCalibrating == PASS){
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.setTextSize(2);
                s_backgroundSprite.drawString("Calibrating", 10, 60);
            }

            if(m_isVerifyingMPU1 == NOT_STARTED){
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }
            else if(m_isVerifyingMPU1 == STARTED){
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }
            else if(m_isVerifyingMPU1 == PASS){
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }
             else if(m_isVerifyingMPU1 == FAILED){
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("mpu 1", 10, 90);
            }

            if(m_isVerifyingMPU2 == NOT_STARTED){
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == STARTED){
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == PASS){
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
             else if(m_isVerifyingMPU2 == FAILED){
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
           

            if(m_isVerifyingMPU2 == NOT_STARTED){
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == STARTED){
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
            else if(m_isVerifyingMPU2 == PASS){
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }
             else if(m_isVerifyingMPU2 == FAILED){
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("mpu 2", 10, 120);
            }


            if(m_isVerifyingRadio == NOT_STARTED){
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("radio", 10, 150);
            }
            else if(m_isVerifyingRadio == STARTED){
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("radio", 10, 150);
            }
            else if(m_isVerifyingRadio == PASS){
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("radio", 10, 150);
            }
             else if(m_isVerifyingRadio == FAILED){
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("radio", 10, 150);
            }

            
        if (m_isVerifyingMPU1 != NOT_STARTED) {
            float lineLen = 25;     

            s_backgroundSprite.setTextSize(1);
            draw_leveling(-40, 200,  m_yprMpu1.roll,           lineLen, TFT_WHITE); 
            draw_leveling(0,   200, -m_yprMpu1.pitch,          lineLen, TFT_WHITE); 
            draw_leveling(40,  200,  m_yprMpu1.yaw + (M_PI/2), lineLen, TFT_WHITE); 

            s_backgroundSprite.setTextColor(TFT_WHITE);
            s_backgroundSprite.drawString("roll",  10,  225);
            s_backgroundSprite.drawString("pitch", 50,  225);
            s_backgroundSprite.drawString("yaw",   100, 225);
        }

        if (m_isVerifyingMPU2 != NOT_STARTED) {

            float lineLen = 25;     

            s_backgroundSprite.setTextSize(1);
            draw_leveling(-35,  195,  m_yprMpu2.roll,           lineLen, TFT_BLUE); 
            draw_leveling(5,    195, -m_yprMpu2.pitch,          lineLen, TFT_BLUE); 
            draw_leveling(45,   195,  m_yprMpu2.yaw + (M_PI/2), lineLen, TFT_BLUE); 

        }

        s_backgroundSprite.pushSprite(0, 0);
        xSemaphoreTake(s_newData, portMAX_DELAY);
    }
    cleanup();

}

void Display::cleanup(){ 
    s_backgroundSprite.fillSprite(TFT_BLACK);
    s_backgroundSprite.pushSprite(0, 0);
}


void Display::drone_state(){
    const uint8_t borderWidth{15};
    const uint16_t centerPos = (c_screenWidth / 2) - (48/2); 
    
    cleanup();
    while(m_state == DRONE){
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

void Display::pid_menu(){


    uint16_t x{45};
    uint16_t y{5};
    uint16_t y_dist{40};
    uint16_t x_offset{30};
    Pid prev_pid{};

    char p_str[25];
    char i_str[25];
    char d_str[25];

    m_pidSelected = P;

    s_backgroundSprite.setTextSize(1);
    s_backgroundSprite.drawString("PRESS L2 TO SELECT" , x - x_offset, (y_dist * 5) + 5);
    //s_backgroundSprite.loadFont(fontHemi);


    while(m_state == PID_CONFIG){

        (void)snprintf(p_str, 25, "P: %-3.2f",  m_pid.kP);
        (void)snprintf(i_str, 25, "I:  %-3.2f", m_pid.kI);
        (void)snprintf(d_str, 25, "D: %-3.2f",  m_pid.kD);

        switch(m_pidSelected){
            case P:
                s_backgroundSprite.fillRect(x - x_offset, y, 150, 30, TFT_BLACK);
                s_backgroundSprite.setTextColor(TFT_WHITE, TFT_BLACK);
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + (y_dist));
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
                break;
            case I:
                s_backgroundSprite.fillRect(x - x_offset, y + y_dist, 150, 30, TFT_BLACK);
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.setTextColor(TFT_WHITE, TFT_BLACK);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + y_dist);
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
            break;
            case D:
                s_backgroundSprite.fillRect(x - x_offset, y + y_dist * 2, 150, 30, TFT_BLACK);
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + y_dist);
                s_backgroundSprite.setTextColor(TFT_WHITE, TFT_BLACK);
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
            break;
            case APPLY:
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + y_dist);
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
            break;
            case CANCEL_PID:
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + y_dist);
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.setTextColor(TFT_WHITE);
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
            break;
            case PID_CONFIG_APPLY:
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + y_dist);
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.setTextColor(TFT_GREEN);
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
            break;
            case PID_CONFIG_CANCEL:
                s_backgroundSprite.drawString(p_str , x - x_offset, y);
                s_backgroundSprite.drawString(i_str , x - x_offset, y + y_dist);
                s_backgroundSprite.drawString(d_str , x - x_offset, y + (y_dist * 2));
                s_backgroundSprite.drawString("APPLY", x - x_offset, y + (y_dist * 3));
                s_backgroundSprite.setTextColor(TFT_RED);
                s_backgroundSprite.drawString("CANCEL", x - x_offset, y + (y_dist * 4));
                s_backgroundSprite.setTextColor(TFT_DARKGREY);
            break;
        }


        s_backgroundSprite.pushSprite(0,0);


        prev_pid = m_pid;
        xSemaphoreTake(s_newData, pdMS_TO_TICKS(100000));
    }
}



void Display::draw_btn(
        int32_t x1, 
        int32_t y1, 
        int32_t w, 
        int32_t h, 
        uint16_t borderColor, 
        uint16_t backgroundColor, 
        const char* str, 
        uint16_t strColor){
    
    float borderWidth = 5.0f; 

    float x2 = x1 + w;
    float y2 = y1 + h;

    s_backgroundSprite.fillRect(x1, y1, w, h, backgroundColor);

    s_backgroundSprite.drawWideLine(x1, y1, x2, y1, borderWidth, borderColor, backgroundColor); // top edge
    s_backgroundSprite.drawWideLine(x2, y1, x2, y2, borderWidth, borderColor, backgroundColor); // right edge
    s_backgroundSprite.drawWideLine(x2, y2, x1, y2, borderWidth, borderColor, backgroundColor); // bottom edge
    s_backgroundSprite.drawWideLine(x1, y2, x1, y1, borderWidth, borderColor, backgroundColor); // left edge

    if (str != nullptr) {
        int textX = x1 + w / 2;
        int textY = y1 + h / 2;

        s_backgroundSprite.setTextDatum(MC_DATUM);  // center alignment
        s_backgroundSprite.setTextColor(strColor, backgroundColor);
        s_backgroundSprite.drawString(str, textX, textY);
    }
    s_backgroundSprite.pushSprite(0, 0);
}

void Display::display_task(void* args){
        
    init();

    while(true){
        cleanup();
        switch(m_state){
            case BOOTING:
                boot_menu();
                break;
            case DRONE:
                drone_state();
                break;
            case PID_CONFIG:
                pid_menu();
                break;
            default:
                break;
        }
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

    for(int i=0;i<2304;i++)
    {
        if(icon[i] != 0)
            icon[i] = color;
    }
    
}

void Display::set_armed_status(bool isArmed){
    m_isArmed = isArmed;
    m_isArmed_bad_state = isArmed? false : m_isArmed_bad_state;
    xSemaphoreGive(s_newData);
}

void Display::set_pid_config_data(Pid pid, enum PidConfigDisplay state){
    m_pidSelected = state;
    m_pid = pid;
    xSemaphoreGive(s_newData);
}
