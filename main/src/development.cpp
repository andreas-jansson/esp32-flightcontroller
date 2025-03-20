
#include "math.h"

#include "development.h"
#include "common_data.h"
#include "drone.h"
#include "debug.h"

#define WEB_TASK


#define log_tag "console_telemetry"


static RingbufHandle_t ringBuffer_web = nullptr;


double mapValue(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
    if(x == 992){
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void draw_progress_char_single(){

    wchar_t block = '*';
    print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"%lc", block);
    fflush(stdout);
}

void draw_progress_bar_pos(std::string num, int value, int nrOfChars,  int slivers, int maxChars){

    print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"channel (%s) %5d                   |", num.c_str(), value);
    for(int i=0;i<nrOfChars;i++) {
        draw_progress_char_single();
    }
    print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"\n");
}

void draw_progress_bar_neg(std::string num, int value, int nrOfChars,  int slivers, int maxChars){

    print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"channel (%s) %5d         ", num.c_str(), value);//          |", value, nrOfChars, slivers);

    for(int i=0;i<(10 - nrOfChars);i++) {
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA," ");
    }
    for(int i=0;i<(nrOfChars);i++) {
        draw_progress_char_single();
    }
    print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"|\n");
    
}

void draw_channels(std::string num, int rawValue, int newRawValue, int counter) {

    if(rawValue != newRawValue || (counter % 100 == 0)){
        setlocale(LC_ALL, "en_US.UTF-8");

        int value = mapValue(rawValue, 174, 1811, -100, 100);
        int completeChars = abs((value / 10));
        int slivers = abs( (value % 10) % 10);
        int nrOfChars = slivers? completeChars + 1  : completeChars;
        constexpr int maxChars = 10;
    
        //delete current line and move cursor to start of line
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"\u001B[K");
        if(value>0)
            draw_progress_bar_pos(num, value, nrOfChars, slivers, maxChars);
        else if(value < 0)
            draw_progress_bar_neg(num, value, nrOfChars, slivers, maxChars);
        else{
            print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"channel (%s) %5d                   |\n", num.c_str(), value);
        }
    }
    else{
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"\n");
    }
}

void telemetry_task(void* args){

    static bool firstPrint{true};
    init_telemetry_buffer();
    RingbufHandle_t ringBuffer_telemetry = Drone::GetInstance()->get_queue_handle();

    constexpr float radToDegree = (180.0 / M_PI);
    // constexpr char moveCursorUp[] = "[4A";
    // constexpr char clearRow[] = "[K";
    constexpr char hideCursor[] = "\033[?25l";
    printf("%s", hideCursor);

    Channel ch{}, chPrev{};
    YawPitchRoll yprPrev{};
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint64_t counter = 0;

    size_t item_size2 = sizeof(TelemetryData);
    std::map<int, std::string> mode = {{ACRO_MODE, "Acro mode"}, {SELFLEVL_MODE, "Self level mode"}, {ANGLE_MODE, "Angle mode"}};




    while (true)
    {

        TelemetryData telemetry{};
        TelemetryData* data = (TelemetryData*)xRingbufferReceive(ringBuffer_telemetry, &item_size2, portMAX_DELAY);
        if(data != nullptr){
            telemetry = *data;
            vRingbufferReturnItem(ringBuffer_telemetry, (void*)data);
        }

        #ifdef WEB_TASK
        BaseType_t res = xRingbufferSend(ringBuffer_web, &telemetry, sizeof(TelemetryData), 0);
        if (res != pdTRUE) {
            printf("Failed to send telemetry item: handle %p size %u\n", ringBuffer_web, sizeof(TelemetryData));
        } 
        #endif
        
        if(firstPrint){
            firstPrint = false;
        }
        else{
            print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"[23A");
        }
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"[K  Roll    Pitch     Yaw\n");
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"[K%6.3f °C %6.3f °C %6.3f °C\n", 
            telemetry.ypr.roll * radToDegree, 
            telemetry.ypr.pitch * radToDegree, 
            telemetry.ypr.yaw * radToDegree);

        draw_channels("1", telemetry.channel.ch1,  chPrev.ch1,  counter);
        draw_channels("2", telemetry.channel.ch2,  chPrev.ch2,  counter);
        draw_channels("3", telemetry.channel.ch3,  chPrev.ch3,  counter);
        draw_channels("4", telemetry.channel.ch4,  chPrev.ch4,  counter);
        draw_channels("5", telemetry.channel.ch5,  chPrev.ch5,  counter);
        draw_channels("6", telemetry.channel.ch6,  chPrev.ch6,  counter);
        draw_channels("7", telemetry.channel.ch7,  chPrev.ch7,  counter);
        draw_channels("8", telemetry.channel.ch8,  chPrev.ch8,  counter);
        draw_channels("9", telemetry.channel.ch9,  chPrev.ch9,  counter);
        draw_channels("10", telemetry.channel.ch10, chPrev.ch10, counter);
        draw_channels("11", telemetry.channel.ch11, chPrev.ch11, counter);
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA,"[KArmed: %3s mode: %-17s (%d) dmp: %lu Hz radio: %lu Hz loop Hz: %lu\n", 
            telemetry.drone.isArmed? "Yes" : "No", 
            mode[telemetry.drone.mode].c_str(), 
            telemetry.drone.mode, 
            telemetry.drone.dmpFreq,
            telemetry.drone.radioFreq,
            telemetry.drone.loopFreq
        );

        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[Ktarget: %5.2f curr: %5.2f\n", telemetry.drone.targetPitch, telemetry.drone.currPitch);
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[Ktarget: %5.2f curr: %5.2f\n", telemetry.drone.targetRoll, telemetry.drone.currRoll);
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[Kfl: %4u fr: %4u\n", telemetry.drone.motorFlSpeed, telemetry.drone.motorFrSpeed);
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[Krl: %4u rr: %4u\n", telemetry.drone.motorRlSpeed, telemetry.drone.motorRrSpeed);
        
        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[K[Rear   Left] %4u °C     %4.2f V     %4.2f A     %4.2f mA    %6u rpm\n", 
            telemetry.drone.escState[MOTOR1].temperature, 
            telemetry.drone.escState[MOTOR1].voltage * 0.01, 
            telemetry.drone.escState[MOTOR1].current * 0.01, 
            telemetry.drone.escState[MOTOR1].consumption * 0.01, 
            telemetry.drone.escState[MOTOR1].rpm * 100 / 7);

        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[K[Rear  Right] %4u °C     %4.2f V     %4.2f A     %4.2f mA    %6u rpm\n", 
            telemetry.drone.escState[MOTOR2].temperature, 
            telemetry.drone.escState[MOTOR2].voltage * 0.01, 
            telemetry.drone.escState[MOTOR2].current * 0.01, 
            telemetry.drone.escState[MOTOR2].consumption * 0.01, 
            telemetry.drone.escState[MOTOR2].rpm * 100 / 7);

        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[K[Front  Left] %4u °C     %4.2f V     %4.2f A     %4.2f mA    %6u rpm\n", 
            telemetry.drone.escState[MOTOR3].temperature, 
            telemetry.drone.escState[MOTOR3].voltage * 0.01, 
            telemetry.drone.escState[MOTOR3].current * 0.01, 
            telemetry.drone.escState[MOTOR3].consumption * 0.01, 
            telemetry.drone.escState[MOTOR3].rpm * 100 / 7);

        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[K[Front Right] %4u °C     %4.2f V     %4.2f A     %4.2f mA    %6u rpm\n", 
            telemetry.drone.escState[MOTOR4].temperature, 
            telemetry.drone.escState[MOTOR4].voltage * 0.01, 
            telemetry.drone.escState[MOTOR4].current * 0.01, 
            telemetry.drone.escState[MOTOR4].consumption * 0.01, 
            telemetry.drone.escState[MOTOR4].rpm * 100 / 7);

        print_debug(DEBUG_TELEMETRY, DEBUG_DATA, "[Kcurrent: %.2f A\n", telemetry.drone.currentDraw);  

        chPrev = ch;
        counter++;
        vTaskDelay(2/portTICK_PERIOD_MS);
    }

}


void init_telemetry_buffer(){
    static bool isInitiated = false;
    if(!isInitiated){
        ringBuffer_web = xRingbufferCreate(sizeof(TelemetryData) * 100, RINGBUF_TYPE_NOSPLIT);
        if (ringBuffer_web == NULL) {
            printf("Failed to create web ring buffer\n");
        }
        isInitiated = true;
    }
}


RingbufHandle_t get_telemetry_handle(){
    return ringBuffer_web;
}