

#include <cstring>
#include <chrono>

#include "drone.h"
#include "esp_check.h"

#define TELEMETRY_TASK

#define log_tag "drone"


Drone* Drone::drone = nullptr;

Drone::Drone(Dshot600* dshot, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle){
    ESP_ERROR_CHECK_WITHOUT_ABORT((dmp_queue_handle == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((radio_queue_handle == nullptr));

    this->telemetry_queue_handle = xRingbufferCreate(sizeof(TelemetryData) * 10, RINGBUF_TYPE_NOSPLIT);
    if (this->telemetry_queue_handle == NULL) {
        printf("Failed to create drone ring buffer\n");
    }

    this->dmp_queue_handle = dmp_queue_handle;
    this->radio_queue_handle = radio_queue_handle;
    this->dshot = dshot;
}

Drone* Drone::GetInstance(Dshot600* dshot, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle){
    if(drone==nullptr){
        drone = new Drone(dshot, dmp_queue_handle, radio_queue_handle);
    }
    return drone;
}

Drone* Drone::GetInstance(){
    return drone;
}

bool Drone::verify_imu_data(YawPitchRoll ypr){

    float yawHigh = 10.0;
    float yawLow = -10.0;
    float pitchHigh = 10.0;
    float pitchLow = -10.0;
    float rollHigh = 10.0;
    float rollLow = -10.0;

    if(ypr.yaw < yawLow || ypr.yaw > yawHigh){
        printf("bad yaw: %.2f\n", ypr.yaw);
        return false;
    }
    else if(ypr.pitch < pitchLow || ypr.pitch > pitchHigh){
        printf("bad pitch: %.2f\n", ypr.pitch);
        return false;
    }
    else if(ypr.roll < rollLow || ypr.roll > rollHigh){
        printf("bad roll: %.2f\n", ypr.roll);
        return false;
    }
    return true;
}

bool Drone::verify_controller_data(Channel channel) {
    if (channel.ch1 < Radio::minChannelValue || channel.ch1 > Radio::maxChannelValue) {
        printf("Invalid channel 1 value: %d\n", channel.ch1);
        return false;
    }
    if (channel.ch2 < Radio::minChannelValue || channel.ch2 > Radio::maxChannelValue) {
        printf("Invalid channel 2 value: %d\n", channel.ch2);
        return false;
    }
    if (channel.ch3 < Radio::minChannelValue || channel.ch3 > Radio::maxChannelValue) {
        printf("Invalid channel 3 value: %d\n", channel.ch3);
        return false;
    }
    if (channel.ch4 < Radio::minChannelValue || channel.ch4 > Radio::maxChannelValue) {
        printf("Invalid channel 4 value: %d\n", channel.ch4);
        return false;
    }
    if (channel.ch5 < Radio::minChannelValue || channel.ch5 > Radio::maxChannelValue) {
        printf("Invalid channel 5 value: %d\n", channel.ch5);
        return false;
    }
    if (channel.ch6 < Radio::minChannelValue || channel.ch6 > Radio::maxChannelValue) {
        printf("Invalid channel 6 value: %d\n", channel.ch6);
        return false;
    }
    if (channel.ch7 < Radio::minChannelValue || channel.ch7 > Radio::maxChannelValue) {
        printf("Invalid channel 7 value: %d\n", channel.ch7);
        return false;
    }
    if (channel.ch8 < Radio::minChannelValue || channel.ch8 > Radio::maxChannelValue) {
        printf("Invalid channel 8 value: %d\n", channel.ch8);
        return false;
    }
    if (channel.ch9 < Radio::minChannelValue || channel.ch9 > Radio::maxChannelValue) {
        printf("Invalid channel 9 value: %d\n", channel.ch9);
        return false;
    }
    if (channel.ch10 < Radio::minChannelValue || channel.ch10 > Radio::maxChannelValue) {
        printf("Invalid channel 10 value: %d\n", channel.ch10);
        return false;
    }
    if (channel.ch11 < Radio::minChannelValue || channel.ch11 > Radio::maxChannelValue) {
        printf("Invalid channel 11 value: %d\n", channel.ch11);
        return false;
    }

    return true;
}

esp_err_t Drone::verify_components(){

    Channel channelPrev{};
    YawPitchRoll yprPrev{};

    size_t item_size = sizeof(YawPitchRoll);
    size_t item_size2 = sizeof(Channel);

    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;

    uint64_t yprValidCounter = 0;
    uint64_t radioValidCounter = 0;

 

    auto start = std::chrono::steady_clock::now();

    while(true){

        bool yprIsValid{};
        bool radioIsValid{};

        YawPitchRoll* data = (YawPitchRoll*)xRingbufferReceive(dmp_queue_handle, &item_size, pdMS_TO_TICKS(1));
        if(data != nullptr){
            this->ypr = *data;
            vRingbufferReturnItem(dmp_queue_handle, (void*)data);
            yprCounter++;
            yprIsValid = true;
        }

        Channel* channelData = (Channel*)xRingbufferReceive(radio_queue_handle, &item_size2, pdMS_TO_TICKS(1));
        if(channelData != nullptr){
            this->channel = *channelData;
            vRingbufferReturnItem(radio_queue_handle, (void*)channelData);
            radioCounter++;
            radioIsValid = true;
        }

        /* start */
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        if(elapsed < 2000){
            yprCounter = 0;
            radioCounter = 0;
            continue;
        }


        if(verify_imu_data(ypr) && yprIsValid)
            yprValidCounter++;

        if(verify_controller_data(channel) && radioIsValid)
            radioValidCounter++;
 

        if(yprCounter > 1000){
            if((yprValidCounter / yprCounter) > 0.9)
                yprIsValid = true;
            else if((yprValidCounter / yprCounter) < 0.9)
                yprIsValid = false;
        }

        if(radioCounter > 1000){
            if(((float)radioValidCounter / radioCounter) > 0.9)
                radioIsValid = true;
            else if(((float)radioValidCounter / radioCounter) < 0.9)
                radioIsValid = false;
        }

     
        if((yprCounter > 1000) && yprCounter % 1000 == 0){
            printf("Error bad rates IMU %.2f%%\n", ((float)yprValidCounter / yprCounter)*100);
        }

        if((radioCounter > 1000)){
            printf("Error bad rates radio %.2f%%\n", ((float)radioValidCounter / radioCounter)*100);
            printf("radioValidCounter: %llu  radioCounter: %llu\n", radioValidCounter, radioCounter);
        }

        if(yprCounter == UINT64_MAX){
            printf("ERROR MAX VALUE YPRCOUNTER: %llu", yprCounter);
        }
        if(radioCounter == UINT64_MAX){
            printf("ERROR MAX VALUE RADIOCOUNTER: %llu", radioCounter);
        }

        if(yprCounter && radioIsValid)
            break;

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    return ESP_OK;
}


void Drone::drone_task(void* args){

    Channel channelPrev{};
    YawPitchRoll yprPrev{};
    
    bool newYpr{};
    bool newChannel{};

    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;

    size_t item_size = sizeof(YawPitchRoll);
    size_t item_size2 = sizeof(Channel);

    auto start = std::chrono::system_clock::now();
    auto start_telemetry = std::chrono::system_clock::now();

    /* await valid controller & gyro */
    verify_components();
    printf("verify_components complete\n");

    while(true){
    




        /* await arming */




        /* armed*/
        YawPitchRoll* data = (YawPitchRoll*)xRingbufferReceive(dmp_queue_handle, &item_size, 0);
        if(data != nullptr){
            this->ypr = *data;
            vRingbufferReturnItem(dmp_queue_handle, (void*)data);
            newYpr = true;
            yprCounter++;
        }

        Channel* channelData = (Channel*)xRingbufferReceive(radio_queue_handle, &item_size2, 0);
        if(channelData != nullptr){
            this->channel = *channelData;
            vRingbufferReturnItem(radio_queue_handle, (void*)channelData);
            parse_channel_state(channel);
            newChannel = true;
            radioCounter++;
        }

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        // calculate and set the frequenzy of received data
        if(elapsed >= 1000){
            this->dmpFreq = yprCounter;
            this->radioFreq = radioCounter;
            yprCounter = 0;
            radioCounter = 0;
            start = std::chrono::system_clock::now();
        }

        #ifdef TELEMETRY_TASK

        auto end_telemetry = std::chrono::system_clock::now();
        auto elapsed_telemetry = std::chrono::duration_cast<std::chrono::milliseconds>(end_telemetry - start_telemetry).count();
        if(elapsed_telemetry >= 50){
            TelemetryData telemetry{};

            telemetry.ypr = this->ypr;
            telemetry.channel = this->channel;
        
            telemetry.drone.isArmed = this->isArmed;
            telemetry.drone.mode = this->mode;
            telemetry.drone.dmpFreq = this->dmpFreq;
            telemetry.drone.radioFreq = this->radioFreq;

            BaseType_t res = xRingbufferSend(telemetry_queue_handle, &telemetry, sizeof(TelemetryData), 1);
            if (res != pdTRUE) {
                printf("Failed to send telemetry item: handle %p size %u\n", telemetry_queue_handle, sizeof(TelemetryData));
            } 

            start_telemetry = std::chrono::system_clock::now();
        }   
        #endif
    
        vTaskDelay(2 / portTICK_PERIOD_MS);



    }
}


esp_err_t Drone::parse_channel_state(const Channel& newChannel){
    esp_err_t status = 0;

    if(this->channel.ch1 != newChannel.ch1){

    }
    else if(this->channel.ch2 != newChannel.ch2){
        
    }
    else if(this->channel.ch3 != newChannel.ch3){
        
    }
    else if(this->channel.ch4 != newChannel.ch4){
        
    }
    else if(this->channel.ch5 != newChannel.ch5){
        status = set_armed(newChannel.ch5);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to arm drone");
    }
    else if(this->channel.ch6 != newChannel.ch6){
        status = set_flight_mode(newChannel.ch6);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set flight mode");
    }
    else if(this->channel.ch7 != newChannel.ch7){
        
    }
    else if(this->channel.ch8 != newChannel.ch8){
        
    }
    else if(this->channel.ch9 != newChannel.ch9){
        
    }
    else if(this->channel.ch10 != newChannel.ch10){
        
    }
    else{
        return ESP_OK;
    }

    this->channel = newChannel;

    return status;
}

esp_err_t Drone::set_armed(int value){
    esp_err_t status = 0;
    if(value > (maxRadioValue - 100)){
        status = start_arm_process();
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to start arm process");
        this->isArmed = true;
    }
    else if(value < (minRadioValue + 100)){
        status = start_dissarm_process();
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to start dissarm process");
        this->isArmed = false;
    }
    return ESP_OK;
}

esp_err_t Drone::start_arm_process(){
    esp_err_t status = 0;
    return status;
}

esp_err_t Drone::start_dissarm_process(){
    esp_err_t status = 0;
    return status;
}

esp_err_t Drone::set_flight_mode(int value){
    esp_err_t status = 0;


    if(value > (neutralRadioValue - 100) && value < (neutralRadioValue + 100)){
        this->mode = SELFLEVL_MODE;
    }
    else if(value > (maxRadioValue - 100)){
        this->mode = ACRO_MODE;
    }
    else if(value < (minRadioValue + 100)){
        this->mode = ANGLE_MODE;
    }
    else{
        return ESP_ERR_INVALID_STATE;    
    }

    return status;
}

esp_err_t Drone::set_motor_direction(enum Motor motorNum, enum MotorDirection direction){

    //parse data etc etc
    //dshot->write();
    return ESP_FAIL;
}

esp_err_t Drone::get_motor_direction(enum Motor motorNum, enum MotorDirection& direction){

    //parse data etc etc
    //dshot->read();
    return ESP_FAIL;
}

