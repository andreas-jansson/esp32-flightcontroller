

#include <cstring>
#include <chrono>
#include <cmath>

#include "esp_check.h"

#include "drone.h"
#include "debug.h"


#define TELEMETRY_TASK

#define log_tag "drone"


Drone* Drone::drone = nullptr;

Drone::Drone(RingbufHandle_t dshot_queue_handle, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle){
    ESP_ERROR_CHECK_WITHOUT_ABORT((dmp_queue_handle == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((radio_queue_handle == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((dshot_queue_handle == nullptr));

    this->telemetry_queue_handle = xRingbufferCreate(sizeof(TelemetryData) * 10, RINGBUF_TYPE_NOSPLIT);
    if (this->telemetry_queue_handle == NULL) {
        printf("Failed to create drone ring buffer\n");
    }

    this->dmp_queue_handle = dmp_queue_handle;
    this->radio_queue_handle = radio_queue_handle;
    this->m_dshot_queue_handle = dshot_queue_handle;

}

esp_err_t Drone::set_motor_lane_mapping(const MotorLaneMapping motorMapping){

    esp_err_t status = 0;

    if(motorMapping.rearLeftlane != motorMapping.rearRightlane &&
        motorMapping.rearLeftlane != motorMapping.frontLeftlane &&
        motorMapping.rearLeftlane != motorMapping.frontRightlane &&
        motorMapping.rearRightlane != motorMapping.frontLeftlane &&
        motorMapping.rearRightlane != motorMapping.frontRightlane &&
        motorMapping.frontLeftlane != motorMapping.frontRightlane)
        status = ESP_OK;
    else
        status = ESP_FAIL;
    

    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set valid motor lane mappings");

    m_motorLaneMapping = motorMapping;

    return status;
}

Drone* Drone::GetInstance(RingbufHandle_t dshot_queue_handle, RingbufHandle_t dmp_queue_handle, RingbufHandle_t radio_queue_handle){
    if(drone==nullptr){
        drone = new Drone(dshot_queue_handle, dmp_queue_handle, radio_queue_handle);
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
        //printf("Invalid channel 1 value: %d\n", channel.ch1);
        return false;
    }
    if (channel.ch2 < Radio::minChannelValue || channel.ch2 > Radio::maxChannelValue) {
        //printf("Invalid channel 2 value: %d\n", channel.ch2);
        return false;
    }
    if (channel.ch3 < Radio::minChannelValue || channel.ch3 > Radio::maxChannelValue) {
        //printf("Invalid channel 3 value: %d\n", channel.ch3);
        return false;
    }
    if (channel.ch4 < Radio::minChannelValue || channel.ch4 > Radio::maxChannelValue) {
        //printf("Invalid channel 4 value: %d\n", channel.ch4);
        return false;
    }
    if (channel.ch5 < Radio::minChannelValue || channel.ch5 > Radio::maxChannelValue) {
        //printf("Invalid channel 5 value: %d\n", channel.ch5);
        return false;
    }
    if (channel.ch6 < Radio::minChannelValue || channel.ch6 > Radio::maxChannelValue) {
        //printf("Invalid channel 6 value: %d\n", channel.ch6);
        return false;
    }
    if (channel.ch7 < Radio::minChannelValue || channel.ch7 > Radio::maxChannelValue) {
        //printf("Invalid channel 7 value: %d\n", channel.ch7);
        return false;
    }
    if (channel.ch8 < Radio::minChannelValue || channel.ch8 > Radio::maxChannelValue) {
        //printf("Invalid channel 8 value: %d\n", channel.ch8);
        return false;
    }
    if (channel.ch9 < Radio::minChannelValue || channel.ch9 > Radio::maxChannelValue) {
        //printf("Invalid channel 9 value: %d\n", channel.ch9);
        return false;
    }
    if (channel.ch10 < Radio::minChannelValue || channel.ch10 > Radio::maxChannelValue) {
        //printf("Invalid channel 10 value: %d\n", channel.ch10);
        return false;
    }
    if (channel.ch11 < Radio::minChannelValue || channel.ch11 > Radio::maxChannelValue) {
        //printf("Invalid channel 11 value: %d\n", channel.ch11);
        return false;
    }

    return true;
}

esp_err_t Drone::arming_process(){

    esp_err_t status = 0;
    Channel channel{};
    YawPitchRoll ypr{};
    static bool warningIssued{};

    ESP_LOGI(log_tag, "Arming process started");
    while(true){
        get_imu_data(ypr, 0); // just to prevent queue from filling up
        status = get_radio_data(channel, pdMS_TO_TICKS(2));
        if(status == ESP_OK){
            if(channel.ch5 < Radio::minChannelValue + 100){
                break;
            }
            else{
                if(!warningIssued){
                    ESP_LOGI(log_tag, "WARNING! Drone controller set to armed. Disarm the controller now!");
                    warningIssued = true;
                }
            }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }



    ESP_LOGI(log_tag, "Awaiting arm signal");
    while(true){
        get_imu_data(ypr, 0); // just to prevent queue from filling up
        status = get_radio_data(channel, pdMS_TO_TICKS(2));
        if(status == ESP_OK){
            if(channel.ch5 > Radio::maxChannelValue - 100){
                break;
            }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    status = set_armed(1);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to arm drone");

    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(log_tag, "Drone armed");
    return ESP_OK;
}

esp_err_t Drone::get_imu_data(YawPitchRoll& newData, TickType_t ticks){

    size_t item_size = sizeof(YawPitchRoll);

    YawPitchRoll* data = (YawPitchRoll*)xRingbufferReceive(dmp_queue_handle, &item_size, ticks);
    if(data != nullptr){
        newData = *data;
        vRingbufferReturnItem(dmp_queue_handle, (void*)data);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t Drone::get_radio_data(Channel& newData, TickType_t ticks){

    size_t item_size = sizeof(Channel);

    Channel* data = (Channel*)xRingbufferReceive(radio_queue_handle, &item_size, ticks);
    if(data != nullptr){
        newData = *data;
        vRingbufferReturnItem(radio_queue_handle, (void*)data);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t Drone::verify_components_process(){

    esp_err_t status = 0;

    Channel channelPrev{};
    YawPitchRoll yprPrev{};

    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;
    uint64_t yprValidCounter = 0;
    uint64_t radioValidCounter = 0;

    auto start = std::chrono::steady_clock::now();

    while(true){

        bool yprIsValid{};
        bool radioIsValid{};

        Channel channel{};
        YawPitchRoll ypr{};

        /* get new data */
        status = get_imu_data(ypr, pdMS_TO_TICKS(1));
        if(status == ESP_OK){
            yprCounter++;
            yprIsValid = true;
        }

        status = get_radio_data(channel, pdMS_TO_TICKS(1));
        if(status == ESP_OK){
            radioCounter++;
            radioIsValid = true;
        }

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

        /* incase overflow */
        if(yprCounter == UINT64_MAX){
            printf("ERROR MAX VALUE YPRCOUNTER: %llu", yprCounter);
            yprCounter = 0;
            yprValidCounter = 0;
        }
        if(radioCounter == UINT64_MAX){
            printf("ERROR MAX VALUE RADIOCOUNTER: %llu", radioCounter);
            radioCounter = 0;
            radioValidCounter = 0;
        }

        if(yprCounter && radioIsValid)
            break;

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    return ESP_OK;
}

ToggleState Drone::did_channel_state_switch(uint16_t newChannelValue, uint8_t channelNr) {
    uint16_t prevChannelValue{};

    // Map the channel number to the corresponding member in the channel struct
    switch (channelNr) {
        case 1:  prevChannelValue = this->channel.ch1;  break;
        case 2:  prevChannelValue = this->channel.ch2;  break;
        case 3:  prevChannelValue = this->channel.ch3;  break;
        case 4:  prevChannelValue = this->channel.ch4;  break;
        case 5:  prevChannelValue = this->channel.ch5;  break;
        case 6:  prevChannelValue = this->channel.ch6;  break;
        case 7:  prevChannelValue = this->channel.ch7;  break;
        case 8:  prevChannelValue = this->channel.ch8;  break;
        case 9:  prevChannelValue = this->channel.ch9;  break;
        case 10: prevChannelValue = this->channel.ch10; break;
        case 11: prevChannelValue = this->channel.ch11; break;
        case 12: prevChannelValue = this->channel.ch12; break;
        case 13: prevChannelValue = this->channel.ch13; break;
        case 14: prevChannelValue = this->channel.ch14; break;
        case 15: prevChannelValue = this->channel.ch15; break;
        case 16: prevChannelValue = this->channel.ch16; break;
        default:
            printf("Error: Invalid channel number %u\n", channelNr);
            return TOGGLE_UNCHANGED;
    }

    if (newChannelValue >= Radio::maxChannelThld && newChannelValue != prevChannelValue) {
        return TOGGLE_HIGH;
    }
    else if (newChannelValue <= Radio::minChannelThld && newChannelValue != prevChannelValue) {
        return TOGGLE_LOW;
    }
    else if (newChannelValue == prevChannelValue) {
        return TOGGLE_UNCHANGED;
    }

    printf("Error: Bad toggle state detected on channel %u | new: %u prev: %u\n", 
           channelNr, newChannelValue, prevChannelValue);
    return TOGGLE_UNCHANGED;
}

uint16_t Drone::mapValue(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    if(x == 992){
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int Drone::mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
    if(x == 992){
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

float Drone::mapValue(int x, int in_min, int in_max, float out_min, float out_max) {
    if(x == 992){
        return 0;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Drone::drone_task(void* args){

    esp_err_t status = 0;

    constexpr uint8_t minStepSize{2};

    Channel channelPrev{};
    YawPitchRoll yprPrev{};

    uint16_t prevMotorRlSpeed{Dshot::invalidThrottle};
    uint16_t prevMotorRrSpeed{Dshot::invalidThrottle};
    uint16_t prevMotorFlSpeed{Dshot::invalidThrottle};
    uint16_t prevMotorFrSpeed{Dshot::invalidThrottle};

    uint16_t prevCh3{};

    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;
    uint64_t loopCounter = 0;

    auto start = std::chrono::system_clock::now();
    auto start_telemetry = std::chrono::system_clock::now();


    /* await valid controller & gyro */
    ESP_LOGI(log_tag, "Verifying IMU and Radio");
    verify_components_process();
    ESP_LOGI(log_tag, "Verifying complete");


    arming_process();

    Pid pidPitch{};
    Pid pidRoll{};
    Pid pidYaw{};

    while(true){
    
        status = get_imu_data(ypr, 0);
        if(status == ESP_OK){
            this->ypr = ypr;
            yprCounter++;
            m_state.currPitch = ypr.pitch;
            m_state.currYaw = ypr.yaw;
            m_state.currRoll = ypr.roll;
        }

        Channel channelNew{};
        status = get_radio_data(channelNew, 0);
        if(status == ESP_OK){
            parse_channel_state(channelNew);
            this->channel = channelNew;
            radioCounter++;
        }

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        // calculate and set the frequenzy of received data
        if(elapsed >= 1000){
            m_state.dmpFreq = yprCounter;
            m_state.radioFreq = radioCounter;
            m_state.loopFreq = loopCounter;
            yprCounter = 0;
            radioCounter = 0;
            loopCounter = 0;
            start = std::chrono::system_clock::now();
        }


        #ifdef TELEMETRY_TASK
        auto end_telemetry = std::chrono::system_clock::now();
        auto elapsed_telemetry = std::chrono::duration_cast<std::chrono::milliseconds>(end_telemetry - start_telemetry).count();
        if(elapsed_telemetry >= 10){
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_telemetry());
            start_telemetry = std::chrono::system_clock::now();
        }   
        #endif


        /* PID calculations*/
        pid(m_state.targetPitch, m_state.currPitch, pidPitch);
        pid(m_state.targetRoll, m_state.currRoll, pidRoll);
        pid(m_state.targetYaw, m_state.currYaw, pidYaw);

        uint16_t newMotorRlSpeed = m_state.throttle + pidPitch.c + pidRoll.c + pidYaw.c;
        uint16_t newMotorRrSpeed = m_state.throttle + pidPitch.c - pidRoll.c - pidYaw.c;
        uint16_t newMotorFlSpeed = m_state.throttle - pidPitch.c + pidRoll.c + pidYaw.c;  
        uint16_t newMotorFrSpeed = m_state.throttle - pidPitch.c - pidRoll.c - pidYaw.c;

        auto minMax = [] (uint16_t& value, uint16_t min, uint16_t max){if(value < min) value = min; if(value > max) value = max;};

        minMax(newMotorRlSpeed, Dshot::minThrottleValue, Dshot::maxThrottleValue);
        minMax(newMotorRrSpeed, Dshot::minThrottleValue, Dshot::maxThrottleValue);
        minMax(newMotorFlSpeed, Dshot::minThrottleValue, Dshot::maxThrottleValue);
        minMax(newMotorFrSpeed, Dshot::minThrottleValue, Dshot::maxThrottleValue);


        /* send updated speed values */
        struct Dshot::DshotMessage msg{};
        bool anyNewSpeed{};
        if(abs(newMotorRlSpeed - prevMotorRlSpeed) >= minStepSize){
            msg.writeTo[m_motorLaneMapping.rearLeftlane] = true;
            msg.speed[m_motorLaneMapping.rearLeftlane] = newMotorRlSpeed;
            msg.loops[m_motorLaneMapping.rearLeftlane] = -1;
            m_state.motorRlSpeed = newMotorRlSpeed;
            anyNewSpeed = true;
        }
        if(abs(newMotorRrSpeed - prevMotorRrSpeed) >= minStepSize){
            msg.writeTo[m_motorLaneMapping.rearRightlane] = true;
            msg.speed[m_motorLaneMapping.rearRightlane] = newMotorRrSpeed;
            msg.loops[m_motorLaneMapping.rearRightlane] = -1;
            m_state.motorRrSpeed = newMotorRrSpeed;
            anyNewSpeed = true;
        }
        if(abs(newMotorFlSpeed - prevMotorFlSpeed) >= minStepSize){
            msg.writeTo[m_motorLaneMapping.frontLeftlane] = true;
            msg.speed[m_motorLaneMapping.frontLeftlane] = newMotorFlSpeed;
            msg.loops[m_motorLaneMapping.frontLeftlane] = -1;
            m_state.motorFlSpeed = newMotorFlSpeed;
            anyNewSpeed = true;
        }
        if(abs(newMotorFrSpeed - prevMotorFrSpeed) >= minStepSize ){
            msg.writeTo[m_motorLaneMapping.frontRightlane] = true;
            msg.speed[m_motorLaneMapping.frontRightlane] = newMotorFrSpeed;
            msg.loops[m_motorLaneMapping.frontRightlane] = -1;
            m_state.motorFrSpeed = newMotorFrSpeed;
            anyNewSpeed = true;
        }

        prevMotorRlSpeed = newMotorRlSpeed;
        prevMotorRrSpeed = newMotorRrSpeed;
        prevMotorFlSpeed = newMotorFlSpeed;
        prevMotorFrSpeed = newMotorFrSpeed;
        
        if(newMotorRlSpeed > 70 && newMotorRrSpeed > 70 && newMotorFlSpeed > 70 && newMotorFrSpeed > 70)
            if(anyNewSpeed){
                msg.msgType = Dshot::THROTTLE;
                //print_debug(DEBUG_DRONE, DEBUG_LOGIC, "sending new dshot msg\n");
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_dshot_message(msg));
            //for(int i=0;i<Dshot::maxChannels;i++)
            //     printf("%d: writeTo %u speed %u\n", i, msg.writeTo[i], msg.speed[i]);
            //send_dshot_message(msg);
            }



        loopCounter++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

esp_err_t Drone::send_telemetry()
{
    TelemetryData telemetry{};
    telemetry.ypr = this->ypr;
    telemetry.channel = this->channel;

    telemetry.drone.isArmed = m_state.isArmed;
    telemetry.drone.mode = m_state.mode;
    telemetry.drone.dmpFreq = m_state.dmpFreq;
    telemetry.drone.radioFreq = m_state.radioFreq;
    telemetry.drone.loopFreq = m_state.loopFreq;

    telemetry.drone.motorRlSpeed = m_state.motorRlSpeed;
    telemetry.drone.motorRrSpeed = m_state.motorRrSpeed;
    telemetry.drone.motorFlSpeed = m_state.motorFlSpeed;
    telemetry.drone.motorFrSpeed = m_state.motorFrSpeed;

    telemetry.drone.targetPitch = m_state.targetPitch;
    telemetry.drone.targetYaw = m_state.targetYaw;
    telemetry.drone.targetRoll = m_state.targetRoll;
    telemetry.drone.currPitch = m_state.currPitch;
    telemetry.drone.currYaw = m_state.currYaw;
    telemetry.drone.currRoll = m_state.currRoll;

    return send_telemetry_message(telemetry);
}

esp_err_t Drone::parse_channel_state(const Channel& newChannel){
    esp_err_t status = 0;

    if(this->channel.ch1 != newChannel.ch1){
        m_state.targetRoll = mapValue(newChannel.ch1, Radio::minChannelValue, Radio::maxChannelValue, m_minRoll, m_maxRoll);
    }
    else if(this->channel.ch2 != newChannel.ch2){
        m_state.targetPitch = mapValue(newChannel.ch2, Radio::minChannelValue, Radio::maxChannelValue, m_minPitch, m_maxPitch);
    }
    else if(this->channel.ch3 != newChannel.ch3){
        m_state.throttle = mapValue(static_cast<uint16_t>(newChannel.ch3), static_cast<uint16_t>(Radio::minChannelValue), static_cast<uint16_t>(Radio::maxChannelValue), Dshot::minThrottleValue, Dshot::maxThrottleValue);
    }
    else if(this->channel.ch4 != newChannel.ch4){
        //TODO yaw
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
        status = blink_led(newChannel.ch8);
    }
    else if(newChannel.ch9 > Radio::maxChannelValue - 100 && this->channel.ch9 < Radio::minChannelValue + 100){
        status = send_beep();
    }
    else if(this->channel.ch10 != newChannel.ch10){
        
    }
    else{
        return ESP_OK;
    }

    this->channel = newChannel;

    return status;
}

esp_err_t Drone::send_dshot_message(Dshot::DshotMessage& msg){

    print_debug(DEBUG_DRONE, DEBUG_ARGS, "send_dshot_message: msg <todo>\n");
    BaseType_t res = xRingbufferSend(m_dshot_queue_handle, &msg, sizeof(Dshot::DshotMessage), 1);
    if (res != pdTRUE) {
        //printf("Failed to send dshot item: handle %p size %u\n", m_dshot_queue_handle, sizeof(Dshot::DshotMessage));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t Drone::send_telemetry_message(TelemetryData& msg){

    BaseType_t res = xRingbufferSend(telemetry_queue_handle, &msg, sizeof(TelemetryData), 1);
    if (res != pdTRUE) {
        //printf("Failed to send telemetry item: handle %p size %u\n", telemetry_queue_handle, sizeof(TelemetryData));
        return ESP_OK; // to avoid ugly error prints
        return ESP_FAIL;
    } 
    return ESP_OK;
}

esp_err_t Drone::set_armed(int value){
    esp_err_t status = 0;
    print_debug(DEBUG_DRONE, DEBUG_ARGS, "set_armed: value %d\n", value);

    Dshot::DshotMessage msg{};

    for(int i=0;i<Dshot::maxChannels;i++){
        msg.writeTo[i] = true;
        msg.cmd[i] = DSHOT_CMD_MOTOR_STOP;
        msg.loops[i] = -1;
    }


    print_debug(DEBUG_DRONE, DEBUG_DATA, "arm msg: writeTo %d cmd:%u speed: %u\n", msg.writeTo[MOTOR1], msg.cmd[MOTOR1], msg.speed[MOTOR1] );

    status = send_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to start arm process");

    m_state.isArmed = true;
  
    return ESP_OK;
}

esp_err_t Drone::send_beep(){
    esp_err_t status = 0;

    Dshot::DshotMessage msg{};
    msg.writeTo[MOTOR1] = true;
    msg.cmd[MOTOR1] = Dshot::BeepNum::BEEP1;
    msg.loops[MOTOR1] = 0;
    status = send_dshot_message(msg);


    return ESP_OK;
}

esp_err_t Drone::blink_led(uint16_t newChannelValue){
    esp_err_t status = 0;
    constexpr uint8_t channelBtn{9};

    toggleState_t state = did_channel_state_switch(newChannelValue, channelBtn);

    if(TOGGLE_HIGH){
        //status = dshot->blink_led_cmd();
        printf("not implemented!\n");
    }

    return status;
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

    if(value > (Radio::neutralChannelValue - 100) && value < (Radio::neutralChannelValue + 100)){
        m_state.mode = SELFLEVL_MODE;
    }
    else if(value > (Radio::maxChannelValue - 100)){
        m_state.mode = ACRO_MODE;
    }
    else if(value < (Radio::minChannelValue  + 100)){
        m_state.mode = ANGLE_MODE;
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

void Drone::pid(float target, float current, Pid& pid){

    float kPOut{};
    float kIOut{};
    float kDOut{};
    float err{};
    float integral{};
    float derivative{};

    err = target - current;

    kPOut = pid.kP * err;

    integral = err * pid.dt;
    kIOut = pid.kI * integral;

    derivative = (err - pid.prevErr) / pid.dt;
    kDOut = pid.kD * derivative;

    pid.prevErr = err;
    pid.c = kPOut + kIOut + kDOut;

}



 