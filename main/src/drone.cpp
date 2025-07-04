


#include <cstring>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "esp_check.h"
#include "hal/gpio_hal.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/adc_types_legacy.h"

#include "drone.h"
#include "debug.h"
#include "display.h"

#define TELEMETRY_TASK

#define log_tag "drone"

Drone *Drone::drone = nullptr;



static auto startTel = std::chrono::steady_clock::now();
static auto endTel = std::chrono::steady_clock::now();


Drone::Drone(RingbufHandle_t dshot_queue_handle, RingbufHandle_t dmp_queue_handle1, RingbufHandle_t dmp_queue_handle2, CircularHandle_t radio_queue_handle, CircularHandle_t radio_statistics_queue_handle){
    ESP_ERROR_CHECK_WITHOUT_ABORT((dmp_queue_handle1             == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((dmp_queue_handle2             == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((radio_queue_handle            == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((radio_statistics_queue_handle == nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT((dshot_queue_handle            == nullptr));

    //this->telemetry_queue_handle = xRingbufferCreate(sizeof(TelemetryData) * 20, RINGBUF_TYPE_NOSPLIT);
    this->m_telemetry_queue_handle = CircularBufCreate(10, sizeof(TelemetryData), "telemetry");
    if (this->m_telemetry_queue_handle == nullptr)
    {
        printf("Failed to create drone ring buffer\n");
    }

    printf("telemetry_queue_handle: %p m_data: %p\n", m_telemetry_queue_handle, m_telemetry_queue_handle->m_data);


    this->m_dmp_queue_handle1 = dmp_queue_handle1;
    this->m_dmp_queue_handle2 = dmp_queue_handle2;
    this->m_radio_queue_handle = radio_queue_handle;
    this->m_radio_statistics_queue_handle = radio_statistics_queue_handle;
    this->m_dshot_queue_handle = dshot_queue_handle;


}

esp_err_t Drone::set_motor_lane_mapping(const MotorLaneMapping motorMapping){

    esp_err_t status = 0;

    if (motorMapping.rearLeftlane != motorMapping.rearRightlane &&
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

Drone *Drone::GetInstance(RingbufHandle_t dshot_queue_handle, 
    RingbufHandle_t dmp_queue_handle1,
    RingbufHandle_t dmp_queue_handle2,  
    CircularHandle_t radio_queue_handle, 
    CircularHandle_t radio_statistics_queue_handle){
    if (drone == nullptr)
    {
        drone = new Drone(dshot_queue_handle, dmp_queue_handle1, dmp_queue_handle2, radio_queue_handle, radio_statistics_queue_handle);
    }
    return drone;
}

Drone *Drone::GetInstance(){
    return drone;
}

bool Drone::verify_imu_data(YawPitchRoll ypr){

    float yawHigh = 10.0;
    float yawLow = -10.0;
    float pitchHigh = 10.0;
    float pitchLow = -10.0;
    float rollHigh = 10.0;
    float rollLow = -10.0;

    if (ypr.yaw < yawLow || ypr.yaw > yawHigh)
    {
        printf("bad yaw: %.2f\n", ypr.yaw);
        return false;
    }
    else if (ypr.pitch < pitchLow || ypr.pitch > pitchHigh)
    {
        printf("bad pitch: %.2f\n", ypr.pitch);
        return false;
    }
    else if (ypr.roll < rollLow || ypr.roll > rollHigh)
    {
        printf("bad roll: %.2f\n", ypr.roll);
        return false;
    }
    return true;
}

bool Drone::verify_controller_data(Channel channel)
{
    if (channel.ch1 < Radio::minChannelValue || channel.ch1 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 1 value: %d\n", channel.ch1);
        return false;
    }
    if (channel.ch2 < Radio::minChannelValue || channel.ch2 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 2 value: %d\n", channel.ch2);
        return false;
    }
    if (channel.ch3 < Radio::minChannelValue || channel.ch3 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 3 value: %d\n", channel.ch3);
        return false;
    }
    if (channel.ch4 < Radio::minChannelValue || channel.ch4 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 4 value: %d\n", channel.ch4);
        return false;
    }
    if (channel.ch5 < Radio::minChannelValue || channel.ch5 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 5 value: %d\n", channel.ch5);
        return false;
    }
    if (channel.ch6 < Radio::minChannelValue || channel.ch6 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 6 value: %d\n", channel.ch6);
        return false;
    }
    if (channel.ch7 < Radio::minChannelValue || channel.ch7 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 7 value: %d\n", channel.ch7);
        return false;
    }
    if (channel.ch8 < Radio::minChannelValue || channel.ch8 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 8 value: %d\n", channel.ch8);
        return false;
    }
    if (channel.ch9 < Radio::minChannelValue || channel.ch9 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 9 value: %d\n", channel.ch9);
        return false;
    }
    if (channel.ch10 < Radio::minChannelValue || channel.ch10 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 10 value: %d\n", channel.ch10);
        return false;
    }
    if (channel.ch11 < Radio::minChannelValue || channel.ch11 > Radio::maxChannelValue)
    {
        // printf("Invalid channel 11 value: %d\n", channel.ch11);
        return false;
    }

    return true;
}

esp_err_t Drone::arming_process()
{

    esp_err_t status = 0;
    Channel channel{};
    YawPitchRoll ypr{};
    static bool warningIssued{};

    ESP_LOGI(log_tag, "Arming process started");
    while (true)
    {
        get_imu_data(1, ypr, 0); // just to prevent queue from filling up
        status = get_radio_data(channel, pdMS_TO_TICKS(2));
        if (status == ESP_OK)
        {
            if (channel.ch5 < Radio::minChannelValue + 100)
            {
                break;
            }
            else
            {
                if (!warningIssued)
                {
                    ESP_LOGE(log_tag, "WARNING! Drone controller set to armed. Disarm the controller now!");
                    Display::set_armed_bad_state_status(true);
                
                    warningIssued = true;
                }
            }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(log_tag, "Awaiting arm signal");
    while (true)
    {
        get_imu_data(1, ypr, 0); // just to prevent queue from filling up
        status = get_radio_data(channel, pdMS_TO_TICKS(2));
        if (status == ESP_OK)
        {
            if (channel.ch5 > Radio::maxChannelValue - 100)
            {
                break;
            }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    status = set_armed(channel.ch5);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to arm drone");

    vTaskDelay(pdMS_TO_TICKS(3500));

    ramp_up_motors();


    if(status != ESP_OK){
        ESP_LOGE(log_tag, "Drone arming failed");
        status = set_armed(Radio::minChannelValue);
        return ESP_FAIL;
    }

    ESP_LOGI(log_tag, "Drone armed");
    //Display::set_armed_status(true);

    return ESP_OK;
}

esp_err_t Drone::get_imu_data(int imuNr, YawPitchRoll &newData, TickType_t ticks)
{

    size_t item_size = sizeof(YawPitchRoll);

    RingbufHandle_t handle = imuNr == 1? m_dmp_queue_handle1 : m_dmp_queue_handle2;

    YawPitchRoll *data = (YawPitchRoll *)xRingbufferReceive(handle, &item_size, ticks);
    if (data != nullptr)
    {
        newData = *data;
        vRingbufferReturnItem(handle, (void *)data);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t Drone::get_radio_data(Channel &newData, TickType_t ticks){

    size_t item_size = sizeof(Channel);
    
    esp_err_t status = CircularBufDequeue(m_radio_queue_handle, &newData, 0);

    return status;
}

esp_err_t Drone::get_radio_statistics(RadioStatistics &newData, TickType_t ticks){

    size_t item_size = sizeof(radio_statistics);
    
    esp_err_t status = CircularBufDequeue(m_radio_statistics_queue_handle, &newData, 0);

    return status;
}


esp_err_t Drone::verify_components_process()
{

    esp_err_t status = 0;

    Channel channelPrev{};
    YawPitchRoll yprPrev{};

    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;
    uint64_t yprValidCounter = 0;
    uint64_t radioValidCounter = 0;

    ESP_LOGI(log_tag, "Verifying IMU and Radio");

    auto start = std::chrono::steady_clock::now();

    while (true)
    {

        bool yprIsValid{};
        bool radioIsValid{};

        Channel channel{};
        YawPitchRoll ypr{};

        /* get new data */
        status = get_imu_data(1, ypr, pdMS_TO_TICKS(1));
        if (status == ESP_OK)
        {
            yprCounter++;
            yprIsValid = true;

        }

        status = get_radio_data(channel, pdMS_TO_TICKS(1));
        if (status == ESP_OK)
        {
            radioCounter++;
            radioIsValid = true;
        }
        else{
            continue;
        }

        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (elapsed < 2000)
        {
            yprCounter = 0;
            radioCounter = 0;
            continue;
        }

        if (verify_imu_data(ypr) && yprIsValid)
            yprValidCounter++;

        if (verify_controller_data(channel) && radioIsValid)
            radioValidCounter++;

        if (yprCounter > 1000)
        {
            if ((yprValidCounter / yprCounter) > 0.9)
                yprIsValid = true;
            else if ((yprValidCounter / yprCounter) < 0.9)
                yprIsValid = false;
        }

        if (radioCounter > 1000)
        {
            if (((float)radioValidCounter / radioCounter) > 0.9)
                radioIsValid = true;
            else if (((float)radioValidCounter / radioCounter) < 0.9)
                radioIsValid = false;
        }

        /* incase overflow */
        if (yprCounter == UINT64_MAX)
        {
            printf("ERROR MAX VALUE YPRCOUNTER: %llu", yprCounter);
            yprCounter = 0;
            yprValidCounter = 0;
        }
        if (radioCounter == UINT64_MAX)
        {
            printf("ERROR MAX VALUE RADIOCOUNTER: %llu", radioCounter);
            radioCounter = 0;
            radioValidCounter = 0;
        }

        if (yprCounter && radioIsValid)
            break;

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(log_tag, "Verifying complete");
    return ESP_OK;
}

ToggleState Drone::did_channel_state_switch(uint16_t newChannelValue, uint8_t channelNr)
{
    uint16_t prevChannelValue{};

    // Map the channel number to the corresponding member in the channel struct
    switch (channelNr)
    {
    case 1:
        prevChannelValue = this->channel.ch1;
        break;
    case 2:
        prevChannelValue = this->channel.ch2;
        break;
    case 3:
        prevChannelValue = this->channel.ch3;
        break;
    case 4:
        prevChannelValue = this->channel.ch4;
        break;
    case 5:
        prevChannelValue = this->channel.ch5;
        break;
    case 6:
        prevChannelValue = this->channel.ch6;
        break;
    case 7:
        prevChannelValue = this->channel.ch7;
        break;
    case 8:
        prevChannelValue = this->channel.ch8;
        break;
    case 9:
        prevChannelValue = this->channel.ch9;
        break;
    case 10:
        prevChannelValue = this->channel.ch10;
        break;
    case 11:
        prevChannelValue = this->channel.ch11;
        break;
    case 12:
        prevChannelValue = this->channel.ch12;
        break;
    case 13:
        prevChannelValue = this->channel.ch13;
        break;
    case 14:
        prevChannelValue = this->channel.ch14;
        break;
    case 15:
        prevChannelValue = this->channel.ch15;
        break;
    case 16:
        prevChannelValue = this->channel.ch16;
        break;
    default:
        printf("Error: Invalid channel number %u\n", channelNr);
        return TOGGLE_UNCHANGED;
    }

    if (newChannelValue >= Radio::maxChannelThld && newChannelValue != prevChannelValue)
    {
        return TOGGLE_HIGH;
    }
    else if (newChannelValue <= Radio::minChannelThld && newChannelValue != prevChannelValue)
    {
        return TOGGLE_LOW;
    }
    else if (newChannelValue == prevChannelValue)
    {
        return TOGGLE_UNCHANGED;
    }

    printf("Error: Bad toggle state detected on channel %u | new: %u prev: %u\n",
           channelNr, newChannelValue, prevChannelValue);
    return TOGGLE_UNCHANGED;
}

uint16_t Drone::mapValue(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max){
    if (x == 992)
    {
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int Drone::mapValue(int x, int in_min, int in_max, int out_min, int out_max){
    if (x == 992)
    {
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

float Drone::mapValue(int x, int in_min, int in_max, float out_min, float out_max){
    if (x == 992)
    {
        return 0;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Drone::mapValue2(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min, float out_max){
    if (x == 992)
    {
        return 0;
    }
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// return the motor to request telemetry from if telemetry is true. Occurs ever x ms
esp_err_t Drone::signal_telemetry_request(Dshot::DshotMessage& msg, bool& newTelemetryReq){

    esp_err_t status = 0;
    uint8_t constexpr interval{5};
    constexpr uint8_t modNr = Dshot::maxChannels;
    static uint64_t motorCounter{};


    static auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if(elapsed >= interval){
        uint8_t idx = motorCounter % modNr;
        msg.telemetryReq[idx] = true;
        msg.writeTo[idx] = true;
        m_telemetryReqIdx.store(idx);
        start = std::chrono::system_clock::now();
        motorCounter++;
        newTelemetryReq = true;
    }


    return status;
}

void Drone::drone_task(void *args)
{

    esp_err_t status = 0;

    constexpr uint8_t minStepSize{2};
    constexpr float radToDegree = (180.0 / M_PI);

    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;
    uint64_t loopCounter = 0;
    bool prevIsArmed{};

    auto start = std::chrono::system_clock::now();
    auto start_telemetry = std::chrono::system_clock::now();


    /* await valid controller & gyro */
    verify_components_process();

    // redo arming until successfull
    while(true){
        status = arming_process();
        if(status == ESP_OK){
            break;
        }
    }
    

    while (true)
    {

        Dshot::DshotMessage msg{};
        Channel channelNew{};
        RadioStatistics radioStatisticsNew{};

        bool newTelemetryReq{};
        bool newSpeedValues{};
        bool newImuData1{};
        bool newImuData2{};

        status = get_imu_data(1, ypr1, 0);
        if (status == ESP_OK)
        {
            this->ypr1 = ypr1;
            yprCounter++;
            m_state.currPitch = ypr1.pitch * radToDegree;
            m_state.currYaw = ypr1.yaw * radToDegree;
            m_state.currRoll = ypr1.roll * radToDegree;
            newImuData1 = true;
        }

        status = get_imu_data(2, ypr2, 0);
        if (status == ESP_OK)
        {
            this->ypr2 = ypr2;
            yprCounter++;
            m_state.currPitch = ypr2.pitch * radToDegree;
            m_state.currYaw = ypr2.yaw * radToDegree;
            m_state.currRoll = ypr2.roll * radToDegree;
            newImuData2 = true;
        }



        status = get_radio_data(channelNew, 0);
        if (status == ESP_OK)
        {
            parse_channel_state(channelNew);
            this->channel = channelNew;
            radioCounter++;
        }

        status = get_radio_statistics(radioStatisticsNew, 0);
        if (status == ESP_OK)
        {
            this->radio_statistics = radioStatisticsNew;
            handle_signal_lost();

        }


        /* calculate and set the frequenzy of received data */
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if (elapsed >= 1000)
        {
            m_state.dmpFreq = yprCounter;
            m_state.radioFreq = radioCounter;
            m_state.loopFreq = loopCounter;
            yprCounter = 0;
            radioCounter = 0;
            loopCounter = 0;
            start = std::chrono::system_clock::now();
        }


        /* send dshot telemetry command */
        ESP_ERROR_CHECK_WITHOUT_ABORT(signal_telemetry_request(msg, newTelemetryReq));
        startTel = std::chrono::steady_clock::now();

        /*  PID for new throttle values*/
        get_new_speed(msg, newSpeedValues);

        /*  send telemetry and state data */
        #ifdef TELEMETRY_TASK
        auto end_telemetry = std::chrono::system_clock::now();
        auto elapsed_telemetry = std::chrono::duration_cast<std::chrono::milliseconds>(end_telemetry - start_telemetry).count();
        if (elapsed_telemetry >= 100)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(measure_current());
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_telemetry());
            start_telemetry = std::chrono::system_clock::now();
        }
        #endif

        loopCounter++;

        /* avoid dshot messages if unarmed*/
        if(!m_state.isArmed){
            vTaskDelay(pdMS_TO_TICKS(20));
            prevIsArmed = true;
            continue;
        }
        else if(m_state.isArmed && !prevIsArmed){
            prevIsArmed = true;
            ramp_up_motors();
        }


        if(!newSpeedValues && !newTelemetryReq){
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }


        ESP_ERROR_CHECK_WITHOUT_ABORT(send_dshot_message(msg));

        taskYIELD();
    }
}

void Drone::esc_telemetry_task(void* args){

    uart_event_t event;
    uint8_t data[512]{};
    size_t dataLength{}; 
    constexpr uint8_t chunkSize{10};
    constexpr uint8_t crcIdx{9};

    vTaskDelay(pdMS_TO_TICKS(5000));


    while(true){

        // wait until armed for telemetry data
        while(m_state.isArmed == false){
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
            if (event.type == UART_DATA) {
                memset(data, 0, 512);
                dataLength = 0; //should be 10 bytes

                ESP_ERROR_CHECK(uart_get_buffered_data_len(this->uartNum, &dataLength));
                dataLength = uart_read_bytes(this->uartNum, data, dataLength, 100);

                if(!dataLength){
                    vTaskDelay(pdMS_TO_TICKS(2));
                    continue;
                }
            }
        }
 

        for(int i=0;i<dataLength;i+=chunkSize){

            uint16_t chksum = calculateCrc8(data, 9);

            if(data[crcIdx] == chksum){

                endTel = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endTel-startTel).count();

                if(elapsed <= 20000){   //only collect data if sample was within the time window or it might belong to another motor than expected
                    //printf("telemetry elapsed time: %lld us\n", elapsed);
                    uint8_t temperature = data[i];
                    uint16_t voltage = (data[i+1] << 8 | data[i+2]); //scaling factor
                    uint16_t current = (data[i+3] << 8 | data[i+4]);      //maybe needs scaling factor
                    uint16_t consumption = (data[i+5] << 8 | data[i+6]);  // idk?
                    uint16_t rpm = (data[i+7] << 8 | data[i+8]);      // pole pairs of engine  
    
                    uint8_t idx = m_telemetryReqIdx.load();
                    set_esc_telemetry_data(temperature, voltage, current, consumption, rpm, idx);
                }
                else{
                    print_debug(DEBUG_DRONE, DEBUG_DATA, "failed to recieve amp reading in time\n");
                }

            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void Drone::set_esc_telemetry_data(uint8_t temperature, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t rpm, uint8_t motorPos){

    m_escTelemetryLock.lock();

    m_state.escState[motorPos].temperature = temperature;
    m_state.escState[motorPos].voltage = voltage;
    m_state.escState[motorPos].current = current;
    m_state.escState[motorPos].consumption = consumption;
    m_state.escState[motorPos].rpm = rpm;

    m_escTelemetryLock.unlock();

    //printf("setting motor %u, voltage: %.2fv\n", motorPos, m_state.escState[motorPos].voltage * 0.01);

}

esp_err_t Drone::init_uart(int rxPin, int txPin, int baudrate){

    esp_err_t status = 0;

    constexpr int rtsPin = UART_PIN_NO_CHANGE;
    constexpr int ctsPin = UART_PIN_NO_CHANGE;
    constexpr int uart_buffer_size = (512);
    uart_config_t uart_config;

    uart_config.baud_rate = baudrate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;  

    this->uartNum = UART_NUM_1;

    ESP_ERROR_CHECK(uart_param_config(this->uartNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(this->uartNum, txPin, rxPin, rtsPin, ctsPin));
    ESP_ERROR_CHECK(uart_driver_install(this->uartNum, uart_buffer_size, uart_buffer_size, 10, &this->uart_queue, 0));


    return status;
}

void Drone::get_new_speed(Dshot::DshotMessage& msg, bool& didChange){

    constexpr uint8_t minStepSize{2};
    const uint16_t maxThrottleValue = c_saftyParams? 500u : Dshot::maxThrottleValue;

    static uint16_t prevMotorRlSpeed{c_minThrottleValue};
    static uint16_t prevMotorRrSpeed{c_minThrottleValue};
    static uint16_t prevMotorFlSpeed{c_minThrottleValue};
    static uint16_t prevMotorFrSpeed{c_minThrottleValue};


    /* PID calculations*/
    pid(m_state.targetPitch, m_state.currPitch, m_pid[PITCH]);
    pid(m_state.targetRoll, m_state.currRoll, m_pid[ROLL]);
    pid(m_state.targetYaw, m_state.currYaw, m_pid[YAW]);

    int _rlSpeed = m_state.throttle + m_pid[PITCH].c + m_pid[ROLL].c + m_pid[YAW].c;
    int _rrSpeed = m_state.throttle + m_pid[PITCH].c - m_pid[ROLL].c - m_pid[YAW].c;
    int _flSpeed = m_state.throttle - m_pid[PITCH].c + m_pid[ROLL].c + m_pid[YAW].c;
    int _frSpeed = m_state.throttle - m_pid[PITCH].c - m_pid[ROLL].c - m_pid[YAW].c;
    //printf("_flSpeed: %u = throttle: %u + pitch: %.2f + roll: %.2f yaw: %.2f\n", _flSpeed, m_state.throttle, m_pid[PITCH].c, m_pid[ROLL].c, m_pid[YAW].c);

    //avoid underflow
    uint16_t rlSpeed =  _rlSpeed >= 0? static_cast<uint16_t>(_rlSpeed) : 0u;  
    uint16_t rrSpeed =  _rrSpeed >= 0? static_cast<uint16_t>(_rrSpeed) : 0u;  
    uint16_t flSpeed =  _flSpeed >= 0? static_cast<uint16_t>(_flSpeed) : 0u;  
    uint16_t frSpeed =  _frSpeed >= 0? static_cast<uint16_t>(_frSpeed) : 0u;  

    //printf("flSpeed: %u frSpeed: %u rlSpeed: %u rrSpeed: %u\n", flSpeed, frSpeed, rlSpeed, rrSpeed);

    // fit to range
    auto minMax = [](uint16_t& value, uint16_t min, uint16_t max)
    {if(value < min) value = min; if(value > max) value = max;};

    minMax(rlSpeed, c_minThrottleValue, maxThrottleValue);  
    minMax(rrSpeed, c_minThrottleValue, maxThrottleValue);  
    minMax(flSpeed, c_minThrottleValue, maxThrottleValue);  
    minMax(frSpeed, c_minThrottleValue, maxThrottleValue);  

    if (abs(rlSpeed - prevMotorRlSpeed) >= minStepSize)
    {
        msg.writeTo[m_motorLaneMapping.rearLeftlane] = true;
        didChange = true;
    }
    if (abs(rrSpeed - prevMotorRrSpeed) >= minStepSize)
    {
        msg.writeTo[m_motorLaneMapping.rearRightlane] = true;
        didChange = true;
    }
    if (abs(flSpeed - prevMotorFlSpeed) >= minStepSize)
    {
        msg.writeTo[m_motorLaneMapping.frontLeftlane] = true;
        didChange = true;
    }
    if (abs(frSpeed - prevMotorFrSpeed) >= minStepSize)
    {
        msg.writeTo[m_motorLaneMapping.frontRightlane] = true;
        didChange = true;
    }    

    msg.speed[m_motorLaneMapping.rearLeftlane] = rlSpeed;
    msg.loops[m_motorLaneMapping.rearLeftlane] = -1;
    m_state.motorRlSpeed = rlSpeed;

    msg.speed[m_motorLaneMapping.rearRightlane] = rrSpeed;
    msg.loops[m_motorLaneMapping.rearRightlane] = -1;
    m_state.motorRrSpeed = rrSpeed;

    msg.speed[m_motorLaneMapping.frontLeftlane] = flSpeed;
    msg.loops[m_motorLaneMapping.frontLeftlane] = -1;
    m_state.motorFlSpeed = flSpeed;

    msg.speed[m_motorLaneMapping.frontRightlane] = frSpeed;
    msg.loops[m_motorLaneMapping.frontRightlane] = -1;
    m_state.motorFrSpeed = frSpeed;


    if(didChange){
        prevMotorRlSpeed = rlSpeed;
        prevMotorRrSpeed = rrSpeed;
        prevMotorFlSpeed = flSpeed;
        prevMotorFrSpeed = frSpeed;
    }

    msg.msgType = Dshot::THROTTLE;


    if(flSpeed >= 2048){
        printf("ERROR bad fl throttle: %u\n", flSpeed);
    }
    else if(frSpeed >= 2048){
        printf("ERROR bad fr throttle: %u\n", frSpeed);
    }
    else if(rlSpeed >= 2048){
        printf("ERROR bad rl throttle: %u\n", rlSpeed);
    }
    else if(rrSpeed >= 2048){
        printf("ERROR bad rr throttle: %u\n", rrSpeed);
    }

}


#include "esp_adc_cal.h"

#define DEFAULT_VREF     1100  // in mV
#define ESC_MV_PER_AMP   50    // ESC outputs 50mV per Amp

esp_err_t Drone::measure_current() {
    static bool init = false;
    if (!init) {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1 << 36;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);  // up to ~3.3V
        init = true;
    }

    const uint8_t samples = 100;
    std::vector<uint32_t> voltageSamples;
    
    for (int i = 0; i < samples; i++) {
        uint32_t raw = adc1_get_raw(ADC1_CHANNEL_0);
        //printf("raw: %lu\n", raw);
        voltageSamples.push_back(raw);
    }

    std::sort(voltageSamples.begin(), voltageSamples.end());
    uint32_t medianRaw = voltageSamples[samples / 2];

    // Convert raw ADC to voltage
    float voltage = (medianRaw / 4095.0f) * 3.3f;  // volts

    // Convert voltage to current (A)
    float current = voltage / (ESC_MV_PER_AMP / 1000.0f);  // mV per Amp → V per Amp

    m_state.currentDraw = current;

    //printf("ADC raw: %lu -> voltage: %.3f V -> current: %.2f A\n", medianRaw, voltage, current);

    return ESP_OK;
}

esp_err_t Drone::ramp_up_motors(){
    esp_err_t status = 0;
    // ramp from min throttle to our min value

    for(uint16_t i=48;i<c_minThrottleValue;i++){ 
        Dshot::DshotMessage msg{};

        msg.msgType = Dshot::THROTTLE;

        for(int j=0;j<Dshot::maxChannels;j++){ 
            msg.writeTo[j] = true;
            msg.loops[j] = -1;
            msg.speed[j] = i;
        }

        status = send_dshot_message(msg);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to send dshot message");

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    m_state.motorFlSpeed = c_minThrottleValue;
    m_state.motorFrSpeed = c_minThrottleValue;
    m_state.motorRlSpeed = c_minThrottleValue;
    m_state.motorRrSpeed = c_minThrottleValue;

    return status;
    
    uint16_t max = c_minThrottleValue + 150u;
    printf("MAX: %u\n", max);
    while(true){
  
        printf("increasing...\n");
        for(uint16_t i=max;i>c_minThrottleValue;i--){ 
            Dshot::DshotMessage msg{};
    
            msg.msgType = Dshot::THROTTLE;
    
            for(int j=0;j<Dshot::maxChannels;j++){ 
                msg.writeTo[j] = true;
                msg.loops[j] = -1;
                msg.speed[j] = i;
            }
    
            status = send_dshot_message(msg);
    
            vTaskDelay(pdMS_TO_TICKS(10));
        } 

        printf("increasing...\n");
        for(uint16_t i=c_minThrottleValue;i<max;i++){ 
            Dshot::DshotMessage msg{};
    
            msg.msgType = Dshot::THROTTLE;
    
            for(int j=0;j<Dshot::maxChannels;j++){ 
                msg.writeTo[j] = true;
                msg.loops[j] = -1;
                msg.speed[j] = i;
            }
    
            status = send_dshot_message(msg);
    
            vTaskDelay(pdMS_TO_TICKS(10));
        } 

    }



    return status;

}

esp_err_t Drone::send_telemetry(){
    TelemetryData telemetry{};
    telemetry.ypr = this->ypr1;
    telemetry.channel = this->channel;
    telemetry.radioStatistics = radio_statistics;

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

    telemetry.drone.currentDraw = m_state.currentDraw;

    m_escTelemetryLock.lock();

    for(int i=0;i<Dshot::maxChannels;i++){
        telemetry.drone.escState[i].temperature = m_state.escState[i].temperature;
        telemetry.drone.escState[i].voltage = m_state.escState[i].voltage;
        telemetry.drone.escState[i].current = m_state.escState[i].current;
        telemetry.drone.escState[i].consumption = m_state.escState[i].consumption;
        telemetry.drone.escState[i].rpm = m_state.escState[i].rpm;   
    }
    m_escTelemetryLock.unlock();


    return send_telemetry_message(telemetry);
}

esp_err_t Drone::parse_channel_state(const Channel &newChannel)
{
    esp_err_t status = 0;

    if (this->channel.ch1 != newChannel.ch1)
    {
        m_state.targetRoll = mapValue(newChannel.ch1, Radio::minChannelValue, Radio::maxChannelValue, m_minRoll, m_maxRoll);
    }
    if (this->channel.ch2 != newChannel.ch2)
    {
        m_state.targetPitch = mapValue(newChannel.ch2, Radio::minChannelValue, Radio::maxChannelValue, m_minPitch, m_maxPitch);
    }
    if (this->channel.ch3 != newChannel.ch3)
    {
        // controller sends ~300 min value 
        m_state.throttle = mapValue(static_cast<uint16_t>(newChannel.ch3), 300u/*static_cast<uint16_t>(Radio::minChannelValue) */, static_cast<uint16_t>(Radio::maxChannelValue), c_minThrottleValue, Dshot::maxThrottleValue);
    }
    if (this->channel.ch4 != newChannel.ch4)
    {
        // TODO yaw
    }
    if (this->channel.ch5 != newChannel.ch5)
    {
        //printf("%u - %u\n", this->channel.ch5, newChannel.ch5);
        status = set_armed(newChannel.ch5);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to arm drone");
    }
    if (this->channel.ch6 != newChannel.ch6)
    {
        status = set_flight_mode(newChannel.ch6);
        ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set flight mode");
    }
    if (this->channel.ch7 != newChannel.ch7)
    {
    }
    if (this->channel.ch8 != newChannel.ch8)
    {
        //status = blink_led(newChannel.ch8);
    }
    if (newChannel.ch9 > Radio::maxChannelThld && this->channel.ch9 < Radio::minChannelThld)
    {
        status = toggle_motor_direction();
    }
    if (this->channel.ch10 != newChannel.ch10)
    {

    }

    this->channel = newChannel;

    return status;
}

esp_err_t Drone::send_dshot_message(Dshot::DshotMessage &msg)
{

    //msg.writeTo[m_motorLaneMapping.frontLeftlane] = false;
    //msg.writeTo[m_motorLaneMapping.frontRightlane] = false;
    //msg.writeTo[m_motorLaneMapping.rearLeftlane] = false;
    //msg.writeTo[m_motorLaneMapping.rearRightlane] = false;

    print_debug(DEBUG_DRONE, DEBUG_ARGS, "send_dshot_message: msg <todo>\n");
    BaseType_t res = xRingbufferSend(m_dshot_queue_handle, &msg, sizeof(Dshot::DshotMessage), 1);
    if (res != pdTRUE)
    {
        printf("Failed to send dshot item: handle %p size %u\n", m_dshot_queue_handle, sizeof(Dshot::DshotMessage));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t Drone::send_telemetry_message(TelemetryData &msg)
{

    esp_err_t status = CircularBufEnqueue(m_telemetry_queue_handle, &msg);

    //BaseType_t res = xRingbufferSend(telemetry_queue_handle, &msg, sizeof(TelemetryData), 1);
    //if (res != pdTRUE)
    //{
    //    //printf("Failed to send telemetry item: handle %p size %u\n", telemetry_queue_handle, sizeof(TelemetryData));
    //    //return ESP_FAIL;
    //    return ESP_OK;
    //}
    //printf("send ok: %p\n", telemetry_queue_handle);
    return ESP_OK;
}

esp_err_t Drone::set_armed(uint32_t value)
{
    esp_err_t status = 0;
    print_debug(DEBUG_DRONE, DEBUG_ARGS, "set_armed: value %d\n", value);

    //channel.ch5 = value;

    Dshot::DshotMessage msg{};

    for (int i = 0; i < Dshot::maxChannels; i++)
    {
        msg.writeTo[i] = true;
        msg.cmd[i] = DSHOT_CMD_MOTOR_STOP;
        msg.loops[i] = -1;
    }
    
    print_debug(DEBUG_DRONE, DEBUG_DATA, "arm msg: writeTo %d cmd:%u speed: %u\n", msg.writeTo[MOTOR1], msg.cmd[MOTOR1], msg.speed[MOTOR1]);
    
    status = send_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to start arm process");

    if(value >= Radio::maxChannelThld){
        m_state.isArmed = true;
        Display::set_armed_status(true);
    }
    else if(value <= Radio::minChannelThld){
        m_state.isArmed = false;
        Display::set_armed_status(false);
    }



    return ESP_OK;
}

esp_err_t Drone::send_beep()
{
    esp_err_t status = 0;

    Dshot::DshotMessage msg{};
    msg.writeTo[MOTOR1] = true;
    msg.cmd[MOTOR1] = Dshot::BeepNum::BEEP1;
    msg.loops[MOTOR1] = 0;
    status = send_dshot_message(msg);

    return ESP_OK;
}

esp_err_t Drone::toggle_motor_direction(){
    esp_err_t status = 0;

    static bool toggle = false;

    Dshot::DshotMessage msg{};
    msg.writeTo[MOTOR1] = true;
    msg.cmd[MOTOR1] = toggle? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_2;
    msg.loops[MOTOR1] = 0;
    status = send_dshot_message(msg);

    toggle = !toggle;

    return ESP_OK;
}

esp_err_t Drone::blink_led(uint16_t newChannelValue){
    esp_err_t status = 0;
    constexpr uint8_t channelBtn{9};

    toggleState_t state = did_channel_state_switch(newChannelValue, channelBtn);

    if (TOGGLE_HIGH)
    {
        // status = dshot->blink_led_cmd();
        printf("blink_led not implemented!\n");
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

    if (value > (Radio::neutralChannelValue - 100) && value < (Radio::neutralChannelValue + 100))
    {
        m_state.mode = SELFLEVL_MODE;
    }
    else if (value > (Radio::maxChannelValue - 100))
    {
        m_state.mode = ACRO_MODE;
    }
    else if (value < (Radio::minChannelValue + 100))
    {
        m_state.mode = ANGLE_MODE;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
    return status;
}

esp_err_t Drone::set_motor_direction(enum Motor motorNum, enum MotorDirection direction)
{

    // parse data etc etc
    // dshot->write();
    return ESP_FAIL;
}

esp_err_t Drone::get_motor_direction(enum Motor motorNum, enum MotorDirection &direction)
{

    // parse data etc etc
    // dshot->read();
    return ESP_FAIL;
}

void Drone::pid(float target, float current, Pid &pid){

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

uint8_t Drone::updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

uint8_t Drone::calculateCrc8(const uint8_t *Buf, const uint8_t BufLen){
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++) {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
}

esp_err_t Drone::handle_signal_lost(){

    static uint32_t counter{};

    if(counter >= 10){
        // cut power
    }

    if(radio_statistics.upplink_quality == 0){

        counter++;
    }
    else{
        counter = 0;
    }



 return 0;
}