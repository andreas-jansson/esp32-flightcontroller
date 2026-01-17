

#include <cstring>
#include <chrono>
#include <cmath>
#include <algorithm>

#include "esp_check.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
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
QueueHandle_t Drone::m_pid_conf_sem;

SemaphoreHandle_t timer_sem = nullptr;
Channel Drone::channel{};
Pid Drone::m_pid[3]{};
char* Drone::s_name{};
DroneState Drone::m_state{};
FwVersion Drone::m_fwVersion{};
BatteryInfo Drone::m_batInfo{};
RadioStatistics Drone::radio_statistics{};

static bool oneshot_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    xSemaphoreGive(timer_sem);
    return true;
}

Drone::Drone(Dshot600 *dshotObj, std::vector<ImuIf*> mpu, CircularHandle_t radio_queue_handle, CircularHandle_t radio_statistics_queue_handle)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(mpu.size() == 0);
    ESP_ERROR_CHECK_WITHOUT_ABORT(radio_queue_handle == nullptr);
    ESP_ERROR_CHECK_WITHOUT_ABORT(radio_statistics_queue_handle == nullptr);
    ESP_ERROR_CHECK_WITHOUT_ABORT(dshotObj == nullptr);

    //printf("ryssdodarn\n");
    //char name[]{"ryssdodarn"};
    //s_name = static_cast<char*>(calloc(0, strlen(name) + 1));
    //if(s_name != nullptr)
    //    memcpy(s_name, name, strlen(name));
    //printf("s_name[%s]\n", s_name);

    this->m_telemetry_queue_handle = CircularBufCreate(10, sizeof(TelemetryData), "telemetry");
    if (this->m_telemetry_queue_handle == nullptr)
    {
        printf("Failed to create drone ring buffer\n");
    }

    printf("telemetry_queue_handle: %p m_data: %p\n", m_telemetry_queue_handle, m_telemetry_queue_handle->m_data);

    this->m_blackbox_telemetry_queue_handle = CircularBufCreate(10, sizeof(TelemetryData), "bb_telemetry");
    if (this->m_blackbox_telemetry_queue_handle == nullptr){
        printf("Failed to create drone blackbox buffer\n");
    }

    this->m_mpuObj1 = mpu;

    for(const auto& m : m_mpuObj1){
        RingbufHandle_t handle = m->get_queue_handle();
        ESP_ERROR_CHECK_WITHOUT_ABORT(handle == nullptr);
        m_dmp_queue_handle.emplace_back(handle);
    }

    this->m_radio_queue_handle = radio_queue_handle;
    this->m_radio_statistics_queue_handle = radio_statistics_queue_handle;
    this->m_dshotObj = dshotObj;

    vSemaphoreCreateBinary(m_pid_conf_sem);
    ESP_ERROR_CHECK_WITHOUT_ABORT((m_pid_conf_sem == nullptr));
    xSemaphoreTake(m_pid_conf_sem, 0);

    vSemaphoreCreateBinary(timer_sem);
    ESP_ERROR_CHECK_WITHOUT_ABORT((timer_sem == nullptr));
    xSemaphoreTake(timer_sem, 0);

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
        .direction = GPTIMER_COUNT_UP,      // Counting direction is up
        .resolution_hz = 1 * 1000 * 1000,   // Resolution is 1 MHz, i.e., 1 tick equals 1 microsecond
    };
    // Create a timer instance
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    const uint32_t loop_hz = 2000;
    const uint32_t alarm_ticks = timer_config.resolution_hz / loop_hz; // 1e6/2000 = 500

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = alarm_ticks,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = 1u,
        }
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = oneshot_timer_callback,
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_sem));

    // Enable the timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // Start the timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

esp_err_t Drone::set_motor_lane_mapping(const MotorLaneMapping motorMapping)
{

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

Drone *Drone::GetInstance(Dshot600 *dshotObj,
                          std::vector<ImuIf*> mpu,
                          CircularHandle_t radio_queue_handle,
                          CircularHandle_t radio_statistics_queue_handle)
{
    if (drone == nullptr)
    {
        drone = new Drone(dshotObj, mpu, radio_queue_handle, radio_statistics_queue_handle);
    }
    return drone;
}

Drone *Drone::GetInstance()
{
    return drone;
}

bool Drone::verify_imu_data(YawPitchRoll ypr)
{

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

    status = arm_drone(channel.ch5);
    ESP_RETURN_ON_ERROR(status, log_tag, "Failed to arm drone");

    m_state.motorFlSpeed = c_minThrottleValue;
    m_state.motorFrSpeed = c_minThrottleValue;
    m_state.motorRlSpeed = c_minThrottleValue;
    m_state.motorRrSpeed = c_minThrottleValue;

    if (status != ESP_OK)
    {
        ESP_LOGE(log_tag, "Drone arming failed");
        status = arm_drone(Radio::minChannelValue);
        return ESP_FAIL;
    }

    ESP_LOGI(log_tag, "Drone armed");

    return ESP_OK;
}

esp_err_t Drone::get_imu_data(int imuNr, YawPitchRoll &newData, TickType_t ticks)
{

    size_t item_size = sizeof(YawPitchRoll);

    RingbufHandle_t handle = m_dmp_queue_handle[imuNr - 1];
    YawPitchRoll *data = (YawPitchRoll *)xRingbufferReceive(handle, &item_size, ticks);
    if (data != nullptr)
    {
        newData = *data;
        vRingbufferReturnItem(handle, (void *)data);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t Drone::get_radio_data(Channel &newData, TickType_t ticks)
{

    size_t item_size = sizeof(Channel);

    esp_err_t status = CircularBufDequeue(m_radio_queue_handle, &newData, 0);

    return status;
}

esp_err_t Drone::get_radio_statistics(RadioStatistics &newData, TickType_t ticks)
{

    size_t item_size = sizeof(radio_statistics);

    esp_err_t status = CircularBufDequeue(m_radio_statistics_queue_handle, &newData, 0);

    return status;
}

esp_err_t Drone::verify_components_process()
{

    esp_err_t status = 0;

    Channel channelPrev{};
    //YawPitchRoll yprPrev{};

    //uint64_t yprCounter1 = 0;
    //uint64_t yprCounter2 = 0;
    uint64_t radioCounter = 0;
    //uint64_t yprValidCounter1 = 0;
    //uint64_t yprValidCounter2 = 0;
    uint64_t radioValidCounter = 0;

    ESP_LOGI(log_tag, "Verifying IMU and Radio");
    Display::set_verify_mpu1_status(STARTED);
    Display::set_verify_mpu2_status(STARTED);
    Display::set_verify_radio_status(STARTED);

    bool mpu1Complete{};
    bool mpu2Complete{};
    bool radioComplete{};

    struct yprContext{
        YawPitchRoll ypr{};
        YawPitchRoll yprPrev{};
        bool complete{};
        bool isValidSample{};
        uint64_t counter{};
        uint64_t validCounter = 0;
        RingbufHandle_t handle{};
    };

    std::vector<yprContext> yprCtx;

    for(const auto& m : m_mpuObj1){
        yprContext ctx{};
        yprCtx.emplace_back(ctx);
    }

    auto start = std::chrono::steady_clock::now();

    while (true)
    {

        // bool yprIsValid1{};
        // bool yprIsValid2{};
        bool radioIsValid{};

        Channel channel{};

        uint8_t imuIdx{1};
        for(auto& ctx : yprCtx){

            status = get_imu_data(imuIdx, ctx.ypr, 0);

            if (status == ESP_OK)
            {
                ctx.counter++;
                ctx.isValidSample = true;
                if(imuIdx == 1)
                    Display::set_mpu1_angle(ctx.ypr);
                else if(imuIdx == 2)
                    Display::set_mpu1_angle(ctx.ypr);
            }
            imuIdx++;
        }

        status = get_radio_data(channel, 0);
        if (status == ESP_OK)
        {
            radioCounter++;
            radioIsValid = true;
        }

        /* wait for 2s before verifying, allows draining the ringbuffers while waiting */
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (elapsed < 2000)
        {
            for(auto& ctx : yprCtx){
                ctx.counter = 0;
            }
            radioCounter = 0;
            continue;
        }

        /* verify recieved data to be within margins */
        for(auto& ctx : yprCtx){
            if (verify_imu_data(ctx.ypr) && ctx.isValidSample)
                ctx.validCounter++;
        }

        if (verify_controller_data(channel) && radioIsValid)
            radioValidCounter++;

        /* if 90% of 300 sampels are valid, approve component*/

        uint8_t mpuIdx{1};
        for(auto& ctx : yprCtx){
            if (ctx.counter > 100)
            {
                if ((ctx.validCounter / ctx.counter) >= 0.9)
                {
                    if(mpuIdx == 1)
                        Display::set_verify_mpu1_status(PASS);
                    else if(mpuIdx == 2)
                        Display::set_verify_mpu2_status(PASS);
                    ctx.complete = true;
                }
                else
                {
                    if(mpuIdx == 1)
                        Display::set_verify_mpu1_status(FAILED);
                    else if(mpuIdx == 2)
                        Display::set_verify_mpu2_status(FAILED);
                    ctx.complete = true;
                }
            }
            mpuIdx++;
        }

        if (radioCounter > 1000)
        {
            if (((float)radioValidCounter / radioCounter) > 0.9)
            {
                Display::set_verify_radio_status(PASS);
                radioComplete = true;
            }
            else if (((float)radioValidCounter / radioCounter) < 0.9)
            {
                Display::set_verify_radio_status(FAILED);
                radioComplete = true;
            }
        }

        bool allMpuComplete{true};
        for(auto& ctx : yprCtx){
            if(!ctx.complete)
                allMpuComplete = false;
        }

        if (allMpuComplete && radioComplete)
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

uint16_t Drone::mapValue(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
    if (x == 992)
    {
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int Drone::mapValue(int x, int in_min, int in_max, int out_min, int out_max)
{
    if (x == 992)
    {
        return 0;
    }
    return roundf((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

float Drone::mapValue(int x, int in_min, int in_max, float out_min, float out_max)
{
    if (x == 992)
    {
        return 0;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Drone::mapValueInverted(int x, int in_min, int in_max, float out_min, float out_max)
{
    if (x == 992)
    {
        return 0;
    }

    return (in_max - x) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Drone::mapValue2(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min, float out_max)
{
    if (x == 992)
    {
        return 0;
    }
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

esp_err_t Drone::signal_telemetry_request(Dshot::DshotMessage &msg, bool &newTelemetryReq)
{

    esp_err_t status = 0;
    uint8_t constexpr intervalMs{5};
    constexpr uint8_t modNr = Dshot::maxChannels;
    static uint64_t motorCounter{};

    static auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (elapsed >= intervalMs)
    {
        uint8_t idx = motorCounter % modNr;
        msg.telemetryReq[idx] = true;
        m_telemetryReqIdx.store(idx);
        start = std::chrono::system_clock::now();
        motorCounter++;
        newTelemetryReq = true;
    }

    return status;
}

void Drone::drone_task(void *args){
    using namespace std::chrono_literals;
    using namespace std::chrono;

    esp_err_t status = 0;
    uint64_t yprCounter = 0;
    uint64_t radioCounter = 0;
    volatile uint64_t loopCounter = 0;
    bool pid_config_running{};
    auto start = std::chrono::system_clock::now();
    auto ok_radio_timer = std::chrono::steady_clock::now();
#ifdef TELEMETRY_TASK
    auto telemetry_period_ms = 200ms;
    auto next_log_time = std::chrono::steady_clock::now() + telemetry_period_ms;
#endif

    m_dshotObj->set_extended_telemetry(true);
    vTaskDelay(pdMS_TO_TICKS(50));
    status = calibrate_gyro();
    verify_components_process();
    Display::set_display_state(DRONE);

    while (true)
    {
        xSemaphoreTake(timer_sem, portMAX_DELAY);
        Dshot::DshotMessage msg{};
        Channel channelNew{};
        RadioStatistics radioStatisticsNew{};

        bool newTelemetryReq{};
        bool newSpeedValues{};
        bool radio_ok{true};

        m_newImuData = false;

        if(m_mpuObj1.size() > 1){
            YawPitchRoll tmp_ypr{};
            YawPitchRoll ypr{};
            uint8_t nrOkSamples{};

            for(const auto& m : m_mpuObj1){
                status = get_imu_data(1, ypr, 0);

                if (status == ESP_OK){
                    tmp_ypr.yaw   += ypr.yaw;
                    tmp_ypr.pitch += ypr.pitch;
                    tmp_ypr.roll  += ypr.roll;

                    nrOkSamples++;
                    yprCounter++;
                    m_newImuData = true;
                }
            }

            tmp_ypr.yaw   /= nrOkSamples;
            tmp_ypr.pitch /= nrOkSamples;
            tmp_ypr.roll  /= nrOkSamples;

            m_ypr1 = tmp_ypr;
        }
        else{
            status = get_imu_data(1, m_ypr1, 0);
            if (status == ESP_OK){
                yprCounter++;
                m_newImuData = true;
            }
        }

        if(m_newImuData){
            m_state.currPitch = m_ypr1.pitch * RAD_TO_DEGREE;
            m_state.currYaw   = m_ypr1.yaw   * RAD_TO_DEGREE;
            m_state.currYaw   = m_ypr1.yaw   * RAD_TO_DEGREE;
            m_state.currRoll  = m_ypr1.roll  * RAD_TO_DEGREE;

            // printf("pitch[%f] yaw[%f] roll[%f] x_gyro[%4d], y_gyro[%4d], z_gyro[%4d]\n",
            // m_state.currPitchRateDegSec,
            // m_state.currRollhRateDegSec,
            // m_state.currYawhRateDegSec,
            // m_ypr1.x_gyro,
            // m_ypr1.y_gyro,
            // m_ypr1.z_gyro);

            m_state.currPitchRateDegSec = m_ypr1.pitchRateDegSec;
            m_state.currRollRateDegSec = m_ypr1.rollRateDegSec;
            m_state.currYawRateDegSec = m_ypr1.yawRateDegSec;
        }

        /*********  Radio processing **********/
        status = get_radio_data(channelNew, 0);
        if (status == ESP_OK){
            parse_channel_state(channelNew);
            this->channel = channelNew;
            radioCounter++;
            ok_radio_timer = std::chrono::steady_clock::now();
        }

        status = get_radio_statistics(radioStatisticsNew, 0);
        if (status == ESP_OK){
            this->radio_statistics = radioStatisticsNew;
        }

        /* if more than 250ms since last radio msg, send 0 throttle */
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_radio_msg = std::chrono::duration_cast<std::chrono::milliseconds>(now - ok_radio_timer).count();
        if (time_since_last_radio_msg >= 250){
            radio_ok = false;
        }

        /********* Launch PID config mode **********/
        if (!m_state.isControllerArmed && configure_pid()){
            printf("PID config active!\n");

            if (!pid_config_running){
                pid_config_running = true;
                TaskHandle_t pid_configure_handle{};
                xTaskCreatePinnedToCore(Drone::pid_configure_task, "pid_config_task", 4048, nullptr, 3, &pid_configure_handle, 0);
            }
            else{
                BaseType_t status = xSemaphoreTake(m_pid_conf_sem, pdMS_TO_TICKS(10));
                if (status == pdTRUE)
                    pid_config_running = false;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* send dshot telemetry command */
        ESP_ERROR_CHECK_WITHOUT_ABORT(signal_telemetry_request(msg, newTelemetryReq));
        startTel = std::chrono::steady_clock::now();

        if (m_state.isControllerArmed && radio_ok && !pid_config_running){
            m_state.isDroneArmed = true;
        }
        else{
            m_state.isDroneArmed = false;
        }

        get_speed(msg);
        write_speed(msg);

#ifdef TELEMETRY_TASK
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        /* calculate and set the frequenzy of received data */
        if (elapsed >= 10000)
        {
            m_state.dmpFreq = yprCounter / 10;
            m_state.radioFreq = radioCounter / 10;
            m_state.loopFreq = loopCounter / 10;
            yprCounter = 0;
            radioCounter = 0;
            loopCounter = 0;
            start = std::chrono::system_clock::now();
        }

        now = steady_clock::now();
        if (now >= next_log_time)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(measure_current());
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_telemetry(false));
            RadioController::send_attitude(m_ypr1);
            RadioController::send_battery_data(m_state.escState->voltage, m_state.escState->current, 100.0);
            next_log_time = now + telemetry_period_ms;
        }
        else{
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_telemetry(true));
        }
#endif
        loopCounter++;
    }
}

void Drone::write_speed(Dshot::DshotMessage &msg)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_dshot_message(msg));
}

void Drone::get_speed(Dshot::DshotMessage &msg)
{
    bool zeroThrottle{};

    // send 0 throttle while not armed
    if (!m_state.isDroneArmed){
        msg.msgType = Dshot::COMMAND;
        zeroThrottle = true;
    }
    else if (channel.ch3 >= m_deadzone && m_newImuData){
        get_new_speed(msg);
        m_prevSpeedMsg = msg;
    }
    else if(channel.ch3 >= m_deadzone && !m_newImuData){
        msg.msgType = Dshot::THROTTLE;
        // avoid assigning telemetry requests
        for(int i=0;i<Dshot::maxChannels;i++)
        {
            msg.speed[i] = m_prevSpeedMsg.speed[i];
        }
    }
    else{
        /* send 0 if thottle close to bottom */
        zeroThrottle = true;
    }

    if(zeroThrottle){
        auto reset_pid = [](Pid& p){ p.integral = 0; p.prevErr = 0; p.c = 0; };

        reset_pid(m_pid[PITCH]);
        reset_pid(m_pid[ROLL]);
        reset_pid(m_pid[YAW]);

        m_state.motorRlSpeed = 0;
        m_state.motorRrSpeed = 0;
        m_state.motorFlSpeed = 0;
        m_state.motorFrSpeed = 0;

        for (int i = 0; i < Dshot::maxChannels; i++){
            msg.speed[i] = 0;
            msg.loops[i] = 0;
        }
    }
}

void Drone::esc_telemetry_task(void *args)
{

    uart_event_t event;
    uint8_t data[512]{};
    size_t dataLength{};
    constexpr uint8_t chunkSize{10};
    constexpr uint8_t crcIdx{9};

    vTaskDelay(pdMS_TO_TICKS(50000));

    // wait until armed for telemetry data
    while (m_state.isControllerArmed == false)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    while (true)
    {

        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
            if (event.type == UART_DATA)
            {
                memset(data, 0, 512);
                dataLength = 0; // should be 10 bytes

                ESP_ERROR_CHECK(uart_get_buffered_data_len(this->uartNum, &dataLength));
                dataLength = uart_read_bytes(this->uartNum, data, dataLength, 100);

                if (!dataLength)
                {
                    vTaskDelay(pdMS_TO_TICKS(2));
                    continue;
                }
            }
        }

        for (int i = 0; i < dataLength; i += chunkSize)
        {

            uint16_t chksum = calculateCrc8(data, 9);

            if (data[crcIdx] == chksum)
            {

                endTel = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endTel - startTel).count();

                if (elapsed <= 20000)
                { // only collect data if sample was within the time window or it might belong to another motor than expected
                    // printf("telemetry elapsed time: %lld us\n", elapsed);
                    uint8_t temperature = data[i];
                    uint16_t voltage = (data[i + 1] << 8 | data[i + 2]);     // scaling factor
                    uint16_t current = (data[i + 3] << 8 | data[i + 4]);     // maybe needs scaling factor
                    uint16_t consumption = (data[i + 5] << 8 | data[i + 6]); // idk?
                    uint16_t rpm = (data[i + 7] << 8 | data[i + 8]);         // pole pairs of engine
                    uint8_t idx = m_telemetryReqIdx.load();
                    set_esc_telemetry_data(temperature, voltage, current, consumption, rpm, idx);
                }
                else
                {
                    print_debug(DEBUG_DRONE, DEBUG_DATA, "failed to recieve amp reading in time\n");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void Drone::set_esc_telemetry_data(uint8_t temperature, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t rpm, uint8_t motorPos)
{

    m_escTelemetryLock.lock();

    m_state.escState[motorPos].temperature = temperature;
    m_state.escState[motorPos].voltage = voltage;
    m_state.escState[motorPos].current = current;
    m_state.escState[motorPos].consumption = consumption;
    m_state.escState[motorPos].rpm = rpm;

    m_escTelemetryLock.unlock();
}

esp_err_t Drone::init_esc_telemetry_uart(int rxPin, int txPin, int baudrate)
{

    esp_err_t status = 0;

    constexpr int rtsPin = UART_PIN_NO_CHANGE;
    constexpr int ctsPin = UART_PIN_NO_CHANGE;
    constexpr int uart_buffer_size = (512);
    uart_config_t uart_config{};

    uart_config.baud_rate = baudrate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;

    this->uartNum = UART_NUM_0;

    ESP_ERROR_CHECK(uart_param_config(this->uartNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(this->uartNum, UART_PIN_NO_CHANGE, rxPin, rtsPin, ctsPin));
    ESP_ERROR_CHECK(uart_driver_install(this->uartNum, uart_buffer_size, uart_buffer_size, 10, &this->uart_queue, 0));

    return status;
}

void Drone::get_new_speed(Dshot::DshotMessage &msg)
{
    const uint16_t maxThrottleValue = c_saftyParams ? 1000u : Dshot::maxThrottleValue;
    msg.msgType = Dshot::THROTTLE;

    // Single PID update that includes I-term clamp + output clamp
    update_pids(0.20f); // max 20% authority

    int _rlSpeed = m_state.throttle + m_pid[PITCH].c + m_pid[ROLL].c + m_pid[YAW].c;
    int _rrSpeed = m_state.throttle + m_pid[PITCH].c - m_pid[ROLL].c - m_pid[YAW].c;
    int _flSpeed = m_state.throttle - m_pid[PITCH].c + m_pid[ROLL].c - m_pid[YAW].c;
    int _frSpeed = m_state.throttle - m_pid[PITCH].c - m_pid[ROLL].c + m_pid[YAW].c;

    // avoid underflow
    uint16_t rlSpeed = _rlSpeed >= 0 ? static_cast<uint16_t>(_rlSpeed) : 0u;
    uint16_t rrSpeed = _rrSpeed >= 0 ? static_cast<uint16_t>(_rrSpeed) : 0u;
    uint16_t flSpeed = _flSpeed >= 0 ? static_cast<uint16_t>(_flSpeed) : 0u;
    uint16_t frSpeed = _frSpeed >= 0 ? static_cast<uint16_t>(_frSpeed) : 0u;

    auto minMax = [](uint16_t &value, uint16_t min, uint16_t max)
    {if(value < min) value = min; if(value > max) value = max; };

    minMax(rlSpeed, c_minThrottleValue, maxThrottleValue);
    minMax(rrSpeed, c_minThrottleValue, maxThrottleValue);
    minMax(flSpeed, c_minThrottleValue, maxThrottleValue);
    minMax(frSpeed, c_minThrottleValue, maxThrottleValue);

    msg.speed[m_motorLaneMapping.rearLeftlane] = rlSpeed;
    msg.loops[m_motorLaneMapping.rearLeftlane] = 0;
    m_state.motorRlSpeed = rlSpeed;

    msg.speed[m_motorLaneMapping.rearRightlane] = rrSpeed;
    msg.loops[m_motorLaneMapping.rearRightlane] = 0;
    m_state.motorRrSpeed = rrSpeed;

    msg.speed[m_motorLaneMapping.frontLeftlane] = flSpeed;
    msg.loops[m_motorLaneMapping.frontLeftlane] = 0;
    m_state.motorFlSpeed = flSpeed;

    msg.speed[m_motorLaneMapping.frontRightlane] = frSpeed;
    msg.loops[m_motorLaneMapping.frontRightlane] = 0;
    m_state.motorFrSpeed = frSpeed;
}

#include "esp_adc_cal.h"

#define DEFAULT_VREF 1100   // in mV
#define ESC_MV_PER_AMP 35.0 // ESC outputs 50mV per Amp

// https://micoair.com/esc/ 12.75 mV per amp

static esp_adc_cal_characteristics_t adc_chars;
esp_err_t Drone::measure_current()
{
    static bool init = false;
    if (!init)
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << 36);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0); // GPIO36

        esp_adc_cal_characterize(
            ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);

        init = true;
    }

    static uint32_t v_offset_mv = 150;        // set during calibration
    static float k_mv_per_A = ESC_MV_PER_AMP; // mV per amp, set during calibration

    uint64_t acc = 0;
    const int N = 10;
    for (int i = 0; i < N; ++i){
        acc += adc1_get_raw(ADC1_CHANNEL_0); // GPIO36 (wired from 17)
        //ets_delay_us(1);
    }

    uint32_t avg_raw = acc / N;
    uint32_t mv = esp_adc_cal_raw_to_voltage(avg_raw, &adc_chars);

    //static uint64_t cntr{};
    //if(cntr % 10 == 0){
    //    printf("mv[%lu]\n", mv);
    //}
    //cntr++;

    int32_t mv_net = (int32_t)mv - (int32_t)v_offset_mv;
    if (mv_net < 0)
        mv_net = 0;

    float current_A = (k_mv_per_A > 1e-3f) ? (mv_net / k_mv_per_A) : 0.0f;
    m_state.currentDraw = current_A;

    return ESP_OK;
}

esp_err_t Drone::send_telemetry(bool isBlackbox){
    TelemetryData telemetry{};
    telemetry.ypr1 = this->m_ypr1;
    //telemetry.ypr2 = this->m_ypr2;
    telemetry.channel = this->channel;
    telemetry.radioStatistics = radio_statistics;

    telemetry.drone.isControllerArmed = m_state.isControllerArmed;
    telemetry.drone.isDroneArmed = m_state.isDroneArmed;
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

    telemetry.drone.targetPitchRateDegSec = m_state.targetPitchRateDegSec;
    telemetry.drone.targetRollRateDegSec = m_state.targetRollRateDegSec;
    telemetry.drone.targetYawRateDegSec = m_state.targetYawRateDegSec;
    telemetry.drone.currPitchRateDegSec = m_state.currPitchRateDegSec;
    telemetry.drone.currRollRateDegSec = m_state.currRollRateDegSec;
    telemetry.drone.currYawRateDegSec = m_state.currYawRateDegSec;

    telemetry.drone.currentDraw = m_state.currentDraw;

    m_escTelemetryLock.lock();

    m_bidiTelemetry = m_dshotObj->get_bidi_telemetry();

    for (int i = 0; i < Dshot::maxChannels; i++)
    {
        telemetry.drone.escState[i].temperature = m_state.escState[i].temperature;
        telemetry.drone.escState[i].voltage = m_state.escState[i].voltage;
        telemetry.drone.escState[i].current = m_state.escState[i].current;
        telemetry.drone.escState[i].consumption = m_state.escState[i].consumption;
        telemetry.drone.escState[i].rpm = m_bidiTelemetry.rpm[i];
    }

    for(int i=0;i<3;i++){
        telemetry.drone.pid[i] = m_pid[i];
    }

    m_escTelemetryLock.unlock();

    if(m_state.isDroneArmed && isBlackbox && false)
        return send_blackbox_telemetry_message(telemetry);
    else 
        return send_telemetry_message(telemetry);

}

esp_err_t Drone::parse_channel_state(const Channel &newChannel)
{
    esp_err_t status = 0;

    if (this->channel.ch1 != newChannel.ch1)
    {
        if (m_state.mode == ACRO_MODE)
            m_state.targetRoll = mapValue(newChannel.ch1, Radio::minChannelValue, Radio::maxChannelValue, m_minRollRate, m_maxRollRate);
        else 
            m_state.targetRoll = mapValue(newChannel.ch1, Radio::minChannelValue, Radio::maxChannelValue, m_minRollDeg, m_maxRollDeg);
    }
    if (this->channel.ch2 != newChannel.ch2)
    {
        if (m_state.mode == ACRO_MODE)
            m_state.targetPitch = mapValue(newChannel.ch2, Radio::minChannelValue, Radio::maxChannelValue, m_minPitchRate, m_maxPitchRate);
        else
            m_state.targetPitch = mapValue(newChannel.ch2, Radio::minChannelValue, Radio::maxChannelValue, m_minPitchDeg, m_maxPitchDeg);
    }
    if (this->channel.ch3 != newChannel.ch3)
    {
        // controller sends ~300 min value
        m_state.throttle = mapValue(static_cast<uint16_t>(newChannel.ch3), 300u /*static_cast<uint16_t>(Radio::minChannelValue) */, static_cast<uint16_t>(Radio::maxChannelValue), c_minThrottleValue, Dshot::maxThrottleValue);
    }
    if (this->channel.ch4 != newChannel.ch4)
    {
        m_state.targetYaw = mapValueInverted(newChannel.ch4, Radio::minChannelValue, Radio::maxChannelValue, m_minYawRate, m_maxYawRate);
    }
    if (this->channel.ch5 != newChannel.ch5)
    {
        status = arm_drone(newChannel.ch5);
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
    }
    if (this->channel.ch9 != newChannel.ch9)
    {
        // status = set_mpu_calibration_mode(newChannel.ch7);
        // ESP_RETURN_ON_ERROR(status, log_tag, "Failed to set mpu calibration mode");
    }
    if (this->channel.ch10 != newChannel.ch10)
    {
    }

    this->channel = newChannel;

    return status;
}

esp_err_t Drone::send_dshot_message(Dshot::DshotMessage &msg)
{
    print_debug(DEBUG_DRONE, DEBUG_ARGS, "send_dshot_message:");
    for (int i = 0; i < Dshot::maxChannels; i++)
    {
        print_debug(DEBUG_DRONE, DEBUG_ARGS, " m%d type: %s cmd: %u speed: %u teleReq: %d ::\n",
                    i,
                    msg.msgType == Dshot::COMMAND ? "CMD" : "THROTTLE",
                    msg.cmd,
                    msg.speed,
                    msg.telemetryReq);
    }
    print_debug(DEBUG_DRONE, DEBUG_ARGS, "\n");

    esp_err_t status = m_dshotObj->set_speed(msg);

    return status;
}

esp_err_t Drone::send_telemetry_message(TelemetryData &msg)
{

    return CircularBufEnqueue(m_telemetry_queue_handle, &msg);
}

esp_err_t Drone::send_blackbox_telemetry_message(TelemetryData &msg)
{
    return CircularBufEnqueue(m_blackbox_telemetry_queue_handle, &msg);
}

esp_err_t Drone::arm_drone(uint32_t value)
{

    if (value <= Radio::minChannelThld)
    {
        m_state.isControllerArmed = false;
        Display::set_armed_status(false);
        return ESP_OK;
    }

    esp_err_t status = 0;

    m_state.isControllerArmed = true;

    Display::set_armed_status(true);

    return ESP_OK;
}

esp_err_t Drone::send_beep()
{
    esp_err_t status = 0;

    Dshot::DshotMessage msg{};
    // msg.writeTo[MOTOR1] = true;
    msg.cmd[MOTOR1] = Dshot::BeepNum::BEEP1;
    msg.loops[MOTOR1] = 0;
    status = send_dshot_message(msg);

    return ESP_OK;
}

esp_err_t Drone::toggle_motor_direction()
{
    esp_err_t status = 0;

    static bool toggle = false;

    Dshot::DshotMessage msg{};
    // msg.writeTo[MOTOR1] = true;
    msg.cmd[MOTOR1] = toggle ? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_2;
    msg.loops[MOTOR1] = 0;
    status = send_dshot_message(msg);

    toggle = !toggle;

    return ESP_OK;
}

esp_err_t Drone::blink_led(uint16_t newChannelValue)
{
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

esp_err_t Drone::start_arm_process()
{
    esp_err_t status = 0;
    return status;
}

esp_err_t Drone::start_dissarm_process()
{
    esp_err_t status = 0;
    return status;
}

esp_err_t Drone::set_flight_mode(int value)
{
    esp_err_t status = 0;

    if (m_state.isDroneArmed)
    {
        return status;
    }
    else if (value > (Radio::neutralChannelValue - 100) && value < (Radio::neutralChannelValue + 100))
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
    return ESP_FAIL;
}

esp_err_t Drone::get_motor_direction(enum Motor motorNum, enum MotorDirection &direction)
{
    return ESP_FAIL;
}

void Drone::clamp_pid_output_pct(Pid& pid, float baseThrottle, float maxPct)
{
    maxPct = std::clamp(maxPct, 0.0f, 1.0f);

    // If baseThrottle is tiny, fall back to "no authority"
    const float base = std::max(std::fabsf(baseThrottle), 1.0f);
    const float maxDelta = base * maxPct;

    pid.c = std::clamp(pid.c, -maxDelta, +maxDelta);
}

void Drone::clamp_pid_integral_pct(Pid& pid, float baseThrottle, float maxPct)
{
    maxPct = std::clamp(maxPct, 0.0f, 1.0f);
    const float base = std::max(std::fabsf(baseThrottle), 1.0f);
    const float maxDelta = base * maxPct;

    const float kI = fabsf(pid.kI);
    if (kI > 1e-9f){
        const float iMax = maxDelta / kI;
        pid.integral = std::clamp(pid.integral, -iMax, +iMax);
    }
    else
        pid.integral = 0;
}

void Drone::pid(float target, float current, Pid &pid)
{
    float kPOut{};
    float kIOut{};
    float kDOut{};
    float err{};
    float derivative{};

    err = target - current;

    kPOut = pid.kP * err;
    pid.integral += err * pid.dt;
    kIOut = pid.kI * pid.integral;

    pid.integral = 0; // FIXME remove this, only for debugging

    derivative = (err - pid.prevErr) / pid.dt;
    kDOut = pid.kD * derivative;

    pid.prevErr = err;
    pid.c = kPOut + kIOut + kDOut;
}

void Drone::pidRate(float target, float current, Pid &pid)
{
    float kPOut{};
    float kIOut{};
    float kDOut{};
    float err{};
    float derivative{};

    err = target - current;

    kPOut = pid.kP * err;
    pid.integral += err * pid.dt;
    kIOut = pid.kI * pid.integral;

    derivative = (err - pid.prevErr) / pid.dt;
    kDOut = pid.kD * derivative;

    pid.prevErr = err;
    pid.c = kPOut + kIOut + kDOut;
}

void Drone::pid_step(float target, float current, Pid &p, float baseThrottle, float maxPct)
{
    // Authority relative to throttle (like you intended)
    maxPct = std::clamp(maxPct, 0.0f, 1.0f);
    const float base = std::max(std::fabsf(baseThrottle), 1.0f);
    const float maxDelta = base * maxPct;
    const float err = target - current;

    const float pTerm = p.kP * err;

    // I (integrate, then clamp integral so I output can’t exceed maxDelta)
    p.integral += err * p.dt;

    if (std::fabsf(p.kI) > 1e-9f){
        const float iMax = maxDelta / std::fabsf(p.kI);
        p.integral = std::clamp(p.integral, -iMax, +iMax);
    }
    else{
        p.integral = 0.0f;
    }
    const float iTerm = p.kI * p.integral;

    const float derivative = (err - p.prevErr) / p.dt;
    const float dTerm = p.kD * derivative;
    p.prevErr = err;

    float out = pTerm + iTerm + dTerm;
    out = std::clamp(out, -maxDelta, +maxDelta);

    p.c = out;
}

void Drone::update_pids(float maxPct)
{
    const float thr = (float)m_state.throttle;

    if (m_state.mode == SELFLEVL_MODE)
    {
        // pid_step(m_state.targetRoll,  m_state.currRoll,  m_pid[ROLL],  thr, maxPct);
        // pid_step(m_state.targetPitch, m_state.currPitch, m_pid[PITCH], thr, maxPct);
        // pid_step(m_state.targetYaw,   m_state.currYaw,   m_pid[YAW],   thr, maxPct);

        pid_step(m_state.targetPitch, m_state.currPitch, m_pid[PITCH], thr, maxPct);
        pid_step(m_state.targetRoll, m_state.currRoll, m_pid[ROLL], thr, maxPct);
        pid_step(m_state.targetYaw, m_state.currYaw, m_pid[YAW], thr, maxPct);
    }
    else if (m_state.mode == ACRO_MODE)
    {
        pid_step(m_state.targetPitch, m_state.currPitchRateDegSec, m_pid[PITCH], thr, maxPct);
        pid_step(m_state.targetRoll, m_state.currRollRateDegSec, m_pid[ROLL], thr, maxPct);
        pid_step(m_state.targetYaw, m_state.currYawRateDegSec, m_pid[YAW], thr, maxPct);
    }
}

uint8_t Drone::updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i = 0; i < 8; i++)
    {
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    }

    return (crc_u);
}

uint8_t Drone::calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++)
    {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
}

esp_err_t Drone::calibrate_gyro()
{
    esp_err_t status = 0;
    uint32_t counter{};
    bool startCalibration{};

    auto start = std::chrono::steady_clock::now();

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (true)
    {
        Channel channel{};
        status = 0;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();

        auto start2 = std::chrono::steady_clock::now();
        status = get_radio_data(channel, pdMS_TO_TICKS(1000));
        auto end2 = std::chrono::steady_clock::now();
        auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count();

        if (status == ESP_OK && channel.ch9 >= Radio::maxChannelThld)
        {
            counter++;
        }

        if (elapsed >= 2000)
        {
            break;
        }
        else if (counter >= 100)
        {
            startCalibration = true;
        }
    }

    if (startCalibration)
    {
        Display::set_calibration_status(STARTED);
        printf("calibrating...\n");

        for(const auto& m : m_mpuObj1){
            m->calibrate_mpu();
        }

        // takes rougly 2.7s
        vTaskDelay(pdMS_TO_TICKS(3500));
        Display::set_calibration_status(PASS);
    }
    else
        printf("NO CALIBRATION counter[%lu]\n", counter);

    return status;
}

esp_err_t Drone::set_mpu_calibration_mode(int value)
{

    esp_err_t status = 0;

    if (value > Radio::maxChannelValue && value < Radio::minChannelValue)
    {
        return ESP_FAIL;
    }

    m_state.calibrateGyro = value > 0 ? true : false;

    return ESP_OK;
}

bool Drone::configure_pid()
{
    static bool onging_check{};
    static uint32_t counter{};
    static std::chrono::_V2::steady_clock::time_point configBtnPressed{};

    if (channel.ch9 >= Radio::maxChannelThld && !onging_check)
    {
        configBtnPressed = std::chrono::steady_clock::now();
        onging_check = true;
    }

    if (channel.ch9 >= Radio::maxChannelThld && onging_check)
    {
        counter++;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - configBtnPressed).count();
    if (counter >= 250 && elapsed >= 500)
    {
        onging_check = false;
        counter = 0;
        return true;
    }
    return false;
}

void Drone::pid_configure_task(void *args)
{

    enum PidConfigDisplay prev_selected_row
    {
    };
    enum PidConfigDisplay selected_row
    {
    };

    Display::set_display_state(PID_CONFIG);

    Pid temp_pid = Drone::m_pid[0];
    bool apply{};
    bool cancel{};
    bool exit_config{};

    uint32_t speed_counter{};

    while (true)
    {
        bool increase{};
        bool decrease{};

        if (Drone::channel.ch2 > Radio::maxChannelThld)
        { // down
            if (selected_row == 0)
                selected_row = CANCEL_PID;
            else
                selected_row = static_cast<enum PidConfigDisplay>((selected_row - 1) % 5);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (Drone::channel.ch2 < Radio::minChannelThld)
        { // up
            selected_row = static_cast<enum PidConfigDisplay>((selected_row + 1) % 5);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (Drone::channel.ch1 < Radio::minChannelThld)
        { // left
            increase = true;
            if (speed_counter >= 10)
            {
                vTaskDelay(pdMS_TO_TICKS(70));
            }
            else
                vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (Drone::channel.ch1 > Radio::maxChannelThld)
        { // right
            decrease = true;
            if (speed_counter >= 10)
            {
                vTaskDelay(pdMS_TO_TICKS(70));
            }
            else
                vTaskDelay(pdMS_TO_TICKS(100));
        }
        else if (Drone::channel.ch9 > Radio::maxChannelThld)
        { // confirm
            if (selected_row == APPLY)
            {
                for (int i = 0; i < 3; i++)
                {
                    m_pid[i].kP = temp_pid.kP;
                    m_pid[i].kI = temp_pid.kI;
                    m_pid[i].kD = temp_pid.kD;
                }
                exit_config = true;
            }
            else if (selected_row == CANCEL_PID)
            {
                exit_config = true;
            }
        }

        switch (selected_row)
        {
        case P:
            if (increase)
                temp_pid.kP += 0.01;
            else if (decrease)
                temp_pid.kP -= 0.01;
            break;
        case I:
            if (increase)
                temp_pid.kI += 0.01;
            else if (decrease)
                temp_pid.kI -= 0.01;
            break;
        case D:
            if (increase)
                temp_pid.kD += 0.01;
            else if (decrease)
                temp_pid.kD -= 0.01;
            break;
        case APPLY:
            break;
        case CANCEL_PID:
            break;
        default:
            break;
        }
        if (selected_row != prev_selected_row || increase || decrease)
        {
            printf("%d\n", selected_row);
            Display::set_pid_config_data(temp_pid, selected_row);
        }

        if (increase || decrease)
        {
            if (speed_counter >= 10)
                speed_counter += 4;
            else
                speed_counter++;
        }
        else
            speed_counter = 0;

        prev_selected_row = selected_row;

        if (exit_config)
        {
            printf("deleteing task");
            if (selected_row == APPLY)
                Display::set_pid_config_data(temp_pid, PID_CONFIG_APPLY);
            else
                Display::set_pid_config_data(temp_pid, PID_CONFIG_CANCEL);

            vTaskDelay(pdMS_TO_TICKS(700));

            Display::set_display_state(DRONE);
            xSemaphoreGive(m_pid_conf_sem);
            vTaskDelete(nullptr);
            printf("should not see this!\n");
        }

        if (speed_counter <= 5)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void Drone::set_fw_version(uint8_t major, uint8_t minor, uint8_t patch){

    m_fwVersion.major = major;
    m_fwVersion.minor = minor;
    m_fwVersion.patch = patch;
}

void Drone::get_fw_version(uint8_t& major, uint8_t& minor, uint8_t& patch){
    major = m_fwVersion.major;
    minor = m_fwVersion.minor;
    patch = m_fwVersion.patch;
}

void Drone::get_channel_values(Channel& ch)
{
    ch = channel;
}

void Drone::set_battery_status(uint8_t cellCount, uint16_t capacityMah){
    m_batInfo.cellCount = cellCount;
    m_batInfo.capacityMah = capacityMah;
}

void Drone::get_battery_status(BatteryInfo &info){
    info = m_batInfo;
    info.voltage = m_state.escState[0].voltage;
    info.current = static_cast<int16_t>(m_state.currentDraw);
}

esp_err_t Drone::get_pid(Pid *pid, uint8_t size){

    if(size != 3){
        return ESP_ERR_INVALID_ARG;
    }

    for(int i=0;i<size;i++){
        pid[i] = m_pid[i];
    }

    return ESP_OK;
}

void Drone::get_radio_status(RadioStatistics& radio){
    radio = radio_statistics;
}

