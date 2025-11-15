
#include <chrono>
#include <cstring>

#include "dshot600.h"
#include "common_data.h"
#include "debug.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rom/ets_sys.h"
#include "freertos/queue.h"
#include "driver/gpio.h"



#define DSHOT_ESC_RESOLUTION_HZ 80000000 
static const char *TAG = "dshot_encoder";

using namespace Dshot;

Dshot600* Dshot600::dshot = nullptr;
bool Dshot600::m_isBidi{true};


// Could argue to remove 0 values and handle that seperately
constexpr TickType_t dshotWaitLookup[48] = {
    pdMS_TO_TICKS(0),    //DSHOT_CMD_MOTOR_STOP
    pdMS_TO_TICKS(260),  //DSHOT_CMD_BEEP1
    pdMS_TO_TICKS(260),  //DSHOT_CMD_BEEP2
    pdMS_TO_TICKS(260),  //DSHOT_CMD_BEEP3
    pdMS_TO_TICKS(260),  //DSHOT_CMD_BEEP4
    pdMS_TO_TICKS(260),  //DSHOT_CMD_BEEP5
    pdMS_TO_TICKS(12),   //DSHOT_CMD_ESC_INFO
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SPIN_DIRECTION_1
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SPIN_DIRECTION_2
    pdMS_TO_TICKS(0),    //DSHOT_CMD_3D_MODE_OFF
    pdMS_TO_TICKS(0),    //DSHOT_CMD_3D_MODE_ON
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SETTINGS_REQUEST
    pdMS_TO_TICKS(35),   //DSHOT_CMD_SAVE_SETTINGS
    pdMS_TO_TICKS(0),    //DSHOT_EXTENDED_TELEMETRY_ENABLE
    pdMS_TO_TICKS(0),    //DSHOT_EXTENDED_TELEMETRY_DISABLE
    pdMS_TO_TICKS(0),    //NOT_KNOWN1
    pdMS_TO_TICKS(0),    //NOT_KNOWN2
    pdMS_TO_TICKS(0),    //NOT_KNOWN3
    pdMS_TO_TICKS(0),    //NOT_KNOWN4
    pdMS_TO_TICKS(0),    //NOT_KNOWN5
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SPIN_DIRECTION_NORMAL
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SPIN_DIRECTION_REVERSED
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED0_ON
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED1_ON
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED2_ON
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED3_ON
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED0_OFF
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED1_OFF
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED2_OFF
    pdMS_TO_TICKS(0),    //DSHOT_CMD_LED3_OFF
    pdMS_TO_TICKS(0),    //AUDIO_STREAM_MODE
    pdMS_TO_TICKS(0),    //SILENT_MODE
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY
    pdMS_TO_TICKS(0),    //NOT_KNOWN6
    pdMS_TO_TICKS(0),    //NOT_KNOWN7
    pdMS_TO_TICKS(0),    //NOT_KNOWN8
    pdMS_TO_TICKS(0),    //NOT_KNOWN9
    pdMS_TO_TICKS(0),    //NOT_KNOWN10
    pdMS_TO_TICKS(0),    //NOT_KNOWN11
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY
    pdMS_TO_TICKS(0),    //DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY
    pdMS_TO_TICKS(0)     //DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY
};

constexpr uint16_t dshotLoopsLookup[48] = {
    1,      //DSHOT_CMD_MOTOR_STOP
    1,      //DSHOT_CMD_BEEP1
    1,      //DSHOT_CMD_BEEP2
    1,      //DSHOT_CMD_BEEP3
    1,      //DSHOT_CMD_BEEP4
    1,      //DSHOT_CMD_BEEP5
    1,      //DSHOT_CMD_ESC_INFO
    6,      //DSHOT_CMD_SPIN_DIRECTION_1
    6,      //DSHOT_CMD_SPIN_DIRECTION_2
    6,      //DSHOT_CMD_3D_MODE_OFF
    6,      //DSHOT_CMD_3D_MODE_ON
    1,      //DSHOT_CMD_SETTINGS_REQUEST
    6,      //DSHOT_CMD_SAVE_SETTINGS
    6,      //DSHOT_EXTENDED_TELEMETRY_ENABLE
    6,      //DSHOT_EXTENDED_TELEMETRY_DISABLE
    1,      //NOT_KNOWN1
    1,      //NOT_KNOWN2
    1,      //NOT_KNOWN3
    1,      //NOT_KNOWN4
    1,      //NOT_KNOWN5
    6,      //DSHOT_CMD_SPIN_DIRECTION_NORMAL
    6,      //DSHOT_CMD_SPIN_DIRECTION_REVERSED
    6,      //DSHOT_CMD_LED0_ON
    1,      //DSHOT_CMD_LED1_ON
    1,      //DSHOT_CMD_LED2_ON
    1,      //DSHOT_CMD_LED3_ON
    1,      //DSHOT_CMD_LED0_OFF
    1,      //DSHOT_CMD_LED1_OFF
    1,      //DSHOT_CMD_LED2_OFF
    1,      //DSHOT_CMD_LED3_OFF
    1,      //AUDIO_STREAM_MODE
    1,      //SILENT_MODE
    6,      //DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE
    6,      //DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE
    6,      //DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY
    6,      //DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY
    1,      //NOT_KNOWN6
    1,      //NOT_KNOWN7
    1,      //NOT_KNOWN8
    1,      //NOT_KNOWN9
    1,      //NOT_KNOWN10
    1,      //NOT_KNOWN11
    1,      //DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY
    1,      //DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY
    1,      //DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY
    1,      //DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY
    1,      //DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY
    1       //DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY
};



/*          rmt RX          */
static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data){
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}



/********* RX END ***********/




/*      My logic        */

Dshot600::Dshot600(gpio_num_t motorPin[Dshot::maxChannels], bool isBidi){
    rmt_tx_channel_config_t tx_chan_config{};
    rmt_rx_channel_config_t rx_chan_config{};

    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;               // select source clock
    tx_chan_config.mem_block_symbols = 128;                     // memory block size, 64 * 4 = 256Bytes
    tx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;     // 1MHz tick resolution, i.e. 1 tick = 1us
    tx_chan_config.trans_queue_depth = 2;                       // set the number of transactions that can pend in the background
   
    if(isBidi)
    {
        tx_chan_config.flags.invert_out = 1;
        //tx_chan_config.flags.io_od_mode = 1;    
    }

    //rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    //rx_chan_config.mem_block_symbols = 64;
    //rx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;

    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 600000, 
        .post_delay_us = 26, // extra delay between each frame
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_dshot_esc_encoder(&encoder_config, &this->dshot_encoder));

    for(int i=0;i<Dshot::maxChannels;i++){
        tx_chan_config.gpio_num = motorPin[i];
        rx_chan_config.gpio_num = motorPin[i];
        m_gpioMotorPin[i]= motorPin[i];

        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[i]));
        printf("motor chan %d: %p pin %d\n", i, this->esc_motor_chan[i], tx_chan_config.gpio_num);
        //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_rx_channel(&rx_chan_config, &this->rmt_rx_handle[i]));

    }

    for(int i=0;i<Dshot::maxChannels;i++){
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_pull_mode(m_gpioMotorPin[i], GPIO_PULLUP_ONLY));
        //gpio_pullup_en(m_gpioMotorPin[i]);
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
        //ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(m_gpioMotorPin[i], GPIO_MODE_OUTPUT_OD));
    }

    m_isBidi = isBidi;  
}

Dshot600* Dshot600::GetInstance(gpio_num_t motorPin[Dshot::maxChannels], bool isBidi){
    if(dshot==nullptr){
        dshot = new Dshot600(motorPin, isBidi);
    }
    return dshot;
}

Dshot600* Dshot600::GetInstance(){
    return dshot;
}

void Dshot600::dshot_task(void* args){
    esp_err_t status = 0;
    uint16_t prevSpeed[Dshot::maxChannels]{};
    TickType_t lastSleep = xTaskGetTickCount();
    TickType_t sleepThld = pdMS_TO_TICKS(100);
    TickType_t sleepDur = pdMS_TO_TICKS(1);
    TickType_t waitMs{portMAX_DELAY};
    bool firstMsg{true};

    while(true){

        DshotMessage msg{};

        if(firstMsg){
            waitMs = firstMsg? portMAX_DELAY : pdMS_TO_TICKS(50);
            firstMsg = false;

            status = get_message(msg, waitMs);
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);
        }
        else{
            status = get_message(msg, waitMs);
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);
        }

   
        if(status == ESP_OK){
            status = parse_dshot_message(msg);
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);

            if(status){
                print_debug(DEBUG_DSHOT, DEBUG_LOGIC, "no valid msg\n");
                continue;
            }

            if(msg.msgType == COMMAND){
                ESP_ERROR_CHECK_WITHOUT_ABORT(write_command(msg));
            }
            else if(msg.msgType == THROTTLE){
                ESP_ERROR_CHECK_WITHOUT_ABORT(set_speed(msg));
            }
        }
        else{
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - lastSleep) >= sleepThld) {
            //vTaskDelay(sleepDur);
            taskYIELD();
            lastSleep = now;
        }

    }
}

esp_err_t Dshot600::get_message(struct Dshot::DshotMessage& msg, TickType_t ticks){

    print_debug(DEBUG_DSHOT, DEBUG_ARGS, "%s %d: ticks %lu", __FILE__, __LINE__, ticks);


    //size_t itemSize = sizeof(Dshot::DshotMessage);
    //DshotMessage* data = (Dshot::DshotMessage*)xRingbufferReceive(this->m_dshot_queue_handle, &itemSize, ticks);
    //esp_err_t status = CircularBufDequeue(m_dshot_queue_handle, &msg, portMAX_DELAY);

    //if(data != nullptr){
    //    msg = *data;
    //    vRingbufferReturnItem(this->m_dshot_queue_handle, (void*)data);
//
    //}
    //else{
    //    return ESP_ERR_NOT_FOUND;
    //}

    //return status;
    return ESP_FAIL;
}

#include "include/soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"

esp_err_t Dshot600::write_speed(struct Dshot::DshotMessage& msg){

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};
    rmt_receive_config_t rx_config[Dshot::maxChannels]{};


    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.speed[i],
            .telemetry_req = msg.telemetryReq[i], 
        };

        tx_config[i] = {
            .loop_count = msg.loops[i] == -1? -1 : 0,
            //.flags{.eot_level = 1,},
        };    

        rx_config[i] = {
            .signal_range_min_ns = 	uint32_t(400),
            .signal_range_max_ns = uint32_t(1600),
        };
    }


    if(m_isBidi){
        for(int i=0;i<Dshot::maxChannels;i++){

                //rmt_enable(esc_motor_chan[i]);

                //gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);
                //gpio_set_direction(m_gpioMotorPin[i], GPIO_MODE_OUTPUT);
                gpio_pullup_dis(m_gpioMotorPin[i]);
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i])); 
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(2)));
                gpio_pullup_en(m_gpioMotorPin[i]);

                //gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);

                //rmt_disable(this->esc_motor_chan[i]);
                //
                //ets_delay_us(6);  
                //rmt_enable(rmt_rx_handle[i]);

                //gpio_set_direction(m_gpioMotorPin[i], GPIO_MODE_INPUT);  
                //uint8_t buffer[128]={0};
                //rmt_receive(rmt_rx_handle[i], buffer, 128, &rx_config[i]);
                //rmt_disable(rmt_rx_handle[i]);

        }
    }
    else{
        for(int i=0;i<Dshot::maxChannels;i++){
                print_debug(DEBUG_DSHOT, DEBUG_DATA, "m%d throttle: %u telemetry: %d channel: 0x%x", i, throttle[i].throttle, throttle[i].telemetry_req, this->esc_motor_chan[i]);
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i])); 
                //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(1)));
        }
        int64_t start[4]{};
        int64_t end[4]{};

        for(int i=0;i<Dshot::maxChannels;i++){
            //start[i] = esp_timer_get_time();
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(20)));
            //end[i] = esp_timer_get_time();
        }

        //printf("rmt_tx_wait_all_done took [%llu] [%llu] [%llu] [%llu] us\n", end[0]-start[0], end[1]-start[1], end[2]-start[2], end[3]-start[3]);

        //print_debug(DEBUG_DSHOT, DEBUG_DATA, "\n");
    }


    return status;
}

esp_err_t Dshot600::write_command(struct Dshot::DshotMessage& msg){

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};

    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.cmd[i],
            .telemetry_req = msg.telemetryReq[i], 
        };

        tx_config[i] = {
            .loop_count = msg.loops[i] == -1? -1 : 0,
        };    
    }
    
    for(int i=0;i<Dshot::maxChannels;i++){

        for(uint16_t j=0;j<dshotLoopsLookup[msg.cmd[i]];j++){
            print_debug(DEBUG_DSHOT, DEBUG_DATA, "m%d command: %u  channel: 0x%x ", i, throttle[i].throttle, this->esc_motor_chan[i]);
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));
            if(dshotWaitLookup[msg.cmd[i]] > 0) 
                vTaskDelay(dshotWaitLookup[msg.cmd[i]]);
        }

    }
    print_debug(DEBUG_DSHOT, DEBUG_DATA, "\n");

    return status;
}

esp_err_t Dshot600::set_idle(){

    esp_err_t status = 0;
    DshotMessage msg = {
        .msgType = COMMAND,
        .loops = {-1, -1, -1, -1},
        .cmd = {0, 0, 0, 0},
    };

    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}

/**
 * speed_mX should be between 0 - 2000
 * loops_mX should be larger than 0 if write_to_mX is true
 */
esp_err_t Dshot600::set_speed(struct Dshot::DshotMessage& msg){

    esp_err_t status = 0;

    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write_speed(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}

esp_err_t Dshot600::parse_dshot_message(struct Dshot::DshotMessage& msg) {

    print_debug(DEBUG_DSHOT, DEBUG_ARGS, "%s %d:", __FILE__, __LINE__);

    bool badMsg = false;

    // Validate motor speed and loop count
    auto validate_motor = [](uint16_t speed, uint8_t loops) {
        return (speed >= Dshot::minThrottleValue && speed <= Dshot::maxThrottleValue);
    };

    for(int i=0;i<Dshot::maxChannels;i++){
        if(msg.speed[i] != 0)
            if (!validate_motor(msg.speed[i], msg.loops[i])) {
                printf("bad msg: %d cmd %u speed %u\n", i, msg.cmd[i], msg.speed[i]);
                badMsg = true;
            }
    }

    for(int i=0;i<Dshot::maxChannels;i++){

        // Ensure telemetry request is only valid with throttle values (not ESC commands)
        if (msg.telemetryReq[i] && (msg.cmd[i] != 0)) {
            ESP_LOGE(TAG, "Invalid: Telemetry request must be used with throttle, not commands.");
            badMsg = true;
        }

        // Validate ESC command range (if used)
        if (msg.cmd[i] > 47) {
            ESP_LOGE(TAG, "Invalid: ESC command out of range.");
            badMsg = true;
        }
    }

    return badMsg ? ESP_FAIL : ESP_OK;
}

esp_err_t Dshot600::set_motor_spin_cmd(enum Motor motorNum, enum MotorDirection direction){

    esp_err_t status = 0;
    DshotMessage msg{};

    //msg.writeTo[motorNum] = true;
    msg.cmd[motorNum] = NORMAL? DSHOT_CMD_SPIN_DIRECTION_NORMAL : DSHOT_CMD_SPIN_DIRECTION_REVERSED;

    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}

esp_err_t Dshot600::beep_cmd(enum Dshot::BeepNum num){

    esp_err_t status = 0;
    DshotMessage msg{};
    
    msg.msgType = COMMAND;
    //msg.writeTo[0] = true;
    msg.cmd[0] = DSHOT_CMD_BEEP1 + num;
    msg.loops[0] = 1;


    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;

}

esp_err_t Dshot600::blink_led_cmd(){

    esp_err_t status = 0;
    DshotMessage msg{};
    
    msg.msgType = COMMAND;
    msg.cmd[0] = DSHOT_CMD_LED0_ON;
    msg.loops[0] = 1;


    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    vTaskDelay(pdMS_TO_TICKS(1000));

    msg.cmd[0] = DSHOT_CMD_LED0_OFF;
    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;

}

esp_err_t Dshot600::arm_esc_cmd(){

    esp_err_t status = 0;
    DshotMessage msg{};
    
    for(int i=0;i<1/*Dshot::maxChannels*/;i++){
        msg.msgType = COMMAND;
        msg.cmd[i] = DSHOT_CMD_MOTOR_STOP;
        msg.loops[i] = 0;

        status = parse_dshot_message(msg);
        ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");
    }

    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}


/*      RMT example functions       */

esp_err_t Dshot600::rmt_del_dshot_encoder(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_del_encoder(dshot_encoder->bytes_encoder);
    rmt_del_encoder(dshot_encoder->copy_encoder);
    free(dshot_encoder);
    return ESP_OK;
}

esp_err_t Dshot600::rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_encoder_reset(dshot_encoder->bytes_encoder);
    rmt_encoder_reset(dshot_encoder->copy_encoder);
    dshot_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

void Dshot600::make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
{
    frame->throttle = throttle;
    frame->telemetry = telemetry;
    uint16_t val = frame->val;
    uint8_t crc = ((val ^ (val >> 4) ^ (val >> 8)) & 0xF0) >> 4;
    frame->crc = crc;
    val = frame->val;
    // change the endian
    frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}

void Dshot600::make_bidi_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
{
    frame->throttle = throttle;
    frame->telemetry = telemetry;
    uint16_t val = frame->val;
    uint8_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0F;
    frame->crc = crc;
    val = frame->val;
    // change the endian
    frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}



size_t Dshot600::rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
    const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = dshot_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = dshot_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    // convert user data into dshot frame
    dshot_esc_throttle_t *throttle = (dshot_esc_throttle_t *)primary_data;
    dshot_esc_frame_t frame = {};
    if(m_isBidi){
        make_bidi_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);
    }
    else{
        make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);
    }

    switch (dshot_encoder->state) {
        case 0: // send the dshot frame
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &frame, sizeof(frame), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                dshot_encoder->state = 1; // switch to next state when current encoding session finished
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = static_cast<rmt_encode_state_t>(static_cast<int>(state) | static_cast<int>(RMT_ENCODING_MEM_FULL));           
                goto out; // yield if there's no free space for encoding artifacts
            }
        // fall-through
        case 1:
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &dshot_encoder->dshot_delay_symbol,
                            sizeof(rmt_symbol_word_t), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                state = static_cast<rmt_encode_state_t>(static_cast<int>(state) | static_cast<int>(RMT_ENCODING_COMPLETE));           
                dshot_encoder->state = RMT_ENCODING_RESET; // switch to next state when current encoding session finished
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = static_cast<rmt_encode_state_t>(static_cast<int>(state) | static_cast<int>(RMT_ENCODING_MEM_FULL));           
                goto out; // yield if there's no free space for encoding artifacts
            }
    }
    out:
    *ret_state = state;
    return encoded_symbols;
}


typedef struct tx_callback_datapack_s
{
    gpio_num_t gpio_num;

    rmt_channel_handle_t channel_handle; //rx_chan
    rmt_receive_config_t channel_config; //dshot_config.tx_callback_datapack
    rmt_symbol_word_t* raw_symbols; //where the gotten symbols should go
    size_t raw_sym_size; //size of the storage space for the raw symbols


} tx_callback_datapack_t;

static bool rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
	//init return value
	//high task wakeup will be true if pushing data to the queue started a task with a higher priority than this inturrupt.
    BaseType_t high_task_wakeup = pdFALSE;

	//get pointer to the settings passed in by us when we initialized the callback
	//rx_callback_datapack_t* config = (rx_callback_datapack_t*)user_data;

	//rx_frame_data_t out_data = {};

	//copy the edata symbols to the frame data
	//size_t sym_count = edata->num_symbols > RX_SYMBOL_MAX ? RX_SYMBOL_MAX : edata->num_symbols; //cap copy size to 11
	//memcpy(&out_data.received_symbols, edata->received_symbols, sym_count * sizeof(rmt_symbol_word_t));
	//out_data.num_symbols = sym_count;

	//TEST
	//Serial.printf("~%d\n", edata->num_symbols);
	//for(int i = 0; i < edata->num_symbols; ++i)
	//	Serial.printf("%d,%d|%d,%d\n",edata->received_symbols[i].duration0,edata->received_symbols[i].level0,
	//			edata->received_symbols[i].duration1,edata->received_symbols[i].level1);

	//we're getting a packet back whose last symbol does not end in the terminating value (which is a symbol with 0 length values)
	size_t last_sym = edata->num_symbols - 1;
	if(edata->received_symbols[last_sym].duration0 != 0
		&& edata->received_symbols[last_sym].duration1 != 0)
	{
		//this results in a guru meditation error, but we shouldn't be getting these anyway... I wonder what the packet looks like...
		//Serial.printf("~\n");
		//out_data.non_termination = true;
	}




    //send the received RMT symbols to the parser task
	//xQueueSendFromISR(config->receive_queue, &out_data, &high_task_wakeup);

    return high_task_wakeup == pdTRUE; //return xQueueSendFromISR result (does it wake up a higher task?)
}

static bool tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
	BaseType_t high_task_wakeup = pdFALSE;
	tx_callback_datapack_t* config = (tx_callback_datapack_t*)user_ctx;
	//size_t symbol_count = edata->num_symbols; //we don't need to know how many symbols we sent

	//restart rx channel
	rmt_enable(config->channel_handle);

	//enable open drain mode before listening for a response (no need for snap logic here, we only do callbacks in bidirectional mode)
	gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), config->gpio_num);

	//start listening for a response
	rmt_receive(config->channel_handle, config->raw_symbols, config->raw_sym_size, &config->channel_config);
	
	return high_task_wakeup; //nothing to wake up; no data needs to be sent back
}

/* configure timings for rmt data */
esp_err_t Dshot600::rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_dshot_esc_encoder_t *dshot_encoder = NULL;
    dshot_encoder = (rmt_dshot_esc_encoder_t*)calloc(1, sizeof(rmt_dshot_esc_encoder_t));

    dshot_encoder->base.encode = rmt_encode_dshot_esc;
    dshot_encoder->base.del = rmt_del_dshot_encoder;
    dshot_encoder->base.reset = rmt_dshot_encoder_reset;
    uint32_t delay_ticks = config->resolution / 1e6 * config->post_delay_us;
    printf("delay_ticks: %lu\n", delay_ticks); 

    	//rmt_tx_event_callbacks_t callback = 
		//{
		//	.on_trans_done = tx_done_callback
		//};

        //	rmt_rx_event_callbacks_t callback = 
		//{
		//	.on_recv_done = rx_done_callback
		//};
	

    rmt_symbol_word_t dshot_delay_symbol{};

    dshot_delay_symbol.level0 = 0;
    dshot_delay_symbol.duration0 = delay_ticks / 1;
    dshot_delay_symbol.level1 = 0;
    dshot_delay_symbol.duration1 = delay_ticks / 1;
    printf("duration0: %d\n", dshot_delay_symbol.duration0); 
    printf("duration1: %d\n", dshot_delay_symbol.duration1); 


    dshot_encoder->dshot_delay_symbol = dshot_delay_symbol;
    // different dshot protocol have its own timing requirements,
    float period_ticks = (float)config->resolution / config->baud_rate;
    printf("period_ticks: %f ns\n", (period_ticks / DSHOT_ESC_RESOLUTION_HZ) * 1e9);

    // 1 and 0 is represented by a 74.850% and 37.425% duty cycle respectively
    unsigned int t1h_ticks = (unsigned int)(period_ticks * 0.7485);
    unsigned int t1l_ticks = (unsigned int)(period_ticks - t1h_ticks);
    unsigned int t0h_ticks = (unsigned int)(period_ticks * 0.37425);
    unsigned int t0l_ticks = (unsigned int)(period_ticks - t0h_ticks);

                                          
    printf("t1h_ticks: %d (%.2f µs)\n", t1h_ticks, t1h_ticks * 25.0e-3);
    printf("t1l_ticks: %d (%.2f µs)\n", t1l_ticks, t1l_ticks * 25.0e-3);
    printf("t0h_ticks: %d (%.2f µs)\n", t0h_ticks, t0h_ticks * 25.0e-3);
    printf("t0l_ticks: %d (%.2f µs)\n", t0l_ticks, t0l_ticks * 25.0e-3);

    rmt_bytes_encoder_config_t bytes_encoder_config{};

        bytes_encoder_config.bit0.level0 = 1;
        bytes_encoder_config.bit0.duration0 = t0h_ticks;
        bytes_encoder_config.bit0.level1 = 0;
        bytes_encoder_config.bit0.duration1 = t0l_ticks;
        bytes_encoder_config.bit1.level0 = 1;
        bytes_encoder_config.bit1.duration0 = t1h_ticks;
        bytes_encoder_config.bit1.level1 = 0;
        bytes_encoder_config.bit1.duration1 = t1l_ticks;

    bytes_encoder_config.flags.msb_first = 1;


    rmt_new_bytes_encoder(&bytes_encoder_config, &dshot_encoder->bytes_encoder);
    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_new_copy_encoder(&copy_encoder_config, &dshot_encoder->copy_encoder);
    *ret_encoder = &dshot_encoder->base;

    return ret;
} 


