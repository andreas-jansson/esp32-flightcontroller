
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





#define DSHOT_ESC_RESOLUTION_HZ 80000000 
static const char *TAG = "dshot_encoder";

using namespace Dshot;

Dshot600* Dshot600::dshot = nullptr;


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


static bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    // return whether any task is woken up
    return high_task_wakeup == pdTRUE;
}



/********* RX END ***********/




/*      My logic        */

Dshot600::Dshot600(gpio_num_t motorPin[Dshot::maxChannels]){

    rmt_tx_channel_config_t tx_chan_config{};

    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;               // select source clock
    tx_chan_config.mem_block_symbols = 128;                     // memory block size, 64 * 4 = 256Bytes
    tx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;     // 1MHz tick resolution, i.e. 1 tick = 1us
    tx_chan_config.trans_queue_depth = 2;                       // set the number of transactions that can pend in the background

    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 600000, 
        .post_delay_us = 26, // extra delay between each frame
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_dshot_esc_encoder(&encoder_config, &this->dshot_encoder));

    for(int i=0;i<Dshot::maxChannels;i++){
        tx_chan_config.gpio_num = motorPin[i];
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[i]));
        printf("motor chan %d: %p pin %d\n", i, this->esc_motor_chan[i], tx_chan_config.gpio_num);
    }

    for(int i=0;i<Dshot::maxChannels;i++){
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
    }

    //this->m_dshot_queue_handle = xRingbufferCreate(sizeof(DshotMessage) * 10, RINGBUF_TYPE_NOSPLIT);
    //if (this->m_dshot_queue_handle == NULL) {
    //    printf("Failed to create drone ring buffer\n");
    //}

    //this->m_dshot_queue_handle = CircularBufCreate(10, sizeof(DshotMessage), "dshot600");
    //if (this->m_dshot_queue_handle == nullptr)
    //{
    //    printf("Failed to create dshot600 ring buffer\n");
    //}


    ////////// DEBUGGING ///////////

    /*
    auto start = std::chrono::steady_clock::now();
    DshotMessage msg;
    msg.msgType = Dshot::COMMAND;

    printf("Arming....\n");
    for(int i=0;i<Dshot::maxChannels;i++){
        msg.writeTo[i] = true;
        msg.loops[i] = 0;
        msg.cmd[i] = DSHOT_CMD_MOTOR_STOP;
        msg.telemetryReq[i] = false;

    }


    while(true){
        write_command(msg);

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();

        if(elapsed >= 5000){
            break;
        }
        vTaskDelay(1);
    }

    msg.msgType = Dshot::THROTTLE;
    printf("Ramping up....\n");

    for(int i=48;i<1000;i++){

        for(int j=0;j<Dshot::maxChannels;j++){
            msg.writeTo[j] = true;
            msg.loops[j] = 0;
            msg.telemetryReq[j] = false;
            msg.speed[j] = i;
        }   
        write_speed(msg);
        vTaskDelay(pdMS_TO_TICKS(10));

    }

    printf("Ramping up complete\n");
    */

}

Dshot600* Dshot600::GetInstance(gpio_num_t motorPin[Dshot::maxChannels]){
    if(dshot==nullptr){
        dshot = new Dshot600(motorPin);
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

esp_err_t Dshot600::write_speed(struct Dshot::DshotMessage& msg){

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};

    static DshotMessage prev_msg;

    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.speed[i],
            .telemetry_req = msg.telemetryReq[i], 
        };

        tx_config[i] = {
            .loop_count = msg.loops[i] == -1? -1 : 0,
        };    

    }
    prev_msg = msg;

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



    print_debug(DEBUG_DSHOT, DEBUG_DATA, "\n");

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
        //.writeTo = {true, true, true, true},
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
    //msg.writeTo[0] = true;
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
        //msg.writeTo[i] = i == 0? true : false;
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

static esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_del_encoder(dshot_encoder->bytes_encoder);
    rmt_del_encoder(dshot_encoder->copy_encoder);
    free(dshot_encoder);
    return ESP_OK;
}

static esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_encoder_reset(dshot_encoder->bytes_encoder);
    rmt_encoder_reset(dshot_encoder->copy_encoder);
    dshot_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static void make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
{
    frame->throttle = throttle;
    frame->telemetry = telemetry;
    uint16_t val = frame->val;
    uint8_t crc = ((val ^ (val >> 4) ^ (val >> 8)) & 0xF0) >> 4;;
    frame->crc = crc;
    val = frame->val;
    // change the endian
    frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}

static size_t rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
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
    make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);

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

