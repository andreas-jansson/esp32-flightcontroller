
#include <chrono>
#include <cstring>

#include "dshot600.h"
#include "common_data.h"
#include "debug.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rom/ets_sys.h"



#define DSHOT_ESC_RESOLUTION_HZ 40000000 
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
    1,      //DSHOT_CMD_SPIN_DIRECTION_NORMAL
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






/*      My logic        */

Dshot600::Dshot600(gpio_num_t motorPin[Dshot::maxChannels]){

    rmt_tx_channel_config_t tx_chan_config{};

    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;               // select source clock
    tx_chan_config.mem_block_symbols = 128;                     // memory block size, 64 * 4 = 256Bytes
    tx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;     // 1MHz tick resolution, i.e. 1 tick = 1us
    tx_chan_config.trans_queue_depth = 1;                       // set the number of transactions that can pend in the background


    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 600000, 
        .post_delay_us = 25, // extra delay between each frame
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_dshot_esc_encoder(&encoder_config, &this->dshot_encoder));

    for(int i=0;i<Dshot::maxChannels;i++){
        tx_chan_config.gpio_num = motorPin[i];
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[i]));
        printf("motor chan %d: %p\n", i, this->esc_motor_chan[i]);
    }

    for(int i=0;i<Dshot::maxChannels;i++){
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
    }

    this->m_dshot_queue_handle = xRingbufferCreate(sizeof(DshotMessage) * 100, RINGBUF_TYPE_NOSPLIT);
    if (this->m_dshot_queue_handle == NULL) {
        printf("Failed to create drone ring buffer\n");
    }

    /*

    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};
   

    printf("sending arm sequence\n");
    DshotMessage msg2{};

    msg2.speed[MOTOR1] = 0;
    msg2.telemetryReq[MOTOR1] = false;
    msg2.loops[MOTOR1] = -1;

    throttle[MOTOR1] = {
        .throttle = msg2.speed[MOTOR1],
        .telemetry_req = msg2.telemetryReq[MOTOR1], 
    };

    tx_config[MOTOR1] = {
        .loop_count = msg2.loops[MOTOR1] == -1? -1 : 0,
    };    

    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[MOTOR1], this->dshot_encoder, &throttle[MOTOR1], sizeof(throttle[MOTOR1]), &tx_config[MOTOR1]));   
    ets_delay_us(10);
    vTaskDelay(pdTICKS_TO_MS(3000));    

    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_disable(this->esc_motor_chan[MOTOR1]));
    vTaskDelay(pdTICKS_TO_MS(3000));    
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[MOTOR1]));



    DshotMessage msg3{};

    msg3.speed[MOTOR1] = DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE;
    msg3.telemetryReq[MOTOR1] = false;
    msg3.loops[MOTOR1] = 0;

    throttle[MOTOR1] = {
        .throttle = msg3.speed[MOTOR1],
        .telemetry_req = msg3.telemetryReq[MOTOR1], 
    };

    tx_config[MOTOR1] = {
        .loop_count = msg3.loops[MOTOR1] == -1? -1 : 0,
    };    

    for(int i=0;i<dshotLoopsLookup[DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE];i++){
        printf("sending telemetry enable sequence\n");
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[MOTOR1], this->dshot_encoder, &throttle[MOTOR1], sizeof(throttle[MOTOR1]), &tx_config[MOTOR1]));   
        ets_delay_us(100);
    }  
    vTaskDelay(pdTICKS_TO_MS(3000));    


    DshotMessage msg{};

    msg.speed[MOTOR1] = 100;
    msg.telemetryReq[MOTOR1] = true;
    msg.loops[MOTOR1] = 0;

    throttle[MOTOR1] = {
        .throttle = msg.speed[MOTOR1],
        .telemetry_req = msg.telemetryReq[MOTOR1], 
    };

    tx_config[MOTOR1] = {
        .loop_count = msg.loops[MOTOR1] == -1? -1 : 0,
    };    

    printf("sending valid throttle\n");
    while(true){
        vTaskDelay(pdTICKS_TO_MS(3000));    
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[MOTOR1], this->dshot_encoder, &throttle[MOTOR1], sizeof(throttle[MOTOR1]), &tx_config[MOTOR1]));   
        ets_delay_us(10);
    }  

    printf("DONE\n");

    vTaskDelay(pdTICKS_TO_MS(1000000));    

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


    while(true){

        DshotMessage msg{};
        status = get_message(msg, portMAX_DELAY);
        ESP_ERROR_CHECK_WITHOUT_ABORT(status);
   
        if(status == ESP_OK){
            status = parse_dshot_message(msg);
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);

            if(status){
                print_debug(DEBUG_DSHOT, DEBUG_LOGIC, "no valid msg\n");
                continue;
            }

            if(msg.msgType == COMMAND){
                print_debug(DEBUG_DSHOT, DEBUG_LOGIC, "writing new command\n");
                ESP_ERROR_CHECK_WITHOUT_ABORT(write_command(msg));
            }
            else if(msg.msgType == THROTTLE){
                print_debug(DEBUG_DSHOT, DEBUG_LOGIC, "writing new throttle\n");
                ESP_ERROR_CHECK_WITHOUT_ABORT(set_speed(msg));
            }
        }

        else{
            ESP_ERROR_CHECK_WITHOUT_ABORT(status);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void Dshot600::esc_telemetry_task(void* args){

    uart_event_t event;
    uint8_t data[512]{};
    size_t dataLength{}; 

    while(true){

        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)){
            if (event.type == UART_DATA) {
                memset(data, 0, 512);
                dataLength = 0; //should be 10 bytes

                ESP_ERROR_CHECK(uart_get_buffered_data_len(this->uartNum, &dataLength));
                dataLength = uart_read_bytes(this->uartNum, data, dataLength, 100);

                if(!dataLength){
                    continue;
                }
            }
        }

        if(dataLength == 80){
            printf("esc read:");
            for(int i=0;i<dataLength;i++){
                printf(" 0x%x", data[i]);
            }
            printf("\n");
        }
    }
}

esp_err_t Dshot600::get_message(struct Dshot::DshotMessage& msg, TickType_t ticks){
    esp_err_t status = 0;

    size_t itemSize = sizeof(Dshot::DshotMessage);
    DshotMessage* data = (Dshot::DshotMessage*)xRingbufferReceive(this->m_dshot_queue_handle, &itemSize, ticks);
    if(data != nullptr){
        msg = *data;
        vRingbufferReturnItem(this->m_dshot_queue_handle, (void*)data);
    }
    else{
        return ESP_ERR_NOT_FOUND;
    }

    return status;
}

esp_err_t Dshot600::write_speed(struct Dshot::DshotMessage& msg){

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};

    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.speed[i],
            .telemetry_req = msg.telemetryReq[i], 
        };

        tx_config[i] = {
            .loop_count = msg.loops[i] == -1? -1 : 0,
        };    
    }

    for(int i=0;i<Dshot::maxChannels;i++){
        if(msg.writeTo[i]){
            //print_debug(DEBUG_DSHOT, DEBUG_DATA, "throttle: %u tele: %d\n", throttle[i].throttle, throttle[i].telemetry_req);
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_disable(this->esc_motor_chan[i]));
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));   
            ets_delay_us(10);
        }
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

        if(msg.writeTo[i]){
            //print_debug(DEBUG_DSHOT, DEBUG_DATA, "cmd: %u\n", throttle[i].throttle, throttle[i].telemetry_req);

            for(uint16_t j=0;j<dshotLoopsLookup[msg.cmd[i]];j++){
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_disable(this->esc_motor_chan[i]));
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));
                ets_delay_us(10);

                if(dshotWaitLookup[msg.cmd[i]] > 0) 
                    vTaskDelay(dshotWaitLookup[msg.cmd[i]]);
            }
        }
    }
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
    bool badMsg = false;
    uint8_t writeToCheck{};

    for(int i=0;i<Dshot::maxChannels;i++){
        writeToCheck += msg.writeTo[i];
    }
    badMsg = writeToCheck == 0? true : false;

    if (writeToCheck == 0) {
        ESP_LOGE(TAG, "Invalid: No motor write request");
        for(int i=0;i<Dshot::maxChannels;i++){
            printf("m%d writeTo: %d speed: %d cmd: %d\n", i, msg.writeTo[i], msg.speed[i], msg.cmd[i]);
        }
        return ESP_FAIL;
    }

    // Validate motor speed and loop count
    auto validate_motor = [](uint16_t speed, uint8_t loops) {
        return (speed >= Dshot::minThrottleValue && speed <= Dshot::maxThrottleValue);
    };

    for(int i=0;i<Dshot::maxChannels;i++){
        if(msg.speed[i] != 0)
            if (msg.writeTo[i] && !validate_motor(msg.speed[i], msg.loops[i])) {
                printf("bad msg: %d writeTo %d cmd %u speed %u\n", i, msg.writeTo[i], msg.cmd[i], msg.speed[i]);
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

    msg.writeTo[motorNum] = true;
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
    msg.writeTo[0] = true;
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
    msg.writeTo[0] = true;
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
        msg.writeTo[i] = i == 0? true : false;
        msg.cmd[i] = DSHOT_CMD_MOTOR_STOP;
        msg.loops[i] = 10000;

        status = parse_dshot_message(msg);
        ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");
    }

    status = write_command(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}

esp_err_t Dshot600::init_uart(int rxPin, int txPin, int baudrate){

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