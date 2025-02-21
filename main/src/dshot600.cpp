#include "dshot600.h"
#include "common_data.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"



#define DSHOT_ESC_RESOLUTION_HZ 40000000 
static const char *TAG = "dshot_encoder";

/*      My logic        */

Dshot600::Dshot600(gpio_num_t motorPin[Dshot::maxChannels]){

    rmt_tx_channel_config_t tx_chan_config{};

    tx_chan_config.clk_src = RMT_CLK_SRC_APB;                   // select source clock
    //tx_chan_config.gpio_num = m1_pin;                         // GPIO number
    tx_chan_config.mem_block_symbols = 64;                      // memory block size, 64 * 4 = 256Bytes
    tx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;     // 1MHz tick resolution, i.e. 1 tick = 1us
    tx_chan_config.trans_queue_depth = 10;                      // set the number of transactions that can pend in the background

    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[0]));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[1]));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[2]));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[3]));

    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 600000, 
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_dshot_esc_encoder(&encoder_config, &this->dshot_encoder));


    //rmt_enable(this->esc_motor_chan[0]);
    //rmt_enable(this->esc_motor_chan[1]);
    //rmt_enable(this->esc_motor_chan[2]);
    //rmt_enable(this->esc_motor_chan[3]);

    for(int i=0;i<Dshot::maxChannels;i++){
        tx_chan_config.gpio_num = motorPin[i];
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[i]));
    }

    for(int i=0;i<Dshot::maxChannels;i++){
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
    }


    //rmt_sync_manager_config_t synchro_config = {
    //    .tx_channel_array = this->esc_motor_chan,
    //    .array_size = 4,
    //};

   //ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, &synchro));


    //rmt_transmit_config_t tx_config = {
    //    .loop_count = -1, // infinite loop
    //};

    //dshot_esc_throttle_t throttle = {
    //    .throttle = 0,
    //    .telemetry_req = false, // telemetry is not supported in this example
    //};


    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_chan_m1, this->dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_chan_m2, this->dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_chan_m3, this->dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_chan_m4, this->dshot_encoder, &throttle, sizeof(throttle), &tx_config));

}

esp_err_t Dshot600::write(struct DshotMessage& msg, bool sendCommand){

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};

    for(int i=0;i<Dshot::maxChannels;i++){

        throttle[i] = {
            .throttle = static_cast<uint16_t>(sendCommand? msg.cmd[i] : msg.speed[i]),
            .telemetry_req = msg.telemetryReq[i], 
        };

        tx_config[i] = {
            .loop_count = msg.loops[i],
        };
    }

    //status = rmt_transmit(this->esc_chan, this->dshot_encoder, &throttle, sizeof(throttle), &tx_config);
    //ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value %u telemetry: %s", throttleValue, telemetryReq? "yes" : "no");


    // currently writes to all 4 synced.
    for(int i=0;i<Dshot::maxChannels;i++){
        status |= rmt_transmit(this->esc_motor_chan[0], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]);
    }

    return ESP_OK;
}

/**
 * speed_mX should be between 0 - 2000
 * loops_mX should be larger than 0 if write_to_mX is true
 */
esp_err_t Dshot600::set_speed(struct DshotMessage& msg){

    esp_err_t status = 0;
    constexpr uint8_t throttle_offset{48}; // value 0 - 47 is reserved for commands, 48 - 2047 is speed


    for(int i=0;i<Dshot::maxChannels;i++){
        msg.speed[i] += throttle_offset;
    }

    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write(msg, false);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}

esp_err_t Dshot600::parse_dshot_message(struct DshotMessage& msg) {
    bool badMsg = false;
    uint8_t writeToCheck{};

    for(int i=0;i<Dshot::maxChannels;i++){
        writeToCheck += msg.writeTo[i];
    }
    badMsg = writeToCheck == 0? true : false;

    if (writeToCheck == 0) {
        ESP_LOGE(TAG, "Invalid: No motor write request");
        return ESP_FAIL;
    }


    // Validate motor speed and loop count
    auto validate_motor = [](uint16_t speed, uint8_t loops) {
        return (speed >= 48 && speed <= 2047) && (loops > 0);
    };

    for(int i=0;i<Dshot::maxChannels;i++){
        if (msg.writeTo[i] && !validate_motor(msg.speed[i], msg.loops[i])) badMsg = true;
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

    status = write(msg, true);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to write dshoot value msg");

    return status;
}

esp_err_t Dshot600::beep_cmd(enum BeepNum num){

    esp_err_t status = 0;
    DshotMessage msg{};
    
    for(int i=0;i<Dshot::maxChannels;i++){
        msg.writeTo[i] = true;
        msg.cmd[i] = DSHOT_CMD_BEEP1 + num;
    }

    status = parse_dshot_message(msg);
    ESP_RETURN_ON_ERROR(status, TAG, "Failed to parse msg");

    status = write(msg, true);
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