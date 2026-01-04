#include "djiO4Pro.h"

#include "debug.h"
#include "drone.h"



using namespace Msp;

DjiO4Pro::DjiO4Pro(){
    printf("dji o4 pro started\n");
    constexpr int txPin = 25;
    constexpr int rxPin = 26;
    constexpr int rtsPin = UART_PIN_NO_CHANGE;
    constexpr int ctsPin = UART_PIN_NO_CHANGE;
    constexpr int uart_buffer_size = (512);
    uart_config_t uart_config{};

    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;  

    s_uartNum = UART_NUM_1;

    ESP_ERROR_CHECK(uart_param_config(s_uartNum, &uart_config));
    printf("s_uartNum[%d] txPin[%d] rxPin[%d] rtsPin[%d] ctsPin[%d]\n", s_uartNum, txPin, rxPin, rtsPin, ctsPin);
    ESP_ERROR_CHECK(uart_set_pin(s_uartNum, txPin, rxPin, rtsPin, ctsPin));
    ESP_ERROR_CHECK(uart_driver_install(s_uartNum, uart_buffer_size, uart_buffer_size, 10, &this->uart_queue, 0));
    printf("dji o4 pro done\n");
}


void DjiO4Pro::vtx_task(void* args){
    esp_err_t status = 0;


    // runs until completion
    printf("starting osd display setup\n");
    init_displayport();
    printf("finished osd display setup\n");



    while(true){

        uint8_t data[512]{};
        size_t dataLength{};
        uart_event_t event;

        if (xQueueReceive(uart_queue, (void *)&event, pdMS_TO_TICKS(1000)) == pdTRUE){
            if (event.type == UART_DATA) {
                memset(data, 0, 512);
                dataLength = 0;
                status = uart_read(data, dataLength);

                if(!dataLength){
                    continue;
                }

                parse_data(data, dataLength);
            }
        }
        else{

        }
    }
}

void DjiO4Pro::parse_data(const uint8_t* buf, size_t buf_len)
{
    // MSPv1: '$' 'M' dir size cmd [payload...] checksum
    for (size_t i = 0; i + 5 < buf_len; i++) {              // need at least 6 bytes
        if (buf[i] != 0x24 || buf[i+1] != 0x4D) continue;   // '$''M'

        uint8_t dir  = buf[i+dir_idx];                            // '<' or '>' etc
        uint8_t plen = buf[i+size_idx];                            // payload length
        cmd_e cmd  = static_cast<cmd_e>(buf[i+cmd_idx]);

        size_t frame_len = 6u + plen;                       // header(5) + payload + csum(1)
        if (i + frame_len > buf_len) continue;              // incomplete frame, stop/continue

        const uint8_t* payload = &buf[i+payload_idx];
        uint8_t csum = buf[i+5+plen];

        // verify checksum
        uint8_t calc = plen ^ cmd;
        for (size_t k = 0; k < plen; k++) calc ^= payload[k];

        //printf("MSP v1 dir[%c] plen[%u] cmd[0x%02X, %u] csum[0x%02X] %s - name[%s] desc[%s]\n",
        //       (char)dir, plen, cmd, cmd, csum, (calc == csum ? "OK" : "BAD"), cmds.at(cmd).name.c_str(), cmds.at(cmd).desc.c_str());

        // probe message you showed: $ M < 00 64 64
        //if (dir == '<' && plen == 0 && cmd == 0x64) {
        //    printf("  -> PROBE request (cmd 0x64)\n");
        //}
        if (dir != dir_vtx_to_fc) { i += frame_len - 1; continue; }  // only '<'
        if (calc != csum)         { i += frame_len - 1; continue; }  // only valid frames

        parse_msg(cmd, payload, plen);


        i += frame_len - 1; // skip to next frame start
    }
}

void DjiO4Pro::parse_msg(cmd_e cmd, const uint8_t* buf, uint8_t len){

    switch(cmd){
        case MSP_STATUS:
            respond_status();
            break;
        case MSP_RC:
            respond_rc_channel();
            break;
        case MSP_ANALOG:
            respond_analog();
            break;
        case MSP_RC_TUNING:
            respond_rc_tuning();
            break;
        case MSP_PID:
            respond_pid();
            break;
        case MSP_BATTERY_STATE:
            respond_battery_status();
            break;
        case MSP_STATUS_EX:
            respond_status_ex();
            break;
        case MSP_FC_VERSION:
            respond_fc_version();
            break;
        case MSP_NAME:
            respond_name();
            break;
        case MSP_FILTER_CONFIG:
            respond_filter_config();
            break;
        case MSP_PID_ADVANCED:
            respond_pid_advanced();
            break;
        case MSP_SET_OSD_CANVAS:
            set_canvas(buf, len);
            break;
        case MSP_DISPLAYPORT:
            break;
        default:
            printf("unsupported MSP cmd[0x%x] len[%u] recieved\n", cmd, len);
    }

}

esp_err_t DjiO4Pro::set_canvas(const uint8_t* buf, size_t len){

    printf("***************************\n");
    printf("revieved MSP_SET_OSD_CANVAS\n");
    uint8_t* payload{};
    uint8_t pSize{};
    extract_payload(buf, len, &payload, pSize);

    for(int i=0;i<pSize;i++){
        printf("data[0x%x]\n", static_cast<uint8_t>(*(payload + i)));
    }


    printf("***************************\n");
    return 0;
}

esp_err_t DjiO4Pro::init_displayport(){
    esp_err_t status = 0;

    bool canvasRecieved{}; //  MSP_SET_OSD_CANVAS

    while(true){

        uint8_t data[512]{};
        size_t dataLength{};
        uart_event_t event;

        if(!canvasRecieved){
            send_dp_heartbeat();
        }

        if (xQueueReceive(uart_queue, (void *)&event, pdMS_TO_TICKS(100)) == pdTRUE){
            if (event.type == UART_DATA) {
                memset(data, 0, 512);
                dataLength = 0;
                status = uart_read(data, dataLength);

                if(!dataLength){
                    continue;
                }

                parse_data(data, dataLength);
            }
        }
    }

    return 0;
}


esp_err_t DjiO4Pro::send_dp_heartbeat(){

    uint8_t msg[] = {MSP_DP_HEARTBEAT};

    msp_write(MSP_DISPLAYPORT, msg, sizeof(msg));
    return 0;
}


esp_err_t DjiO4Pro::respond_status(){
    msp_status_t msg{};
    msg.cycleTime = 0;
    msg.i2cErrorCounter = 0;
    msg.sensor = 0;
    msg.flightModeFlags = 0xffff'ffff;
    msg.configProfileIndex = 0;

    ESP_ERROR_CHECK(msp_write(MSP_STATUS, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    
    return 0;
}

esp_err_t DjiO4Pro::respond_status_ex(){
    msp_status_ex_t msg{};
    msg.cycleTime = 0;
    msg.i2cErrorCounter = 0;
    msg.sensor = 0;
    msg.flightModeFlags = 0xfff0'0000;
    msg.configProfileIndex = 0;
    msg.averageSystemLoadPercent = 0;
    msg.armingFlags = 0;
    msg.accCalibrationAxisFlags = 0;

    ESP_ERROR_CHECK(msp_write(MSP_STATUS_EX, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    return 0;
}

esp_err_t DjiO4Pro::respond_fc_version(){
    msp_fc_version_t msg{};
    
    Drone::get_fw_version(msg.versionMajor, msg.versionMinor, msg.versionPatchLevel);
    ESP_ERROR_CHECK(msp_write(MSP_FC_VERSION, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    return 0;
}



esp_err_t DjiO4Pro::respond_rc_channel(){
    msp_rc_t msg{};
    Channel ch{};

    Drone::get_channel_values(ch);

    msg.channelValue[0]  = ch.ch1;
    msg.channelValue[1]  = ch.ch2;
    msg.channelValue[2]  = ch.ch3;
    msg.channelValue[3]  = ch.ch4;
    msg.channelValue[4]  = ch.ch5;
    msg.channelValue[5]  = ch.ch6;
    msg.channelValue[6]  = ch.ch7;
    msg.channelValue[7]  = ch.ch8;
    msg.channelValue[8]  = ch.ch9;
    msg.channelValue[9]  = ch.ch10;
    msg.channelValue[10] = ch.ch11;

    ESP_ERROR_CHECK(msp_write(MSP_RC, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    return 0;
}

esp_err_t DjiO4Pro::respond_analog(){
    msp_analog_t msg{};
    BatteryInfo bat_info{};
    RadioStatistics radio_info{};

    Drone::get_battery_status(bat_info);
    Drone::get_radio_status(radio_info);

    msg.vbat = bat_info.voltage / 10;
    msg.mAhDrawn = 0;
    msg.rssi = radio_info.upplink_quality * 100;
    msg.amperage = bat_info.current;

    ESP_ERROR_CHECK(msp_write(MSP_ANALOG, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));

    return 0;
}

esp_err_t DjiO4Pro::respond_rc_tuning(){
    msp_rc_tuning_t msg{};
    msg.rcRate8 = 0;
    msg.rcExpo8 = 0;
    msg.rates[0] = 0;
    msg.rates[1] = 0;
    msg.rates[2] = 0;
    msg.dynThrPID = 0;
    msg.thrMid8 = 0;
    msg.thrExpo8 = 0;
    msg.tpa_breakpoint = 0;
    msg.rcYawExpo8 = 0;

    ESP_ERROR_CHECK(msp_write(MSP_RC_TUNING ,reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));

    return 0;
}

esp_err_t DjiO4Pro::respond_pid(){

    msp_pid_t msg{};

    Pid pid[3]{};
    Drone::get_pid(pid, 3);

    for(int i=0;i<3;i++){
        msg.roll[i] = pid[i].kP;
        msg.pitch[i] = pid[i].kI;
        msg.yaw[i] = pid[i].kD;
        msg.pos_z[i] = 0;
        msg.pos_xy[i] = 0;
        msg.vel_xy[i] = 0;
        msg.surface[i] = 0;
        msg.level[i] = 0;
        msg.heading[i] = 0;
        msg.vel_z[i] = 0;
    }
    ESP_ERROR_CHECK(msp_write(MSP_PID, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    return 0;
}

esp_err_t DjiO4Pro::respond_name(){

    uint8_t msg[] = "ryssdodarn";

    ESP_ERROR_CHECK(msp_write(MSP_NAME, msg, sizeof(msg)));
    return 0;
}

esp_err_t DjiO4Pro::respond_filter_config(){

    msp_filter_config msg{};
    ESP_ERROR_CHECK(msp_write(MSP_FILTER_CONFIG, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    return 0;
}

esp_err_t DjiO4Pro::respond_pid_advanced(){

    msp_pid_advanced msg{};
    ESP_ERROR_CHECK(msp_write(MSP_PID_ADVANCED, reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
    return 0;
}


esp_err_t DjiO4Pro::respond_battery_status(){

    /*
    uint8_t msg[maxMspPayload] {};

    msg[0] = 6; // cell count u8
    msg[1] = 3500u; // bat capacity mAH u16
    msg[3] = 15*10 ; // bat voltage in 0.1V steps u8
    msg[4] = 100; // mAH drawn u16
    msg[6] = static_cast<int16_t>(60);  // send current in 0.01 A steps, range is -320A to 320A i16
    msg[8] = 0; // battery alerts u8
    msg[9] = 13; // bat voltage in 0.1V steps u16
    */

    msp_battery_state_s msg{};
    BatteryInfo info{};
    Drone::get_battery_status(info);

    msg.voltage = info.voltage / 10;
    msg.voltageLegacy = info.voltage / 10;
    msg.current = info.current;
    msg.cellCount = info.cellCount;
    msg.alerts = 0;
    msg.mahDrawn = 0;

    msp_write(MSP_BATTERY_STATE, reinterpret_cast<uint8_t*>(&msg), sizeof(msp_battery_state_s));

    return 0;
}

esp_err_t DjiO4Pro::uart_write(uint8_t* data, uint8_t dataLength){
    esp_err_t status = 0;

    //printf("responding[%s] ", 
    //    cmds.contains(static_cast<cmd_e>(data[type_idx]))? cmds.at(static_cast<cmd_e>(data[type_idx])).name.c_str() : "unknown");

    //for(int i=0;i<dataLength;i++){
    //    printf("[0x%x] ", data[i]);
    //}
    //printf("\n");

    uart_write_bytes(s_uartNum, data, dataLength);
    return status;
}

esp_err_t DjiO4Pro::uart_read(uint8_t* data, size_t& dataLength){
    esp_err_t status = 0;
    print_debug(DEBUG_DJIO4PRO, DEBUG_ARGS, "%-33s: uartNum: %d dataLength: %u\n", __func__, s_uartNum, dataLength);

    ESP_ERROR_CHECK(uart_get_buffered_data_len(s_uartNum, &dataLength));
    dataLength = uart_read_bytes(s_uartNum, data, dataLength, 100);

    return status;
}

esp_err_t DjiO4Pro::msp_write(cmd_e cmd, uint8_t* payload, uint8_t payloadLen){

    uint8_t msg[maxMspPayload] = { start_sym, version_1, dir_fc_to_vtx, payloadLen, cmd};

    if(cmds.at(cmd).name == "MSP_DISPLAYPORT")
    {
        msg[2] = dir_vtx_to_fc;
    }

    memcpy(&msg[payload_idx], payload, payloadLen);
    msg[payload_idx + msg[size_idx]] = crc8_v1(msg[cmd_idx], &msg[payload_idx], msg[size_idx]);

    ESP_ERROR_CHECK(uart_write(msg, 6 + msg[size_idx]));

    return ESP_OK;
}


esp_err_t DjiO4Pro::send_data(uint8_t* msg, size_t len){
    int w = uart_write_bytes(s_uartNum, (const char*)msg, sizeof(msg));
    if (w != (int)sizeof(msg)) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t DjiO4Pro::get_data(uint8_t* msg, size_t len){
        return ESP_OK;
}

uint8_t DjiO4Pro::crc8_v1(uint8_t cmd, uint8_t* payload, uint8_t len) {
        uint8_t calc = len ^ cmd;
        for (size_t k = 0; k < len; k++) calc ^= payload[k];
        return calc;
}

uint8_t DjiO4Pro::crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}


esp_err_t DjiO4Pro::extract_payload(const uint8_t* data, uint8_t len, uint8_t** payload, uint8_t& pLen){

    if(len <= 6){
        len = 0;
        return 0;
    }
    *payload = const_cast<uint8_t*>(&data[payload_idx]);
    pLen = data[size_idx] + len -6;

    return 0;
}
