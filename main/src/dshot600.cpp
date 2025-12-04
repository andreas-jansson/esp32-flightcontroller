
#include <chrono>
#include <cstring>
#include <algorithm>
#include <bitset>
#include <cmath>

#include "dshot600.h"
#include "common_data.h"
#include "debug.h"
#include "helpers.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "rom/ets_sys.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#include "esp_cpu.h"
#include "esp_private/esp_clk.h"
#include "esp_attr.h"
#include "include/soc/gpio_sig_map.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "hal/rmt_ll.h"
#include "soc/rmt_struct.h"
#include "soc/rmt_periph.h"
#include "esp_rom_sys.h"
#include "rom/gpio.h"


#define DSHOT_150 150 
#define DSHOT_300 300 
#define DSHOT_600 600 
#define DSHOT_SPEED DSHOT_600 

#define DSHOT_ESC_RESOLUTION_HZ 80000000 
static const char *TAG = "dshot_encoder";

using namespace Dshot;

Dshot600* Dshot600::dshot = nullptr;
bool Dshot600::m_isBidi{};
SemaphoreHandle_t s_newData = nullptr;


rmt_symbol_word_t raw_symbols[64];
rmt_symbol_word_t edata_rx_symbol[4][16]{};
static rmt_rx_done_event_data_t edata_rx[4]{};

static uint64_t rx_cntr[4]{};

static uint64_t rx_fail_cntr{};
static uint64_t rx_fail_bits_cntr{};
static uint64_t rx_fail_crc_cntr{};
static uint64_t rx_fail_gcr_cntr{};
static uint64_t rx_ok_cntr{};

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


rx_symbol_t rx_sym_data{};
rx_symbol_t rx_sym_data_bac_crc[20]{};
rx_symbol_t rx_sym_data_good[20]{};


static void IRAM_ATTR rx_handler(void *args)
{
    if(rx_sym_data.num_symbols<32){
        rx_sym_data.sym[rx_sym_data.num_symbols] = esp_cpu_get_cycle_count();
        rx_sym_data.bit_info[rx_sym_data.num_symbols].sym_lvl = gpio_ll_get_level(GPIO_LL_GET_HW(GPIO_PORT_0), 12);    
        rx_sym_data.num_symbols++;
    }
}

static bool IRAM_ATTR rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx){

    cb_dshot_ctx* ctx = reinterpret_cast<cb_dshot_ctx*>(user_ctx);
    //gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), ctx->pin);

    if(edata->num_symbols == 16 && rx_cntr[ctx->motor] % 1000 == 0){
        //uint16_t shift = (edata->received_symbols->val & 0xE000) >> 9;
        //uint16_t erpm = (((edata->received_symbols->val & 0x1FF0) >> 4) >> shift);
        //rpm_data[ctx->motor] = erpm / 14;
        memcpy(&edata_rx[ctx->motor], edata, sizeof(rmt_rx_done_event_data_t));

        for(int i=0;i<16;i++){
            memcpy(static_cast<void*>(&edata_rx_symbol[ctx->motor][i]), static_cast<void*>(&edata->received_symbols[i]), sizeof(rmt_symbol_word_t));
        }

    }
    
    rx_cntr[ctx->motor]++;


    return  pdTRUE;
}


static bool IRAM_ATTR tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    cb_dshot_ctx* ctx = reinterpret_cast<cb_dshot_ctx*>(user_ctx);
    //gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), ctx->pin);
    //rmt_receive(ctx->rx_handle, raw_symbols, 64, const_cast<rmt_receive_config_t*>(&ctx->rx_chan_config));
	return true; 
}

static intr_handle_t s_my_rmt_intr = nullptr;
static volatile bool s_rmt_ch0_tx_done = false;

static void IRAM_ATTR my_rmt_tx_isr(void *arg)
{
    uint32_t status = RMT.int_raw.val;

    //RMT.int_clr.val = status;

    if (status) {
        s_rmt_ch0_tx_done = true;
        gpio_set_level(static_cast<gpio_num_t>(25), 0);
        gpio_set_level(static_cast<gpio_num_t>(25), 1);
    }

}

void Dshot600::attach_my_rmt_tx_isr(void)
{
    int group_id{0};  
    int isr_flags{0x50e};
    int chanId{};

    // rmt_channel_handle_t == rmt_channel_t, first int is channel id in rmt_private.h
    memcpy(&chanId, esc_motor_chan[0], sizeof(int));

    esp_err_t err = esp_intr_alloc_intrstatus(
        rmt_periph_signals.groups[group_id].irq,               
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED,
        (uint32_t)rmt_ll_get_interrupt_status_reg(&RMT),       
        RMT_LL_EVENT_TX_MASK(chanId),
        my_rmt_tx_isr,
        nullptr,
        &s_my_rmt_intr
    );

    printf("********** my isr ********\n");
    printf("rmt_periph_signals.groups[group_id].irq\t\t[%d]\n", rmt_periph_signals.groups[group_id].irq);
    printf("ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED\t\t[0x%x]\n", ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED);
    printf("rmt_ll_get_interrupt_status_reg(&RMT)\t\t[%p]\n", rmt_ll_get_interrupt_status_reg(&RMT));
    printf("RMT_LL_EVENT_TX_MASK(%d)\t\t[0x%x]\n", chanId, RMT_LL_EVENT_TX_MASK(chanId));

    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
}


Dshot600::Dshot600(gpio_num_t motorPin[Dshot::maxChannels], bool isBidi){
    rmt_tx_channel_config_t tx_chan_config{};
    rmt_rx_channel_config_t rx_chan_config{};
    uint32_t baud_rate{};

    m_isBidi = isBidi;  

    vSemaphoreCreateBinary(s_newData);

    c_cpuFreq = esp_clk_cpu_freq();
    c_cycleToUs = (1.0 / c_cpuFreq) * 1000'000;

    gpio_config_t conf = {
    .pin_bit_mask = 1ULL << motorPin[0],
    .mode = GPIO_MODE_INPUT_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_ANYEDGE,
        };
    gpio_config(&conf);

    printf("***** pre isr setup ******\n");
    my_gpio_dump(motorPin[0]);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(static_cast<gpio_num_t>(motorPin[0]), rx_handler, nullptr);

    printf("***** post isr setup ******\n");
    my_gpio_dump(motorPin[0]);

    switch(DSHOT_SPEED){
        case DSHOT_600:
        baud_rate = DSHOT_600 * 1000;
        break;
        case DSHOT_300:
        baud_rate = DSHOT_300 * 1000;
        break;
        case DSHOT_150:
        baud_rate = DSHOT_150 * 1000;
        break;
        default:
        baud_rate = DSHOT_600 * 1000;
    }

    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = baud_rate,
        .post_delay_us = (m_isBidi? 0u : 26u), // extra delay between each frame
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_dshot_esc_encoder(&encoder_config, &this->dshot_encoder));


    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;               // select source clock
    tx_chan_config.mem_block_symbols = 64;                      // memory block size, 64 * 4 = 256Bytes
    tx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;     // 1MHz tick resolution, i.e. 1 tick = 1us
    tx_chan_config.trans_queue_depth = 1;                       // set the number of transactions that can pend in the background
    //tx_chan_config.intr_priority = 3;
    //tx_chan_config.flags.with_dma = 0;

    if(isBidi)
    {
        tx_chan_config.flags.invert_out = 1;
        tx_chan_config.flags.io_od_mode = 1;    
    }

    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.mem_block_symbols = 64;
    rx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;
    rx_chan_config.flags.invert_in = 1;

    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 	uint32_t(200),
        .signal_range_max_ns = uint32_t(6000),
    };

    rmt_rx_event_callbacks_t rx_callback = 
    {
        .on_recv_done = rx_done_callback
    };
    rmt_tx_event_callbacks_t tx_callback = 
    {
        .on_trans_done = tx_done_callback
    };

    
    for(int i=0;i<Dshot::maxChannels;i++){

        m_gpioMotorPin[i]= motorPin[i];
        tx_chan_config.gpio_num = m_gpioMotorPin[i];
        rx_chan_config.gpio_num = m_gpioMotorPin[i];

        gpio_ll_output_disable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_pull_mode(m_gpioMotorPin[i], GPIO_PULLUP_ONLY));
        gpio_ll_set_drive_capability(GPIO_LL_GET_HW(GPIO_PORT_0), static_cast<uint32_t>(m_gpioMotorPin[i]), GPIO_DRIVE_CAP_3);

        // start rx before to not mess upp gpio func
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_rx_channel(&rx_chan_config, &this->rmt_rx_handle[i]));
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[i]));
        printf("motor chan %d: %p pin %d\n", i, this->esc_motor_chan[i], tx_chan_config.gpio_num);

        /* setup callbacks */
        m_ctx[i] = static_cast<cb_dshot_ctx*>(calloc(1,sizeof(cb_dshot_ctx)));
        m_ctx[i]->tx_handle = esc_motor_chan[i];
        m_ctx[i]->rx_handle = rmt_rx_handle[i];
        m_ctx[i]->pin = m_gpioMotorPin[i];
        m_ctx[i]->motor = static_cast<motor_type_t>(i);
        m_ctx[i]->rx_chan_config = rx_config;


        //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_rx_register_event_callbacks(rmt_rx_handle[i], &rx_callback, reinterpret_cast<void*>(m_ctx[i])));
        //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_register_event_callbacks(esc_motor_chan[i], &tx_callback, reinterpret_cast<void*>(m_ctx[i])));

        /* start up RMT channels*/
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->rmt_rx_handle[i]));
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
    }

    printf("***** post rmt setup ******\n");
    my_gpio_dump(m_gpioMotorPin[0]);

    init_dshot_debug_pin(25);

    attach_my_rmt_tx_isr();

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
    //}
    //else{
    //    return ESP_ERR_NOT_FOUND;
    //}

    //return status;
    return ESP_FAIL;
}
//extern rmt_dev_t RMT;

esp_err_t Dshot600::write_speed(struct Dshot::DshotMessage& msg){
    using namespace std::chrono_literals;

    esp_err_t status{};
    static uint8_t recieve_error_counter{};
    constexpr uint8_t recieve_error_thld{100};
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};
    std::chrono::_V2::steady_clock::time_point rx_ready_us[Dshot::maxChannels]{};

    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.speed[i],
            .telemetry_req = true,
        };

        tx_config[i].loop_count = msg.loops[i] == -1? -1 : 0;
    }

    // semaphore to sync recieve if ESC is powered

    xSemaphoreTake(s_newData, pdMS_TO_TICKS(1));

    if(m_isBidi){

        for(int i=0;i<Dshot::maxChannels;i++){


            if(i==0){
                memset(static_cast<void*>(&rx_sym_data), 0, sizeof(rx_symbol_t));
            }
            gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);
            auto tx_called =  std::chrono::steady_clock::now();
            rx_ready_us[i] = tx_called + 65us;

            // median 11us avg 10us min 6us max 122us (first)
            //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));


            //printf("motor %d starting tx done measurement...raw[0x%lx]\n", i, RMT.int_raw.val);
            //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));
            //esp_cpu_cycle_count_t cycles_start = esp_cpu_get_cycle_count();
            //while (!RMT.int_raw.val);
            //esp_cpu_cycle_count_t cycles_end = esp_cpu_get_cycle_count();
            //constexpr float c_cycleToUs = (1.0/240000000.0) * 1000000;
            //float tx_done_us = static_cast<float>(( cycles_end - cycles_start ) * c_cycleToUs);
            //printf("motor %d took %f us for tx done, raw[0x%lx]\n", i, tx_done_us, RMT.int_raw.val);
            
            if(i==0){
                printf("motor %d starting tx done measurement...raw[0x%lx]\n", i, RMT.int_raw.val);
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));
                esp_cpu_cycle_count_t cycles_start = esp_cpu_get_cycle_count();

                while (!s_rmt_ch0_tx_done) {
                    __asm__ __volatile__("nop");
                }
                esp_cpu_cycle_count_t cycles_end = esp_cpu_get_cycle_count();
                constexpr float c_cycleToUs = (1.0/240000000.0) * 1000000;
                float tx_done_us = static_cast<float>(( cycles_end - cycles_start ) * c_cycleToUs);
                printf("motor %d took %f us for tx done, raw[0x%lx]\n", i, tx_done_us, RMT.int_raw.val);
            }            
            s_rmt_ch0_tx_done = false;


            

            if(i==0 && msg.speed[0] > 50 && false){                

                ets_delay_us(110);
                gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[0]);
                ets_delay_us(15);
                gpio_intr_enable(static_cast<gpio_num_t>(m_gpioMotorPin[0]));
                //gpio_set_level(static_cast<gpio_num_t>(25), 0);
                //gpio_set_level(static_cast<gpio_num_t>(25), 1);
                uint32_t mini_counter = 0;
                while(rx_sym_data.num_symbols<16 && mini_counter<200){
                    ets_delay_us(1);
                    mini_counter++;
                }            
                rx_symbol_t rx_sym{};
                memcpy(&rx_sym, static_cast<void*>(&rx_sym_data), sizeof(rx_symbol_t));    
                if(process_erpm_data(rx_sym) == -1){
                    rx_fail_cntr++;
                }
                else
                    rx_ok_cntr++;
                gpio_intr_disable(static_cast<gpio_num_t>(m_gpioMotorPin[0]));

            }
            
            //printf("***** post isr enable isr setup ******\n");
            //my_gpio_dump(m_gpioMotorPin[0]);
        }

        /*
        uint8_t rx_done{};
        //  trigger rx when time window is correct instead of busy waiting 
        for(uint8_t j=rx_done;j<=Dshot::maxChannels;j++){
            auto now = std::chrono::steady_clock::now();
            if(now >= rx_ready_us[j] && now <= rx_ready_us[j] + 40us){
                rx_done = j + 1;
            }
            else if(now < rx_ready_us[j] && j == Dshot::maxChannels - 1){ //if last motor, wait for window
                auto delay = std::chrono::duration_cast<std::chrono::microseconds>(rx_ready_us[j] - now).count();
                if(delay>0)
                    ets_delay_us(delay);
            }
            else{
                break;
            }
            gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[j]);

            // avoid listning if ESC without power 
            if(recieve_error_counter <= recieve_error_thld)
                status = rmt_receive(m_ctx[j]->rx_handle, raw_symbols, 64, const_cast<rmt_receive_config_t*>(&m_ctx[j]->rx_chan_config));
            if(status != ESP_OK){
                recieve_error_counter++;
            }
        }*/
    }
    else{
        for(int i=0;i<Dshot::maxChannels;i++){
                print_debug(DEBUG_DSHOT, DEBUG_DATA, "m%d throttle: %u telemetry: %d channel: 0x%x", i, throttle[i].throttle, throttle[i].telemetry_req, this->esc_motor_chan[i]);
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i])); 
        }

        for(int i=0;i<Dshot::maxChannels;i++){
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(20)));
        }
    }

    static uint64_t cntr_2{};
    if(cntr_2 % 20000 == 0 && cntr_2 != 0){
        rx_symbol_t rx_sym{};
        memcpy(&rx_sym, static_cast<void*>(&rx_sym_data), sizeof(rx_symbol_t));    
        for(int i=0;i<Dshot::maxChannels;i++){
            if(rx_sym.num_symbols)
                printf("num_symbols[%d]\n", rx_sym.num_symbols);
            else
                printf("nada\n");
            if(i==0){
                process_erpm_data(rx_sym);
                for(int j=0;j<rx_sym.num_symbols;j++){
                    printf("[%2d] period[%-4.2f]us level[%d]\n", j, rx_sym.bit_info[j].period_us, rx_sym.bit_info[j].sym_lvl);
                }

            }
        }
        printf("\n");
        printf("ok %llu fail %llu crc %llu bits %llu gcr %llu success %f\n", rx_ok_cntr, 
            rx_fail_cntr, rx_fail_crc_cntr, rx_fail_bits_cntr, rx_fail_gcr_cntr,
            static_cast<float>(rx_ok_cntr) / static_cast<float>((rx_ok_cntr + rx_fail_cntr)));

    }
    cntr_2++;

    return status;
}

esp_err_t Dshot600::write_command(struct Dshot::DshotMessage& msg){

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};
    
    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.cmd[i],
        };

        tx_config[i] = {
            .loop_count = msg.loops[i]  == -1? -1 : 0,
        };    
    }

    for(int i=0;i<Dshot::maxChannels;i++){
        for(uint16_t j=0;j<dshotLoopsLookup[msg.cmd[i]] * 2;j++){ //send more than specified incase jitter etc
            print_debug(DEBUG_DSHOT, DEBUG_DATA, "m%d command: %u  channel: 0x%x ", i, throttle[i].throttle, this->esc_motor_chan[i]);
            
            if(m_isBidi)
                gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);

            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(20)));

            if(m_isBidi)
                gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[0]);

            if(dshotWaitLookup[msg.cmd[i]] > 0) 
                vTaskDelay(dshotWaitLookup[msg.cmd[i]]);
         
        }
    }
    print_debug(DEBUG_DSHOT, DEBUG_DATA, "\n");

    return status;
}

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


/*      RMT example functions       */
esp_err_t Dshot600::rmt_del_dshot_encoder(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_del_encoder(dshot_encoder->bytes_encoder);
    rmt_del_encoder(dshot_encoder->copy_encoder);
    free(dshot_encoder);
    return ESP_OK;
}

esp_err_t IRAM_ATTR Dshot600::rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_encoder_reset(dshot_encoder->bytes_encoder);
    rmt_encoder_reset(dshot_encoder->copy_encoder);
    dshot_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

void IRAM_ATTR Dshot600::make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
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

void IRAM_ATTR Dshot600::make_bidi_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
{
    frame->throttle = throttle;
    frame->telemetry = telemetry;
    uint16_t val = (throttle <<1) | telemetry;
    uint8_t crc  = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0F;
    frame->crc = crc;
    val = frame->val;
    // change the endian
    frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}

size_t IRAM_ATTR Dshot600::rmt_encode_dshot_esc(
    rmt_encoder_t *encoder,
    rmt_channel_handle_t channel,
    const void *primary_data,
    size_t data_size,
    rmt_encode_state_t *ret_state)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);

    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    dshot_esc_throttle_t *throttle = (dshot_esc_throttle_t *)primary_data;

    dshot_esc_frame_t frame = {};
    if (m_isBidi)
        make_bidi_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);
    else
        make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);

    switch (dshot_encoder->state) {

    case 0: // Encode 16-bit DSHOT frame
        encoded_symbols += dshot_encoder->bytes_encoder->encode(
            dshot_encoder->bytes_encoder,
            channel, &frame, sizeof(frame),
            &session_state);

        if (session_state & RMT_ENCODING_COMPLETE)
            dshot_encoder->state = 1;

        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = (rmt_encode_state_t)(RMT_ENCODING_MEM_FULL);
            goto out;
        }
    case 1:
        encoded_symbols += dshot_encoder->copy_encoder->encode(
            dshot_encoder->copy_encoder,
            channel,
            &dshot_encoder->dshot_delay_symbol,
            sizeof(rmt_symbol_word_t),
            &session_state);

        if (session_state & RMT_ENCODING_COMPLETE) {
            state = RMT_ENCODING_COMPLETE;
            dshot_encoder->state = 0;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = RMT_ENCODING_MEM_FULL;
            goto out;
        }
        break;
    }

out:
    *ret_state = state;
    return encoded_symbols;
}




/* configure timings for rmt data */
esp_err_t IRAM_ATTR Dshot600::rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_dshot_esc_encoder_t *dshot_encoder = NULL;
    dshot_encoder = (rmt_dshot_esc_encoder_t*)calloc(1, sizeof(rmt_dshot_esc_encoder_t));

    dshot_encoder->base.encode = rmt_encode_dshot_esc;
    dshot_encoder->base.del = rmt_del_dshot_encoder;
    dshot_encoder->base.reset = rmt_dshot_encoder_reset;
    uint32_t delay_ticks = config->resolution / 1e6 * config->post_delay_us;
    printf("delay_ticks: %lu\n", delay_ticks); 

    
	//handle_error(rmt_tx_register_event_callbacks(dshot_config.tx_chan, &callback, &dshot_config.tx_callback_datapack));


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


int Dshot600::decode_timings_to_gcr(rx_symbol_t& rx_sym){
    const float max_thld{1.3};
    const float min_thld{0.7};
    float total_msg_us{};
    constexpr float c_cycleToUs = (1.0/240000000.0) * 1000000;
    constexpr float bit_period_us{6.67}; //dshot150
    constexpr float single_bit_us{(4.0/5) * bit_period_us};
    rx_sym.gcr_data = 0;

    for(uint8_t i=0; i<rx_sym.num_symbols-1; i++) {
        rx_sym.bit_info[i].period_us = static_cast<float>(( rx_sym.sym[i+1] - rx_sym.sym[i] ) * c_cycleToUs);
    }

    uint8_t bit_index = 0;

    for(uint8_t i=0; i<rx_sym.num_symbols - 1; i++) {

        float bits_f = rx_sym.bit_info[i].period_us / single_bit_us;
        uint8_t nr_bits = static_cast<uint8_t>(std::round(bits_f));

        if (nr_bits < 1 || nr_bits > 4) {
            rx_fail_bits_cntr++;
            return -1;
        }

        uint8_t level = rx_sym.bit_info[i].sym_lvl ? 0 : 1;

        for (uint8_t b = 0; b < nr_bits; ++b) {
            rx_sym.gcr_data |= (uint32_t(level) << (20 - bit_index));
            ++bit_index;
        }
    }
    return 0;
}


int Dshot600::extract_nibbles(rx_symbol_t& rx_sym){
    uint8_t s0 = (rx_sym.gcr_data >> 15) & 0x1F;
    uint8_t s1 = (rx_sym.gcr_data >> 10) & 0x1F;
    uint8_t s2 = (rx_sym.gcr_data >>  5) & 0x1F;
    uint8_t s3 = (rx_sym.gcr_data >>  0) & 0x1F;

    if(!unmapGcr.contains(s0) || !unmapGcr.contains(s1) || !unmapGcr.contains(s2) || !unmapGcr.contains(s3)){
        rx_fail_gcr_cntr++;
        return -1;
    }

    uint8_t n0 = unmapGcr.at(s0);
    uint8_t n1 = unmapGcr.at(s1);
    uint8_t n2 = unmapGcr.at(s2);
    uint8_t n3 = unmapGcr.at(s3);

    rx_sym.data_bits = (static_cast<uint16_t>(n0) << 12) |
                       (static_cast<uint16_t>(n1) <<  8) |
                       (static_cast<uint16_t>(n2) <<  4) |
                       (static_cast<uint16_t>(n3) <<  0);

    return 0;
}

int Dshot600::crc_4(uint16_t& word16){

    uint16_t payload = word16 >> 4;   // upper 12 bits
    uint8_t crc = payload ^ (payload >> 4) ^ (payload >> 8);

    crc = ~crc;
    crc &= 0x0F;

    uint8_t crc_rx = word16 & 0x0F;

    if (crc != crc_rx) {
        rx_fail_crc_cntr++;
        return -1;
    }
    return 0;
}


int Dshot600::decode_msg(rx_symbol_t& rx_sym){
    if(crc_4(rx_sym.data_bits) == -1)
        return -1;

    if(rx_sym.data_bits & 0x1000) {
        if(rx_sym.data_bits == 0xFFF0) {
          rx_sym.data = 0;
        }else{
            rx_sym.msgType = RPM;
            uint8_t exponent  = (rx_sym.data_bits >> 13) & 0x07;      // 3 bits
            uint16_t mantissa = (rx_sym.data_bits >> 4) & 0x01FF;     // 9 bits (value field)
            rx_sym.data = 1e6 / (mantissa << exponent);
        }
    }
    else{
        // pppp mmmmmmmm
        uint8_t type = rx_sym.data_bits >> 12;
        if(type == TEMP){
            rx_sym.msgType = TEMP;
        }
        else if(type == VOLT){
            rx_sym.msgType = VOLT;
        }
        else if(type == AMP){
            rx_sym.msgType = AMP;
        }
        else if(type == DEBUG1){
            rx_sym.msgType = DEBUG1;
        }
        else if(type == DEBUG2){
            rx_sym.msgType = DEBUG2;
        }
        else if(type == DEBUG3){
            rx_sym.msgType = DEBUG3;
        }
        else if(type == EVENT){
            rx_sym.msgType = EVENT;
        }

        if(rx_sym.msgType == RPM)
            rx_sym.data = (rx_sym.data_bits & 0x1FF0) >> 4;
        else
            rx_sym.data = (rx_sym.data_bits & 0x0FF0) >> 4;
    }
    
    if(rx_sym.msgType == RPM){
        //printf("rpm: %u\n", rx_sym.data);
    }
    else if(rx_sym.msgType == TEMP){
         printf("temp: %u\n", rx_sym.data);
    }
    else if(rx_sym.msgType == VOLT){
         printf("voltage: %f\n", static_cast<float>(rx_sym.data * 0.25));
    }
    else if(rx_sym.msgType == AMP){
         printf("AMP: %f\n", rx_sym.data / 100.0);
    }
    else if(rx_sym.msgType == DEBUG1){
        rx_sym.msgType = DEBUG1;
        printf("debug1: %u\n", rx_sym.data);
    }
    else if(rx_sym.msgType == DEBUG2){
        printf("debug2: %u\n", rx_sym.data);
    }
    else if(rx_sym.msgType == DEBUG3){
        printf("debug3: %u\n", rx_sym.data);
    }
    else if(rx_sym.msgType == EVENT){
        printf("event: %u\n", rx_sym.data);
    }
    
    return 0;
}


int Dshot600::process_erpm_data(rx_symbol_t& rx_sym) {

    if(decode_timings_to_gcr(rx_sym) == -1)
        return -1;

    uint32_t gcr = rx_sym.gcr_data & 0x1FFFFF;
    rx_sym.gcr_data = gcr ^ (gcr >> 1);

    if(extract_nibbles(rx_sym) == -1)
        return -1;

    if(decode_msg(rx_sym) == -1)
        return -1;
    
    return 0;
}

esp_err_t Dshot600::set_extended_telemetry(bool enable){

    Dshot::DshotMessage msg{};

     for(int i=0;i<Dshot::maxChannels;i++){
            msg.msgType = COMMAND;
            msg.loops[i] = 0;
            msg.cmd[i] = enable? DSHOT_EXTENDED_TELEMETRY_ENABLE : DSHOT_EXTENDED_TELEMETRY_DISABLE;
    }

    write_command(msg);
    
    return ESP_OK;
}


