
#include <chrono>
#include <cstring>
#include <algorithm>

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



static constexpr gpio_num_t DSHOT_DEBUG_PIN = GPIO_NUM_25; // or any free pin

SemaphoreHandle_t s_newData = nullptr;

#include "esp_cpu.h"
#include "esp_private/esp_clk.h"
#include "esp_attr.h"

static inline void IRAM_ATTR delay_ns(uint32_t ns)
{
    // CPU frequency in Hz (e.g. 240000000)
    uint32_t cpu_hz = esp_clk_cpu_freq();
    // Convert ns -> cycles: cycles = cpu_hz * ns / 1e9
    uint32_t cycles = (uint32_t)(((uint64_t)cpu_hz * ns) / 1000000000ULL);

    uint32_t start = esp_cpu_get_cycle_count();
    while ((uint32_t)(esp_cpu_get_cycle_count() - start) < cycles) {
        __asm__ __volatile__("nop");
    }
}


void foo(){
    xSemaphoreTake( s_newData, portMAX_DELAY);
    gpio_set_level(DSHOT_DEBUG_PIN, 0);
    //ets_delay_us(1);
    delay_ns(2);
    gpio_set_level(DSHOT_DEBUG_PIN, 1);
    //ets_delay_us(1);
    delay_ns(2);
    gpio_set_level(DSHOT_DEBUG_PIN, 0);
    //ets_delay_us(1);
    delay_ns(2);
    gpio_set_level(DSHOT_DEBUG_PIN, 1);
}


void init_dshot_debug_pin()
{
    vSemaphoreCreateBinary(s_newData);
    xSemaphoreTake( s_newData, portMAX_DELAY);
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << DSHOT_DEBUG_PIN;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(DSHOT_DEBUG_PIN, 1);
}


#include <stdio.h>
#include "driver/gpio.h"
#include "hal/gpio_hal.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"        // BIT() etc

// Your existing helper
static uint32_t get_iomux_reg(gpio_num_t gpio_num)
{
    switch (gpio_num) {
        case 0:  return IO_MUX_GPIO0_REG;
        case 1:  return IO_MUX_GPIO1_REG;
        case 2:  return IO_MUX_GPIO2_REG;
        case 3:  return IO_MUX_GPIO3_REG;
        case 4:  return IO_MUX_GPIO4_REG;
        case 5:  return IO_MUX_GPIO5_REG;
        case 6:  return IO_MUX_GPIO6_REG;
        case 7:  return IO_MUX_GPIO7_REG;
        case 8:  return IO_MUX_GPIO8_REG;
        case 9:  return IO_MUX_GPIO9_REG;
        case 10: return IO_MUX_GPIO10_REG;
        case 11: return IO_MUX_GPIO11_REG;
        case 12: return IO_MUX_GPIO12_REG;
        case 13: return IO_MUX_GPIO13_REG;
        case 14: return IO_MUX_GPIO14_REG;
        case 15: return IO_MUX_GPIO15_REG;
        case 16: return IO_MUX_GPIO16_REG;
        case 17: return IO_MUX_GPIO17_REG;
        case 18: return IO_MUX_GPIO18_REG;
        case 19: return IO_MUX_GPIO19_REG;
        case 21: return IO_MUX_GPIO21_REG;
        case 22: return IO_MUX_GPIO22_REG;
        case 23: return IO_MUX_GPIO23_REG;
        case 25: return IO_MUX_GPIO25_REG;
        case 26: return IO_MUX_GPIO26_REG;
        case 27: return IO_MUX_GPIO27_REG;
        case 32: return IO_MUX_GPIO32_REG;
        case 33: return IO_MUX_GPIO33_REG;
        case 34: return IO_MUX_GPIO34_REG;
        case 35: return IO_MUX_GPIO35_REG;
        case 36: return IO_MUX_GPIO36_REG;
        case 37: return IO_MUX_GPIO37_REG;
        case 38: return IO_MUX_GPIO38_REG;
        case 39: return IO_MUX_GPIO39_REG;
        default: return 0;
    }
}

static const char* drive_cap_str(gpio_drive_cap_t cap)
{
    switch (cap) {
        case GPIO_DRIVE_CAP_0: return "0 (weakest)";
        case GPIO_DRIVE_CAP_1: return "1";
        case GPIO_DRIVE_CAP_2: return "2";
        case GPIO_DRIVE_CAP_3: return "3 (strongest)";
        default:               return "unknown";
    }
}

// Read RTC_IO pull config for the RTC/touch pins we care about
static bool get_rtc_pull(gpio_num_t gpio, bool *pullup, bool *pulldown)
{
    uint32_t reg_val = 0;

    switch (gpio) {
    case GPIO_NUM_2:   // TOUCH_PAD2
        reg_val   = REG_READ(RTC_IO_TOUCH_PAD2_REG);
        *pullup   = (reg_val & RTC_IO_TOUCH_PAD2_RUE) != 0;
        *pulldown = (reg_val & RTC_IO_TOUCH_PAD2_RDE) != 0;
        return true;

    case GPIO_NUM_15:  // TOUCH_PAD3
        reg_val   = REG_READ(RTC_IO_TOUCH_PAD3_REG);
        *pullup   = (reg_val & RTC_IO_TOUCH_PAD3_RUE) != 0;
        *pulldown = (reg_val & RTC_IO_TOUCH_PAD3_RDE) != 0;
        return true;

    case GPIO_NUM_13:  // TOUCH_PAD4
        reg_val   = REG_READ(RTC_IO_TOUCH_PAD4_REG);
        *pullup   = (reg_val & RTC_IO_TOUCH_PAD4_RUE) != 0;
        *pulldown = (reg_val & RTC_IO_TOUCH_PAD4_RDE) != 0;
        return true;

    case GPIO_NUM_12:  // TOUCH_PAD5
        reg_val   = REG_READ(RTC_IO_TOUCH_PAD5_REG);
        *pullup   = (reg_val & RTC_IO_TOUCH_PAD5_RUE) != 0;
        *pulldown = (reg_val & RTC_IO_TOUCH_PAD5_RDE) != 0;
        return true;

    default:
        return false;  // not one of the special RTC/touch pads we handle here
    }
}

void my_gpio_dump(gpio_num_t gpio)
{
    printf("==== GPIO %d ====\n", gpio);

    // Logic level
    printf("Level            : %d\n", gpio_get_level(gpio));

    // Drive strength
    gpio_drive_cap_t cap;
    if (gpio_get_drive_capability(gpio, &cap) == ESP_OK) {
        printf("Drive strength   : %d (%s)\n", cap, drive_cap_str(cap));
    }

    gpio_dev_t *hw = GPIO_LL_GET_HW(GPIO_PORT_0);

    // Output enable (fix union-& issue by using .val)
    bool out_en;
    if (gpio < 32) {
        out_en = (hw->enable & BIT(gpio)) != 0;
    } else {
        out_en = (hw->enable1.val & BIT(gpio - 32)) != 0;
    }
    printf("Output enabled   : %s\n", out_en ? "yes" : "no");

    // Open drain
    printf("Open-drain       : %s\n",
           hw->pin[gpio].pad_driver ? "yes" : "no");

    // ---- Pull-up / Pull-down & input enable ----
    bool pullup = false;
    bool pulldown = false;
    bool input_en = false;

    // First try RTC_IO for the special RTC/touch pins
    if (get_rtc_pull(gpio, &pullup, &pulldown)) {
        // Input enable for these: FUN_IE still reflects digital input enable
        uint32_t iomux_reg = get_iomux_reg(gpio);
        if (iomux_reg) {
            uint32_t iomux_val = REG_READ(iomux_reg);
            input_en = (iomux_val & FUN_IE) != 0;
        }
    } else {
        // Normal GPIOs: use IO_MUX FUN_PU/FUN_PD/FUN_IE
        uint32_t iomux_reg = get_iomux_reg(gpio);
        if (iomux_reg) {
            uint32_t iomux_val = REG_READ(iomux_reg);
            pullup   = (iomux_val & FUN_PU) != 0;
            pulldown = (iomux_val & FUN_PD) != 0;
            input_en = (iomux_val & FUN_IE) != 0;
        }
    }

    printf("Input enabled    : %s\n", input_en ? "yes" : "no");

    const char *pull_str = "none";
    if (pullup && !pulldown)      pull_str = "pull-up";
    else if (!pullup && pulldown) pull_str = "pull-down";
    else if (pullup && pulldown)  pull_str = "both? (invalid)";

    printf("Pull mode        : %s\n", pull_str);

    // ---- Function select / GPIO matrix ----
    uint32_t iomux_reg = get_iomux_reg(gpio);
    if (iomux_reg) {
        uint32_t iomux_val = REG_READ(iomux_reg);
        uint32_t func = (iomux_val & MCU_SEL_M) >> MCU_SEL_S;
        printf("Function         : %lu\n", (unsigned long)func);
    } else {
        printf("Function         : (no IO_MUX)\n");
    }

    printf("FUNC_OUT_SEL_CFG : 0x%08lx\n",
           (unsigned long)GPIO.func_out_sel_cfg[gpio].val);

    printf("=================\n\n");
}


#define DSHOT_150 150 
#define DSHOT_300 300 
#define DSHOT_600 600 
#define DSHOT_SPEED DSHOT_600 

#define DSHOT_ESC_RESOLUTION_HZ 80000000 
static const char *TAG = "dshot_encoder";

using namespace Dshot;

Dshot600* Dshot600::dshot = nullptr;
bool Dshot600::m_isBidi{};



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


rmt_symbol_word_t raw_symbols[64];
/*          rmt RX          */

static bool IRAM_ATTR example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx){

    cb_dshot_ctx* ctx = reinterpret_cast<cb_dshot_ctx*>(user_ctx);
    gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), ctx->pin);

    return  pdTRUE;
}


static bool IRAM_ATTR tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    cb_dshot_ctx* ctx = reinterpret_cast<cb_dshot_ctx*>(user_ctx);
    xSemaphoreGive(s_newData);
    //gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), ctx->pin);
    //rmt_receive(ctx->rx_handle, raw_symbols, 64, const_cast<rmt_receive_config_t*>(&ctx->rx_chan_config));
	return true; 
}



/********* RX END ***********/




/*      My logic        */

Dshot600::Dshot600(gpio_num_t motorPin[Dshot::maxChannels], bool isBidi){
    rmt_tx_channel_config_t tx_chan_config{};
    rmt_rx_channel_config_t rx_chan_config{};

    tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;               // select source clock
    tx_chan_config.mem_block_symbols = 64;                     // memory block size, 64 * 4 = 256Bytes
    tx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;     // 1MHz tick resolution, i.e. 1 tick = 1us
    tx_chan_config.trans_queue_depth = 2;                       // set the number of transactions that can pend in the background
   
    if(isBidi)
    {
        tx_chan_config.flags.invert_out = 1;
        tx_chan_config.flags.io_od_mode = 1;    
    }

    rx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_chan_config.mem_block_symbols = 64;
    rx_chan_config.resolution_hz = DSHOT_ESC_RESOLUTION_HZ;
    rx_chan_config.flags.invert_in = 1;

    uint32_t baud_rate{};
    switch(DSHOT_SPEED){
        case DSHOT_600:
        baud_rate = DSHOT_600 * 1000;
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

    for(int i=0;i<Dshot::maxChannels;i++){
        gpio_ll_output_disable(GPIO_LL_GET_HW(GPIO_PORT_0), motorPin[i]);
        //gpio_ll_input_enable(GPIO_LL_GET_HW(GPIO_PORT_0), motorPin[i]);
        ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_pull_mode(motorPin[i], GPIO_PULLUP_ONLY));
    }


    for(int i=0;i<Dshot::maxChannels;i++){
        tx_chan_config.gpio_num = motorPin[i];
        rx_chan_config.gpio_num = motorPin[i];
        m_gpioMotorPin[i]= motorPin[i];

        printf("motor chan %d: %p pin %d\n", i, this->esc_motor_chan[i], tx_chan_config.gpio_num);
        // start rx before to not mess upp gpio func
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_rx_channel(&rx_chan_config, &this->rmt_rx_handle[i]));
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_new_tx_channel(&tx_chan_config, &this->esc_motor_chan[i]));
        printf("motor chan %d: %p pin %d\n", i, this->esc_motor_chan[i], tx_chan_config.gpio_num);

    }

    init_dshot_debug_pin();

    rmt_rx_event_callbacks_t rx_callback = 
    {
        .on_recv_done = example_rmt_rx_done_callback
    };
    rmt_tx_event_callbacks_t tx_callback = 
    {
        .on_trans_done = tx_done_callback
    };

    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 	uint32_t(200),
        .signal_range_max_ns = uint32_t(3000),
    };

    for(int i=0;i<Dshot::maxChannels;i++){

        cb_dshot_ctx* ctx = reinterpret_cast<cb_dshot_ctx*>(calloc(1, sizeof(cb_dshot_ctx)));

        ctx->tx_handle = esc_motor_chan[i];
        ctx->rx_handle = rmt_rx_handle[i];
        ctx->pin = m_gpioMotorPin[i];
        ctx->motor = static_cast<motor_type_t>(i);
        ctx->rx_chan_config = rx_config;

        m_ctx[i].tx_handle = esc_motor_chan[i];
        m_ctx[i].rx_handle = rmt_rx_handle[i];
        m_ctx[i].pin = m_gpioMotorPin[i];
        m_ctx[i].motor = static_cast<motor_type_t>(i);
        m_ctx[i].rx_chan_config = rx_config;

        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_rx_register_event_callbacks(rmt_rx_handle[i], &rx_callback, ctx));
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_register_event_callbacks(esc_motor_chan[i], &tx_callback, ctx));
    }

    for(int i=0;i<Dshot::maxChannels;i++){
    
        // start rx before to not mess upp gpio func
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->rmt_rx_handle[i]));
        ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_enable(this->esc_motor_chan[i]));
        gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), motorPin[i]);
        printf("************** setup ***************\n");
        my_gpio_dump(m_gpioMotorPin[i]);
        printf("************************************\n");

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


double average(int64_t a[], uint64_t n)
{
    // Find sum of array element
    int64_t sum = 0;
    for (int i = 0; i < n; i++)
        sum += a[i];

    return (double)sum / n;
}


esp_err_t Dshot600::write_speed(struct Dshot::DshotMessage& msg){
    using namespace std::chrono_literals;

    esp_err_t status = 0;
    dshot_esc_throttle_t throttle[Dshot::maxChannels]{};
    rmt_transmit_config_t tx_config[Dshot::maxChannels]{};
    std::chrono::_V2::steady_clock::time_point rx_ready_us[Dshot::maxChannels]{};

    for(int i=0;i<Dshot::maxChannels;i++){
        throttle[i] = {
            .throttle = msg.speed[i],
            .telemetry_req = true,//msg.telemetryReq[i], 
        };

        tx_config[i].loop_count = msg.loops[i] == -1? -1 : 0;
        //tx_config[i].flags.eot_level = m_isBidi? 1 : 0;
    }



    const uint64_t bm_max{10000};
    static uint64_t bm_cntr{};
    static std::chrono::_V2::steady_clock::time_point bm_start{};
    static std::chrono::_V2::steady_clock::time_point bm_end{};
    static int64_t bm_elapsed[10000]{};

    //if(bm_cntr < bm_max){
    //      bm_start = std::chrono::steady_clock::now();
    //
    //      bm_end = std::chrono::steady_clock::now();
    //      bm_elapsed[bm_cntr] = std::chrono::duration_cast<std::chrono::microseconds>(bm_end - bm_start).count(); 
    //      bm_cntr++;
    //}
    //else{
    //    
    //}



    if(m_isBidi){

        uint8_t rx_done{};
        bool rx_done_complete[Dshot::maxChannels]{};

        for(int i=0;i<Dshot::maxChannels;i++){
            gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);

            auto tx_called = std::chrono::steady_clock::now();
            // median 11us avg 10us min 6us max 122us (first)
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i]));
            auto tx_done =  std::chrono::steady_clock::now();
            rx_ready_us[i] = std::chrono::steady_clock::now() + (65us - (tx_done - tx_called));      
            foo();         
            //auto delay = std::chrono::duration_cast<std::chrono::microseconds>(65us - (tx_done - tx_called)).count();
            //if(delay > 0)
            //{
            //    ets_delay_us(static_cast<uint32_t>(delay));
            //    gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[i]);
            //}

            /*  trigger rx when time window is correct instead of busy waiting */
            /*
            for(uint8_t j=rx_done;j<Dshot::maxChannels;j++){
                auto now = std::chrono::steady_clock::now();
                if(now >= rx_ready_us[j] && now <= rx_ready_us[j] + 40us){
                    rx_done = j + 1;
                }
                else if(now < rx_ready_us[j] && i == Dshot::maxChannels - 1){ //if last motor, wait for window
                    auto delay = std::chrono::duration_cast<std::chrono::microseconds>(rx_ready_us[j] - now).count();
                    if(delay>0)
                        ets_delay_us(delay);
                }
                else{
                    break;
                }

                gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), m_gpioMotorPin[j]);
                // median 5us avg 5s min 4us max 183us (first)
                rmt_receive(m_ctx[j].rx_handle, raw_symbols, 64, const_cast<rmt_receive_config_t*>(&m_ctx[j].rx_chan_config));

                rx_done_complete[j] = true;
            }
            */
            
        }

        for(int i=0;i<Dshot::maxChannels;i++){
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(20)));
        }
        ets_delay_us(100);

        if(bm_cntr == bm_max){
            uint64_t n = sizeof(bm_elapsed) / sizeof(bm_elapsed[0]);
            std::sort(bm_elapsed, bm_elapsed + static_cast<int64_t>(n));
            double avg = average(bm_elapsed, static_cast<int64_t>(n));

            printf("**** all results ****\n");
            for(uint64_t i=0;i<bm_max;i++){
                printf("[%llu]    us: %-3lld\n", i, bm_elapsed[i]);
            }

            printf("****** stats ******\n");
            printf("avg    us: %f\n", avg);
            printf("median us: %lld\n", bm_elapsed[(bm_max-1) / 2]);
            printf("min    us: %lld\n", bm_elapsed[0]);
            printf("max    us: %lld\n", bm_elapsed[bm_max-1]);

            bm_cntr = bm_max + 100; // to prevent multiple prints
        }

    }
    else{
        for(int i=0;i<Dshot::maxChannels;i++){
                print_debug(DEBUG_DSHOT, DEBUG_DATA, "m%d throttle: %u telemetry: %d channel: 0x%x", i, throttle[i].throttle, throttle[i].telemetry_req, this->esc_motor_chan[i]);
                ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_transmit(this->esc_motor_chan[i], this->dshot_encoder, &throttle[i], sizeof(throttle[i]), &tx_config[i])); 
                //ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(1)));
        }


        for(int i=0;i<Dshot::maxChannels;i++){
            ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_tx_wait_all_done(this->esc_motor_chan[i], pdMS_TO_TICKS(20)));
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
    uint16_t val = frame->val;
    uint8_t crc = (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0F;
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
    rmt_dshot_esc_encoder_t *dshot_encoder =
        __containerof(encoder, rmt_dshot_esc_encoder_t, base);

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
        // fallthrough intentional

    case 1:
        // >>>>>>>>>> THIS IS THE MAGIC <<<<<<<<<<<<<<
        // Switch pin to input/OD exactly when delay symbol encoding begins
        if (m_isBidi) {
            //gpio_ll_output_disable(GPIO_LL_GET_HW(GPIO_PORT_0), dshot_encoder->pin);
            //gpio_ll_input_enable (GPIO_LL_GET_HW(GPIO_PORT_0), dshot_encoder->pin);
            gpio_ll_od_enable    (GPIO_LL_GET_HW(GPIO_PORT_0), dshot_encoder->pin);
        }

        // Encode the "delay" symbol
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


size_t Dshot600::rmt_encode_dshot_esc_old(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
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





/*********************** Bit Banger **********************************/

