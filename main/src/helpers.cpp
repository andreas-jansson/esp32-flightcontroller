#include <stdint.h>
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




void init_dshot_debug_pin(uint8_t pin)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << pin;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

}
