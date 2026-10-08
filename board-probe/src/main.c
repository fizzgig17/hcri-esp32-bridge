// Board probe: tells a LilyGo T-Display-S3 Pro from a plain T-Display S3.
//
// The Pro has a SY6970 power chip (I2C address 0x6A) on GPIO5 (SDA) / GPIO6 (SCL). On the plain
// T-Display S3 those pins are the screen's reset and chip-select lines and nothing answers there.
// Prints "HCRI-PROBE board=pro|not-pro ..." once a second for a minute; the flasher reads it.
// Nothing is written to the board's storage and the screen is never touched.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define SDA_PIN 5
#define SCL_PIN 6

static int acks(uint8_t addr) {
    uint8_t b;
    return i2c_master_read_from_device(I2C_NUM_0, addr, &b, 1, pdMS_TO_TICKS(50)) == ESP_OK;
}

void app_main(void) {
    i2c_config_t c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &c);
    i2c_driver_install(I2C_NUM_0, c.mode, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    for (int i = 0; i < 60; i++) {
        int pmu = acks(0x6A);    // SY6970 power chip
        int touch = acks(0x5A);  // CST226SE touch controller
        int light = acks(0x23);  // LTR-553 light sensor
        printf("HCRI-PROBE board=%s pmu=%d touch=%d light=%d\n", pmu ? "pro" : "not-pro", pmu, touch, light);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
