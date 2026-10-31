// Torch Bearer scanner for the LilyGo T-Display S3 and the T-Display-S3 Pro.
//
// One source, two builds: the default build is the plain T-Display S3; the
// `tdisplay-s3-pro` PlatformIO environment defines BOARD_PRO and targets the Pro
// (SPI ST7796 screen, SY6970 power chip) and also shows the battery percentage.
//
// Press the right-hand button (GPIO14; GPIO12 on the Pro) or BOOT (GPIO0) to take one scan.
// The ESP32-S3 acts as a USB host and talks to the spectrometer's CH340
// USB-to-serial bridge (VID 1A86 / PID 7523) at 115200 8N1.
//
// Protocol and spectrum de-obfuscation are ported from the GPLv3 projects
// ZoidTechnology/Torch-Bearer-Tools and wejn/tobes-ui, so this firmware is
// GPLv3 as well.

#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#ifdef BOARD_PRO
#include "driver/spi_master.h"
#include "driver/i2c.h"
#endif

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp.hpp"

#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "mbedtls/sha256.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "metrics.h"

using namespace esp_usb;

static const char *TAG = "tobes";

// The git commit this firmware was built from (tools/fw_version.py passes -DFW_GIT=<commit>), shown on the idle screens.
#ifndef FW_GIT
#define FW_GIT gdev   // the leading g is dropped below: a bare number like 0465550 would lose its zero
#endif
#define FW_STR2(x) #x
#define FW_STR(x) FW_STR2(x)
static const char *FW_VERSION = FW_STR(FW_GIT) + 1;

// True while the phone is sending a new firmware (BLE OTA, see "Firmware update" below). The idle-loop screen updates, the
// buttons and the sleep timer stand still meanwhile, so nothing draws over the progress screen or puts the board to sleep.
static volatile bool ota_active = false;
static volatile bool ota_redraw = false;   // an update ended without a restart: put the normal screen back
static volatile int64_t ota_redraw_at = 0; // ... but not before this time (esp_timer us), so a failure message can be read

// ---------------------------------------------------------------- pins ----
#ifdef BOARD_PRO
// LilyGo T-Display-S3 Pro: ST7796 on SPI, SY6970 power chip on I2C.
#define PIN_LCD_BL    48
#define PIN_LCD_CS    39
#define PIN_LCD_DC    9
#define PIN_LCD_RST   47
#define PIN_SPI_SCK   18
#define PIN_SPI_MOSI  17
#define PIN_SD_CS     14   // shares the SPI bus; keep it deselected
#define PIN_I2C_SDA   5
#define PIN_I2C_SCL   6
#define PIN_BTN_A     0    // the two buttons seen on the Pro: GPIO 0 is the upper one with the screen unflipped...
#define PIN_BTN_B     16   // ...and GPIO 16 the lower one
#define PIN_BTN_A2    12   // earlier firmware used GPIO 12 as the upper button; still counts as A, in case a board has it
#else
#define PIN_POWER_ON 15
#define PIN_LCD_BL   38
#define PIN_LCD_RD   9
#define PIN_LCD_WR   8
#define PIN_LCD_DC   7
#define PIN_LCD_CS   6
#define PIN_LCD_RST  5
static const int PIN_LCD_D[8] = {39, 40, 41, 42, 45, 46, 47, 48};
#define PIN_BTN_A    0   // BOOT
#define PIN_BTN_B    14
#endif

// ------------------------------------------------------------- display ----
#define W 320
#define H 170

static uint16_t *fb;  // W*H RGB565, byte-swapped for the panel
static esp_lcd_panel_handle_t panel;

// Display orientation. The default is the same way up on both boards (buttons at the bottom-left of the picture);
// "flipped" is that turned 180 degrees. It is chosen from the app (BLE command 0x03) and kept in flash so it survives a power cycle.
static bool nvs_ready = false;
static void nvs_boot() {
    if (nvs_ready) return;
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_ready = true;
}
static bool flip_load() {
    nvs_boot();
    nvs_handle_t h;
    uint8_t v = 0;
    if (nvs_open("hcri", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "flip", &v);
        nvs_close(h);
    }
    return v != 0;
}
static void flip_save(bool f) {
    nvs_boot();
    nvs_handle_t h;
    if (nvs_open("hcri", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "flip", f ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}
static bool flipped = false;                // what the panel is set to now
static volatile int8_t flip_req = -1;       // -1 none, else 0/1 asked for by the phone

static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    return __builtin_bswap16(c);
}
static const uint16_t BLACK = 0, WHITE = rgb(255, 255, 255), GREY = rgb(110, 110, 110),
                      GREEN = rgb(60, 220, 90), RED = rgb(255, 70, 70),
                      YELLOW = rgb(255, 210, 50), CYAN = rgb(70, 200, 255);

#ifdef BOARD_PRO
// The UI is drawn into a 320x170 frame buffer (as on the plain T-Display S3) and shown centred on
// the Pro's 480x222 landscape screen.
#define PANEL_W 480
#define PANEL_H 222
#define PANEL_COL_OFFSET 49   // the 222-pixel-wide glass starts at column 49 of the ST7796's 320
#define FB_X ((PANEL_W - W) / 2)
#define FB_Y ((PANEL_H - H) / 2)
static esp_lcd_panel_io_handle_t lcd_io;

static void st_cmd(uint8_t c, const uint8_t *d = nullptr, size_t n = 0) {
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(lcd_io, c, d, n));
}
static void st_cmd1(uint8_t c, uint8_t v) { st_cmd(c, &v, 1); }

// MADCTL: 0xA8 = landscape with the USB-C end on the right, 0x68 = turned 180 degrees (the glass is centred, so the column offset is the same).
static void apply_flip(bool f) { st_cmd1(0x36, f ? 0x68 : 0xA8); }

// Write a w*h block of RGB565 to the landscape position (x, y).
static void blit(int x, int y, int w, int h, const void *data) {
    uint8_t ca[4] = {(uint8_t)(x >> 8), (uint8_t)x, (uint8_t)((x + w - 1) >> 8), (uint8_t)(x + w - 1)};
    int ys = y + PANEL_COL_OFFSET, ye = y + h - 1 + PANEL_COL_OFFSET;
    uint8_t ra[4] = {(uint8_t)(ys >> 8), (uint8_t)ys, (uint8_t)(ye >> 8), (uint8_t)ye};
    st_cmd(0x2A, ca, 4);
    st_cmd(0x2B, ra, 4);
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(lcd_io, 0x2C, data, (size_t)w * h * 2));
}

static void lcd_init() {
    gpio_config_t o = {};
    o.mode = GPIO_MODE_OUTPUT;
    o.pin_bit_mask = (1ULL << PIN_LCD_BL) | (1ULL << PIN_SD_CS) | (1ULL << PIN_LCD_RST);
    gpio_config(&o);
    gpio_set_level((gpio_num_t)PIN_LCD_BL, 0);
    gpio_set_level((gpio_num_t)PIN_SD_CS, 1);

    spi_bus_config_t bc = {};
    bc.mosi_io_num = PIN_SPI_MOSI;
    bc.miso_io_num = -1;
    bc.sclk_io_num = PIN_SPI_SCK;
    bc.quadwp_io_num = -1;
    bc.quadhd_io_num = -1;
    bc.max_transfer_sz = W * H * 2 + 64;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bc, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t ic = {};
    ic.cs_gpio_num = PIN_LCD_CS;
    ic.dc_gpio_num = PIN_LCD_DC;
    ic.spi_mode = 0;
    ic.pclk_hz = 40 * 1000 * 1000;
    ic.trans_queue_depth = 10;
    ic.lcd_cmd_bits = 8;
    ic.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &ic, &lcd_io));

    // Hardware reset
    gpio_set_level((gpio_num_t)PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level((gpio_num_t)PIN_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level((gpio_num_t)PIN_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // ST7796 start-up sequence (same values as LilyGo's Arduino_GFX driver for this board)
    st_cmd1(0x3A, 0x55);                           // 16-bit colour
    st_cmd1(0xF0, 0xC3); st_cmd1(0xF0, 0x96);      // command set control
    st_cmd1(0xB4, 0x01);
    { const uint8_t d[] = {0x80, 0x22, 0x3B}; st_cmd(0xB6, d, sizeof d); }
    { const uint8_t d[] = {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}; st_cmd(0xE8, d, sizeof d); }
    st_cmd1(0xC1, 0x06); st_cmd1(0xC2, 0xA7); st_cmd1(0xC5, 0x18);
    { const uint8_t d[] = {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B}; st_cmd(0xE0, d, sizeof d); }
    { const uint8_t d[] = {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B}; st_cmd(0xE1, d, sizeof d); }
    st_cmd1(0xF0, 0x3C); st_cmd1(0xF0, 0x69);
    st_cmd(0x11);                                  // sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    st_cmd(0x38);
    st_cmd(0x21);                                  // IPS panel: inversion on
    flipped = flip_load();
    apply_flip(flipped);                           // landscape (MY | MV | BGR), see apply_flip
    st_cmd(0x29);                                  // display on

    fb = (uint16_t *)heap_caps_malloc(W * H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(fb);
    memset(fb, 0, W * H * 2);
    // Blank the whole glass (the frame buffer is zeroed, so reuse it as the source).
    blit(0, 0, W, H, fb);
    blit(W, 0, PANEL_W - W, H, fb);
    blit(0, H, W, PANEL_H - H, fb);
    blit(W, H, PANEL_W - W, PANEL_H - H, fb);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)PIN_LCD_BL, 1);
}

static void present() {
    blit(FB_X, FB_Y, W, H, fb);
    vTaskDelay(pdMS_TO_TICKS(40));  // let the DMA finish before fb is touched again
}
#else
// The default here is the Pro's default turned 180 degrees (this board's old "flipped"), so both boards start the same
// way up: buttons at the bottom-left of the picture. Flipping swaps both mirrors.
static void apply_flip(bool f) { esp_lcd_panel_mirror(panel, f, !f); }

static void lcd_init() {
    gpio_config_t o = {};
    o.mode = GPIO_MODE_OUTPUT;
    o.pin_bit_mask = (1ULL << PIN_POWER_ON) | (1ULL << PIN_LCD_BL) | (1ULL << PIN_LCD_RD);
    gpio_config(&o);
    gpio_set_level((gpio_num_t)PIN_POWER_ON, 1);
    gpio_set_level((gpio_num_t)PIN_LCD_RD, 1);
    gpio_set_level((gpio_num_t)PIN_LCD_BL, 0);

    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bc = {};
    bc.dc_gpio_num = PIN_LCD_DC;
    bc.wr_gpio_num = PIN_LCD_WR;
    bc.clk_src = LCD_CLK_SRC_DEFAULT;
    for (int i = 0; i < 8; i++) bc.data_gpio_nums[i] = PIN_LCD_D[i];
    bc.bus_width = 8;
    bc.max_transfer_bytes = W * H * 2;
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bc, &bus));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_i80_config_t ic = {};
    ic.cs_gpio_num = PIN_LCD_CS;
    ic.pclk_hz = 20 * 1000 * 1000;
    ic.trans_queue_depth = 10;
    ic.dc_levels.dc_idle_level = 0;
    ic.dc_levels.dc_cmd_level = 0;
    ic.dc_levels.dc_dummy_level = 0;
    ic.dc_levels.dc_data_level = 1;
    ic.lcd_cmd_bits = 8;
    ic.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus, &ic, &io));

    esp_lcd_panel_dev_config_t pc = {};
    pc.reset_gpio_num = PIN_LCD_RST;
    pc.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    pc.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &pc, &panel));

    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, true);
    esp_lcd_panel_swap_xy(panel, true);
    // Default: the text reads upright with the USB-C end on the RIGHT, so the buttons sit at the right edge (the READY
    // arrow points up and to the right at the top one). The 35-pixel gap is the same either way (the 170-pixel-high
    // glass sits centred in the controller's 240 rows).
    flipped = flip_load();
    apply_flip(flipped);
    esp_lcd_panel_set_gap(panel, 0, 35);
    esp_lcd_panel_disp_on_off(panel, true);

    fb = (uint16_t *)heap_caps_malloc(W * H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    assert(fb);
    memset(fb, 0, W * H * 2);
    esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, fb);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)PIN_LCD_BL, 1);
}

static void present() {
    esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, fb);
    vTaskDelay(pdMS_TO_TICKS(40));  // let the DMA finish before fb is touched again
}

#endif

static int shown = 0;      // which screen is on show: 1 READY, 2 result (both carry the button arrow); 0 any other
static bool result_ok = true;  // the result screen's "ok", to redraw it after a flip
static bool waiting_shown = false;  // the NO USB screen is on show
// Torch Bearer screens on the Pro carry a READ arrow (top button) and a LIVE arrow (bottom button) on the button side; while
// this is set the text steps clear of them. clear() drops it, and the Torch Bearer screens raise it again.
static bool tb_ui = false;
// Pro, ColorMunki screens: a READ label by the single button arrow (cleared by clear() like tb_ui).
static bool arrow_label = false;
// What the progress screen says: 0 plain, 1 "PERFORMING READ", 2 "PERFORMING LIVE READ", 3 "SIMULATED READ".
static int prog_mode = 0;
// While a simulated Live read runs, the result screen says so (header and the line under the figures).
static bool sim_live_banner = false;
static inline bool arrow_bottom_left() { return !flipped; }   // (declared early: the app link uses it) both boards now start with the buttons at the bottom-left
static void clear() { memset(fb, 0, W * H * 2); shown = 0; waiting_shown = false; tb_ui = false; arrow_label = false; }

static inline void px(int x, int y, uint16_t c) {
    if (x >= 0 && x < W && y >= 0 && y < H) fb[y * W + x] = c;
}
static void fill(int x, int y, int w, int h, uint16_t c) {
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) px(x + i, y + j, c);
}
static void vline(int x, int y0, int y1, uint16_t c) {
    for (int y = y0; y <= y1; y++) px(x, y, c);
}

// Classic 5x7 font, ASCII 0x20..0x5A (upper case only), column-major, LSB at top.
static const uint8_t FONT[59][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}};

static void text(int x, int y, const char *s, int scale, uint16_t c) {
    for (; *s; s++, x += 6 * scale) {
        int ch = toupper((unsigned char)*s);
        if (ch < 0x20 || ch > 0x5A) ch = '?';
        const uint8_t *g = FONT[ch - 0x20];
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 7; row++)
                if (g[col] & (1 << row)) fill(x + col * scale, y + row * scale, scale, scale, c);
    }
}
static void textf(int x, int y, int scale, uint16_t c, const char *fmt, ...) {
    char b[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    text(x, y, b, scale, c);
}

// ------------------------------------------------------------- battery ----
// Pro only: the SY6970 power chip (I2C 0x6A) reports battery voltage and whether USB power is present.
// Shown in the header as "87%" on battery, or a lightning bolt plus "87%" while USB power is connected.
#ifdef BOARD_PRO
#define SY6970_ADDR 0x6A
#define BATT_X 150   // left edge of the indicator in the header
#define BATT_W 62

struct BattState {
    int pct = -1;        // -1 = no valid reading yet
    bool plugged = false;
};
static BattState batt;

static bool pmu_read(uint8_t reg, uint8_t *v) {
    return i2c_master_write_read_device(I2C_NUM_0, SY6970_ADDR, &reg, 1, v, 1, pdMS_TO_TICKS(50)) == ESP_OK;
}
static bool pmu_write(uint8_t reg, uint8_t v) {
    uint8_t b[2] = {reg, v};
    return i2c_master_write_to_device(I2C_NUM_0, SY6970_ADDR, b, 2, pdMS_TO_TICKS(50)) == ESP_OK;
}

// Battery-powered host: with no outside power on the USB-C port, switch the SY6970 from charging to its
// 5 V boost so the port powers the spectrometer (through a USB-C to USB-A adapter) while the ESP32 reads it.
// While outside power is present the boost is off and the battery charges normally. Compile out with -DPRO_OTG_BOOST=0.
#ifndef PRO_OTG_BOOST
#define PRO_OTG_BOOST 1
#endif
static bool otg_on = false;
// Last power-chip readings, shown on the "NO USB" screen so a dropout can be diagnosed without a laptop.
static int dbg_mv = 0, dbg_stat = -1, dbg_fault = -1, dbg_chrg = -1;
static int unplug_votes = 0;
static void otg_update(bool plugged) {
#if PRO_OTG_BOOST
    uint8_t r03;
    if (!pmu_read(0x03, &r03)) return;
    // Turning the boost OFF cuts the spectrometer's power, so it takes several polls in a row that say
    // "outside power is present" (a single odd status read must not drop the USB device). Turning it ON is immediate.
    unplug_votes = plugged ? unplug_votes + 1 : 0;
    bool want = !(plugged && (unplug_votes >= 3 || !otg_on));
    if (want && !(r03 & 0x20)) {
        pmu_write(0x03, (uint8_t)((r03 | 0x20) & ~0x10));   // OTG_CONFIG on, CHG_CONFIG off
        otg_on = true;
        ESP_LOGW("pmu", "boost ON");
    } else if (!want && (r03 & 0x20)) {
        pmu_write(0x03, (uint8_t)((r03 & ~0x20) | 0x10));   // boost off, charging back on
        otg_on = false;
        ESP_LOGW("pmu", "boost OFF (outside power seen %d polls)", unplug_votes);
    }
#else
    (void)plugged;
#endif
}

static void battery_init() {
    i2c_config_t c = {};
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = PIN_I2C_SDA;
    c.scl_io_num = PIN_I2C_SCL;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &c);
    i2c_driver_install(I2C_NUM_0, c.mode, 0, 0, 0);
    uint8_t r02;
    if (pmu_read(0x02, &r02)) pmu_write(0x02, r02 | 0x40);   // continuous battery-voltage conversion
    uint8_t r0b = 0, r11 = 0;
    bool plugged = pmu_read(0x0B, &r0b) && pmu_read(0x11, &r11) && (r11 & 0x80) && ((r0b >> 5) != 7);
    otg_update(plugged);
    if (otg_on) vTaskDelay(pdMS_TO_TICKS(400));              // let the spectrometer power up before USB starts
}

// Rough single-cell LiPo state of charge from the voltage (4.2 V full). The voltage is the real
// measurement; this is only an estimate for the on-screen percentage.
static int approx_percent(int mv) {
    static const int v[] = {3300, 3500, 3600, 3700, 3750, 3800, 3850, 3900, 3950, 4000, 4100, 4200};
    static const int p[] = {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    if (mv <= v[0]) return 0;
    if (mv >= v[11]) return 100;
    for (int i = 1; i < 12; i++)
        if (mv < v[i]) return p[i - 1] + (p[i] - p[i - 1]) * (mv - v[i - 1]) / (v[i] - v[i - 1]);
    return 100;
}

static bool was_plugged = false;
static bool full_latch = false;
static void was_plugged_reset() { was_plugged = false; full_latch = false; }

// Reads the chip; returns true if the displayed values changed.
static bool battery_poll() {
    uint8_t r02 = 0, r0b = 0, r0e = 0, r11 = 0;
    if (!pmu_read(0x02, &r02) || !pmu_read(0x0B, &r0b) || !pmu_read(0x0E, &r0e) || !pmu_read(0x11, &r11)) return false;
    if (!(r02 & 0x40)) pmu_write(0x02, r02 | 0x40);   // keep the ADC converting continuously
    int mv = 2304 + (r0e & 0x7F) * 20;                // REG0E: battery voltage, 20 mV steps
    if (mv < 2500) return false;                      // ADC hasn't produced a value yet
    uint8_t r0c = 0;
    pmu_read(0x0C, &r0c);                              // REG0C: fault flags (boost overload, battery, thermal)
    dbg_mv = mv; dbg_stat = (r0b >> 5) & 7; dbg_chrg = (r0b >> 3) & 3; dbg_fault = r0c;
    if (r0c) ESP_LOGW("pmu", "fault REG0C=0x%02x stat=%d mv=%d", r0c, dbg_stat, mv);
    bool boosting = ((r0b >> 5) & 7) == 7;            // REG0B VBUS_STAT 7: the chip itself is sourcing 5 V
    bool plugged = (r11 & 0x80) != 0 && !boosting;    // REG11 bit 7: VBUS good (outside power)
    otg_update(plugged);
    int chrg = (r0b >> 3) & 3;                        // REG0B: 0 idle, 1 pre-charge, 2 fast charge, 3 done
    int pct = approx_percent(mv);
    ESP_LOGI("pmu", "mv=%d stat=%d chrg=%d plugged=%d raw=%d%%", mv, dbg_stat, chrg, (int)plugged, pct);
    if (plugged) {
        // The voltage is not a clean battery reading while the charger is running (it jumps with the charge current),
        // so show the middle of the last 7 samples (about 35 s), and 100% only once "charge done" has been
        // reported on 3 polls in a row. The bolt shows that it is charging; the number only moves slowly.
        static int win[7], wn = 0, done_votes = 0;
        if (!was_plugged) { wn = 0; done_votes = 0; full_latch = false; }       // just plugged in: start a fresh window
        was_plugged = true;
        win[wn++ % 7] = pct;
        if (wn < 5) return false;                           // too few samples to trust yet: keep the last value
        int n = wn < 7 ? wn : 7, tmp[7];
        for (int i = 0; i < n; i++) tmp[i] = win[i];
        for (int i = 1; i < n; i++) for (int j = i; j > 0 && tmp[j] < tmp[j - 1]; j--) { int t = tmp[j]; tmp[j] = tmp[j - 1]; tmp[j - 1] = t; }
        // Near the top the charger flips between "fast charge" and "done" every few seconds and the voltage
        // follows it, so "done" is not a steady signal. Count a poll as full when the chip says done OR the
        // voltage is at the charge target; latch 100% after 3 such polls in a row, and keep it until the
        // middle of the recent readings falls well below full (a real discharge) or the charger is removed.
        bool near_full = (chrg == 3) || mv >= 4190;
        done_votes = near_full ? done_votes + 1 : 0;
        if (done_votes >= 3) full_latch = true;
        if (tmp[n / 2] < 85) full_latch = false;
        pct = full_latch ? 100 : (tmp[n / 2] > 99 ? 99 : tmp[n / 2]);
        if (wn > 1000) wn = 7;
    }
    if (!plugged) was_plugged_reset();
    bool changed = pct != batt.pct || plugged != batt.plugged;
    batt.pct = pct;
    batt.plugged = plugged;
    return changed;
}

// Filled lightning bolt, about 10 x 14 pixels, top-left at (x, y).
static void draw_bolt(int x, int y, uint16_t c) {
    static const int P[6][2] = {{6, 0}, {1, 7}, {4, 7}, {2, 13}, {9, 5}, {6, 5}};
    for (int j = 0; j < 14; j++) {
        float yy = j + 0.5f;
        float xs[6];
        int n = 0;
        for (int i = 0; i < 6; i++) {
            const int *a = P[i], *b = P[(i + 1) % 6];
            if ((a[1] <= yy) != (b[1] <= yy)) xs[n++] = a[0] + (yy - a[1]) * (b[0] - a[0]) / (float)(b[1] - a[1]);
        }
        for (int i = 0; i < n; i++)
            for (int k = i + 1; k < n; k++)
                if (xs[k] < xs[i]) { float t = xs[i]; xs[i] = xs[k]; xs[k] = t; }
        for (int i = 0; i + 1 < n; i += 2)
            for (int xx = (int)ceilf(xs[i] - 0.5f); xx < (int)ceilf(xs[i + 1] - 0.5f); xx++) px(x + xx, y + j, c);
    }
}

static void draw_battery() {
    fill(BATT_X, 3, BATT_W, 16, BLACK);
    if (batt.pct < 0) return;
    uint16_t col = batt.pct < 15 && !batt.plugged ? RED : WHITE;
    char b[16];
    snprintf(b, sizeof b, "%d%%", batt.pct);
    int w = (int)strlen(b) * 12;
    int x = BATT_X + BATT_W - w;                      // right-aligned
    if (batt.plugged) draw_bolt(x - 13, 4, YELLOW);
    text(x, 4, b, 2, col);
}

// Power-chip state line on the idle screens (NO USB / READY): input status, charge state, fault flags, battery mV.
static bool diag_on = false;
static void draw_diag() {
    // Stops above the flipped button arrow, and clear of the LIVE label on the button side of the Torch Bearer screens.
    const bool lbl = tb_ui || arrow_label;
    const int x0 = (lbl && arrow_bottom_left()) ? 28 : 0, x1 = (lbl && !arrow_bottom_left()) ? W - 28 : W;
    fill(x0, 134, x1 - x0, 19, BLACK);
    if (dbg_stat < 0) return;
    char d[40];
    snprintf(d, sizeof d, "S%d C%d F%02X %dmV", dbg_stat, dbg_chrg, dbg_fault, dbg_mv);
    text(x0 + 20, 140, d, 2, GREY);
}

// Called from the idle loops: refresh the indicator (at most every 5 s) and redraw if it changed.
static void battery_tick() {
    if (ota_active) return;
    static int64_t last = -5000000;
    int64_t now = esp_timer_get_time();
    if (now - last < 5000000) return;
    last = now;
    bool ch = battery_poll();
    if (ch) draw_battery();
    if (diag_on) draw_diag();
    if (ch || diag_on) present();
}
#else
static void battery_init() {}
static void draw_battery() {}
static void battery_tick() {}
static bool diag_on = false;
static void draw_diag() {}
#endif

// ----------------------------------------------------------- USB serial ----
static std::unique_ptr<CdcAcmDevice> vcp;
static volatile bool usb_connected = false;
static volatile bool usb_lost = false;
static StreamBufferHandle_t rx_stream;

static bool on_rx(const uint8_t *data, size_t len, void *) {
    xStreamBufferSend(rx_stream, data, len, 0);
    return true;
}
static void on_event(const cdc_acm_host_dev_event_data_t *ev, void *) {
    if (ev->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
        ESP_LOGW(TAG, "spectrometer unplugged");
        usb_lost = true;
    }
}
static void usb_lib_task(void *) {
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}
static void usb_start() {
    rx_stream = xStreamBufferCreate(8192, 1);
    usb_host_config_t hc = {};
    hc.skip_phy_setup = false;
    hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&hc));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    VCP::register_driver<CH34x>();
}
static bool usb_try_open() {
    cdc_acm_host_device_config_t dc = {};
    dc.connection_timeout_ms = 1000;
    dc.out_buffer_size = 512;
    dc.in_buffer_size = 512;
    dc.event_cb = on_event;
    dc.data_cb = on_rx;
    dc.user_arg = NULL;
    vcp.reset(VCP::open(&dc));
    if (!vcp) return false;
    cdc_acm_line_coding_t lc = {};
    lc.dwDTERate = 115200;
    lc.bCharFormat = 0;
    lc.bParityType = 0;
    lc.bDataBits = 8;
    if (vcp->line_coding_set(&lc) != ESP_OK) ESP_LOGW(TAG, "line_coding_set failed");
    vcp->set_control_line_state(true, true);  // DTR + RTS, as pyserial does
    xStreamBufferReset(rx_stream);
    usb_lost = false;
    return true;
}

// ----------------------------------------------------- Torch Bearer proto ----
enum : uint8_t { T_STOP = 0x04, T_ID = 0x08, T_RANGE = 0x0F, T_DATA = 0x33 };
#define NPTS_MAX 700

struct Scan {
    uint8_t status;      // 0 normal, 1 over, 2 under
    float exposure_ms;
    int npts;
    int start_nm;
    float spd[NPTS_MAX];
};
static Scan scan;
static char device_id[40];
static int range_lo = 340, range_hi = 1000;

static bool send_frame(uint8_t type, const uint8_t *payload = NULL, size_t plen = 0) {
    uint8_t b[32];
    size_t n = 9 + plen;
    b[0] = 0xCC; b[1] = 0x01;
    b[2] = n & 0xFF; b[3] = (n >> 8) & 0xFF; b[4] = (n >> 16) & 0xFF;
    b[5] = type;
    if (plen) memcpy(b + 6, payload, plen);
    uint8_t sum = 0;
    for (size_t i = 0; i < 6 + plen; i++) sum += b[i];
    b[6 + plen] = sum;
    b[7 + plen] = 0x0D; b[8 + plen] = 0x0A;
    return vcp && vcp->tx_blocking(b, n, 1000) == ESP_OK;
}

static uint8_t acc[2048];
static size_t acc_n = 0;

// Read one device->host frame. Returns payload length, or -1 on timeout.
static int read_frame(uint8_t *type, uint8_t *payload, size_t cap, int timeout_ms) {
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (true) {
        // try to parse what we have
        while (acc_n >= 5) {
            if (acc[0] != 0xCC || acc[1] != 0x81) {  // resync
                memmove(acc, acc + 1, --acc_n);
                continue;
            }
            size_t n = acc[2] | (acc[3] << 8) | (acc[4] << 16);
            if (n < 9 || n > sizeof acc) { memmove(acc, acc + 1, --acc_n); continue; }
            if (acc_n < n) break;
            uint8_t sum = 0;
            for (size_t i = 0; i < n - 3; i++) sum += acc[i];
            bool ok = sum == acc[n - 3] && acc[n - 2] == 0x0D && acc[n - 1] == 0x0A;
            int plen = (int)n - 9;
            if (ok && (size_t)plen <= cap) {
                *type = acc[5];
                memcpy(payload, acc + 6, plen);
                memmove(acc, acc + n, acc_n - n);
                acc_n -= n;
                return plen;
            }
            memmove(acc, acc + 1, --acc_n);  // bad frame, resync
        }
        int64_t left = deadline - esp_timer_get_time();
        if (left <= 0) return -1;
        size_t got = xStreamBufferReceive(rx_stream, acc + acc_n, sizeof acc - acc_n,
                                          pdMS_TO_TICKS(left / 1000 + 1));
        acc_n += got;
        if (usb_lost) return -1;
    }
}

static void decode(const uint8_t *p, size_t plen, Scan &s) {
    uint8_t status = p[0];
    uint32_t exp_us; uint16_t exp_enc; uint32_t sn; uint64_t ex_info;
    memcpy(&exp_us, p + 1, 4);
    memcpy(&exp_enc, p + 5, 2);
    memcpy(&sn, p + 7, 4);
    memcpy(&ex_info, p + 11, 8);

    double exposure_ms = exp_us / 1000.0;
    float et = (float)exposure_ms;
    uint32_t et_le;
    memcpy(&et_le, &et, 4);
    uint32_t et_be = __builtin_bswap32(et_le);

    uint64_t common = (uint64_t)et_be ^ (ex_info >> 16);
    uint16_t key_a = (uint16_t)((common ^ (((uint64_t)(et_le ^ sn)) >> 16) ^ sn ^ ex_info) & 0xFFFF);
    uint16_t key_b = (uint16_t)((((common >> 16) ^ et_le ^ sn)) & 0xFFFF);
    int exponent = (int)(__builtin_bswap16(exp_enc) ^ 8848);
    float scale = powf(10.0f, (float)exponent);

    int n = (int)((plen - 19) / 2);
    if (n > NPTS_MAX) n = NPTS_MAX;
    int mid = n / 2;
    for (int i = 0; i < n; i++) {
        uint16_t v;
        memcpy(&v, p + 19 + i * 2, 2);
        s.spd[i] = (float)(v ^ (i < mid ? key_a : key_b)) / scale;
    }
    s.npts = n;
    s.status = status;
    s.exposure_ms = (float)exposure_ms;
}

static bool tb_init() {
    uint8_t type, pl[64];
    acc_n = 0;
    xStreamBufferReset(rx_stream);
    const uint8_t q = 0x18;
    if (!send_frame(T_ID, &q, 1)) return false;
    int n = read_frame(&type, pl, sizeof pl, 3000);
    if (n < 0 || type != T_ID) return false;
    n = n > (int)sizeof device_id - 1 ? (int)sizeof device_id - 1 : n;
    memcpy(device_id, pl, n);
    device_id[n] = 0;
    if (!send_frame(T_RANGE)) return false;
    n = read_frame(&type, pl, sizeof pl, 3000);
    if (n == 4 && type == T_RANGE) {
        range_lo = pl[0] | (pl[1] << 8);
        range_hi = pl[2] | (pl[3] << 8);
    }
    return true;
}

// Progress callback: (try number, exposure ms, status of the last frame)
typedef void (*progress_fn)(int, float, int);

// Take one scan with auto exposure. Returns true when a normal-status frame arrived.
static bool tb_scan(progress_fn progress, int timeout_s) {
    static uint8_t pl[1600];
    uint8_t type;
    acc_n = 0;
    xStreamBufferReset(rx_stream);
    if (!send_frame(T_DATA)) return false;

    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_s * 1000000;
    bool ok = false, last_ok = false;
    int tries = 0;
    while (esp_timer_get_time() < deadline && !usb_lost) {
        int n = read_frame(&type, pl, sizeof pl, 5000);
        if (n < 0) continue;
        if (type == T_STOP) { send_frame(T_DATA); continue; }  // same quirk tobes-ui works around
        if (type != T_DATA || n < 19) continue;
        decode(pl, n, scan);
        scan.start_nm = range_lo;
        tries++;
        last_ok = scan.status == 0;
        if (progress) progress(tries, scan.exposure_ms, scan.status);
        if (last_ok) { ok = true; break; }
    }
    send_frame(T_STOP);
    if (last_ok) {  // a STOP ack follows a good frame; drain until we see it
        int64_t until = esp_timer_get_time() + 1500000;
        while (esp_timer_get_time() < until) {
            int n = read_frame(&type, pl, sizeof pl, 300);
            if (n >= 0 && type == T_STOP) break;
        }
    }
    return ok;
}

// Live (continuous) mode: the Torch Bearer streams spectra for as long as it is not told to stop (tobes-ui does the same:
// one GET_DATA, then read frames until STOP). Every frame that is locked (status 0) goes to `emit`, which applies the
// correction and sends it to the phone. Frames that arrive faster than the phone can take them are dropped (only the newest
// is sent). Ends on a cancel from the phone (command 05), a button press (stop_req), the cable being pulled, or 15 s without
// a frame. Returns the number of frames sent.
typedef bool (*stop_fn)();
typedef void (*emit_fn)();
static bool scan_cancelled();   // defined with the BLE state below
static int tb_live(progress_fn progress, emit_fn emit, stop_fn stop_req) {
    static uint8_t pl[1600];
    uint8_t type;
    acc_n = 0;
    xStreamBufferReset(rx_stream);
    if (!send_frame(T_DATA)) return 0;

    int sent = 0, tries = 0;
    bool last_ok = false;
    int64_t last_frame_us = esp_timer_get_time();
    while (!usb_lost && !scan_cancelled() && !(stop_req && stop_req())) {
        int n = read_frame(&type, pl, sizeof pl, 200);
        if (n < 0) {
            if (esp_timer_get_time() - last_frame_us > 15000000) break;   // the spectrometer went quiet
            continue;
        }
        if (type == T_STOP) { send_frame(T_DATA); continue; }             // same quirk as tb_scan / tobes-ui
        if (type != T_DATA || n < 19) continue;
        last_frame_us = esp_timer_get_time();
        // Frames that queued up while the previous one was being sent: keep only the newest.
        int m;
        while ((m = read_frame(&type, pl, sizeof pl, 0)) >= 0) {
            if (type == T_STOP) { send_frame(T_DATA); continue; }
            if (type != T_DATA || m < 19) continue;
            n = m;
        }
        decode(pl, n, scan);
        scan.start_nm = range_lo;
        tries++;
        last_ok = scan.status == 0;
        if (progress) progress(tries, scan.exposure_ms, scan.status);
        if (last_ok) { emit(); sent++; }
    }
    send_frame(T_STOP);
    if (last_ok) {   // a STOP ack follows a good frame; drain until we see it (frames still in flight are skipped)
        int64_t until = esp_timer_get_time() + 1500000;
        while (esp_timer_get_time() < until) {
            int n = read_frame(&type, pl, sizeof pl, 300);
            if (n >= 0 && type == T_STOP) break;
        }
    }
    return sent;
}

// ------------------------------------------------------------- BLE ----
// GATT service 7a1c0001-5b2e-4f0a-9c3d-2e8f6b4a1d00 ("Torch Bearer"):
//   0002 command  (write)  : 01 = scan now, 02 = resend last result, 06 = live: keep scanning and send every locked result
//                            until 05 (Torch Bearer only; each result is a normal summary + header + data + end), 05 = cancel the reading in progress (ColorMunki:
//                            stops at its next step and goes back to ready without sending a result)
//                            07 <kind> = developer simulation: pretend instrument <kind> is attached (0 off, 1 Torch Bearer, 2 ColorMunki,
//                            3 TKLamp tester) when none is plugged in. Held in RAM only (a restart ends it). While it is on, the board
//                            behaves like that instrument (screens, status, fake readings) and reports its own button presses, see 07 below.
//   0003 result   (notify/read): summary packet, then the full spectrum in chunks
//   0004 status   (notify/read): [state u8][try u8][exposure_ms f32]
//   0005 update   (write, write-no-response, notify, read): firmware update over Bluetooth, see "Firmware update" below
//   0006 info     (read): [proto u8 = 1][flags u8: bit0 = can update over Bluetooth][board u8: 0 T-Display S3, 1 S3 Pro]
//                         [running slot u8: 0 or 1][reserved u8] then the firmware's git commit as text (not zero-ended)
//        state: 0 ready, 1 scanning, 2 done, 3 error (no USB / timeout), 4 done but not locked
// Packets (all little-endian):
//   summary  : 01 | status u8 | peak_nm u16 | exposure_ms f32 | peak_val f32 | sum f32 | npts u16 | start_nm u16
//   header   : 02 | npts u16 | start_nm u16 | step_nm u8 | total_bytes u16     (then float32 values)
//   data     : 03 | seq u8 | bytes...                                          (seq counts from 0)
//   end      : 04 | chunks u8
//   live     : 06 | ms u32 | raw u32   (TKLamp tester only: one reading about every 220 ms, ms = board uptime, raw = the
//              tester's sensor count, lumens = raw / ~313.2 on a calibrated tester; scaling is done in the app)
//   button   : 07 | button u8 (1 top, 2 bottom) | pressed u8 (1 down, 0 up) | ms u32 (board uptime) | held_ms u16 (on release, else 0)
//              (only while a simulated instrument is on; tells the app which physical button was pressed)
//   diag     : 05 | id u8 | seq u8 | last u8 | bytes...   (ColorMunki diagnostics, sent after the end marker; apps that
//              don't know 05 ignore it. id 1 = the reading's raw sensor data, id 2 = the instrument's calibration
//              block, once per connection. Used to develop the high-resolution mode offline; see tools/munki_diag.py.)
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x01,0x00,0x1c,0x7a);
static const ble_uuid128_t CMD_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x02,0x00,0x1c,0x7a);
static const ble_uuid128_t RES_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x03,0x00,0x1c,0x7a);
static const ble_uuid128_t STA_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x04,0x00,0x1c,0x7a);
static const ble_uuid128_t OTA_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x05,0x00,0x1c,0x7a);
static const ble_uuid128_t INF_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x06,0x00,0x1c,0x7a);

static uint16_t ble_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile uint32_t ble_conn_count = 0;   // counts connections, so "once per connection" survives reused handles
static uint16_t res_handle, sta_handle;
static bool res_sub = false, sta_sub = false;
static uint8_t own_addr_type;
static volatile bool ble_scan_req = false;
static volatile bool ble_resend_req = false;
static volatile bool ble_live_req = false;   // command 06: keep scanning until cancelled (05)
// Developer simulation (command 07): which instrument to pretend is attached when none is plugged in.
static volatile bool sim_tk_run = false;      // simulated TKLamp: the app has a run going (command 0A 01), so only STOP is offered
static volatile uint8_t sim_inject = 0;      // simulation: a button press sent by the phone (command 09 <1 top|2 bottom>), taken by sim_poll()
static volatile uint8_t sim_kind = 0;        // 0 off, 1 Torch Bearer, 2 ColorMunki, 3 TKLamp tester
static volatile bool sim_changed = false;    // command 07 arrived: restart the simulated instrument
static volatile bool sim_redraw = false;     // a firmware update ended: redraw the simulated screen
[[maybe_unused]] static bool sim_dark = false;                // simulated ColorMunki: dark calibration done
[[maybe_unused]] static int64_t sim_dark_us = 0;
// Cancel (command 05) applies to a reading only if it arrived after that reading was requested, so a late cancel
// can never swallow the next reading.
static volatile int64_t ble_scan_req_us = 0, ble_cancel_us = 0;
static int64_t scan_started_us = 0;
static bool scan_cancelled() { return ble_cancel_us > scan_started_us; }
static bool have_result = false;
// State 3 = no spectrometer: that is what a phone must read when it connects before one is plugged in (it becomes 0 'ready' once USB connects).
// Bytes 6 and 7 are new: [6] meter kind (0 Torch Bearer, 1 ColorMunki, 2 TKLamp tester: live readings, packet 06), [7] ColorMunki flags: bit7 dial known, bits0-1 dial position
// (0 projector, 1 surface, 2 calibration, 3 ambient), bit2 dark calibration done, bit3 high-gain dark references taken,
// bit5 (ColorMunki) readings are high resolution (3.33 nm; standard 10 nm when the reading is flagged low light),
// bit6 the spectrum is already corrected on the board (Torch Bearer: corr_table.h; ColorMunki: munki_corr.h).
// Older apps read only the first 6 bytes.
static uint8_t last_state[10] = {3, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF};
static uint16_t (*meter_cal_age_min)() = NULL;   // ColorMunki: minutes since dark calibration (0xFFFF = none)
static uint8_t meter_kind = 0, meter_flags = 0;
static bool hdr_generic = false;   // no instrument plugged in yet: the header must not claim a Torch Bearer

struct Stats { int pk; float mx, sum; };
static Stats stats_of(const Scan &s) {
    Stats t = {0, 0, 0};
    for (int i = 0; i < s.npts; i++) {
        t.sum += s.spd[i];
        if (s.spd[i] > t.mx) { t.mx = s.spd[i]; t.pk = i; }
    }
    return t;
}

static bool ble_notify(uint16_t handle, bool subscribed, const uint8_t *d, size_t n) {
    if (ble_conn == BLE_HS_CONN_HANDLE_NONE || !subscribed) return false;
    for (int t = 0; t < 60; t++) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(d, n);
        if (!om) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        int rc = ble_gatts_notify_custom(ble_conn, handle, om);
        if (rc == 0) return true;
        if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EBUSY) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        return false;
    }
    return false;
}

static void ble_status(uint8_t state, uint8_t tries, float exp_ms) {
    last_state[0] = state;
    last_state[1] = tries;
    memcpy(last_state + 2, &exp_ms, 4);
    last_state[6] = meter_kind;
    last_state[7] = meter_flags | ((TB_APPLY_CORRECTION && meter_kind == 0) ? 0x40 : 0);   // bit6: spectrum already corrected on board
    {
        uint16_t a = meter_cal_age_min ? meter_cal_age_min() : 0xFFFF;
        last_state[8] = (uint8_t)(a & 0xFF);
        last_state[9] = (uint8_t)(a >> 8);
    }
    ble_notify(sta_handle, sta_sub, last_state, sizeof last_state);
}
// Re-send the current status with the latest meter flags (the dial moved, or calibration finished).
static void ble_status_refresh() {
    last_state[6] = meter_kind;
    last_state[7] = meter_flags | ((TB_APPLY_CORRECTION && meter_kind == 0) ? 0x40 : 0);   // bit6: spectrum already corrected on board
    {
        uint16_t a = meter_cal_age_min ? meter_cal_age_min() : 0xFFFF;
        last_state[8] = (uint8_t)(a & 0xFF);
        last_state[9] = (uint8_t)(a >> 8);
    }
    ble_notify(sta_handle, sta_sub, last_state, sizeof last_state);
}

static size_t summary_packet(uint8_t *b) {
    Stats t = stats_of(scan);
    uint16_t peak_nm = (uint16_t)(scan.start_nm + t.pk), npts = (uint16_t)scan.npts, st = (uint16_t)scan.start_nm;
    size_t o = 0;
    b[o++] = 0x01;
    b[o++] = scan.status;
    memcpy(b + o, &peak_nm, 2); o += 2;
    memcpy(b + o, &scan.exposure_ms, 4); o += 4;
    memcpy(b + o, &t.mx, 4); o += 4;
    memcpy(b + o, &t.sum, 4); o += 4;
    memcpy(b + o, &npts, 2); o += 2;
    memcpy(b + o, &st, 2); o += 2;
    return o;
}

static void ble_send_result() {
    if (!have_result || ble_conn == BLE_HS_CONN_HANDLE_NONE) return;
    uint8_t b[32];
    ble_notify(res_handle, res_sub, b, summary_packet(b));

    uint16_t mtu = ble_att_mtu(ble_conn);
    size_t chunk = mtu > 23 ? mtu - 3 : 20;
    if (chunk > 240) chunk = 240;
    size_t payload = chunk - 2;

    uint16_t npts = (uint16_t)scan.npts, st = (uint16_t)scan.start_nm;
    uint16_t total = (uint16_t)(scan.npts * 4);
    uint8_t h[8] = {0x02};
    memcpy(h + 1, &npts, 2);
    memcpy(h + 3, &st, 2);
    h[5] = 1;
    memcpy(h + 6, &total, 2);
    ble_notify(res_handle, res_sub, h, sizeof h);

    const uint8_t *raw = (const uint8_t *)scan.spd;
    uint8_t seq = 0, pkt[256];
    for (size_t off = 0; off < total; off += payload, seq++) {
        size_t n = total - off < payload ? total - off : payload;
        pkt[0] = 0x03;
        pkt[1] = seq;
        memcpy(pkt + 2, raw + off, n);
        if (!ble_notify(res_handle, res_sub, pkt, n + 2)) break;
        vTaskDelay(pdMS_TO_TICKS(6));
    }
    uint8_t e[2] = {0x04, seq};
    ble_notify(res_handle, res_sub, e, 2);
}

// A diagnostics blob as 05 packets (see the protocol comment above).
static void ble_send_blob(uint8_t id, const uint8_t *d, size_t n) {
    if (ble_conn == BLE_HS_CONN_HANDLE_NONE || !d || !n) return;
    uint16_t mtu = ble_att_mtu(ble_conn);
    size_t chunk = mtu > 23 ? mtu - 3 : 20;
    if (chunk > 240) chunk = 240;
    size_t payload = chunk - 4;
    uint8_t pkt[256], seq = 0;
    for (size_t off = 0; off < n; off += payload, seq++) {
        size_t m = n - off < payload ? n - off : payload;
        pkt[0] = 0x05;
        pkt[1] = id;
        pkt[2] = seq;
        pkt[3] = off + m >= n ? 1 : 0;
        memcpy(pkt + 4, d + off, m);
        if (!ble_notify(res_handle, res_sub, pkt, m + 4)) break;
        vTaskDelay(pdMS_TO_TICKS(6));
    }
}

static int ble_gap_cb(struct ble_gap_event *ev, void *);
static uint16_t ota_handle = 0;   // characteristic 0005 (firmware update): see "Firmware update" below
static bool ota_sub = false;
static void ota_abort_quiet();
// What is plugged into the bridge, sent in the advertisement so the app can show it (and hide an empty bridge) in its
// device list without connecting: 0 nothing, 1 Torch Bearer, 2 ColorMunki, 3 TKLamp tester. Manufacturer data = company id 0xFFFF
// (reserved for testing), then 'h' 'C' and this byte. Old apps ignore it; old firmware simply doesn't send it.
static uint8_t adv_attached = 0;
static void ble_advertise() {
    struct ble_hs_adv_fields f = {};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = (uint8_t *)"hCRI Bridge";
    f.name_len = 12;
    f.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
    struct ble_hs_adv_fields r = {};
    r.uuids128 = (ble_uuid128_t *)&SVC_UUID;
    r.num_uuids128 = 1;
    r.uuids128_is_complete = 1;
    uint8_t mfg[5] = {0xFF, 0xFF, 'h', 'C', adv_attached};
    r.mfg_data = mfg;
    r.mfg_data_len = sizeof mfg;
    ble_gap_adv_rsp_set_fields(&r);
    struct ble_gap_adv_params ap = {};
    ap.conn_mode = BLE_GAP_CONN_MODE_UND;
    ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ap.itvl_min = BLE_GAP_ADV_ITVL_MS(100);  // advertise often so phone scans find it quickly
    ap.itvl_max = BLE_GAP_ADV_ITVL_MS(150);
    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &ap, ble_gap_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) ESP_LOGW(TAG, "adv start failed %d", rc);
}

/** Call when the plugged-in instrument changes. Restarts advertising (only when no phone is connected: a connected
 *  bridge doesn't advertise, and the next disconnect advertises the current value). */
static void adv_set_attached(uint8_t a) {
    if (a == adv_attached) return;
    adv_attached = a;
    if (ble_conn == BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_adv_stop();
        ble_advertise();
    }
}

static int ble_gap_cb(struct ble_gap_event *ev, void *) {
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            ble_conn = ev->connect.conn_handle;
            ble_conn_count++;
            ESP_LOGI(TAG, "BLE connected");
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ble_conn = BLE_HS_CONN_HANDLE_NONE;
        res_sub = sta_sub = ota_sub = false;
        if (sim_kind) { sim_kind = 0; sim_tk_run = false; sim_changed = true; }   // a simulation lasts only as long as the phone link
        if (ota_active) { ota_abort_quiet(); ESP_LOGW(TAG, "update cancelled: phone disconnected"); }
        ble_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == res_handle) res_sub = ev->subscribe.cur_notify;
        if (ev->subscribe.attr_handle == sta_handle) sta_sub = ev->subscribe.cur_notify;
        if (ev->subscribe.attr_handle == ota_handle) ota_sub = ev->subscribe.cur_notify;
        break;
    default:
        break;
    }
    return 0;
}


// ------------------------------------------------------- firmware update ----
// Over-the-air update from the phone, on characteristic 0005. The image goes into the spare app slot (ota_0/ota_1, see
// partitions_ota.csv), is checked against a SHA-256 the phone sends first, and is only then made the boot slot. The new
// firmware must start and run for OTA_VALID_AFTER_S seconds; if it crashes or resets before that, the bootloader goes back to
// the old slot by itself (rollback), so a bad image can't leave the board unusable.
//   phone -> board (first byte = op):
//     01 begin : 01 | size u32 | sha256 [32] | board u8   (board: 0 S3, 1 Pro; refused if it is the other one)
//     02 data  : 02 | offset u32 | bytes...                (offset must be exactly the number of bytes taken so far)
//     03 end   : 03                                         (checks size and SHA-256, switches slot, restarts after ~1.5 s)
//     04 abort : 04
//   board -> phone (notify, and readable as the last one sent; all little-endian):
//     81 | slot u8            begin accepted (slot = the slot being written)
//     82 | received u32       progress, about every 4 KB
//     83                      done: the board restarts into the new firmware
//     E1 | expected u32       an offset did not match: send again from `expected`
//     E0 | code u8            failed: 1 begin refused (size, wrong board, no spare slot), 2 flash write failed,
//                             3 not started, 4 size mismatch, 5 SHA-256 mismatch, 6 image rejected / slot switch failed
#define OTA_VALID_AFTER_S 10
static uint8_t ota_last[8] = {0xE0, 3};
static size_t ota_last_len = 2;
static esp_ota_handle_t ota_h = 0;
static const esp_partition_t *ota_part = NULL;
static uint32_t ota_total = 0, ota_got = 0, ota_acked = 0;
static uint8_t ota_sha_want[32];
static mbedtls_sha256_context ota_sha;
static bool ota_ctx_ready = false;
static void ota_draw(int pct, const char *msg, bool bad);

static void ota_send(const uint8_t *d, size_t n) {
    memcpy(ota_last, d, n);
    ota_last_len = n;
    if (ble_conn == BLE_HS_CONN_HANDLE_NONE || !ota_sub) return;
    // One try, no waiting: this runs in the Bluetooth host task, which must not sit and wait for itself. If a notification is
    // lost the phone reads the characteristic, or sends again after its own timeout.
    struct os_mbuf *om = ble_hs_mbuf_from_flat(d, n);
    if (om) ble_gatts_notify_custom(ble_conn, ota_handle, om);
}
static void ota_fail(uint8_t code, const char *detail = nullptr) {
    uint8_t m[2] = {0xE0, code};
    ota_send(m, 2);
    if (ota_h) { esp_ota_abort(ota_h); ota_h = 0; }
    if (ota_ctx_ready) { mbedtls_sha256_free(&ota_sha); ota_ctx_ready = false; }
    ESP_LOGW(TAG, "update failed (%d)", code);
    ota_draw(0, detail ? detail : "UPDATE FAILED", true);
    ota_active = false;
    ota_redraw_at = esp_timer_get_time() + 5000000;
    ota_redraw = true;   // the idle loop puts the normal screen back after 5 s
}
// SHA-256 of what is actually stored in the new slot (read back from flash, 1 KB at a time). This is the truth about the update.
static bool ota_flash_hash(uint8_t out[32]) {
    if (!ota_part) return false;
    static uint8_t blk[1024];
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    for (uint32_t off = 0; off < ota_total; off += sizeof blk) {
        size_t n = ota_total - off < sizeof blk ? ota_total - off : sizeof blk;
        if (esp_partition_read(ota_part, off, blk, n) != ESP_OK) { mbedtls_sha256_free(&c); return false; }
        mbedtls_sha256_update(&c, blk, n);
    }
    mbedtls_sha256_finish(&c, out);
    mbedtls_sha256_free(&c);
    return true;
}
static void ota_abort_quiet() {   // the phone went away or asked to stop
    if (!ota_active) return;
    if (ota_h) { esp_ota_abort(ota_h); ota_h = 0; }
    if (ota_ctx_ready) { mbedtls_sha256_free(&ota_sha); ota_ctx_ready = false; }
    ota_active = false;
    ota_redraw_at = 0;
    ota_redraw = true;
}
static void ota_restart_cb(void *) { esp_restart(); }

static void ota_handle_write(const uint8_t *d, size_t n) {
    if (n < 1) return;
    switch (d[0]) {
    case 0x01: {   // begin
        if (n < 38) { ota_fail(1); return; }
        if (ota_active) ota_abort_quiet();
        uint32_t size;
        memcpy(&size, d + 1, 4);
        uint8_t board = d[37];
#ifdef BOARD_PRO
        const uint8_t mine = 1;
#else
        const uint8_t mine = 0;
#endif
        ota_part = esp_ota_get_next_update_partition(NULL);
        if (!ota_part || board != mine || size < 100000 || size > ota_part->size) { ota_fail(1); return; }
        if (esp_ota_begin(ota_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_h) != ESP_OK) { ota_h = 0; ota_fail(1); return; }
        memcpy(ota_sha_want, d + 5, 32);
        mbedtls_sha256_init(&ota_sha);
        mbedtls_sha256_starts(&ota_sha, 0);
        ota_ctx_ready = true;
        ota_total = size;
        ota_got = ota_acked = 0;
        ota_active = true;
        ota_draw(0, "UPDATING FIRMWARE", false);
        uint8_t m[2] = {0x81, (uint8_t)(ota_part->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN)};
        ota_send(m, 2);
        ESP_LOGI(TAG, "update begins: %u bytes into %s", (unsigned)size, ota_part->label);
        break;
    }
    case 0x02: {   // data
        if (!ota_active || !ota_h) { uint8_t m[2] = {0xE0, 3}; ota_send(m, 2); return; }
        if (n < 6) return;
        uint32_t off;
        memcpy(&off, d + 1, 4);
        if (off != ota_got) {   // a chunk was lost or arrived twice: tell the phone where to continue
            uint8_t m[5] = {0xE1};
            memcpy(m + 1, &ota_got, 4);
            ota_send(m, 5);
            return;
        }
        size_t len = n - 5;
        if (ota_got + len > ota_total) { ota_fail(4); return; }
        if (esp_ota_write(ota_h, d + 5, len) != ESP_OK) { ota_fail(2); return; }
        mbedtls_sha256_update(&ota_sha, d + 5, len);
        ota_got += len;
        if (ota_got - ota_acked >= 4096 || ota_got == ota_total) {
            ota_acked = ota_got;
            uint8_t m[5] = {0x82};
            memcpy(m + 1, &ota_got, 4);
            ota_send(m, 5);
            ota_draw((int)((uint64_t)ota_got * 100 / ota_total), "UPDATING FIRMWARE", false);
        }
        break;
    }
    case 0x03: {   // end
        if (!ota_active || !ota_h) { uint8_t m[2] = {0xE0, 3}; ota_send(m, 2); return; }
        if (ota_got != ota_total) { ota_fail(4); return; }
        uint8_t got[32];
        mbedtls_sha256_finish(&ota_sha, got);
        mbedtls_sha256_free(&ota_sha);
        ota_ctx_ready = false;
        // The running hash of the bytes received and a fresh hash of what landed in flash. The flash is what will boot, so it decides;
        // if they disagree, the screen shows the first bytes of each (S = stream, F = flash, W = wanted) so the cause can be found.
        uint8_t fl[32] = {0};
        bool fl_ok = ota_flash_hash(fl);
        bool stream_ok = memcmp(got, ota_sha_want, 32) == 0;
        bool flash_ok = fl_ok && memcmp(fl, ota_sha_want, 32) == 0;
        if (!flash_ok) {
            char msg[40];
            snprintf(msg, sizeof msg, "S%02X%02X F%02X%02X W%02X%02X", got[0], got[1], fl[0], fl[1], ota_sha_want[0], ota_sha_want[1]);
            ESP_LOGW(TAG, "update hash mismatch: %s (flash read %s)", msg, fl_ok ? "ok" : "failed");
            ota_fail(5, msg);
            return;
        }
        if (!stream_ok) ESP_LOGW(TAG, "running hash differed but the flash copy is correct; accepting");
        esp_err_t e = esp_ota_end(ota_h);
        ota_h = 0;
        if (e != ESP_OK || esp_ota_set_boot_partition(ota_part) != ESP_OK) { ota_fail(6); return; }
        uint8_t m[1] = {0x83};
        ota_send(m, 1);
        ota_draw(100, "UPDATED - RESTARTING", false);
        ESP_LOGI(TAG, "update done, restarting");
        static esp_timer_handle_t t;
        esp_timer_create_args_t a = {};
        a.callback = ota_restart_cb;
        a.name = "ota_restart";
        if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_once(t, 1500000);
        break;
    }
    case 0x04:   // abort
        ota_abort_quiet();
        break;
    default:
        break;
    }
}

// Once the new firmware has run for OTA_VALID_AFTER_S seconds it is accepted for good; until then a reset puts the old one back.
static void ota_valid_cb(void *) { esp_ota_mark_app_valid_cancel_rollback(); }
static void ota_validate_later() {
    static esp_timer_handle_t t;
    esp_timer_create_args_t a = {};
    a.callback = ota_valid_cb;
    a.name = "ota_valid";
    if (esp_timer_create(&a, &t) == ESP_OK) esp_timer_start_once(t, (uint64_t)OTA_VALID_AFTER_S * 1000000);
}

static int ble_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int which = (int)(intptr_t)arg;  // 0 command, 1 result, 2 status, 3 firmware update, 4 info
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && which == 3) {
        uint8_t buf[260];
        uint16_t n = OS_MBUF_PKTLEN(ctxt->om);
        if (n > sizeof buf) n = sizeof buf;
        if (os_mbuf_copydata(ctxt->om, 0, n, buf) == 0) ota_handle_write(buf, n);
        return 0;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && which == 0) {
        uint8_t cmd = 0;
        if (OS_MBUF_PKTLEN(ctxt->om) >= 1 && os_mbuf_copydata(ctxt->om, 0, 1, &cmd) == 0) {
            if (ota_active) return 0;   // nothing starts while a firmware update is being received
            if (cmd == 0x01) { ble_scan_req_us = esp_timer_get_time(); ble_scan_req = true; }
            if (cmd == 0x05) ble_cancel_us = esp_timer_get_time();   // cancel the reading in progress (ColorMunki)
            if (cmd == 0x02) ble_resend_req = true;
            if (cmd == 0x06) { ble_scan_req_us = esp_timer_get_time(); ble_live_req = true; }   // live (continuous) scanning, Torch Bearer only; 05 stops it
            if (cmd == 0x07) {  // 0x07 <kind>: developer simulation of an attached instrument (nothing is plugged in)
                uint8_t v = 0;
                if (OS_MBUF_PKTLEN(ctxt->om) >= 2 && os_mbuf_copydata(ctxt->om, 1, 1, &v) == 0) { sim_kind = v <= 3 ? v : 0; sim_changed = true; }
            }
            if (cmd == 0x0A) {  // 0x0A <0|1>: the app has a TKLamp run going (simulation only): the screen then offers only STOP
                uint8_t v = 0;
                if (OS_MBUF_PKTLEN(ctxt->om) >= 2 && os_mbuf_copydata(ctxt->om, 1, 1, &v) == 0) { sim_tk_run = v != 0; sim_redraw = true; }
            }
            if (cmd == 0x09) {  // 0x09 <1 top|2 bottom>: a button press from the phone (simulation only)
                uint8_t w = 0;
                if (sim_kind != 0 && OS_MBUF_PKTLEN(ctxt->om) >= 2 && os_mbuf_copydata(ctxt->om, 1, 1, &w) == 0 && (w == 1 || w == 2)) sim_inject = w;
            }
            if (cmd == 0x08) { ESP_LOGW(TAG, "restart requested over BLE"); esp_restart(); }   // 0x08: reset the bridge
            if (cmd == 0x03) {  // 0x03 <0|1>: display orientation, 1 = turned 180 degrees
                uint8_t v = 0;
                if (OS_MBUF_PKTLEN(ctxt->om) >= 2 && os_mbuf_copydata(ctxt->om, 1, 1, &v) == 0) flip_req = v ? 1 : 0;
            }
        }
        return 0;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && which == 3) return os_mbuf_append(ctxt->om, ota_last, ota_last_len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && which == 4) {
        const esp_partition_t *run = esp_ota_get_running_partition();
#ifdef BOARD_PRO
        const uint8_t board = 1;
#else
        const uint8_t board = 0;
#endif
        uint8_t slot = (run && run->subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MIN) ? (uint8_t)(run->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN) : 0;
        uint8_t h[5] = {1, 1, board, slot, 0};
        if (os_mbuf_append(ctxt->om, h, sizeof h) != 0) return BLE_ATT_ERR_INSUFFICIENT_RES;
        return os_mbuf_append(ctxt->om, FW_VERSION, strlen(FW_VERSION)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t b[32];
        if (which == 2) {
            uint16_t a = meter_cal_age_min ? meter_cal_age_min() : 0xFFFF;
            last_state[8] = (uint8_t)(a & 0xFF);
            last_state[9] = (uint8_t)(a >> 8);
        }
        if (which == 2) return os_mbuf_append(ctxt->om, last_state, sizeof last_state) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        if (which == 1 && have_result) return os_mbuf_append(ctxt->om, b, summary_packet(b)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static struct ble_gatt_chr_def ble_chrs[6];
static struct ble_gatt_svc_def ble_svcs[2];

static void ble_on_sync() {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_advertise();
}
static void ble_host_task(void *) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}
static void ble_start() {
    nvs_boot();
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    memset(ble_chrs, 0, sizeof ble_chrs);
    ble_chrs[0].uuid = &CMD_UUID.u;
    ble_chrs[0].access_cb = ble_access_cb;
    ble_chrs[0].arg = (void *)0;
    ble_chrs[0].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;
    ble_chrs[1].uuid = &RES_UUID.u;
    ble_chrs[1].access_cb = ble_access_cb;
    ble_chrs[1].arg = (void *)1;
    ble_chrs[1].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    ble_chrs[1].val_handle = &res_handle;
    ble_chrs[2].uuid = &STA_UUID.u;
    ble_chrs[2].access_cb = ble_access_cb;
    ble_chrs[2].arg = (void *)2;
    ble_chrs[2].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    ble_chrs[2].val_handle = &sta_handle;
    ble_chrs[3].uuid = &OTA_UUID.u;
    ble_chrs[3].access_cb = ble_access_cb;
    ble_chrs[3].arg = (void *)3;
    ble_chrs[3].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    ble_chrs[3].val_handle = &ota_handle;
    ble_chrs[4].uuid = &INF_UUID.u;
    ble_chrs[4].access_cb = ble_access_cb;
    ble_chrs[4].arg = (void *)4;
    ble_chrs[4].flags = BLE_GATT_CHR_F_READ;
    memset(ble_svcs, 0, sizeof ble_svcs);
    ble_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    ble_svcs[0].uuid = &SVC_UUID.u;
    ble_svcs[0].characteristics = ble_chrs;

    ESP_ERROR_CHECK(ble_gatts_count_cfg(ble_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(ble_svcs));
    ble_svc_gap_device_name_set("hCRI Bridge");
    ble_att_set_preferred_mtu(247);
    nimble_port_freertos_init(ble_host_task);
}

// ------------------------------------------------------------- app link ----
// A Bluetooth rune and "APP" in the top-right corner (under the header) while the phone app is connected.
static bool link_shown = false;
static const int LINK_X = 292, LINK_Y = 26, LINK_W = W - 292, LINK_H = 32;
static void thick_line(int x0, int y0, int x1, int y1, int t, uint16_t c) {
    int n = abs(x1 - x0) > abs(y1 - y0) ? abs(x1 - x0) : abs(y1 - y0);
    if (n == 0) n = 1;
    for (int i = 0; i <= n; i++) fill(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, t, t, c);
}
static void draw_app_link() {
    bool up = ble_conn != BLE_HS_CONN_HANDLE_NONE;
    link_shown = up;
    const int dx = ((tb_ui || arrow_label) && !arrow_bottom_left()) ? -34 : 0;   // clear of the READ label when the buttons are on the right
    fill(LINK_X + dx, LINK_Y, LINK_W, LINK_H, BLACK);
    if (!up) return;
    const int x = 300 + dx, y = 27;  // rune, about 9 x 15
    thick_line(x + 4, y, x + 4, y + 14, 2, CYAN);
    thick_line(x + 4, y, x + 8, y + 4, 2, CYAN);
    thick_line(x + 8, y + 4, x, y + 10, 2, CYAN);
    thick_line(x + 4, y + 14, x + 8, y + 10, 2, CYAN);
    thick_line(x + 8, y + 10, x, y + 4, 2, CYAN);
    text(298 + dx, 45, "APP", 1, CYAN);
}
// Called from the idle loops: redraw only when the phone connects or disconnects.
static void ota_restore_ui();
static void link_tick() {
    if (ota_redraw && esp_timer_get_time() >= ota_redraw_at) { ota_redraw = false; ota_restore_ui(); return; }
    if (ota_active) return;
    if ((ble_conn != BLE_HS_CONN_HANDLE_NONE) == link_shown) return;
    draw_app_link();
    present();
}

// ----------------------------------------------------------------- UI ----
// A small yellow arrow pointing at the scan button on the READY and result screens. Unflipped, the buttons are at the
// bottom-left on both boards: S3 arrow bottom-left pointing left, Pro arrow bottom-left pointing down. Flipped, they are
// at the top-right: S3 arrow pointing right, Pro arrow pointing up.

// Pro, Torch Bearer: READ arrow at the top (top button, word under the arrow), LIVE arrow at the bottom (bottom button,
// word above the arrow), on whichever side the buttons are for the current orientation.
static int lpad() { return ((tb_ui || arrow_label) && arrow_bottom_left()) ? 24 : 0; }
static void draw_tb_buttons() {
    const bool left = arrow_bottom_left();
    const bool tk = arrow_label && sim_kind == 3;            // simulated TKLamp: top starts / stops the run in the app, bottom steps the light level
    const bool munki = arrow_label && !tk;                   // ColorMunki screen: bottom button reads, top turns the (simulated) dial
    const bool live = !munki && (sim_live_banner || prog_mode == 2);   // during Live only the bottom button matters, and it stops it
    const int cx = left ? 8 : 312;
    const int lx = left ? 1 : W - 25;
    if (tk) {   // top: START, or STOP while a run is going (then it is the only button offered); bottom: NEXT light level
        const char *tl = sim_tk_run ? "STOP" : "START";
        const int tx = left ? 1 : W - 1 - (int)strlen(tl) * 6;
        for (int i = 0; i <= 5; i++) fill(cx - i, 3 + i, 2 * i + 1, 1, sim_tk_run ? RED : YELLOW);
        fill(cx - 1, 9, 3, 8, sim_tk_run ? RED : YELLOW);
        text(tx, 26, tl, 1, sim_tk_run ? RED : WHITE);
        if (!sim_tk_run) {
            for (int i = 0; i <= 5; i++) fill(cx - i, 166 - i, 2 * i + 1, 1, YELLOW);
            fill(cx - 1, 153, 3, 8, YELLOW);
            text(left ? 1 : W - 25, 142, "NEXT", 1, WHITE);
        }
        return;
    }
    if (!live && (!munki || sim_kind == 2 || tk)) {
        for (int i = 0; i <= 5; i++) fill(cx - i, 3 + i, 2 * i + 1, 1, YELLOW);     // up arrow, tip at the top
        fill(cx - 1, 9, 3, 8, YELLOW);
        text(lx, 26, tk ? "RUN" : munki ? "DIAL" : "READ", 1, WHITE);
    }
    for (int i = 0; i <= 5; i++) fill(cx - i, 166 - i, 2 * i + 1, 1, YELLOW);   // down arrow, tip at the bottom
    fill(cx - 1, 153, 3, 8, YELLOW);
    text(lx, 142, tk ? "NEXT" : munki ? "READ" : live ? "STOP" : "LIVE", 1, live ? RED : WHITE);
}

static void draw_btn_arrow() {
#ifdef BOARD_PRO
    if (tb_ui || arrow_label) { draw_tb_buttons(); return; }
    if (!arrow_bottom_left()) {
        const int cx = 312;
        for (int i = 0; i <= 5; i++) fill(cx - i, 3 + i, 2 * i + 1, 1, YELLOW);   // head, tip at the top
        fill(cx - 1, 9, 3, 8, YELLOW);                                              // shaft
    } else {
        const int cx = 8;
        for (int i = 0; i <= 5; i++) fill(cx - i, 166 - i, 2 * i + 1, 1, YELLOW); // head, tip at the bottom
        fill(cx - 1, 153, 3, 8, YELLOW);
    }
#else
    if (!arrow_bottom_left()) {
        const int tip = 318, cy = 11;
        for (int i = 0; i <= 5; i++) fill(tip - i, cy - i, 1, 2 * i + 1, YELLOW);   // head, pointing right
        fill(tip - 13, cy - 1, 8, 3, YELLOW);                                        // shaft
    } else {
        const int tip = 1, cy = 160;
        for (int i = 0; i <= 5; i++) fill(tip + i, cy - i, 1, 2 * i + 1, YELLOW);   // head, pointing left
        fill(tip + 6, cy - 1, 8, 3, YELLOW);
    }
#endif
}

// ------------------------------------------------------- button readout ----
// Which button pins are held, shown on the idle and result screens and logged, to see what each physical button does.
// The Pro also watches GPIO 12, which counts as button A.
static const int WATCH_PINS[] = {
    PIN_BTN_A, PIN_BTN_B,
#ifdef BOARD_PRO
    PIN_BTN_A2,
#endif
};
static const int N_WATCH = (int)(sizeof(WATCH_PINS) / sizeof(WATCH_PINS[0]));
static char btn_text[40] = "";
static uint32_t btn_seen = 0;

static uint32_t button_mask() {
    uint32_t m = 0;
    for (int i = 0; i < N_WATCH; i++)
        if (gpio_get_level((gpio_num_t)WATCH_PINS[i]) == 0) m |= 1u << i;
    return m;
}
static void btn_note(uint32_t m) {
    if (!m) return;
    int n = snprintf(btn_text, sizeof btn_text, "BTN GPIO");
    bool first = true;
    for (int i = 0; i < N_WATCH && n < (int)sizeof btn_text - 6; i++) {
        if (!(m & (1u << i))) continue;
        n += snprintf(btn_text + n, sizeof btn_text - n, first ? "%d" : "+%d", WATCH_PINS[i]);
        first = false;
    }
    ESP_LOGI(TAG, "button pressed: %s", btn_text);
}
static void draw_btn_note() {
    fill(20 + lpad(), 122, 200, 9, BLACK);
    if (btn_text[0]) text(20 + lpad(), 122, btn_text, 1, CYAN);
}

// "FW <commit>", with SIM added while a simulated instrument is on, so a simulation is never mistaken for a real one.
static const char *fw_label() {
    static char b[40];
    snprintf(b, sizeof b, sim_kind ? "%s SIM" : "%s", FW_VERSION);
    return b;
}

static void header(const char *right, uint16_t rc) {
    text(6 + lpad(), 4, meter_kind == 1 ? "COLORMUNKI" : meter_kind == 2 ? "TKLAMP" : hdr_generic ? "HCRI BRIDGE" : "TORCH BEARER", 2, WHITE);
    // On the arrow screens the upright arrow owns the top-right corner, so the right-hand status steps left of it.
    int edge = (shown != 0 && !arrow_bottom_left()) ? W - 24 : W - 6;
    text(edge - (int)strlen(right) * 12, 4, right, 2, rc);
    draw_battery();
    for (int x = 0; x < W; x++) px(x, 22, GREY);
    draw_app_link();
}

static void ui_waiting() {
    clear();
    hdr_generic = true;
    header("NO USB", RED);
    hdr_generic = false;
    text(20, 48, "PLUG IN AN INSTRUMENT", 2, YELLOW);
    text(20, 74, "TORCH BEARER, COLORMUNKI", 2, GREY);
    text(20, 94, "OR TKLAMP, VIA OTG CABLE", 2, GREY);
    waiting_shown = true;
    draw_btn_note();
    textf(6, 160, 1, GREY, "FW %s", fw_label());
    diag_on = true;
    draw_diag();
    present();
}

static void ui_munki_ready();
static void ui_ready() {
    if (meter_kind == 1) { ui_munki_ready(); return; }
    clear();
    shown = 1;
#ifdef BOARD_PRO
    tb_ui = true;
#endif
    const int L = lpad();
    header("READY", GREEN);
    text(L + 6, 32, device_id, 1, GREY);
    textf(L + 6, 48, 1, GREY, "RANGE %d-%d NM", range_lo, range_hi);
    text(L + 6, 74, "TOP BUTTON: READ", 2, WHITE);      // roles follow the screen: see btn_top_pressed()
    text(L + 6, 98, "BOTTOM BUTTON: LIVE", 2, WHITE);
    draw_btn_note();
    draw_btn_arrow();
    textf(arrow_bottom_left() ? (tb_ui ? 30 : 18) : 6, 160, 1, GREY, "FW %s", fw_label());
    diag_on = true;
    draw_diag();
    present();
}

static void ui_progress(int tries, float exp_ms, int status) {
    diag_on = false;
    ble_status(1, (uint8_t)tries, exp_ms);
    clear();
#ifdef BOARD_PRO
    if (prog_mode == 2) tb_ui = true;   // Live: the bottom button's STOP label is drawn, so the text steps clear of it
#endif
    const int L = lpad();
    header("SCANNING", YELLOW);
    if (prog_mode) text(L + 6, 28, prog_mode == 1 ? "PERFORMING READ" : prog_mode == 2 ? "PERFORMING LIVE READ" : "SIMULATED READ", 2, CYAN);
    if (prog_mode == 2) text(L + 6, 146, "PRESS BUTTON AGAIN TO STOP", 1, YELLOW);
    textf(L + 10, 50, 3, WHITE, "TRY %d", tries);
    textf(L + 10, 90, 2, CYAN, "EXPOSURE %.1f MS", exp_ms);
    text(L + 10, 115, status == 1 ? "TOO BRIGHT - ADJUSTING" : status == 2 ? "TOO DIM - ADJUSTING" : "OK", 2,
         status == 0 ? GREEN : YELLOW);
    if (tb_ui) draw_btn_arrow();
    present();
}

static void ui_scanning() { ui_progress(0, 0, 2); }

// Firmware update screen (BLE OTA, see "Firmware update"): a progress bar while the phone sends the image.
static void ota_draw(int pct, const char *msg, bool bad) {
    diag_on = false;
    clear();
    header("UPDATE", bad ? RED : YELLOW);
    text(10, 50, msg, 2, bad ? RED : WHITE);
    if (!bad) {
        const int bx = 10, by = 90, bw = W - 20, bh = 22;
        fill(bx, by, bw, bh, GREY);
        fill(bx + 2, by + 2, bw - 4, bh - 4, BLACK);
        fill(bx + 2, by + 2, (bw - 4) * pct / 100, bh - 4, GREEN);
        textf(10, 124, 2, WHITE, "%d%%", pct);
        text(10, 150, "KEEP THE PHONE CLOSE", 1, GREY);
    }
    present();
}
// After a failed or cancelled update: the screen the board was on before.
static void ota_restore_ui() {
    if (usb_connected) ui_ready(); else if (sim_kind) sim_redraw = true; else ui_waiting();
}

static uint16_t wl_color(int nm) {   // rough wavelength to colour
    if (nm < 380 || nm > 750) return rgb(80, 80, 80);   // UV / IR: no visible colour, plot in grey
    double r = 0, g = 0, b = 0;
    if (nm < 440) { r = (440 - nm) / 60.0; b = 1; }
    else if (nm < 490) { g = (nm - 440) / 50.0; b = 1; }
    else if (nm < 510) { g = 1; b = (510 - nm) / 20.0; }
    else if (nm < 580) { r = (nm - 510) / 70.0; g = 1; }
    else if (nm < 645) { r = 1; g = (645 - nm) / 65.0; }
    else r = 1;
    if (nm < 420) { double f = 0.3 + 0.7 * (nm - 380) / 40.0; r *= f; g *= f; b *= f; }
    if (nm > 700) { double f = 0.3 + 0.7 * (730 - nm) / 30.0; r *= f; g *= f; b *= f; }
    return rgb((uint8_t)(255 * r), (uint8_t)(255 * g), (uint8_t)(255 * b));
}

static void ui_result(bool ok) {
    diag_on = false;
    int n = scan.npts;
    Stats t = stats_of(scan);
    float mx = t.mx;
    int pk = t.pk;
    Metrics m;
    bool have_m = compute_metrics(scan.spd, n, scan.start_nm, &m);
    clear();
#ifdef BOARD_PRO
    tb_ui = true;
#endif
    const int L = lpad();
    shown = 2;
    result_ok = ok;
    if (sim_live_banner) header("SIM LIVE", YELLOW); else header(ok ? "DONE" : "NO LOCK", ok ? GREEN : RED);

    // colorimetry, computed on the board from the corrected spectrum
    if (have_m) {
        textf(L + 6, 28, 2, WHITE, "CCT %.0f K", m.cct);
        textf(L + 6, 46, 2, WHITE, "DUV %+.4f", m.duv);
        textf(L + 6, 64, 2, WHITE, "RA %d  R9 %d", m.ra, m.r9);
        textf(196, 30, 1, CYAN, "X %.4f", m.x);
        textf(196, 42, 1, CYAN, "Y %.4f", m.y);
    } else {
        text(L + 6, 40, "NO SIGNAL", 2, RED);
    }
    textf(196, 54, 1, GREY, "PEAK %d NM", scan.start_nm + pk);
    textf(196, 66, 1, GREY, "EXP %.1f MS", scan.exposure_ms);
    if (sim_live_banner) text(L + 6, 80, "SIMULATING LIVE READ - PRESS BUTTON AGAIN TO STOP", 1, YELLOW); else
    textf(L + 6, 80, 1, scan.status == 0 ? GREY : YELLOW, "%s  %d PTS",
          scan.status == 0 ? "NORMAL" : scan.status == 1 ? "OVER-EXPOSED" : "UNDER-EXPOSED", n);

    // spectrum plot, x = wavelength, y = value (autoscaled to the peak)
    const int px0 = 10 + L, pw = tb_ui ? 276 : 300, py0 = 92, ph = 66;
    for (int x = 0; x < pw; x++) px(px0 + x, py0 + ph, GREY);
    if (mx > 0 && n > 1) {
        for (int x = 0; x < pw; x++) {
            int i = x * (n - 1) / (pw - 1);
            int h = (int)(scan.spd[i] / mx * (ph - 1));
            if (h < 0) h = 0;
            vline(px0 + x, py0 + ph - h, py0 + ph - 1,
                  wl_color(range_lo + x * (range_hi - range_lo) / (pw - 1)));   // each column in its own wavelength colour
        }
        int pkx = pk * (pw - 1) / (n - 1);
        vline(px0 + pkx, py0, py0 + ph, YELLOW);
    }
    textf(arrow_bottom_left() && !tb_ui ? px0 + 8 : px0, 162, 1, GREY, "%d", range_lo);   // steps right of the flipped arrow
    draw_btn_arrow();
    if (btn_text[0] && !sim_live_banner) text(196, 80, btn_text, 1, CYAN);
    textf(px0 + pw / 2 - 9, 162, 1, GREY, "%d", (range_lo + range_hi) / 2);
    textf(px0 + pw - 24, 162, 1, GREY, "%d", range_hi);
    present();
}

// Apply an orientation asked for over BLE: turn the panel, remember it, and redraw the frame buffer the new way up.
static void flip_tick() {
    if (ota_active) return;
    int8_t r = flip_req;
    if (r < 0) return;
    flip_req = -1;
    bool f = r != 0;
    if (f == flipped) return;
    flipped = f;
    apply_flip(f);
    flip_save(f);
    if (shown == 1) ui_ready();            // redraw with the arrow in the other corner
    else if (shown == 2) ui_result(result_ok);
    else present();
}

// Idle loops: when a watched pin goes low, log it and show it on the idle screens.
// ------------------------------------------------------------- sleep ----
// Hold either button for 2 seconds and the board goes to deep sleep; a press of a wake button starts it again (it reboots,
// so Bluetooth and the instrument reconnect, a few seconds). Wake buttons: both side buttons on the Pro; on the plain S3
// only the GPIO 14 one (BOOT, GPIO 0, doubles as the flashing-mode strap, so it is not used to wake it).
#define SLEEP_HOLD_US 2000000
static bool button_pressed();   // defined below
static void deep_sleep_now() {
    ESP_LOGI(TAG, "deep sleep");
    clear();
    text(88, 70, "SLEEPING", 3, WHITE);
    text(40, 110, "PRESS A BUTTON TO WAKE", 1, GREY);
    present();
    vTaskDelay(pdMS_TO_TICKS(700));
#ifdef BOARD_PRO
    {   // the OTG boost powers the instrument from the battery: cut it, or the battery keeps draining
        uint8_t r03;
        if (pmu_read(0x03, &r03)) pmu_write(0x03, (uint8_t)((r03 & ~0x20) | 0x10));
    }
    const uint64_t wake_mask = (1ULL << PIN_BTN_A) | (1ULL << PIN_BTN_B);
#else
    const uint64_t wake_mask = (1ULL << PIN_BTN_B);
#endif
    gpio_set_level((gpio_num_t)PIN_LCD_BL, 0);
    gpio_hold_en((gpio_num_t)PIN_LCD_BL);
#ifndef BOARD_PRO
    gpio_set_level((gpio_num_t)PIN_POWER_ON, 0);
    gpio_hold_en((gpio_num_t)PIN_POWER_ON);
#endif
    gpio_deep_sleep_hold_en();
    // The wake buttons are active low: keep their pull-ups on in the RTC domain, then wait for them to be let go,
    // or the sleep would end straight away.
    for (int pin = 0; pin < 22; pin++) {
        if (!(wake_mask & (1ULL << pin))) continue;
        rtc_gpio_init((gpio_num_t)pin);
        rtc_gpio_set_direction((gpio_num_t)pin, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_dis((gpio_num_t)pin);
        rtc_gpio_pullup_en((gpio_num_t)pin);
    }
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    while (button_pressed()) vTaskDelay(pdMS_TO_TICKS(20));
    vTaskDelay(pdMS_TO_TICKS(100));
    if (esp_sleep_enable_ext1_wakeup(wake_mask, ESP_EXT1_WAKEUP_ANY_LOW) != ESP_OK) {
        // Fall back to one pin if the chip does not take "any low" on several.
        for (int pin = 0; pin < 22; pin++)
            if (wake_mask & (1ULL << pin)) { esp_sleep_enable_ext0_wakeup((gpio_num_t)pin, 0); break; }
    }
    esp_deep_sleep_start();
}
// A press that is let go within 2 s is a normal press (returns true); one held that long puts the board to sleep (no return).
static bool btn_short_press() {
    int64_t t0 = esp_timer_get_time();
    while (button_pressed()) {
        if (esp_timer_get_time() - t0 > SLEEP_HOLD_US) deep_sleep_now();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}
// For the loops that do not act on a button press: sleep after a 2 s hold.
static int64_t btn_hold_start_us = 0;
static void btn_hold_tick() {
    if (!button_pressed()) { btn_hold_start_us = 0; return; }
    int64_t now = esp_timer_get_time();
    if (!btn_hold_start_us) btn_hold_start_us = now;
    else if (now - btn_hold_start_us > SLEEP_HOLD_US) deep_sleep_now();
}

static void btn_tick() {
    if (ota_active) return;
    btn_hold_tick();
    uint32_t m = button_mask();
    if (m == btn_seen) return;
    btn_seen = m;
    if (!m) return;
    btn_note(m);
    if (shown == 1 || waiting_shown) {
        draw_btn_note();
        present();
    }
}

static inline bool btn_a_low() {
#ifdef BOARD_PRO
    if (gpio_get_level((gpio_num_t)PIN_BTN_A2) == 0) return true;
#endif
    return gpio_get_level((gpio_num_t)PIN_BTN_A) == 0;
}
static bool button_pressed() {
    return btn_a_low() || gpio_get_level((gpio_num_t)PIN_BTN_B) == 0;
}

// Torch Bearer button roles, as seen on the screen: the TOP button starts Live, the BOTTOM button takes one scan. The two
// buttons are fixed pins, so the roles swap when the display is flipped (the screen turns 180 degrees, the buttons do not).
// Unflipped, A is the top one (the Pro's comments above: A = top press, B = bottom press). If a board's labels turn out the
// wrong way round, swap A and B here -- this is the only place that decides it.
static inline bool btn_pin_low(int pin) { return gpio_get_level((gpio_num_t)pin) == 0; }
static bool btn_top_pressed() { return flipped ? btn_pin_low(PIN_BTN_B) : btn_a_low(); }
static bool btn_bottom_pressed() { return flipped ? btn_a_low() : btn_pin_low(PIN_BTN_B); }

#ifdef COLORMUNKI_PROBE
// ------------------------------------------------------- ColorMunki probe ----
// Built with -DCOLORMUNKI_PROBE=1 (env colormunki-probe-pro, Pro only). Instead of the Torch Bearer bridge it shows the
// USB IDs, strings, descriptors and endpoints of whatever is plugged into the USB-C port (through a USB-C to USB-A
// adapter, with the Pro's 5 V boost powering it), so a new instrument can be identified before a driver is written.
// Any button pages through the lines; every line is also logged.
#ifndef BOARD_PRO
#error "the ColorMunki probe is for the T-Display-S3 Pro only (env colormunki-probe-pro)"
#endif
#include "usb/usb_helpers.h"
#include "munki_argyll.h"

static void ui_munki_ready() {}   // the probe has no bridge screens; ui_ready() refers to this one
static usb_host_client_handle_t pr_client = NULL;
static usb_device_handle_t pr_dev = NULL;
static volatile uint8_t pr_new_addr = 0;
static volatile bool pr_gone = false;
#define PR_MAX_LINES 72
#define PR_PER_PAGE 13
static char pr_txt[PR_MAX_LINES][54];
static int pr_n = 0;
static int pr_page = 0;

static void pr_client_cb(const usb_host_client_event_msg_t *m, void *) {
    if (m->event == USB_HOST_CLIENT_EVENT_NEW_DEV) pr_new_addr = m->new_dev.address;
    else if (m->event == USB_HOST_CLIENT_EVENT_DEV_GONE) pr_gone = true;
}

static void pr_add(const char *fmt, ...) {
    if (pr_n >= PR_MAX_LINES) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(pr_txt[pr_n], sizeof pr_txt[0], fmt, ap);
    va_end(ap);
    ESP_LOGI(TAG, "%s", pr_txt[pr_n]);
    pr_n++;
}

static void pr_str(const char *label, const usb_str_desc_t *sd) {
    char b[36] = "-";
    if (sd && sd->bLength >= 4) {
        int n = (sd->bLength - 2) / 2;
        if (n > 34) n = 34;
        for (int i = 0; i < n; i++) {
            uint16_t w = sd->wData[i];
            b[i] = (w >= 32 && w < 127) ? (char)w : '?';
        }
        b[n] = 0;
    }
    pr_add("%s %s", label, b);
}

static void pr_describe() {
    pr_n = 0;
    usb_device_info_t info;
    const usb_device_desc_t *dd = NULL;
    if (usb_host_device_info(pr_dev, &info) != ESP_OK || usb_host_get_device_descriptor(pr_dev, &dd) != ESP_OK || !dd) {
        pr_add("COULD NOT READ THE DEVICE DESCRIPTOR");
        return;
    }
    pr_add("VID %04X  PID %04X  BCD %04X", dd->idVendor, dd->idProduct, dd->bcdDevice);
    pr_add("CLASS %02X/%02X/%02X  EP0 %d  USB %X.%02X", dd->bDeviceClass, dd->bDeviceSubClass, dd->bDeviceProtocol,
           dd->bMaxPacketSize0, dd->bcdUSB >> 8, dd->bcdUSB & 0xFF);
    pr_add("SPEED %s  ADDR %d  CONFIGS %d", info.speed == USB_SPEED_LOW ? "LOW" : info.speed == USB_SPEED_FULL ? "FULL" : "HIGH",
           info.dev_addr, dd->bNumConfigurations);
    pr_str("MFG", info.str_desc_manufacturer);
    pr_str("PRODUCT", info.str_desc_product);
    pr_str("SERIAL", info.str_desc_serial_num);
    const usb_config_desc_t *cd = NULL;
    if (usb_host_get_active_config_descriptor(pr_dev, &cd) != ESP_OK || !cd) {
        pr_add("NO CONFIG DESCRIPTOR");
        return;
    }
    pr_add("CONFIG %d: %d IF  %d MA  %d BYTES", cd->bConfigurationValue, cd->bNumInterfaces, cd->bMaxPower * 2, cd->wTotalLength);
    static const char *const types[] = {"CTRL", "ISO", "BULK", "INT"};
    for (int i = 0; i < cd->bNumInterfaces; i++) {
        for (int alt = 0; alt < 4; alt++) {
            int off = 0;
            const usb_intf_desc_t *id = usb_parse_interface_descriptor(cd, i, alt, &off);
            if (!id) break;
            pr_add("IF %d ALT %d CLASS %02X/%02X/%02X  %d EP", id->bInterfaceNumber, id->bAlternateSetting, id->bInterfaceClass,
                   id->bInterfaceSubClass, id->bInterfaceProtocol, id->bNumEndpoints);
            for (int e = 0; e < id->bNumEndpoints; e++) {
                int eo = off;
                const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(id, e, cd->wTotalLength, &eo);
                if (!ep) break;
                pr_add("  EP %02X %s %s  MAX %d  IVL %d", ep->bEndpointAddress, (ep->bEndpointAddress & 0x80) ? "IN" : "OUT",
                       types[ep->bmAttributes & 3], ep->wMaxPacketSize, ep->bInterval);
            }
        }
    }
}


// Talks to the instrument with the Argyll-derived driver (munki_argyll.cpp) and adds what it says to the pages.
static bool pr_munki = false;
static uint8_t pr_spos = 0xFF, pr_but = 0;
static munki::Eeprom pr_ee;
static munki::Spectrum pr_spec;
static bool pr_have_spec = false;
static munki::Mode pr_spec_mode = munki::MODE_AMBIENT;
static const char *pr_msg = "";
static uint8_t pr_spos_prev = 0xFF;
static int pr_ee_state = 0;   // 0 not read, 1 checksum OK, 2 checksum BAD, 3 read failed
static void pr_query() {
    pr_munki = false;
    if (!munki::open(pr_client, pr_dev)) { pr_add("MUNKI: COULD NOT CLAIM INTERFACE 0"); return; }
    munki::Firmware f;
    if (munki::get_firmware(&f)) {
        pr_munki = true;
        pr_add("MUNKI FW %d.%d  TICK %d  MININT %d", f.fw_major, f.fw_minor, f.tick_dur, f.min_int_count);
        pr_add("EEPROM %d BLOCKS x %d BYTES", f.ee_blocks, f.ee_block_size);
    } else {
        pr_add("MUNKI: FIRMWARE REQUEST FAILED");
    }
    char v[37];
    if (munki::get_version(v)) pr_add("VERSION %s", v);
    pr_ee_state = 0;
    if (pr_munki) {
        if (!munki::read_calibration(&pr_ee)) {
            pr_ee_state = 3;
            pr_add("EEPROM READ FAILED (SIZE %d)", pr_ee.size);
        } else {
            pr_ee_state = pr_ee.sum_ok ? 1 : 2;
            pr_add("EEPROM %d BYTES  CAL V%d (COMPAT %d)", pr_ee.size, pr_ee.cal_version, pr_ee.compat_version);
            pr_add("CHECKSUM %08X / %08X  %s", (unsigned)pr_ee.sum_stored, (unsigned)pr_ee.sum_calc, pr_ee.sum_ok ? "OK" : "BAD");
        }
    }
    uint8_t id[8];
    if (munki::get_chip_id(id)) pr_add("CHIP %02X-%02X%02X%02X%02X%02X%02X%02X", id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
}

// Instrument button: on CALIBRATION it takes the dark reference; on AMBIENT / SURFACE / PROJECTOR it measures the light
// (ambient = diffuser, surface = open emission, projector = projector lens) and shows the 36 band spectrum on page 2.
static void pr_show();
static const char *mode_name(munki::Mode m) { return m == munki::MODE_AMBIENT ? "AMBIENT" : m == munki::MODE_EMISSION ? "EMISSION" : "PROJECTOR"; }
static void pr_measure(uint8_t pos) {
    if (pos == munki::POS_CALIB) {
        pr_msg = "DARK CALIBRATION...";
        pr_show();
        pr_msg = munki::dark_calibrate() ? "" : "DARK CALIBRATION FAILED";
        return;
    }
    munki::Mode mode;
    if (pos == munki::POS_AMBIENT) mode = munki::MODE_AMBIENT;
    else if (pos == munki::POS_SURF) mode = munki::MODE_EMISSION;
    else if (pos == munki::POS_PROJ) mode = munki::MODE_PROJECTOR;
    else { pr_msg = "TURN DIAL TO A MEASURING POSITION"; return; }
    pr_msg = "MEASURING...";
    pr_show();
    munki::Result r = munki::measure_spectrum(mode, &pr_spec);
    if (r == munki::MR_OK || r == munki::MR_LOW_LIGHT) {
        pr_have_spec = true;
        pr_spec_mode = mode;
        pr_msg = r == munki::MR_LOW_LIGHT ? "LOW LIGHT - RESULT IS NOISY" : "";
        pr_page = 1;
        ESP_LOGI(TAG, "spectrum %s: T %.3f s n %d peak %d nm", mode_name(mode), pr_spec.inttime, pr_spec.nmeas, pr_spec.peak_nm);
        for (int j = 0; j < 36; j++) ESP_LOGI(TAG, "  %d nm  %g", 380 + j * 10, pr_spec.v[j]);
    } else {
        pr_msg = munki::result_name(r);
    }
}

static void pr_plot() {
    float mx = 0;
    for (int j = 0; j < 36; j++) if (pr_spec.v[j] > mx) mx = pr_spec.v[j];
    if (mx <= 0) mx = 1;
    const int X0 = 16, BW = 8, BASE = 128, PH = 88;
    for (int j = 0; j < 36; j++) {
        float v = pr_spec.v[j] < 0 ? 0 : pr_spec.v[j];
        int h = (int)(PH * v / mx);
        fill(X0 + j * BW, BASE - h, BW - 1, h, wl_color(380 + j * 10));
    }
    for (int x = X0; x < X0 + 36 * BW; x++) px(x, BASE + 1, GREY);
    text(X0, BASE + 4, "380", 1, GREY);
    text(X0 + 17 * BW, BASE + 4, "550", 1, GREY);
    text(X0 + 32 * BW, BASE + 4, "730", 1, GREY);
}

static void pr_frame(const char *right, uint16_t rc) {
    clear();
    text(6, 4, "COLORMUNKI PROBE", 2, WHITE);
    (void)right;
    (void)rc;
    draw_battery();
    for (int x = 0; x < W; x++) px(x, 22, GREY);
}

static void pr_waiting() {
    pr_frame("", GREY);
    text(6, 40, "PLUG IN THE COLORMUNKI", 2, YELLOW);
    text(6, 64, "USB-C TO USB-A ADAPTER", 1, GREY);
    textf(6, 80, 1, otg_on ? GREEN : RED, "5 V BOOST %s", otg_on ? "ON" : "OFF");
    textf(6, 160, 1, GREY, "FW %s", FW_VERSION);
    present();
}

// Page 0 is the big live status (when a ColorMunki answers); the USB detail pages follow it.
static void pr_show() {
    pr_frame("", GREY);
    int base = pr_munki ? (pr_have_spec ? 2 : 1) : 0;
    int dpages = pr_n ? (pr_n + PR_PER_PAGE - 1) / PR_PER_PAGE : 1;
    int pages = base + dpages;
    if (pr_page >= pages) pr_page = 0;
    if (pr_munki && pr_page == 0) {
        text(6, 28, "DIAL", 1, GREY);
        text(6, 38, pr_spos == 0xFF ? "-" : munki::pos_name(pr_spos), 4, YELLOW);
        textf(6, 74, 2, pr_but ? GREEN : YELLOW, "BUTTON %s", pr_but ? "PRESSED" : "-");
        textf(6, 94, 2, munki::dark_valid() ? GREEN : GREY, "DARK CAL %s", munki::dark_valid() ? "OK" : "NEEDED");
        if (pr_have_spec) textf(6, 112, 2, WHITE, "%s PEAK %d NM", mode_name(pr_spec_mode), pr_spec.peak_nm);
        if (*pr_msg) text(6, 134, pr_msg, 1, RED);
        else text(6, 134, "CAL + BUTTON = DARK CAL.  AMB/SURF/PROJ + BUTTON = MEASURE", 1, GREY);
    } else if (pr_munki && pr_have_spec && pr_page == 1) {
        textf(6, 28, 1, WHITE, "%s  PEAK %d NM  T %.2f S  N %d", mode_name(pr_spec_mode), pr_spec.peak_nm, pr_spec.inttime, pr_spec.nmeas);
        pr_plot();
        if (*pr_msg) text(6, 148, pr_msg, 1, RED);
    } else {
        int dp = pr_page - base;
        for (int i = 0; i < PR_PER_PAGE; i++) {
            int k = dp * PR_PER_PAGE + i;
            if (k >= pr_n) break;
            text(4, 28 + i * 9, pr_txt[k], 1, WHITE);
        }
    }
    textf(4, 160, 1, GREY, "PAGE %d/%d  BUTTON = NEXT  FW %s", pr_page + 1, pages, FW_VERSION);
    present();
}

extern "C" void app_main() {
    gpio_config_t bi = {};
    bi.mode = GPIO_MODE_INPUT;
    bi.pull_up_en = GPIO_PULLUP_ENABLE;
    bi.pin_bit_mask = (1ULL << PIN_BTN_A) | (1ULL << PIN_BTN_B);
    gpio_config(&bi);

    ESP_LOGI(TAG, "ColorMunki probe, firmware commit %s", FW_VERSION);
    lcd_init();
    battery_init();   // switches the 5 V boost on when nothing powers the USB-C port
    usb_start();
    usb_host_client_config_t cc = {};
    cc.is_synchronous = false;
    cc.max_num_event_msg = 5;
    cc.async.client_event_callback = pr_client_cb;
    cc.async.callback_arg = NULL;
    ESP_ERROR_CHECK(usb_host_client_register(&cc, &pr_client));
    pr_waiting();

    while (true) {
        usb_host_client_handle_events(pr_client, pdMS_TO_TICKS(100));
        if (pr_new_addr) {
            uint8_t addr = pr_new_addr;
            pr_new_addr = 0;
            if (pr_dev) {
                usb_host_device_close(pr_client, pr_dev);
                pr_dev = NULL;
            }
            if (usb_host_device_open(pr_client, addr, &pr_dev) == ESP_OK) {
                pr_describe();
                pr_query();
                pr_page = 0;
                pr_show();
            } else {
                pr_dev = NULL;
                ESP_LOGW(TAG, "could not open the new USB device");
            }
        }
        if (pr_gone) {
            pr_gone = false;
            if (pr_dev) {
                munki::close();
                usb_host_device_close(pr_client, pr_dev);
                pr_dev = NULL;
            }
            pr_munki = false;
            pr_waiting();
        }
        if (pr_dev && button_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
            if (button_pressed()) {
                pr_page++;
                pr_show();
                while (button_pressed()) vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        if (pr_munki && pr_dev) {   // live dial position and instrument button; button press on CALIBRATION measures
            static uint32_t last = 0;
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last >= 100) {
                last = now;
                uint8_t sp, bt;
                if (munki::get_status(&sp, &bt)) {
                    bool press = bt && !pr_but;
                    bool changed = (sp != pr_spos || bt != pr_but);
                    pr_spos = sp;
                    pr_but = bt;
                    if (press) {
                        pr_measure(sp);
                        changed = true;
                    } else if (changed && sp != pr_spos_prev && *pr_msg) {
                        pr_msg = "";
                    }
                    pr_spos_prev = sp;
                    if (changed) pr_show();
                }
            }
        }
        battery_tick();
    }
}

#else
// ------------------------------------------------------ ColorMunki mode ----
// A ColorMunki Photo/Design (USB 0971:2007) plugged into the USB port is driven with the Argyll-derived driver in
// munki_argyll.cpp. The phone sees the same Torch Bearer service: dial CALIBRATION + scan = dark calibration, dial AMBIENT +
// scan = a reading, sent as a 1 nm spectrum interpolated from the instrument's 36 bands (380-730 nm, 10 nm).
#include "usb/usb_helpers.h"
#include "munki_argyll.h"
#include "munki_corr.h"   // MK_CORR_FITTED is needed by mk_flags_now() below

static usb_host_client_handle_t mk_client = NULL;
static usb_device_handle_t mk_dev = NULL;
static volatile uint8_t mk_pending[4] = {0, 0, 0, 0};
static volatile bool mk_gone = false;
static uint8_t mk_dial = 0xFF;
enum : uint8_t { MKR_TOO_BRIGHT = 1, MKR_COMMS = 2, MKR_NO_DARK = 3, MKR_NO_DATA = 4, MKR_WRONG_DIAL = 5, MKR_CAL_FAILED = 6 };
static uint8_t mk_reason = 0;
static const char *mk_msg = "";

static void mk_client_cb(const usb_host_client_event_msg_t *e, void *) {
    if (e->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        for (int i = 0; i < 4; i++) if (!mk_pending[i]) { mk_pending[i] = e->new_dev.address; break; }
    } else if (e->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        if (mk_dev && e->dev_gone.dev_hdl == mk_dev) mk_gone = true;
    }
}

static void mk_init() {
    usb_host_client_config_t cc = {};
    cc.is_synchronous = false;
    cc.max_num_event_msg = 5;
    cc.async.client_event_callback = mk_client_cb;
    cc.async.callback_arg = NULL;
    if (usb_host_client_register(&cc, &mk_client) != ESP_OK) { mk_client = NULL; return; }
    uint8_t list[8];
    int n = 0;
    if (usb_host_device_addr_list_fill(sizeof list, list, &n) == ESP_OK)   // devices already attached at boot
        for (int i = 0; i < n && i < 4; i++) mk_pending[i] = list[i];
}

static uint16_t mk_age_min() {
    uint32_t s = munki::dark_age_s();
    if (s == 0xFFFFFFFFu) return 0xFFFF;
    uint32_t m = s / 60;
    return (uint16_t)(m > 0xFFFE ? 0xFFFE : m);
}

static uint8_t mk_flags_now() {
    return (uint8_t)((mk_dial <= 3 ? (0x80 | mk_dial) : 0) | (munki::dark_valid() ? 0x04 : 0) |
                     (munki::high_gain_ready() ? 0x08 : 0) |   // bit3: high-gain dark references taken
                     (munki::hires_ready() ? 0x20 : 0) |      // bit5: high resolution (3.33 nm) readings
                     ((MK_CORR_FITTED && MK_CORR_HR_FITTED) ? 0x40 : 0));   // bit6: spectrum corrected on board (munki_corr.h)
}

// Looks at newly attached devices; returns true once a ColorMunki is open and its calibration data is read.
static bool mk_try_open() {
    if (!mk_client) return false;
    usb_host_client_handle_events(mk_client, 0);
    for (int i = 0; i < 4; i++) {
        uint8_t a = mk_pending[i];
        if (!a) continue;
        mk_pending[i] = 0;
        usb_device_handle_t d = NULL;
        if (usb_host_device_open(mk_client, a, &d) != ESP_OK) continue;
        const usb_device_desc_t *dd = NULL;
        if (usb_host_get_device_descriptor(d, &dd) != ESP_OK || !dd || dd->idVendor != 0x0971 || dd->idProduct != 0x2007) {
            usb_host_device_close(mk_client, d);
            continue;
        }
        mk_dev = d;
        mk_gone = false;
        munki::Eeprom ee;
        munki::Firmware fw;
        if (munki::open(mk_client, mk_dev) && munki::get_firmware(&fw) && munki::read_calibration(&ee) && ee.sum_ok) {
            ESP_LOGI(TAG, "ColorMunki: firmware %d.%d, calibration %d bytes (v%d)", fw.fw_major, fw.fw_minor, ee.size, ee.cal_version);
            return true;
        }
        ESP_LOGW(TAG, "ColorMunki found but could not be read");
        munki::close();
        usb_host_device_close(mk_client, mk_dev);
        mk_dev = NULL;
    }
    return false;
}

// 36 bands at 10 nm -> 351 points at 1 nm (Catmull-Rom, clamped at zero), the grid the rest of the bridge and the app use.
// The ColorMunki code yields mW/m^2/nm; the app (like the Torch Bearer path) expects W/m^2/nm, hence MK_UNITS.
static const float MK_UNITS = 0.001f;
// Catmull-Rom from n bands (band k at first + k * step nm) onto the 1 nm grid 380..730, clamped at zero.
static void mk_to_1nm(const float *v, int n, double first, double step, float *out) {
    for (int i = 0; i < 351; i++) {
        double x = (380 + i - first) / step;
        int k = (int)floor(x);
        if (k < 0) k = 0;
        if (k > n - 2) k = n - 2;
        float t = (float)(x - k);
        float p0 = v[k > 0 ? k - 1 : 0], p1 = v[k], p2 = v[k + 1], p3 = v[k + 2 < n ? k + 2 : n - 1];
        float r = 0.5f * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t);
        out[i] = r < 0 ? 0 : r;
    }
}

static double mk_lum(const float *spd) {   // relative photopic luminance of a 380..730 nm, 1 nm spectrum
    double y = 0;
    for (int i = 0; i < 351; i++) y += spd[i] * tbm::interp5(CMF_Y, 380 + i);
    return y;
}

// The reading as the app gets it: 1 nm, 380-730 nm, W/m^2/nm. Normally the high-resolution (3.33 nm) spectrum,
// scaled to the standard one's luminance (so lux stays on X-Rite's calibration; the two differ by a constant ~25 %);
// in low light the standard 10 nm one, because the narrower bands are noisier there. Each has its own HPCS-fitted
// shape correction (src/munki_corr.h). The app never applies its own curve to ColorMunki readings.
static bool mk_last_hires = false;
static void mk_fill_scan(const munki::Spectrum &s, bool low_light) {
    static float std1[351], hr1[351];
    scan.start_nm = 380;
    scan.npts = 351;
    scan.exposure_ms = (float)(s.inttime * 1000.0);
    scan.status = low_light ? 2 : 0;
    mk_to_1nm(s.v, 36, 380.0, 10.0, std1);
    mk_last_hires = s.hires && !low_light;
    const float *src = std1;
    if (mk_last_hires) {
        int n = munki::hires_bands();
        mk_to_1nm(munki::last_hires(), n, munki::hires_wavelength(0), munki::hires_wavelength(1) - munki::hires_wavelength(0), hr1);
        double ys = mk_lum(std1), yh = mk_lum(hr1);
        float k = (ys > 0 && yh > 0) ? (float)(ys / yh) : 1.0f;
        for (int i = 0; i < 351; i++) hr1[i] *= k;
        src = hr1;
    }
    for (int i = 0; i < 351; i++) {
        float c = 1.0f;
#if MK_CORR_HR_FITTED
        if (mk_last_hires) c = MK_CORR_HR[i];
#endif
#if MK_CORR_FITTED
        if (!mk_last_hires) c = MK_CORR[i];
#endif
        scan.spd[i] = src[i] * c * MK_UNITS;
    }
}

// Raw data behind a reading, for developing the high-resolution mode offline (tools/munki_diag.py decodes it).
// id 1, version 1: u8 version | u8 gain | u16 readings averaged | f32 integration time (s) | f32 absraw[128]
// (dark-subtracted, linearised sensor values per second) | f32 band[36] (380-730 nm, before the unit change).
// The calibration block (id 2) goes once per connection, before the first reading's data.
static uint32_t mk_cal_sent_for = 0;
static void mk_send_diag(const munki::Spectrum &s) {
    if (ble_conn == BLE_HS_CONN_HANDLE_NONE) return;
    if (mk_cal_sent_for != ble_conn_count) {
        ble_send_blob(2, munki::calibration_data(), (size_t)munki::calibration_size());
        mk_cal_sent_for = ble_conn_count;
    }
    static uint8_t b[8 + 128 * 4 + 36 * 4];
    b[0] = 1;
    b[1] = (uint8_t)s.gain;
    uint16_t n = (uint16_t)s.nmeas;
    float t = (float)s.inttime;
    memcpy(b + 2, &n, 2);
    memcpy(b + 4, &t, 4);
    memcpy(b + 8, munki::last_absraw(), 128 * 4);
    memcpy(b + 8 + 128 * 4, s.v, 36 * 4);
    ble_send_blob(1, b, sizeof b);
}

static const char *mk_dial_name(uint8_t d) {
    return d == munki::POS_CALIB ? "CALIBRATION" : d == munki::POS_AMBIENT ? "AMBIENT" : d == munki::POS_SURF ? "SURFACE" : d == munki::POS_PROJ ? "PROJECTOR" : "-";
}

static void ui_munki_ready() {
    clear();
    shown = 1;
#ifdef BOARD_PRO
    arrow_label = true;
#endif
    const int L = lpad();
    header("READY", GREEN);
    text(L + 6, 30, "DIAL", 1, GREY);
    text(L + 6, 40, mk_dial_name(mk_dial), 3, YELLOW);
    bool dv = sim_kind == 2 ? sim_dark : munki::dark_valid();
    text(L + 6, 70, dv ? "CALIBRATED" : "NOT CALIBRATED", 2, dv ? GREEN : YELLOW);
    if (*mk_msg) text(L + 6, 94, mk_msg, 1, RED);
    else if (mk_dial == munki::POS_CALIB) text(L + 6, 94, "PRESS BUTTON TO CALIBRATE", 1, CYAN);
    else if (mk_dial == munki::POS_AMBIENT) text(L + 6, 94, dv ? "PRESS BUTTON TO MEASURE" : "CALIBRATE FIRST (DIAL TO CALIBRATION)", 1, dv ? WHITE : YELLOW);
    else text(L + 6, 94, "SET THE DIAL TO AMBIENT (OR CALIBRATION)", 1, RED);
    draw_btn_note();
    draw_btn_arrow();
    textf(arrow_bottom_left() ? 30 : 6, 160, 1, GREY, "FW %s", fw_label());
    present();
}

static void ui_munki_busy(const char *what) {
    clear();
    header("WORKING", YELLOW);
    text(10, 60, what, 2, WHITE);
    text(10, 90, "KEEP THE DIAL STILL", 1, GREY);
    present();
}

static void mk_fail(uint8_t reason, const char *msg) {
    mk_reason = reason;
    mk_msg = msg;
    ble_status(5, reason, 0);
    ui_munki_ready();
    // Let the phone see the failure, then go back to ready so a later
    // connection does not start out reading a stale error.
    vTaskDelay(pdMS_TO_TICKS(300));
    ble_status(0, 0, 0);
}

static void mk_do(uint8_t dial) {
    mk_msg = "";
    if (scan_cancelled()) { ble_status(0, 0, 0); ui_munki_ready(); return; }   // cancelled before it started
    if (dial == munki::POS_CALIB) {
        ble_status(1, 0, 0);
        ui_munki_busy("CALIBRATING...");
        bool ok = munki::dark_calibrate();
        mk_dial = dial;
        meter_flags = mk_flags_now();
        if (!ok && scan_cancelled()) { mk_msg = "CANCELLED"; ble_status(0, 0, 0); ui_munki_ready(); return; }
        if (!ok) { mk_fail(MKR_CAL_FAILED, munki::last_error()); return; }
        ble_status(0, 0, 0);
        ui_munki_ready();
    } else if (dial == munki::POS_AMBIENT) {
        if (!munki::dark_valid()) { mk_fail(MKR_NO_DARK, "CALIBRATE FIRST (DIAL TO CALIBRATION)"); return; }
        ble_status(1, 0, 0);
        ui_munki_busy("MEASURING...");
        munki::Spectrum s;
        munki::Result r = munki::measure_spectrum(munki::MODE_AMBIENT, &s);
        if (r == munki::MR_CANCELLED) {
            mk_msg = "CANCELLED";
            ble_status(0, 0, 0);
            ui_munki_ready();
        } else if (r == munki::MR_OK || r == munki::MR_LOW_LIGHT) {
            mk_fill_scan(s, r == munki::MR_LOW_LIGHT);
            have_result = true;
            ui_result(true);
            ble_status(2, 0, scan.exposure_ms);
            ble_send_result();
            mk_send_diag(s);
        } else if (r == munki::MR_TOO_BRIGHT) mk_fail(MKR_TOO_BRIGHT, "TOO BRIGHT");
        else if (r == munki::MR_NO_DARK) mk_fail(MKR_NO_DARK, "CALIBRATE FIRST (DIAL TO CALIBRATION)");
        else if (r == munki::MR_COMMS) mk_fail(MKR_COMMS, munki::last_error());
        else mk_fail(MKR_NO_DATA, "MEASUREMENT FAILED");
    } else {
        mk_fail(MKR_WRONG_DIAL, "SET THE DIAL TO AMBIENT (OR CALIBRATION)");
    }
}

// Runs while a ColorMunki is attached; returns when it is unplugged.
static void mk_session() {
    munki::set_cancel_check(scan_cancelled);
    meter_kind = 1;
    adv_set_attached(2);
    meter_cal_age_min = mk_age_min;
    strncpy(device_id, "COLORMUNKI", sizeof device_id);
    range_lo = 380;
    range_hi = 730;
    mk_dial = 0xFF;
    mk_msg = "";
    meter_flags = mk_flags_now();
    ble_status(0, 0, 0);
    ui_munki_ready();
    uint8_t bt_prev = 0;
    int fails = 0;
    while (!mk_gone && fails < 5) {
        uint8_t sp = 0xFF, bt = 0;
        if (munki::get_status(&sp, &bt)) fails = 0; else { fails++; sp = mk_dial; }
        bool dial_changed = sp != mk_dial;
        if (dial_changed) {
            mk_dial = sp;
            mk_msg = "";
        }
        uint8_t fl = mk_flags_now();
        if (fl != meter_flags) {
            meter_flags = fl;
            ble_status_refresh();
        }
        bool trigger = false, from_button = false;
        if (ble_scan_req) trigger = true;
        else if (bt && !bt_prev) trigger = true;
        else if (button_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
            trigger = from_button = button_pressed();
            if (trigger) { btn_seen = button_mask(); btn_note(btn_seen); btn_short_press(); }   // held 2 s: deep sleep
        }
        bt_prev = bt;
        if (ble_resend_req) { ble_resend_req = false; ble_send_result(); }
        if (trigger) {
            // A reading started from the phone counts from its request; one started with a button counts from now.
            scan_started_us = ble_scan_req ? ble_scan_req_us : esp_timer_get_time();
            ble_scan_req = false;
            mk_do(mk_dial);
            meter_flags = mk_flags_now();
            ble_status_refresh();
            if (from_button) while (button_pressed()) vTaskDelay(pdMS_TO_TICKS(20));
            bt_prev = 1;   // don't re-trigger on an instrument button still held
        } else if (dial_changed && shown == 1) {
            ui_munki_ready();
        }
        flip_tick();
        btn_tick();
        battery_tick();
        link_tick();
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    munki::close();
    if (mk_dev) { usb_host_device_close(mk_client, mk_dev); mk_dev = NULL; }
    meter_kind = 0;
    meter_cal_age_min = NULL;
    meter_flags = 0;
    device_id[0] = 0;
    range_lo = 340;
    range_hi = 1000;
    have_result = false;
}

static void tb_live_emit() {
    if (scan.npts > 0) apply_correction(scan.spd, scan.npts, scan.start_nm);   // HPCS-fitted, before display + BLE
    have_result = true;
    ble_status(2, 0, scan.exposure_ms);
    ble_send_result();
    ble_status(1, 0, 0);   // still going: the phone sees state 2 (a result), then 1 again
}
static bool tb_live_stop() { return button_pressed(); }

// ----------------------------------------------------- TKLamp tester ----
// The TKLamp TK2303D integrating-sphere tester (CH340 serial, 9600 8N1) streams one 15-byte frame about every 220 ms while
// it is plugged in, without being asked:
//   a5 0a af a5 09 ae 5a 5a 09 04 00 | v2 v1 v0 | cs        (v = 24-bit big-endian sensor count, cs = v2+v1+v0-0x3f)
// On a calibrated tester lumens = count / ~313.2 (measured against its 2150 lm calibration lamp). The firmware passes the
// raw count on (BLE packet 06) and only converts for the screen; the app owns the scale.
static const uint8_t TK_HDR[11] = {0xA5, 0x0A, 0xAF, 0xA5, 0x09, 0xAE, 0x5A, 0x5A, 0x09, 0x04, 0x00};
#define TK_FRAME 15
#define TK_RAW_PER_LM 313.2f

static void usb_set_baud(uint32_t baud) {
    if (!vcp) return;
    cdc_acm_line_coding_t lc = {};
    lc.dwDTERate = baud;
    lc.bCharFormat = 0;
    lc.bParityType = 0;
    lc.bDataBits = 8;
    if (vcp->line_coding_set(&lc) != ESP_OK) ESP_LOGW(TAG, "line_coding_set %u failed", (unsigned)baud);
}

static uint8_t tkb[64];
static size_t tkn = 0;
// Pull bytes from the USB stream and return the next valid frame's 24-bit count, or -1 if none is complete yet.
static int64_t tk_next(int wait_ms) {
    while (true) {
        while (tkn >= TK_FRAME) {
            if (memcmp(tkb, TK_HDR, sizeof TK_HDR) == 0 &&
                (uint8_t)(tkb[11] + tkb[12] + tkb[13] - 0x3F) == tkb[14]) {
                int64_t v = ((int64_t)tkb[11] << 16) | (tkb[12] << 8) | tkb[13];
                memmove(tkb, tkb + TK_FRAME, tkn - TK_FRAME);
                tkn -= TK_FRAME;
                return v;
            }
            memmove(tkb, tkb + 1, --tkn);   // resync one byte at a time
        }
        size_t got = xStreamBufferReceive(rx_stream, tkb + tkn, sizeof tkb - tkn, pdMS_TO_TICKS(wait_ms));
        if (!got) return -1;
        tkn += got;
        wait_ms = 1;   // got bytes: only look again briefly
    }
}

/** After a CH340 opened and did not answer as a Torch Bearer: is it the TKLamp tester? Listens at 9600 baud for a valid
 *  frame. Leaves the port at 115200 on a miss, so the next attempt starts from the usual state. */
static bool tk_probe() {
    usb_set_baud(9600);
    xStreamBufferReset(rx_stream);
    tkn = 0;
    int64_t until = esp_timer_get_time() + 2000000;
    bool found = false;
    while (esp_timer_get_time() < until) {
        if (tk_next(200) >= 0) { found = true; break; }
        if (usb_lost) break;
    }
    if (!found) {
        usb_set_baud(115200);
        if (vcp) vcp->set_control_line_state(true, true);   // as usb_try_open() leaves it, in case the baud change touched the lines
        xStreamBufferReset(rx_stream);
        tkn = 0;
    }
    return found;
}

static void ui_tk(float lm, bool have, bool stale) {
    clear();
    if (sim_kind == 3) arrow_label = true;   // simulated tester: the RUN / NEXT button labels are drawn
    header(stale ? "NO DATA" : "LIVE", stale ? RED : GREEN);
    if (have && !stale) {
        char s[16];
        snprintf(s, sizeof s, lm >= 100.0f ? "%.0f" : "%.1f", lm);   // like the app: one decimal below 100 lm
        int w = (int)strlen(s) * 36;
        text((W - w) / 2, 52, s, 6, WHITE);
        text(W / 2 - 12, 104, "LM", 2, GREY);
    } else {
        text(20 + lpad(), 70, stale ? "TESTER STOPPED SENDING" : "WAITING FOR DATA", 2, YELLOW);
    }
    if (sim_kind == 3) {   // what the two buttons do (the arrows point at them)
        text(6 + lpad(), 108, sim_tk_run ? "TOP: STOP RUN" : "TOP: START RUN", 1, sim_tk_run ? RED : YELLOW);
        if (!sim_tk_run) text(6 + lpad(), 120, "BOTTOM: NEXT LEVEL", 1, YELLOW);
    }
    draw_btn_note();
    textf(6, 160, 1, GREY, "FW %s", fw_label());
    diag_on = true;
    draw_diag();
    present();
}

// Runs while a TKLamp tester is attached; returns when it is unplugged.
static void tk_session() {
    meter_kind = 2;
    adv_set_attached(3);
    strncpy(device_id, "TKLAMP", sizeof device_id);
    have_result = false;
    ble_scan_req = false;
    ble_status(0, 0, 0);
    ui_tk(0, false, false);
    float hist[5] = {0};
    int nh = 0, hi = 0;
    int64_t last_rx = esp_timer_get_time(), last_draw = 0;
    bool stale = false, drew_stale = false;
    while (!usb_lost) {
        int64_t v = tk_next(100);
        int64_t now = esp_timer_get_time();
        if (v >= 0) {
            last_rx = now;
            stale = false;
            uint8_t pk[9] = {0x06};
            uint32_t ms = (uint32_t)(now / 1000), raw = (uint32_t)v;
            memcpy(pk + 1, &ms, 4);      // little-endian on the ESP32-S3
            memcpy(pk + 5, &raw, 4);
            ble_notify(res_handle, res_sub, pk, sizeof pk);
            hist[hi] = raw / TK_RAW_PER_LM;
            hi = (hi + 1) % 5;
            if (nh < 5) nh++;
            if (now - last_draw > 300000) {   // about 3 screen updates a second, on a 5-reading (about 1 s) average
                float s = 0;
                for (int i = 0; i < nh; i++) s += hist[i];
                ui_tk(s / nh, true, false);
                last_draw = now;
                drew_stale = false;
            }
        } else if (now - last_rx > 5000000) {
            stale = true;
            if (!drew_stale) { ui_tk(0, false, true); drew_stale = true; }
        }
        ble_scan_req = false;       // no scan command on this instrument
        ble_resend_req = false;
        flip_tick();
        btn_tick();
        battery_tick();
        link_tick();
    }
    meter_kind = 0;
    device_id[0] = 0;
    have_result = false;
}

// -------------------------------------------------------------- main ----
// ----------------------------------------------- developer simulation ----
// BLE command 07 <kind> makes the board behave like an instrument that is not plugged in, so each instrument's screens
// can be worked on without the instrument: Torch Bearer (READY / SCANNING / DONE, LIVE), ColorMunki (dial, calibration,
// measuring, errors) and the TKLamp tester (live lumens, waiting, no data). Readings are made up. It lives in RAM only, so a
// restart ends it, and while it is on a plugged-in instrument is ignored (set the kind back to 0 to use it). The board
// advertises and reports itself as that instrument, so the app connects to it as usual. Physical button presses do what they
// would on the real instrument and are also sent to the app (result packet 07), so the app can show them.

// Result packet 08 01: Live was started with the bridge's own button, so the app can go into Live too (it ends by the usual
// "ready" status). Not sent when the phone asked for Live itself.
static void ble_live_by_button() {
    uint8_t pk[2] = {0x08, 0x01};
    ble_notify(res_handle, res_sub, pk, sizeof pk);
}

// Reports a physical button press (pressed = 1) or release (pressed = 0, with how long it was held) to the app.
static void sim_button_event(uint8_t which, bool pressed, uint16_t held_ms) {
    uint8_t pk[9] = {0x07, which, (uint8_t)(pressed ? 1 : 0)};
    uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    memcpy(pk + 3, &ms, 4);
    memcpy(pk + 7, &held_ms, 2);
    ble_notify(res_handle, res_sub, pk, sizeof pk);
    ESP_LOGI(TAG, "sim: %s button %s", which == 1 ? "top" : "bottom", pressed ? "down" : "up");
}

// A physical press, reported once it has been let go: 0 none, 1 top button, 2 bottom button (roles follow the screen
// orientation, as on the real instruments). Held 2 s the board goes to sleep, as usual.
static uint8_t sim_poll() {
    if (!ota_active && sim_inject) {   // pressed from the phone: handled exactly like the physical button, including the press / release reports
        uint8_t w = sim_inject;
        sim_inject = 0;
        sim_button_event(w, true, 0);
        vTaskDelay(pdMS_TO_TICKS(80));
        sim_button_event(w, false, 80);
        return w;
    }
    if (ota_active || !button_pressed()) return 0;
    vTaskDelay(pdMS_TO_TICKS(30));   // debounce
    bool t = btn_top_pressed(), b = btn_bottom_pressed();
    if (!t && !b) return 0;
    uint8_t which = (t && !b) ? 1 : 2;
    btn_seen = button_mask();
    btn_note(btn_seen);
    sim_button_event(which, true, 0);
    int64_t t0 = esp_timer_get_time();
    btn_short_press();   // waits for the release
    sim_button_event(which, false, (uint16_t)((esp_timer_get_time() - t0) / 1000));
    return which;
}

// A plausible white-LED spectrum, 1 nm steps, peak about 1: variants 0 warm, 1 neutral, 2 cool, with a little noise.
static void sim_fill_scan(int lo, int hi, int variant, float exposure_ms) {
    static uint32_t seed = 0x1234567;
    static const float P[3][6] = {
        {450, 11, 0.55f, 610, 58, 1.00f},    // warm
        {452, 11, 0.80f, 590, 55, 0.90f},    // neutral
        {450, 10, 1.00f, 565, 52, 0.62f},    // cool
    };
    const float *v = P[((variant % 3) + 3) % 3];
    int n = hi - lo + 1;
    if (n > NPTS_MAX) n = NPTS_MAX;
    scan.status = 0;
    scan.exposure_ms = exposure_ms;
    scan.npts = n;
    scan.start_nm = lo;
    for (int i = 0; i < n; i++) {
        float nm = (float)(lo + i);
        float b = (nm - v[0]) / v[1], p = (nm - v[3]) / v[4];
        seed = seed * 1664525u + 1013904223u;
        float noise = (((seed >> 16) & 0xFFFF) / 65535.0f - 0.5f) * 0.006f;
        float y = v[2] * expf(-0.5f * b * b) + v[5] * expf(-0.5f * p * p) + noise;
        scan.spd[i] = y > 0 ? y : 0;
    }
}

static uint16_t sim_cal_age() { return sim_dark ? (uint16_t)((esp_timer_get_time() - sim_dark_us) / 60000000) : 0xFFFF; }
static uint8_t sim_mk_flags() {
    return (uint8_t)((mk_dial <= 3 ? (0x80 | mk_dial) : 0) | (sim_dark ? 0x04 : 0) | 0x40);   // 0x40: spectrum already corrected
}

// Torch Bearer: READY, READ (top button or phone: one simulated reading), LIVE (bottom button or phone: only announced,
// only made-up readings are drawn; any press or the phone stops it).
// While a simulated Live read draws its readings (which takes a while), a task watches the buttons so a press is never
// missed: the first press seen is kept in sim_latch (1 top, 2 bottom) and stops the Live read.
static volatile uint8_t sim_latch = 0;
static volatile bool sim_latch_run = false, sim_latch_done = true;
static void sim_latch_task(void *) {
    while (sim_latch_run) {
        if (!sim_latch) {
            uint8_t w = sim_poll();
            if (w) sim_latch = w;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    sim_latch_done = true;
    vTaskDelete(NULL);
}

static void sim_tb_step(int &variant) {
    uint8_t b = sim_poll();
    bool live = ble_live_req || b == 2;
    bool scanning = ble_scan_req || b == 1;
    if (ble_resend_req) { ble_resend_req = false; ble_send_result(); }
    if (live) {
        const bool by_app = ble_live_req;
        scan_started_us = by_app ? ble_scan_req_us : esp_timer_get_time();
        ble_live_req = false;
        ble_scan_req = false;
        if (!by_app) ble_live_by_button();   // the app goes into Live as well
        ble_status(1, 0, 0);
        // Made-up readings, about four a second, drawn like the real Live screen and sent to the app.
        sim_live_banner = true;
        int n = 0;
        sim_latch = 0;
        sim_latch_done = false;
        sim_latch_run = true;
        if (xTaskCreate(sim_latch_task, "simbtn", 4096, NULL, 5, NULL) != pdPASS) { sim_latch_run = false; sim_latch_done = true; }
        while (!sim_latch && sim_kind == 1 && !sim_changed && !scan_cancelled() && !ota_active) {
            sim_fill_scan(340, 1000, variant, 14.0f);   // one light throughout; only the noise moves
            have_result = true;
            ble_status(2, 0, scan.exposure_ms);
            ble_send_result();   // the app (in Live) draws the same readings
            ui_result(true);
            n++;
            for (int i = 0; i < 6 && !sim_latch && sim_kind == 1 && !sim_changed && !scan_cancelled(); i++) {
                vTaskDelay(pdMS_TO_TICKS(30));
                flip_tick();   // a flip asked for from the phone applies during Live too
            }
        }
        sim_latch_run = false;
        while (!sim_latch_done) vTaskDelay(pdMS_TO_TICKS(10));
        sim_latch = 0;
        sim_live_banner = false;
        if (sim_kind == 1) { ui_ready(); ble_status(0, 0, 0); }
    } else if (scanning) {
        ble_scan_req = false;
        prog_mode = 3;
        ui_progress(1, 8.0f, 1);
        vTaskDelay(pdMS_TO_TICKS(450));
        ui_progress(2, 16.0f, 2);
        vTaskDelay(pdMS_TO_TICKS(450));
        ui_progress(3, 12.5f, 0);
        vTaskDelay(pdMS_TO_TICKS(450));
        prog_mode = 0;
        sim_fill_scan(340, 1000, variant++, 12.5f);
        have_result = true;
        ui_result(true);
        ble_status(2, 0, scan.exposure_ms);
        ble_send_result();
    }
    ble_scan_req = false;
    ble_resend_req = false;
}

// ColorMunki: the top button turns the "dial" (calibration, ambient, surface, projector); the bottom button or the phone
// does what the real button does at that dial position.
static void sim_mk_action(int &variant) {
    mk_msg = "";
    if (mk_dial == munki::POS_CALIB) {
        ble_status(1, 0, 0);
        ui_munki_busy("CALIBRATING...");
        vTaskDelay(pdMS_TO_TICKS(1500));
        sim_dark = true;
        sim_dark_us = esp_timer_get_time();
        meter_flags = sim_mk_flags();
        ble_status(0, 0, 0);
        ui_munki_ready();
    } else if (mk_dial == munki::POS_AMBIENT) {
        if (!sim_dark) { mk_fail(MKR_NO_DARK, "CALIBRATE FIRST (DIAL TO CALIBRATION)"); return; }
        ble_status(1, 0, 0);
        ui_munki_busy("MEASURING...");
        vTaskDelay(pdMS_TO_TICKS(1500));
        sim_fill_scan(380, 730, variant++, 20.0f);
        have_result = true;
        ui_result(true);
        ble_status(2, 0, scan.exposure_ms);
        ble_send_result();
    } else {
        mk_fail(MKR_WRONG_DIAL, "SET THE DIAL TO AMBIENT (OR CALIBRATION)");
    }
    meter_flags = sim_mk_flags();
    ble_status_refresh();
}

static void sim_mk_step(int &variant) {
    uint8_t b = sim_poll();
    if (ble_resend_req) { ble_resend_req = false; ble_send_result(); }
    if (b == 1) {
        static const uint8_t ORDER[4] = {munki::POS_CALIB, munki::POS_AMBIENT, munki::POS_SURF, munki::POS_PROJ};
        int i = 0;
        for (int k = 0; k < 4; k++) if (ORDER[k] == mk_dial) i = k;
        mk_dial = ORDER[(i + 1) % 4];
        mk_msg = "";
        meter_flags = sim_mk_flags();
        ble_status_refresh();
        ui_munki_ready();
    } else if (b == 2 || ble_scan_req) {
        ble_scan_req = false;
        sim_mk_action(variant);
    }
}

// TKLamp tester: the bottom button steps forward through five flashlight-like levels (moonlight to turbo), then WAITING FOR DATA and NO DATA; the
// top button is the app's run Start / Stop. The lumen readings drift a little and go to the phone as live packets (06), like the real tester.
static const float SIM_TK_LM[5] = {1.5f, 14.2f, 118.0f, 540.0f, 1250.0f};
static void sim_tk_draw(int st, float lm) {
    if (st < 5) ui_tk(lm, true, false);
    else if (st == 5) ui_tk(0, false, false);
    else ui_tk(0, false, true);
}
static void sim_tk_step(int &st, int64_t &last_us) {
    uint8_t b = sim_poll();
    int64_t now = esp_timer_get_time();
    if (b) {
        if (b == 2 && !sim_tk_run) st = (st + 1) % 7;   // the top button is the run's Start / Stop in the app (nothing to change here); no level changes during a run
        sim_tk_draw(st, st < 5 ? SIM_TK_LM[st] : 0);
        last_us = now;
    }
    if (st < 5 && now - last_us >= 220000) {
        last_us = now;
        static uint32_t seed = 99;
        seed = seed * 1664525u + 1013904223u;
        float lm = SIM_TK_LM[st] * (1.0f + (((seed >> 16) & 0xFFFF) / 65535.0f - 0.5f) * 0.02f);
        uint8_t pk[9] = {0x06};
        uint32_t ms = (uint32_t)(now / 1000), raw = (uint32_t)(lm * TK_RAW_PER_LM);
        memcpy(pk + 1, &ms, 4);
        memcpy(pk + 5, &raw, 4);
        ble_notify(res_handle, res_sub, pk, sizeof pk);
        static int64_t drawn = 0;
        if (now - drawn > 300000) { drawn = now; ui_tk(lm, true, false); }
    }
    ble_scan_req = false;
    ble_resend_req = false;
}

// Runs the simulated instrument until the simulation is changed or turned off.
static void sim_session() {
    uint8_t kind = sim_kind;
    sim_changed = false;
    sim_redraw = false;
    ble_scan_req = ble_live_req = ble_resend_req = false;
    have_result = false;
    int variant = 0, tk_state = 3;   // TKLamp sim starts at a normal high flashlight level (540 lm)
    int64_t tk_last = 0;
    ESP_LOGI(TAG, "simulating instrument %d", kind);
    if (kind == 1) {
        meter_kind = 0;
        strncpy(device_id, "SIMULATED TORCH BEARER", sizeof device_id);
        range_lo = 340;
        range_hi = 1000;
        adv_set_attached(1);
        ble_status(0, 0, 0);
        ui_ready();
    } else if (kind == 2) {
        meter_kind = 1;
        strncpy(device_id, "COLORMUNKI", sizeof device_id);
        range_lo = 380;
        range_hi = 730;
        sim_dark = true;                    // starts calibrated, dial on Ambient, so a reading can be taken at once
        sim_dark_us = esp_timer_get_time();
        mk_dial = munki::POS_AMBIENT;
        mk_msg = "";
        meter_cal_age_min = sim_cal_age;
        meter_flags = sim_mk_flags();
        adv_set_attached(2);
        ble_status(0, 0, 0);
        ui_munki_ready();
    } else {
        meter_kind = 2;
        strncpy(device_id, "TKLAMP", sizeof device_id);
        adv_set_attached(3);
        ble_status(0, 0, 0);
        sim_tk_draw(3, SIM_TK_LM[3]);
    }
    while (sim_kind == kind && !sim_changed) {
        if (ota_active) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        if (sim_redraw) {
            sim_redraw = false;
            if (kind == 1) ui_ready(); else if (kind == 2) ui_munki_ready(); else sim_tk_draw(tk_state, tk_state < 5 ? SIM_TK_LM[tk_state] : 0);
        }
        if (kind == 1) sim_tb_step(variant);
        else if (kind == 2) sim_mk_step(variant);
        else sim_tk_step(tk_state, tk_last);
        flip_tick();
        btn_tick();
        battery_tick();
        link_tick();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    meter_kind = 0;
    meter_cal_age_min = NULL;
    meter_flags = 0;
    device_id[0] = 0;
    range_lo = 340;
    range_hi = 1000;
    have_result = false;
    ble_scan_req = ble_live_req = false;
    sim_dark = false;
    prog_mode = 0;
    sim_live_banner = false;
}

extern "C" void app_main() {
    // Coming back from deep sleep: the pins held during the sleep (backlight, power enable) and the button pins the RTC
    // domain was watching go back to normal use.
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)PIN_LCD_BL);
#ifndef BOARD_PRO
    gpio_hold_dis((gpio_num_t)PIN_POWER_ON);
#endif
    rtc_gpio_deinit((gpio_num_t)PIN_BTN_A);
    rtc_gpio_deinit((gpio_num_t)PIN_BTN_B);
    gpio_config_t bi = {};
    bi.mode = GPIO_MODE_INPUT;
    bi.pull_up_en = GPIO_PULLUP_ENABLE;
    bi.pin_bit_mask = (1ULL << PIN_BTN_A) | (1ULL << PIN_BTN_B);
#ifdef BOARD_PRO
    bi.pin_bit_mask |= (1ULL << PIN_BTN_A2);
#endif
    gpio_config(&bi);

    ESP_LOGI(TAG, "firmware commit %s", FW_VERSION);
    lcd_init();
    battery_init();
    usb_start();
    mk_init();
    ble_start();
    ui_waiting();
    ota_validate_later();   // after a Bluetooth update: keep this firmware once it has run a while (else the bootloader goes back)

    while (true) {
        // 1. wait for the spectrometer (or a ColorMunki)
        while (!usb_connected) {
            if (sim_kind) {
                sim_session();   // developer simulation (BLE command 07): ignores real instruments until turned off
                if (!sim_kind) {
                    adv_set_attached(0);
                    ui_waiting();
                    ble_status(3, 0, 0);
                }
            } else if (mk_try_open()) {
                mk_session();
                adv_set_attached(0);
                ui_waiting();
                ble_status(3, 0, 0);
            } else if (usb_try_open() && tb_init()) {
                usb_connected = true;
                adv_set_attached(1);
                ESP_LOGI(TAG, "connected: %s  %d-%d nm", device_id, range_lo, range_hi);
            } else if (vcp && tk_probe()) {
                // Not a Torch Bearer (it did not answer), but a TKLamp tester is streaming. It is only named once sure.
                tk_session();           // TKLamp tester: streams on its own, no commands to send
                vcp.reset();
                adv_set_attached(0);
                ui_waiting();
                ble_status(3, 0, 0);
            } else {
                vcp.reset();
                if (ble_scan_req) { ble_scan_req = false; ble_status(3, 0, 0); }
                flip_tick();
                btn_tick();
                battery_tick();
                link_tick();
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        ui_ready();
        ble_status(0, 0, 0);

        // 2. scan on button press or BLE command until the cable is pulled
        while (usb_connected && !usb_lost) {
            bool trigger = false, from_button = false, live_from_button = false;
            if (ble_scan_req) {
                trigger = true;
            } else if (!ota_active && button_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));  // debounce
                bool top = btn_top_pressed(), bottom = btn_bottom_pressed();
                if (top || bottom) btn_short_press();   // held 2 s: deep sleep; otherwise the action starts when it is let go
                if (top || bottom) {
                    from_button = true;
                    btn_seen = button_mask(); btn_note(btn_seen);
                    if (bottom && !top) live_from_button = true;   // bottom button: Live
                    else trigger = true;                           // top button (or both): one read
                }
            }
            if (ble_resend_req) {
                ble_resend_req = false;
                ble_send_result();
            }
            if (ble_live_req || live_from_button) {
                // From the phone (06) the reading counts from its request; from the top button, from the moment it is let go.
                if (live_from_button) {
                    while (button_pressed()) vTaskDelay(pdMS_TO_TICKS(20));   // the press that starts Live must not also stop it
                    scan_started_us = esp_timer_get_time();
                } else {
                    scan_started_us = ble_scan_req_us;
                }
                const bool by_button = live_from_button;
                ble_live_req = false;
                ble_scan_req = false;
                if (by_button) ble_live_by_button();   // the app goes into Live as well
                prog_mode = 2;
                ui_scanning();
                ble_status(1, 0, 0);
                int n = tb_live(ui_progress, tb_live_emit, tb_live_stop);
                prog_mode = 0;
                if (usb_lost) break;
                ESP_LOGI(TAG, "live ended after %d results", n);
                have_result = scan.npts > 0;
                ui_result(n > 0);
                ble_status(n > 0 ? 0 : 4, 0, scan.exposure_ms);
                while (tb_live_stop()) vTaskDelay(pdMS_TO_TICKS(20));   // a stopping button press must not start a scan
                ble_scan_req = false;
            }
            if (trigger) {
                ble_scan_req = false;
                prog_mode = 1;
                ui_scanning();
                bool ok = tb_scan(ui_progress, 40);
                prog_mode = 0;
                if (usb_lost) break;
                have_result = ok || scan.npts > 0;
                if (scan.npts > 0) apply_correction(scan.spd, scan.npts, scan.start_nm);  // HPCS-fitted, before display + BLE
                ui_result(ok);
                ble_status(ok ? 2 : 4, 0, scan.exposure_ms);
                ble_send_result();
                if (from_button) while (button_pressed()) vTaskDelay(pdMS_TO_TICKS(20));
                // result stays on screen; the next press or BLE command starts a new scan
            }
            flip_tick();
            btn_tick();
            battery_tick();
            link_tick();
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 3. cable pulled
        usb_connected = false;
        adv_set_attached(0);
        vcp.reset();
        ui_waiting();
        ble_status(3, 0, 0);
    }
}

#endif  // COLORMUNKI_PROBE
