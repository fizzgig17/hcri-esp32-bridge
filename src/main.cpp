// Torch Bearer scanner for the LilyGo T-Display S3 (non-Pro).
//
// Press the right-hand button (GPIO14) or BOOT (GPIO0) to take one scan.
// The ESP32-S3 acts as a USB host and talks to the spectrometer's CH340
// USB-to-serial bridge (VID 1A86 / PID 7523) at 115200 8N1.
//
// Protocol and spectrum de-obfuscation are ported from the GPLv3 projects
// ZoidTechnology/Torch-Bearer-Tools and wejn/tobes-ui, so this firmware is
// GPLv3 as well.

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp.hpp"

#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "metrics.h"

using namespace esp_usb;

static const char *TAG = "tobes";

// ---------------------------------------------------------------- pins ----
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

// ------------------------------------------------------------- display ----
#define W 320
#define H 170

static uint16_t *fb;  // W*H RGB565, byte-swapped for the panel
static esp_lcd_panel_handle_t panel;

static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    return __builtin_bswap16(c);
}
static const uint16_t BLACK = 0, WHITE = rgb(255, 255, 255), GREY = rgb(110, 110, 110),
                      GREEN = rgb(60, 220, 90), RED = rgb(255, 70, 70),
                      YELLOW = rgb(255, 210, 50), CYAN = rgb(70, 200, 255);

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
    esp_lcd_panel_mirror(panel, false, true);
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

static void clear() { memset(fb, 0, W * H * 2); }

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

// ------------------------------------------------------------- BLE ----
// GATT service 7a1c0001-5b2e-4f0a-9c3d-2e8f6b4a1d00 ("Torch Bearer"):
//   0002 command  (write)  : 01 = scan now, 02 = resend last result
//   0003 result   (notify/read): summary packet, then the full spectrum in chunks
//   0004 status   (notify/read): [state u8][try u8][exposure_ms f32]
//        state: 0 ready, 1 scanning, 2 done, 3 error (no USB / timeout), 4 done but not locked
// Packets (all little-endian):
//   summary  : 01 | status u8 | peak_nm u16 | exposure_ms f32 | peak_val f32 | sum f32 | npts u16 | start_nm u16
//   header   : 02 | npts u16 | start_nm u16 | step_nm u8 | total_bytes u16     (then float32 values)
//   data     : 03 | seq u8 | bytes...                                          (seq counts from 0)
//   end      : 04 | chunks u8
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x01,0x00,0x1c,0x7a);
static const ble_uuid128_t CMD_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x02,0x00,0x1c,0x7a);
static const ble_uuid128_t RES_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x03,0x00,0x1c,0x7a);
static const ble_uuid128_t STA_UUID = BLE_UUID128_INIT(0x00,0x1d,0x4a,0x6b,0x8f,0x2e,0x3d,0x9c,0x0a,0x4f,0x2e,0x5b,0x04,0x00,0x1c,0x7a);

static uint16_t ble_conn = BLE_HS_CONN_HANDLE_NONE;
static uint16_t res_handle, sta_handle;
static bool res_sub = false, sta_sub = false;
static uint8_t own_addr_type;
static volatile bool ble_scan_req = false;
static volatile bool ble_resend_req = false;
static bool have_result = false;
static uint8_t last_state[6] = {0};

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

static int ble_gap_cb(struct ble_gap_event *ev, void *);
static void ble_advertise() {
    struct ble_hs_adv_fields f = {};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = (uint8_t *)"Torch Bearer";
    f.name_len = 12;
    f.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
    struct ble_hs_adv_fields r = {};
    r.uuids128 = (ble_uuid128_t *)&SVC_UUID;
    r.num_uuids128 = 1;
    r.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&r);
    struct ble_gap_adv_params ap = {};
    ap.conn_mode = BLE_GAP_CONN_MODE_UND;
    ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ap.itvl_min = BLE_GAP_ADV_ITVL_MS(100);  // advertise often so phone scans find it quickly
    ap.itvl_max = BLE_GAP_ADV_ITVL_MS(150);
    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &ap, ble_gap_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) ESP_LOGW(TAG, "adv start failed %d", rc);
}

static int ble_gap_cb(struct ble_gap_event *ev, void *) {
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            ble_conn = ev->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected");
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ble_conn = BLE_HS_CONN_HANDLE_NONE;
        res_sub = sta_sub = false;
        ble_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == res_handle) res_sub = ev->subscribe.cur_notify;
        if (ev->subscribe.attr_handle == sta_handle) sta_sub = ev->subscribe.cur_notify;
        break;
    default:
        break;
    }
    return 0;
}

static int ble_access_cb(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int which = (int)(intptr_t)arg;  // 0 command, 1 result, 2 status
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && which == 0) {
        uint8_t cmd = 0;
        if (OS_MBUF_PKTLEN(ctxt->om) >= 1 && os_mbuf_copydata(ctxt->om, 0, 1, &cmd) == 0) {
            if (cmd == 0x01) ble_scan_req = true;
            if (cmd == 0x02) ble_resend_req = true;
        }
        return 0;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t b[32];
        if (which == 2) return os_mbuf_append(ctxt->om, last_state, sizeof last_state) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        if (which == 1 && have_result) return os_mbuf_append(ctxt->om, b, summary_packet(b)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static struct ble_gatt_chr_def ble_chrs[4];
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
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
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
    memset(ble_svcs, 0, sizeof ble_svcs);
    ble_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    ble_svcs[0].uuid = &SVC_UUID.u;
    ble_svcs[0].characteristics = ble_chrs;

    ESP_ERROR_CHECK(ble_gatts_count_cfg(ble_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(ble_svcs));
    ble_svc_gap_device_name_set("Torch Bearer");
    ble_att_set_preferred_mtu(247);
    nimble_port_freertos_init(ble_host_task);
}

// ----------------------------------------------------------------- UI ----
static void header(const char *right, uint16_t rc) {
    text(6, 4, "TORCH BEARER", 2, WHITE);
    text(W - 6 - (int)strlen(right) * 12, 4, right, 2, rc);
    for (int x = 0; x < W; x++) px(x, 22, GREY);
}

static void ui_waiting() {
    clear();
    header("NO USB", RED);
    text(20, 70, "PLUG IN SPECTROMETER", 2, YELLOW);
    text(20, 100, "VIA OTG CABLE", 2, GREY);
    present();
}

static void ui_ready() {
    clear();
    header("READY", GREEN);
    text(6, 32, device_id, 1, GREY);
    textf(6, 48, 1, GREY, "RANGE %d-%d NM", range_lo, range_hi);
    text(20, 90, "PRESS BUTTON TO SCAN", 2, WHITE);
    present();
}

static void ui_progress(int tries, float exp_ms, int status) {
    ble_status(1, (uint8_t)tries, exp_ms);
    clear();
    header("SCANNING", YELLOW);
    textf(10, 50, 3, WHITE, "TRY %d", tries);
    textf(10, 90, 2, CYAN, "EXPOSURE %.1f MS", exp_ms);
    text(10, 115, status == 1 ? "TOO BRIGHT - ADJUSTING" : status == 2 ? "TOO DIM - ADJUSTING" : "OK", 2,
         status == 0 ? GREEN : YELLOW);
    present();
}

static void ui_scanning() { ui_progress(0, 0, 2); }

static void ui_result(bool ok) {
    int n = scan.npts;
    Stats t = stats_of(scan);
    float mx = t.mx;
    int pk = t.pk;
    Metrics m;
    bool have_m = compute_metrics(scan.spd, n, scan.start_nm, &m);
    clear();
    header(ok ? "DONE" : "NO LOCK", ok ? GREEN : RED);

    // colorimetry, computed on the board from the corrected spectrum
    if (have_m) {
        textf(6, 28, 2, WHITE, "CCT %.0f K", m.cct);
        textf(6, 46, 2, WHITE, "DUV %+.4f", m.duv);
        textf(6, 64, 2, WHITE, "RA %d  R9 %d", m.ra, m.r9);
        textf(196, 30, 1, CYAN, "X %.4f", m.x);
        textf(196, 42, 1, CYAN, "Y %.4f", m.y);
    } else {
        text(6, 40, "NO SIGNAL", 2, RED);
    }
    textf(196, 54, 1, GREY, "PEAK %d NM", scan.start_nm + pk);
    textf(196, 66, 1, GREY, "EXP %.1f MS", scan.exposure_ms);
    textf(6, 80, 1, scan.status == 0 ? GREY : YELLOW, "%s  %d PTS",
          scan.status == 0 ? "NORMAL" : scan.status == 1 ? "OVER-EXPOSED" : "UNDER-EXPOSED", n);

    // spectrum plot, x = wavelength, y = value (autoscaled to the peak)
    const int px0 = 10, pw = 300, py0 = 92, ph = 66;
    for (int x = 0; x < pw; x++) px(px0 + x, py0 + ph, GREY);
    if (mx > 0 && n > 1) {
        for (int x = 0; x < pw; x++) {
            int i = x * (n - 1) / (pw - 1);
            int h = (int)(scan.spd[i] / mx * (ph - 1));
            if (h < 0) h = 0;
            vline(px0 + x, py0 + ph - h, py0 + ph - 1, ok ? GREEN : RED);
        }
        int pkx = pk * (pw - 1) / (n - 1);
        vline(px0 + pkx, py0, py0 + ph, YELLOW);
    }
    textf(px0, 162, 1, GREY, "%d", range_lo);
    textf(px0 + pw / 2 - 9, 162, 1, GREY, "%d", (range_lo + range_hi) / 2);
    textf(px0 + pw - 24, 162, 1, GREY, "%d", range_hi);
    present();
}

static bool button_pressed() {
    return gpio_get_level((gpio_num_t)PIN_BTN_A) == 0 || gpio_get_level((gpio_num_t)PIN_BTN_B) == 0;
}

// -------------------------------------------------------------- main ----
extern "C" void app_main() {
    gpio_config_t bi = {};
    bi.mode = GPIO_MODE_INPUT;
    bi.pull_up_en = GPIO_PULLUP_ENABLE;
    bi.pin_bit_mask = (1ULL << PIN_BTN_A) | (1ULL << PIN_BTN_B);
    gpio_config(&bi);

    lcd_init();
    usb_start();
    ble_start();
    ui_waiting();

    while (true) {
        // 1. wait for the spectrometer
        while (!usb_connected) {
            if (usb_try_open() && tb_init()) {
                usb_connected = true;
                ESP_LOGI(TAG, "connected: %s  %d-%d nm", device_id, range_lo, range_hi);
            } else {
                vcp.reset();
                if (ble_scan_req) { ble_scan_req = false; ble_status(3, 0, 0); }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        ui_ready();
        ble_status(0, 0, 0);

        // 2. scan on button press or BLE command until the cable is pulled
        while (usb_connected && !usb_lost) {
            bool trigger = false, from_button = false;
            if (ble_scan_req) {
                trigger = true;
            } else if (button_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));  // debounce
                trigger = from_button = button_pressed();
            }
            if (ble_resend_req) {
                ble_resend_req = false;
                ble_send_result();
            }
            if (trigger) {
                ble_scan_req = false;
                ui_scanning();
                bool ok = tb_scan(ui_progress, 40);
                if (usb_lost) break;
                have_result = ok || scan.npts > 0;
                if (scan.npts > 0) apply_correction(scan.spd, scan.npts, scan.start_nm);  // HPCS-fitted, before display + BLE
                ui_result(ok);
                ble_status(ok ? 2 : 4, 0, scan.exposure_ms);
                ble_send_result();
                if (from_button) while (button_pressed()) vTaskDelay(pdMS_TO_TICKS(20));
                // result stays on screen; the next press or BLE command starts a new scan
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 3. cable pulled
        usb_connected = false;
        vcp.reset();
        ui_waiting();
        ble_status(3, 0, 0);
    }
}
