// PC-side simulation of src/munki_argyll.cpp: a fake ColorMunki behind fake ESP-IDF USB calls.
// Checks dark calibration (both gains), the integration-time / gain choice in bright, dim and very dim light,
// saturation, exact (tick-quantised) integration time, and that a timed-out transfer is never freed while queued.
//
//   g++ -std=gnu++17 -O1 -I tools/sim_stubs -I src -o /tmp/munki_sim tools/munki_sim_test.cpp && /tmp/munki_sim
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <set>
#include <string>
#include "usb/usb_host.h"

// ---------------------------------------------------------------- fake clock + instrument ----
static int64_t g_now_us = 0;
int64_t esp_timer_get_time(void) { return g_now_us; }

static const double TICK_S = 0.0000910;      // one integration clock tick
static const int MIN_COUNT = 110;            // -> minimum integration time ~10 ms
static const double HIGHGAIN = 4.0;          // high gain ratio (lin1[1] = 1/4)
static const int SAT = 60000;

static std::vector<uint8_t> g_eeprom(8192);
static double g_light = 0;                   // counts per second at normal gain on every band (peak band 1.0x)
static bool g_hang_next_bulk = false;        // simulate a bulk read that never completes
static int g_trig_clocks = 0, g_trig_n = 0, g_trig_gain = 0;
static std::vector<uint8_t> g_stream;        // pending bulk IN bytes
static int g_eeprom_addr = 0, g_eeprom_size = 0;
static bool g_eeprom_pending = false;

static void put32(uint8_t *b, uint32_t v) { b[0] = v; b[1] = v >> 8; b[2] = v >> 16; b[3] = v >> 24; }
static void putf(int off, float f) { uint32_t v; memcpy(&v, &f, 4); put32(&g_eeprom[off], v); }
static void put16(int off, int v) { g_eeprom[off] = v & 0xFF; g_eeprom[off + 1] = (v >> 8) & 0xFF; }

static void build_eeprom() {
    put16(0, 4);                 // cal version
    put32(&g_eeprom[4], 8192);   // size
    // linearisation, stored reversed (word at off+12 is coefficient 0): lin0 = s, lin1 = s / HIGHGAIN
    putf(4936 + 12, 0); putf(4936 + 8, 1); putf(4936 + 4, 0); putf(4936 + 0, 0);
    putf(4952 + 12, 0); putf(4952 + 8, (float)(1.0 / HIGHGAIN)); putf(4952 + 4, 0); putf(4952 + 0, 0);
    for (int j = 0; j < 36; j++) {               // 36 x 16 matrix: each band averages 16 sensor cells
        put32(&g_eeprom[2488 + j * 4], j * 3);
        for (int k = 0; k < 16; k++) putf(2632 + (j * 16 + k) * 4, 1.0f / 16);
    }
    for (int j = 0; j < 36; j++) { putf(5256 + j * 4, 1.0f); putf(5112 + j * 4, 1.0f); }
    putf(8020, 0.001f);                          // stray light scale; matrix left zero
    put16(5400, 10000); put16(5402, 40000); put16(5404, 55000); put16(5406, SAT);
    putf(5408, 20.0f); put32(&g_eeprom[5412], 0); put32(&g_eeprom[5416], 0); put16(5420, 0); put16(5422, 0);
    uint32_t sum = 0;                            // checksum at offset 8, computed with that word zeroed
    put32(&g_eeprom[8], 0);
    for (int i = 0; i + 3 < 8192; i += 4) sum += (uint32_t)g_eeprom[i] | g_eeprom[i + 1] << 8 | g_eeprom[i + 2] << 16 | (uint32_t)g_eeprom[i + 3] << 24;
    put32(&g_eeprom[8], sum);
}

static double dark_level(int j, double T, int gain) { return 900 + 0.5 * j + (gain ? 400 : 100) * T; }

static void make_readings() {
    double T = g_trig_clocks * TICK_S;
    g_stream.clear();
    for (int i = 0; i < g_trig_n; i++) {
        uint8_t r[274] = {};
        for (int k = 0; k < 4; k++) r[k * 2] = (uint8_t)((int)dark_level(0, T, g_trig_gain) & 0xFF), r[k * 2 + 1] = (uint8_t)((int)dark_level(0, T, g_trig_gain) >> 8);
        for (int j = 0; j < 128; j++) {
            double shape = 0.3 + 0.7 * exp(-pow((j - 40) / 12.0, 2));
            double v = dark_level(j, T, g_trig_gain) + g_light * shape * T * (g_trig_gain ? HIGHGAIN : 1.0);
            int iv = v > 65535 ? 65535 : (int)v;
            r[(6 + j) * 2] = iv & 0xFF; r[(6 + j) * 2 + 1] = iv >> 8;
        }
        g_stream.insert(g_stream.end(), r, r + 274);
    }
    g_now_us += (int64_t)(T * g_trig_n * 1e6);
}

// ---------------------------------------------------------------- fake ESP-IDF USB host ----
struct Xfer { usb_transfer_t t; std::vector<uint8_t> buf; bool queued; };
static std::set<usb_transfer_t *> g_live, g_queued;
static std::vector<usb_transfer_t *> g_done;     // completions to deliver on the next handle_events
static int g_freed_while_queued = 0;

esp_err_t usb_host_transfer_alloc(size_t n, int, usb_transfer_t **out) {
    Xfer *x = new Xfer{{nullptr, 0, 0, 0, 0, nullptr, 0, USB_TRANSFER_STATUS_ERROR, 0, nullptr, nullptr}, std::vector<uint8_t>(n), false};
    *(uint8_t **)&x->t.data_buffer = x->buf.data();
    *(size_t *)&x->t.data_buffer_size = n;
    g_live.insert(&x->t);
    *out = &x->t;
    return ESP_OK;
}
esp_err_t usb_host_transfer_free(usb_transfer_t *t) {
    if (g_queued.count(t)) g_freed_while_queued++;
    g_live.erase(t);
    delete (Xfer *)t;
    return ESP_OK;
}
esp_err_t usb_host_transfer_submit_control(usb_host_client_handle_t, usb_transfer_t *t) {
    usb_setup_packet_t *s = (usb_setup_packet_t *)t->data_buffer;
    uint8_t *d = t->data_buffer + sizeof(usb_setup_packet_t);
    bool in = s->bmRequestType & 0x80;
    t->actual_num_bytes = t->num_bytes;
    t->status = USB_TRANSFER_STATUS_COMPLETED;
    if (in && s->bRequest == 0x86) { memset(d, 0, 24); put32(d + 8, (uint32_t)lround(TICK_S * 1e6)); put32(d + 12, MIN_COUNT); }
    else if (in && s->bRequest == 0x87) { d[0] = 3; d[1] = 0; }
    else if (!in && s->bRequest == 0x81) {
        g_eeprom_addr = d[0] | d[1] << 8 | d[2] << 16 | d[3] << 24;
        g_eeprom_size = d[4] | d[5] << 8 | d[6] << 16 | d[7] << 24;
        g_stream.assign(g_eeprom.begin() + g_eeprom_addr, g_eeprom.begin() + g_eeprom_addr + g_eeprom_size);
        g_eeprom_pending = true;
    } else if (!in && s->bRequest == 0x80) {
        g_trig_gain = d[2];
        g_trig_clocks = d[4] | d[5] << 8 | d[6] << 16 | d[7] << 24;
        g_trig_n = d[8] | d[9] << 8 | d[10] << 16 | d[11] << 24;
        make_readings();
    }
    g_queued.insert(t);
    g_done.push_back(t);
    return ESP_OK;
}
esp_err_t usb_host_transfer_submit(usb_transfer_t *t) {
    g_queued.insert(t);
    if (g_hang_next_bulk) { g_hang_next_bulk = false; return ESP_OK; }   // never completes on its own
    int n = (int)g_stream.size() < t->num_bytes ? (int)g_stream.size() : t->num_bytes;
    if (n == 0) return ESP_OK;                                           // nothing to send: stays queued
    memcpy(t->data_buffer, g_stream.data(), n);
    g_stream.erase(g_stream.begin(), g_stream.begin() + n);
    t->actual_num_bytes = n;
    t->status = USB_TRANSFER_STATUS_COMPLETED;
    g_done.push_back(t);
    return ESP_OK;
}
esp_err_t usb_host_client_handle_events(usb_host_client_handle_t, uint32_t ms) {
    g_now_us += ms * 1000;
    std::vector<usb_transfer_t *> d;
    d.swap(g_done);
    for (auto *t : d) { g_queued.erase(t); t->callback(t); }
    return ESP_OK;
}
esp_err_t usb_host_endpoint_halt(usb_device_handle_t, uint8_t) { return ESP_OK; }
esp_err_t usb_host_endpoint_flush(usb_device_handle_t, uint8_t) {
    for (auto *t : std::vector<usb_transfer_t *>(g_queued.begin(), g_queued.end())) {
        if (t->bEndpointAddress != 0x81) continue;
        t->status = USB_TRANSFER_STATUS_CANCELED;
        t->actual_num_bytes = 0;
        g_done.push_back(t);
    }
    return ESP_OK;
}
esp_err_t usb_host_endpoint_clear(usb_device_handle_t, uint8_t) { return ESP_OK; }
esp_err_t usb_host_interface_claim(usb_host_client_handle_t, usb_device_handle_t, uint8_t, uint8_t) { return ESP_OK; }
esp_err_t usb_host_interface_release(usb_host_client_handle_t, usb_device_handle_t, uint8_t) { return ESP_OK; }

#include "../src/munki_argyll.cpp"

// ---------------------------------------------------------------- tests ----
static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: " __VA_ARGS__); printf("\n"); fails++; } else { printf("ok:   " __VA_ARGS__); printf("\n"); } } while (0)

static munki::Result read(double light, munki::Spectrum *s) {
    g_light = light;
    return munki::measure_spectrum(munki::MODE_AMBIENT, s);
}

int main() {
    build_eeprom();
    usb_host_client_handle_t c = (usb_host_client_handle_t)1;
    usb_device_handle_t d = (usb_device_handle_t)1;
    CHECK(munki::open(c, d), "open");
    munki::Firmware f;
    CHECK(munki::get_firmware(&f) && f.min_int_count == MIN_COUNT, "firmware read");
    munki::Eeprom e;
    CHECK(munki::read_calibration(&e) && e.sum_ok, "calibration block read, checksum ok");
    g_light = 0;
    CHECK(munki::dark_calibrate(), "dark calibration");
    CHECK(munki::high_gain_ready(), "high gain dark reference taken");

    munki::Spectrum s;
    munki::Result r = read(400000, &s);
    CHECK(r == munki::MR_OK && s.gain == 0 && s.inttime < 0.5, "bright: OK, normal gain, short time (T %.4f, n %d)", s.inttime, s.nmeas);
    double q = s.inttime / TICK_S;
    CHECK(fabs(q - lround(q)) < 1e-6, "integration time is a whole number of ticks");
    double expect = 400000 * 1.0;   // absraw at the peak cell = light * shape(=1) per second
    CHECK(fabs(munki::last_absraw()[40] - expect) / expect < 0.01, "absolute value at peak (%.0f vs %.0f)", munki::last_absraw()[40], expect);

    r = read(9000, &s);
    CHECK(r == munki::MR_OK && s.gain == 0 && fabs(s.inttime - 2.0) < 0.01 && s.nmeas >= 3, "dim: 2 s, averaged (T %.3f, n %d, %s)", s.inttime, s.nmeas, munki::result_name(r));
    CHECK(fabs(munki::last_absraw()[40] - 9000) / 9000 < 0.01, "dim absolute value (%.1f)", munki::last_absraw()[40]);

    r = read(300, &s);
    CHECK(r == munki::MR_LOW_LIGHT && s.gain == 1, "very dim: high gain, flagged low light (T %.3f, n %d, gain %d)", s.inttime, s.nmeas, s.gain);
    CHECK(fabs(munki::last_absraw()[40] - 300) / 300 < 0.02, "high gain gives the same absolute units (%.1f vs 300)", munki::last_absraw()[40]);

    r = read(5e8, &s);
    CHECK(r == munki::MR_TOO_BRIGHT, "too bright reported (%s)", munki::result_name(r));

    // A bulk read that never completes: must not be freed while queued, must be cancelled and freed afterwards.
    g_light = 400000;
    g_hang_next_bulk = true;
    munki::Raw raw;
    bool ok = munki::measure_raw(false, 0, 0.05, 0, 2, &raw);
    CHECK(!ok, "hung bulk read fails (%s)", munki::last_error());
    CHECK(g_freed_while_queued == 0, "no transfer freed while still queued");
    CHECK(g_queued.empty(), "nothing left queued after the cancel");
    r = read(400000, &s);
    CHECK(r == munki::MR_OK, "next reading after a failure works (%s)", munki::result_name(r));
    printf("\nlive transfers left: %zu\n", g_live.size());
    printf(fails ? "\n%d FAILED\n" : "\nALL PASSED\n", fails);
    return fails ? 1 : 0;
}
