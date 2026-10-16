// SPDX-License-Identifier: AGPL-3.0-or-later
// Ported from ArgyllCMS spectro/munki_imp.c (Copyright Graeme W. Gill, AGPL-3.0). See munki_argyll.h.
#include "munki_argyll.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <utility>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAGM = "munki";

namespace munki {

static usb_host_client_handle_t g_client = NULL;
static usb_device_handle_t g_dev = NULL;

// A transfer's completion record lives on the heap. If we give up waiting (timeout), the transfer may still be in
// flight inside the USB stack, so neither it nor this record may be freed then: we mark it abandoned and the callback
// frees both whenever it finally runs (or the endpoint flush cancels it). Callbacks run inside
// usb_host_client_handle_events(), i.e. on this same task, so there is no race on these fields.
struct Wait { volatile bool done; volatile bool abandoned; volatile usb_transfer_status_t status; volatile int bytes; };

static void xfer_cb(usb_transfer_t *t) {
    Wait *w = (Wait *)t->context;
    if (w->abandoned) {
        usb_host_transfer_free(t);
        free(w);
        return;
    }
    w->status = t->status;
    w->bytes = t->actual_num_bytes;
    w->done = true;
}

static Wait *new_wait() { return (Wait *)calloc(1, sizeof(Wait)); }

// Services USB events until the transfer completes or timeout_ms (plus a 500 ms grace) of real time has passed.
static void pump(Wait *w, int timeout_ms, int grace_ms = 500) {
    const int64_t deadline = esp_timer_get_time() + (int64_t)(timeout_ms + grace_ms) * 1000;
    while (!w->done && esp_timer_get_time() < deadline)
        usb_host_client_handle_events(g_client, pdMS_TO_TICKS(20));
}

// Lets the USB stack run pending callbacks (e.g. cancelled transfers after a flush) for about ms milliseconds.
static void settle(int ms) {
    const int64_t deadline = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < deadline) usb_host_client_handle_events(g_client, pdMS_TO_TICKS(10));
}

// Cancels whatever is queued on the bulk IN endpoint (halt, flush, clear), so a timed-out read cannot linger.
static void reset_bulk_in() {
    if (!g_dev) return;
    usb_host_endpoint_halt(g_dev, 0x81);
    usb_host_endpoint_flush(g_dev, 0x81);
    settle(60);
    usb_host_endpoint_clear(g_dev, 0x81);
}

static char g_err[48] = "";
const char *last_error() { return g_err; }
static void set_err(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_err, sizeof g_err, fmt, ap);
    va_end(ap);
    ESP_LOGW(TAGM, "%s", g_err);
}

// Vendor control transfer on endpoint 0 (the instrument uses these for almost everything).
static bool ctrl(bool in, uint8_t req, uint8_t *buf, int len, int timeout_ms = 2000) {
    if (!g_dev) return false;
    const int mps = 64;
    int data_len = in ? ((len + mps - 1) / mps) * mps : len;
    usb_transfer_t *t = NULL;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + data_len, 0, &t) != ESP_OK) { set_err("NO CTRL BUFFER"); return false; }
    Wait *w = new_wait();
    if (!w) { usb_host_transfer_free(t); set_err("NO CTRL BUFFER"); return false; }
    usb_setup_packet_t *s = (usb_setup_packet_t *)t->data_buffer;
    s->bmRequestType = (in ? 0x80 : 0x00) | 0x40;   // vendor, device
    s->bRequest = req;
    s->wValue = 0;
    s->wIndex = 0;
    s->wLength = len;
    if (!in && len) memcpy(t->data_buffer + sizeof(usb_setup_packet_t), buf, len);
    t->device_handle = g_dev;
    t->bEndpointAddress = 0;
    t->num_bytes = sizeof(usb_setup_packet_t) + data_len;
    t->callback = xfer_cb;
    t->context = w;
    t->timeout_ms = timeout_ms;
    if (usb_host_transfer_submit_control(g_client, t) != ESP_OK) {
        usb_host_transfer_free(t);
        free(w);
        set_err("REQ %02X NOT SENT", req);
        return false;
    }
    pump(w, timeout_ms);
    if (!w->done) {
        // Still in flight: hand it to the callback to free (the old code freed it here, under the USB stack).
        w->abandoned = true;
        set_err("REQ %02X TIMEOUT", req);
        return false;
    }
    bool ok = w->status == USB_TRANSFER_STATUS_COMPLETED;
    if (ok && in) memcpy(buf, t->data_buffer + sizeof(usb_setup_packet_t), len);
    if (!ok) set_err("REQ %02X ST %d", req, (int)w->status);
    usb_host_transfer_free(t);
    free(w);
    return ok;
}


// Bulk IN read (instrument data comes back on endpoint 0x81), handed to `sink` one transfer at a time so nothing larger
// than a single 4 KB buffer is ever needed (the bridge runs Bluetooth too and has little free RAM for a 40 KB buffer).
// Every transfer is a multiple of 64 bytes except possibly the last, so the stream stays aligned with the instrument.
typedef bool (*SinkFn)(const uint8_t *d, int n, void *ctx);
static bool bulk_in_stream(int total, int timeout_ms, SinkFn sink, void *ctx) {
    usb_transfer_t *t = NULL;
    if (usb_host_transfer_alloc(4096, 0, &t) != ESP_OK) { set_err("NO USB BUFFER"); return false; }
    Wait *w = new_wait();
    if (!w) { usb_host_transfer_free(t); set_err("NO USB BUFFER"); return false; }
    bool ok = true;
    int off = 0;
    while (ok && off < total) {
        int chunk = total - off < 4096 ? total - off : 4096;
        int alloc = ((chunk + 63) / 64) * 64;
        w->done = false;
        w->status = USB_TRANSFER_STATUS_ERROR;
        w->bytes = 0;
        t->device_handle = g_dev;
        t->bEndpointAddress = 0x81;
        t->num_bytes = alloc;
        t->callback = xfer_cb;
        t->context = w;
        t->timeout_ms = timeout_ms;
        esp_err_t e = usb_host_transfer_submit(t);
        if (e != ESP_OK) { set_err("BULK SUBMIT %d", (int)e); ok = false; break; }
        pump(w, timeout_ms);
        if (!w->done) {
            // Still queued: the callback will free the transfer and record once the flush cancels it.
            w->abandoned = true;
            set_err("BULK TIMEOUT");
            reset_bulk_in();
            return false;
        }
        if (w->status != USB_TRANSFER_STATUS_COMPLETED || w->bytes < chunk) {
            set_err("BULK ST %d GOT %d/%d", (int)w->status, w->bytes, chunk);
            ok = false;
            break;
        }
        ok = sink(t->data_buffer, chunk, ctx);
        off += chunk;
    }
    usb_host_transfer_free(t);
    free(w);
    return ok;
}

static int buf2int(const uint8_t *b) {   // little endian, signed 32 bit
    return (int)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}
static void int2buf(uint8_t *b, int v) {
    b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}

static void forget_instrument();

bool open(usb_host_client_handle_t client, usb_device_handle_t dev) {
    g_client = client;
    g_dev = dev;
    forget_instrument();
    esp_err_t e = usb_host_interface_claim(client, dev, 0, 0);
    if (e != ESP_OK) {
        ESP_LOGW(TAGM, "interface claim failed: %d", (int)e);
        g_dev = NULL;
        return false;
    }
    return true;
}

void close() {
    if (g_dev && g_client) usb_host_interface_release(g_client, g_dev, 0);
    g_dev = NULL;
}

static int g_tick_us = 0;
// Firmware details never change while the instrument is attached, so they are read once (open() clears the cache).
static Firmware g_fw;
static bool g_fw_valid = false;

bool get_firmware(Firmware *f) {
    if (g_fw_valid) { *f = g_fw; return true; }
    uint8_t p[24];
    if (!ctrl(true, 0x86, p, 24)) return false;
    f->fw_major = buf2int(&p[0]);
    f->fw_minor = buf2int(&p[4]);
    f->tick_dur = buf2int(&p[8]);
    f->min_int_count = buf2int(&p[12]);
    g_tick_us = f->tick_dur;
    f->ee_blocks = buf2int(&p[16]);
    f->ee_block_size = buf2int(&p[20]);
    g_fw = *f;
    g_fw_valid = true;
    return true;
}

bool get_version(char out[37]) {
    if (!ctrl(true, 0x85, (uint8_t *)out, 36)) return false;
    out[36] = 0;
    for (int i = 0; i < 36; i++) if (out[i] && (out[i] < 32 || out[i] > 126)) out[i] = '?';
    return true;
}

bool get_chip_id(uint8_t out[8]) { return ctrl(true, 0x8A, out, 8); }

bool get_status(uint8_t *spos, uint8_t *button) {
    uint8_t p[2];
    if (!ctrl(true, 0x87, p, 2)) return false;
    if (spos) *spos = p[0];
    if (button) *button = p[1];
    return true;
}

bool read_eeprom(uint8_t *buf, int addr, int size) {
    uint8_t p[8];
    int2buf(&p[0], addr);
    int2buf(&p[4], size);
    if (!ctrl(false, 0x81, p, 8)) return false;
    struct Cp { uint8_t *b; int off; } cp = {buf, 0};
    return bulk_in_stream(size, 6000, [](const uint8_t *d, int n, void *ctx) {
        Cp *x = (Cp *)ctx;
        memcpy(x->b + x->off, d, n);
        x->off += n;
        return true;
    }, &cp);
}

static uint8_t *g_cal = NULL;
static int g_cal_size = 0;
int calibration_size() { return g_cal ? g_cal_size : 0; }

static uint32_t buf2uint(const uint8_t *b) { return (uint32_t)buf2int(b); }

const uint8_t *calibration_data() { return g_cal; }

// Argyll munki_imp_init(): read the size at offset 4, read that many bytes from offset 0, sum the 32 bit words
// with the stored checksum (at offset 8) zeroed, and compare.
static bool build_hires();

bool read_calibration(Eeprom *e) {
    memset(e, 0, sizeof *e);
    free(g_cal);
    g_cal = NULL;
    uint8_t sz[4];
    if (!read_eeprom(sz, 4, 4)) return false;
    int calsize = buf2int(sz);
    if (calsize < 12 || calsize > 2 * 8192) { e->size = calsize; return false; }
    int ru = (calsize + 3) & ~3;
    uint8_t *b = (uint8_t *)calloc(ru, 1);
    if (!b) return false;
    if (!read_eeprom(b, 0, calsize)) { free(b); return false; }
    e->size = calsize;
    g_cal_size = calsize;
    e->read_ok = true;
    e->sum_stored = buf2uint(b + 8);
    memset(b + 8, 0, 4);
    uint32_t sum = 0;
    for (int i = 0; i < ru - 3; i += 4) sum += buf2uint(b + i);
    e->sum_calc = sum;
    e->sum_ok = (sum == e->sum_stored);
    e->cal_version = (int)(b[0] | (b[1] << 8));
    e->compat_version = (int)(b[2] | (b[3] << 8));
    g_cal = b;   // kept (with the checksum word zeroed, as Argyll does) for the parser
    if (e->sum_ok) build_hires();
    return true;
}

static int u16at(const uint8_t *b, int off) { return b[off] | (b[off + 1] << 8); }
static double f32at(const uint8_t *b, int off) {
    uint32_t v = (uint32_t)buf2int(b + off);
    float f;
    memcpy(&f, &v, 4);
    return (double)f;
}

// Offsets from Argyll munki_parse_eeprom(): u16 at 5400..5406, float at 5408, int32 at 5412/5416, u16 at 5420/5422.
bool get_params(Params *p) {
    if (!g_cal || g_tick_us <= 0) return false;
    p->min_sval = u16at(g_cal, 5400);
    p->opt_sval = u16at(g_cal, 5402);
    p->max_sval = u16at(g_cal, 5404);
    p->sat_limit = u16at(g_cal, 5406);
    p->cal_int_time = f32at(g_cal, 5408) * 1e-3;
    p->preheat_s = buf2int(g_cal + 5412) * 1e-3;
    p->wait_s = buf2int(g_cal + 5416) * 1e-3;
    p->hold_temp_duty = u16at(g_cal, 5420);
    p->invalid_s = u16at(g_cal, 5422) * 1e-3;
    p->tick_s = g_tick_us * 1e-6;
    return true;
}

// Argyll munki_triggermeasure() + munki_readmeasurement() + the raw-sample layout from munki_sens_to_raw():
// each reading is 137 little-endian 16 bit values: 6 leading (the first 4 are shielded cells), 128 bands, 3 trailing
// (the last is the LED temperature value).
static const int NSEN = 137, NRAW = 128, RDBYTES = 274;

namespace {
struct RawAcc {
    int ninvalid;
    int idx;                 // readings seen so far
    uint8_t carry[274];
    int carry_len;
    double acc[128];
    double shielded, led;
    double peak;             // highest single band value of any one reading (saturation check)
    int counted;
};
}

static void take_reading(RawAcc *a, const uint8_t *r) {
    if (a->idx++ < a->ninvalid) return;
    for (int k = 0; k < 4; k++) a->shielded += u16at(r, k * 2);
    for (int j = 0; j < 128; j++) {
        int v = u16at(r, (6 + j) * 2);
        a->acc[j] += v;
        if (v > a->peak) a->peak = v;
    }
    a->led += u16at(r, (137 - 1) * 2);
    a->counted++;
}

static bool raw_sink(const uint8_t *d, int n, void *ctx) {
    RawAcc *a = (RawAcc *)ctx;
    while (n > 0) {
        if (a->carry_len > 0 || n < 274) {
            int need = 274 - a->carry_len;
            int take = n < need ? n : need;
            memcpy(a->carry + a->carry_len, d, take);
            a->carry_len += take;
            d += take;
            n -= take;
            if (a->carry_len == 274) { take_reading(a, a->carry); a->carry_len = 0; }
        } else {
            take_reading(a, d);
            d += 274;
            n -= 274;
        }
    }
    return true;
}

// After a failed reading the instrument may still be sending the rest of it. Read and discard whatever is left
// (short waits) so the next command starts from a clean stream.
static void drain_bulk_in() {
    if (!g_dev) return;
    usb_transfer_t *t = NULL;
    if (usb_host_transfer_alloc(4096, 0, &t) != ESP_OK) return;
    Wait *w = new_wait();
    if (!w) { usb_host_transfer_free(t); return; }
    int discarded = 0;
    const int64_t until = esp_timer_get_time() + 3000000;
    while (esp_timer_get_time() < until) {
        w->done = false;
        w->bytes = 0;
        t->device_handle = g_dev;
        t->bEndpointAddress = 0x81;
        t->num_bytes = 4096;
        t->callback = xfer_cb;
        t->context = w;
        t->timeout_ms = 150;
        if (usb_host_transfer_submit(t) != ESP_OK) break;
        pump(w, 150, 50);
        if (!w->done) {             // nothing more is coming: cancel the read; the callback frees t and w
            w->abandoned = true;
            reset_bulk_in();
            ESP_LOGW(TAGM, "drained %d leftover bytes", discarded);
            return;
        }
        if (w->status != USB_TRANSFER_STATUS_COMPLETED || w->bytes <= 0) break;
        discarded += w->bytes;
    }
    ESP_LOGW(TAGM, "drained %d leftover bytes", discarded);
    usb_host_transfer_free(t);
    free(w);
}

bool measure_raw(bool lamp, int gain, double inttime_s, int ninvalid, int nmeas, Raw *out) {
    Params pr;
    if (!get_params(&pr) || nmeas < 1 || ninvalid < 0) { set_err("NO PARAMS"); return false; }
    int total = ninvalid + nmeas;
    // The instrument integrates for a whole number of clock ticks; use that exact time (as Argyll does) for the
    // dark interpolation and the per-second scaling, not the time that was asked for.
    int intclocks = (int)(inttime_s / pr.tick_s + 0.5);
    if (intclocks < 1) intclocks = 1;
    uint8_t cmd[12] = {};
    cmd[0] = lamp ? 1 : 0;   // lamp
    cmd[1] = 0;              // scan: off (spot reading)
    cmd[2] = gain ? 1 : 0;   // gain: 0 normal, 1 high
    cmd[3] = (uint8_t)pr.hold_temp_duty;
    int2buf(&cmd[4], intclocks);
    int2buf(&cmd[8], total);
    if (!ctrl(false, 0x80, cmd, 12)) return false;
    RawAcc a = {};
    a.ninvalid = ninvalid;
    int timeout = 2000 + (int)(intclocks * pr.tick_s * total * 1000.0);
    if (!bulk_in_stream(274 * total, timeout, raw_sink, &a)) {
        drain_bulk_in();
        return false;
    }
    if (a.counted != nmeas) { set_err("GOT %d OF %d READINGS", a.counted, nmeas); drain_bulk_in(); return false; }
    memset(out, 0, sizeof *out);
    out->n = nmeas;
    out->inttime = intclocks * pr.tick_s;
    out->gain = gain ? 1 : 0;
    out->shielded = a.shielded / (4.0 * nmeas);
    out->led = a.led / nmeas;
    double sum = 0;
    for (int j = 0; j < 128; j++) {
        out->band[j] = (float)(a.acc[j] / nmeas);
        sum += out->band[j];
    }
    out->mean = sum / 128;
    out->max = a.peak;   // per reading, so one saturated reading is not hidden by the average
    return true;
}

// ----------------------------------------------------------------------------------------------------------
// Emission / ambient spectrum. Follows Argyll: adaptive dark reference interpolated linearly in integration time
// (munki_prepare_idark / munki_interp_dark, one pair per gain), shielded-cell drift correction and linearisation
// (munki_sub_raw_to_absraw, lin0 normal gain / lin1 high gain), the 36 x 16 wavelength matrix and stray-light matrix
// (munki_absraw_to_abswav1), then the mode's calibration factors. Integration time and gain are chosen as in
// munki_optimise_sensor().
static const double MAX_INT = 4.5;          // Argyll m->max_int_time
static const double TARG_MAX_INT = 2.0;     // Argyll s->targmaxitime
static const double TARG_SCALE = 0.90;      // Argyll s->targoscale
static const double TARG_SCALE2 = 0.15;     // Argyll s->targoscale2
static const double DARK_T1 = 2.0;          // Argyll ADARKINT_MAX: second dark reference time, both gains
static const double READ_TIME = 1.0;        // Argyll s->wreadtime: total time averaged per reading
// Not Argyll: in low light (integration at or above TARG_MAX_INT) average over this much time instead, so a dim
// reading is several integrations, not one. Averaging n readings cuts the random noise by about sqrt(n).
static const double LOW_READ_TIME = 8.0;

// High gain is used only when even the longest integration time cannot reach a usable signal, as in Argyll's
// optimiser. Argyll itself ships with it off (USE_HIGH_GAIN_MODE undefined); set MUNKI_HIGH_GAIN 0 to match that.
#ifndef MUNKI_HIGH_GAIN
#define MUNKI_HIGH_GAIN 1
#endif

struct DarkRef {
    bool valid;
    double t0, t1;
    float d0[NRAW + 1], d1[NRAW + 1];   // [0] = shielded cells, [1..128] = bands
};
static DarkRef g_dark[2] = {};          // [0] normal gain, [1] high gain
static double g_highgain = 0;           // gain ratio of high gain mode (Argyll: 1 / lin1[1]); 0 = unusable

static bool (*g_cancel)() = nullptr;
void set_cancel_check(bool (*fn)()) { g_cancel = fn; }
static bool cancelled() { return g_cancel && g_cancel(); }

static void forget_instrument() {
    g_fw_valid = false;
    g_dark[0].valid = g_dark[1].valid = false;
    g_highgain = 0;
}

static void put_dark(float *dst, const Raw &r) {
    dst[0] = (float)r.shielded;
    for (int j = 0; j < NRAW; j++) dst[1 + j] = r.band[j];
}

static int64_t g_dark_us = 0;
bool dark_valid() { return g_dark[0].valid; }
uint32_t dark_age_s() { return g_dark[0].valid ? (uint32_t)((esp_timer_get_time() - g_dark_us) / 1000000) : 0xFFFFFFFFu; }
bool high_gain_ready() { return MUNKI_HIGH_GAIN && g_dark[1].valid && g_highgain > 1.0; }

static double min_int_time(const Params &pr) {
    Firmware f;
    if (get_firmware(&f) && f.min_int_count > 0) return pr.tick_s * f.min_int_count;
    return pr.tick_s * 224.0;
}

// Two dark readings at one gain: many short integrations at the minimum time, one long one at DARK_T1.
static bool dark_pair(const Params &pr, int gain, DarkRef *d) {
    d->valid = false;
    double tmin = min_int_time(pr);
    Raw r;
    int n0 = (int)(READ_TIME / tmin + 0.5);
    if (n0 < 1) n0 = 1;
    if (n0 > 140) n0 = 140;
    if (cancelled()) { set_err("CANCELLED"); return false; }
    if (!measure_raw(false, gain, tmin, 0, n0, &r)) return false;
    d->t0 = r.inttime;
    put_dark(d->d0, r);
    if (cancelled()) { set_err("CANCELLED"); return false; }
    if (!measure_raw(false, gain, DARK_T1, 0, 1, &r)) return false;
    d->t1 = r.inttime;
    put_dark(d->d1, r);
    d->valid = d->t1 > d->t0;
    return d->valid;
}

bool dark_calibrate() {
    Params pr;
    if (!get_params(&pr)) return false;
    g_dark[1].valid = false;
    if (!dark_pair(pr, 0, &g_dark[0])) return false;
    g_dark_us = esp_timer_get_time();
#if MUNKI_HIGH_GAIN
    // Argyll: the gain of high gain mode is encoded in its linearisation, highgain = 1 / lin1[1].
    double l1 = f32at(g_cal, 4952 + (3 - 1) * 4);
    g_highgain = (l1 > 0 && l1 == l1) ? 1.0 / l1 : 0;
    if (g_highgain > 1.0 && g_highgain < 100.0) {
        if (cancelled()) return true;   // normal-gain references are done; high gain just stays off
        if (!dark_pair(pr, 1, &g_dark[1])) ESP_LOGW(TAGM, "high gain dark reference failed; high gain stays off");
    } else {
        ESP_LOGW(TAGM, "high gain ratio %.3f looks wrong; high gain stays off", g_highgain);
        g_highgain = 0;
    }
#endif
    return true;
}

// Dark level at integration time T and gain: linear in T for every cell (index 0 = shielded cells).
static void dark_at(double T, int gain, double *out) {
    const DarkRef &d = g_dark[gain && g_dark[1].valid ? 1 : 0];
    for (int j = 0; j <= NRAW; j++) {
        double slope = ((double)d.d1[j] - d.d0[j]) / (d.t1 - d.t0);
        double base = d.d0[j] - slope * d.t0;
        out[j] = base + T * slope;
    }
}

static float g_last_absraw[NRAW];

static bool compute_spectrum(const Raw &r, Mode mode, float *out) {
    const uint8_t *e = g_cal;
    const double T = r.inttime;
    int calver = u16at(e, 0);
    double dk[NRAW + 1];
    dark_at(T, r.gain, dk);
    double sub_sh = dk[0];
    double *sub = dk + 1;
    // zero point for the shielded-cell drift correction
    double avgs = r.shielded, submax = -1e6;
    for (int j = 0; j < NRAW; j++) if (sub[j] > submax) submax = sub[j];
    double zero = 1.08 * 0.5 * (avgs + sub_sh);
    if (zero < 1.005 * avgs) zero = 1.005 * avgs;
    if (zero < 1.005 * sub_sh) zero = 1.005 * sub_sh;
    if (zero < 1.005 * submax) zero = 1.005 * submax;
    // Linearisation polynomial (Argyll stores them reversed, rget_32_doubles): lin0 at 4936 for normal gain,
    // lin1 at 4952 for high gain. lin1 also carries the gain, so both give the same absolute units.
    const int lin_off = r.gain ? 4952 : 4936;
    double lin[4];
    for (int i = 0; i < 4; i++) lin[i] = f32at(e, lin_off + (3 - i) * 4);
    double absraw[NRAW];
    for (int j = 0; j < NRAW; j++) {
        double asub = zero - (zero - sub[j]) * (zero - avgs) / (zero - sub_sh);
        double s = r.band[j] - asub;
        double l = lin[3];
        for (int k = 2; k >= 0; k--) l = l * s + lin[k];
        absraw[j] = l / T;
        g_last_absraw[j] = (float)absraw[j];
    }
    double tm[36];
    for (int j = 0; j < 36; j++) {
        int sx = buf2int(e + 2488 + j * 4);
        if (sx < 0 || sx + 16 > NRAW) return false;
        double o = 0;
        for (int k = 0; k < 16; k++) o += f32at(e, 2632 + (j * 16 + k) * 4) * absraw[sx + k];
        tm[j] = o;
    }
    double tdouble = calver >= 4 ? f32at(e, 8020) : 0.001;
    int cal_off = mode == MODE_AMBIENT ? 5256 : mode == MODE_EMISSION ? 5112 : 8024;
    for (int j = 0; j < 36; j++) {
        double o = 0;
        for (int k = 0; k < 36; k++) {
            double sl = tdouble * (int16_t)u16at(e, 5428 + (j * 36 + k) * 2);
            if (j == k) sl += 1.0;
            o += sl * tm[k];
        }
        double cf = f32at(e, cal_off + j * 4);
        if (cf != cf) return false;   // NaN in the reference
        out[j] = (float)(o * cf);
    }
    return true;
}

const float *last_absraw() { return g_last_absraw; }

// ----------------------------------------------------------------------------------------------------------
// High-resolution emission spectrum (3.33 nm), after ArgyllCMS munki_create_hr() for emission with its defaults
// (Gaussian filter shape, BOX_INTEGRATE, DO_CCDNORM + DO_CCDNORMAVG, ONEDSTRAYLIGHTUS). Built once from the
// calibration block when it is read:
//   1. where neighbouring factory 10 nm filters cross over gives the wavelength of points on the CCD; a cubic spline
//      through them maps any CCD cell to its wavelength (Argyll fits an rspl here);
//   2. one narrow filter per 3.33 nm band, each the Gaussian of HR_WIDTH integrated over every CCD cell's wavelength
//      span (Argyll's box integration);
//   3. the filters are scaled so their average CCD weight matches the factory filters' (x 10 / HR_WIDTH);
//   4. the mode's calibration factors are resampled to the new bands (a cubic spline; Argyll fits an rspl) and the
//      stray-light matrix is resampled linearly, as Argyll's standalone build does.
// Below 400 nm Argyll repeats the 400 nm band (too noisy); so does this.
static const int HR_N = 106;                       // (730 - 380) / 3.333 + 1
static const double HR_WIDTH = 10.0 / 3.0;
static const double HR_SHORT = 380.0, HR_LONG = 730.0, HR_MIN_WL = 400.0;
static const int HR_MAXCOEF = 12;                  // a ColorMunki's widest band uses 7 CCD cells

static bool g_hr_ok = false;
static double hr_stray_off(int i, int j);
static int g_hr_start[HR_N], g_hr_count[HR_N];
static float g_hr_coef[HR_N][HR_MAXCOEF];
static float g_hr_slp[36][36];                     // stray light with the near-diagonal entries interpolated
static float g_hr_diag[HR_N];                      // diagonal of the resampled stray-light matrix
static float g_last_hr[HR_N];

static double hr_wl(int j) { return HR_SHORT + j * (HR_LONG - HR_SHORT) / (HR_N - 1.0); }

// Natural cubic spline through (x[i], y[i]), x ascending; evaluated with extrapolation of the end cubics.
struct Spline {
    int n;
    double x[40], y[40], m[40];   // m = second derivatives
    void fit(const double *xs, const double *ys, int cnt) {
        n = cnt;
        for (int i = 0; i < n; i++) { x[i] = xs[i]; y[i] = ys[i]; }
        double u[40];
        m[0] = u[0] = 0;
        for (int i = 1; i < n - 1; i++) {
            double sig = (x[i] - x[i - 1]) / (x[i + 1] - x[i - 1]);
            double p = sig * m[i - 1] + 2.0;
            m[i] = (sig - 1.0) / p;
            u[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
            u[i] = (6.0 * u[i] / (x[i + 1] - x[i - 1]) - sig * u[i - 1]) / p;
        }
        m[n - 1] = 0;
        for (int k = n - 2; k >= 0; k--) m[k] = m[k] * m[k + 1] + u[k];
    }
    double at(double v) const {
        int k = 0;
        if (v >= x[n - 1]) k = n - 2;
        else if (v > x[0]) { k = 0; while (k < n - 2 && x[k + 1] < v) k++; }
        double h = x[k + 1] - x[k], a = (x[k + 1] - v) / h, b = (v - x[k]) / h;
        return a * y[k] + b * y[k + 1] + ((a * a * a - a) * m[k] + (b * b * b - b) * m[k + 1]) * (h * h) / 6.0;
    }
};

// Argyll lanczos2() with USE_GAUSSIAN: a Gaussian of the given width, 1.0 at the centre.
static double hr_shape(double wi, double x) {
    wi = wi / sqrt(2.0 * log(2.0));
    x = x / wi;
    return exp(-(x * x));
}

static bool build_hires() {
    g_hr_ok = false;
    const uint8_t *e = g_cal;
    if (!e) return false;
    // Working memory only needed while building (the main task's stack is small): on the heap, freed on return.
    struct C { int ix; double we; };
    struct Work { C coeff[36][16]; double ccd[2][NRAW]; };
    Work *wk = (Work *)calloc(1, sizeof(Work));
    if (!wk) return false;
    struct Guard { Work *p; ~Guard() { free(p); } } guard{wk};
    C (*coeff)[16] = wk->coeff;
    double (*ccd)[NRAW] = wk->ccd;
    // Factory filters (36 x 16 at 10 nm), skipping the duplicated ones at the short end, as Argyll does.
    int nw = 36, jj = 0;
    double ws = 380.0;
    const double wlong = 730.0;
    for (int j = 0; j < 36; j++) {
        int sx = buf2int(e + 2488 + j * 4);
        if (sx < 0 || sx + 16 > NRAW) return false;
        if (j < 35 && sx == buf2int(e + 2488 + (j + 1) * 4)) { ws += 10.0; nw--; continue; }
        for (int k = 0; k < 16; k++) coeff[jj][k] = {sx + k, f32at(e, 2632 + (j * 16 + k) * 4)};
        jj++;
    }
    if (nw < 4) return false;
    auto wl1 = [&](double i) { return ws + i * (wlong - ws) / (nw - 1.0); };
    // 1. Crossover points between neighbouring filters (raw index, wavelength).
    double xr[40], xw[40], xy[40];
    const double eps = 1e-6;
    for (int i = 0; i < nw - 1; i++) {
        double best = -1e6;
        for (int j = 0; j < 15; j++)
            for (int k = 0; k < 15; k++) {
                if (coeff[i][j].ix != coeff[i + 1][k].ix || coeff[i][j + 1].ix != coeff[i + 1][k + 1].ix) continue;
                double y1 = coeff[i][j].we, y2 = coeff[i][j + 1].we, y3 = coeff[i + 1][k].we, y4 = coeff[i + 1][k + 1].we;
                double den = -y4 + y3 + y2 - y1;
                if (fabs(den) < eps) continue;
                double yn = (y2 * y3 - y1 * y4) / den, xn = (y3 - y1) / den;
                if (xn < -eps || xn > 1.0 + eps) continue;
                if (yn > best) {
                    xw[i + 1] = wl1(i + 0.5);
                    xr[i + 1] = (1.0 - xn) * coeff[i][j].ix + xn * coeff[i][j + 1].ix;
                    xy[i + 1] = yn;
                    best = yn;
                }
            }
        if (best < 0.0) { ESP_LOGW(TAGM, "high res: no crossover %d", i); return false; }
    }
    for (int end = 0; end < 2; end++) {   // extrapolated crossovers outside the first and last filter
        int a = end ? nw - 2 : 1, b = end ? nw - 1 : 2, f = end ? nw - 1 : 0, pos = end ? nw : 0;
        double x5 = xr[a], y5 = xy[a], x6 = xr[b], y6 = xy[b], y1 = 0, y2 = 0, y3 = 0, y4 = 0;
        int j;
        for (j = 0; j < 15; j++) {
            y3 = y5 + (coeff[f][j].ix - x5) / (x6 - x5) * (y6 - y5);
            y4 = y5 + (coeff[f][j + 1].ix - x5) / (x6 - x5) * (y6 - y5);
            y1 = coeff[f][j].we;
            y2 = coeff[f][j + 1].we;
            double c0 = coeff[f][j].ix, c1 = coeff[f][j + 1].ix;
            if (((y1 >= y3 && y2 <= y4) || (y1 <= y3 && y2 >= y4)) &&
                ((c0 < x5 && c0 < x6 && c1 < x5 && c1 < x6) || (c1 > x5 && c1 > x6 && c0 > x5 && c0 > x6))) break;
        }
        if (j >= 15) { ESP_LOGW(TAGM, "high res: no end crossover"); return false; }
        double den = -y4 + y3 + y2 - y1;
        double xn = (y3 - y1) / den;
        xw[pos] = wl1(end ? nw - 0.5 : -0.5);
        xr[pos] = (1.0 - xn) * coeff[f][j].ix + xn * coeff[f][j + 1].ix;
    }
    // Spline raw index -> wavelength (sorted by raw index).
    int np = nw + 1;
    double sr[40], sw[40];
    for (int i = 0; i < np; i++) { sr[i] = xr[i]; sw[i] = xw[i]; }
    for (int i = 1; i < np; i++)
        for (int k = i; k > 0 && sr[k] < sr[k - 1]; k--) { std::swap(sr[k], sr[k - 1]); std::swap(sw[k], sw[k - 1]); }
    static Spline r2w;
    r2w.fit(sr, sw, np);
    // 2. Narrow filters: the Gaussian integrated over each CCD cell's wavelength span (0.05 nm steps).
    double fshmax = 50.0;
    for (; fshmax >= 0.0; fshmax -= 0.1)
        if (fabs(hr_shape(HR_WIDTH, fshmax)) > 0.001) { fshmax += 0.1; break; }
    for (int j = 0; j < HR_N; j++) g_hr_count[j] = 0;
    for (int i = 0; i < NRAW; i++) {
        double w1 = r2w.at(i - 0.5), w2 = r2w.at(i + 0.5);
        for (int j = 0; j < HR_N; j++) {
            double cwl = hr_wl(j);
            if (cwl < HR_MIN_WL) cwl = HR_MIN_WL;
            if (fabs(w1 - cwl) > fshmax && fabs(w2 - cwl) > fshmax) continue;
            int nn = (int)(fabs(w2 - w1) / 0.05 + 0.5);
            double lw = w1, ll = hr_shape(HR_WIDTH, w1 - cwl), we = 0.0;
            for (int k = 0; k < nn; k++) {
                double cw = w1 + (k + 1.0) / (nn + 1.0) * (w2 - w1);
                double cl = hr_shape(HR_WIDTH, cw - cwl);
                we += 0.5 * (cl + ll) * (lw - cw);
                ll = cl;
                lw = cw;
            }
            if (g_hr_count[j] >= HR_MAXCOEF) { ESP_LOGW(TAGM, "high res: filter too wide"); return false; }
            if (g_hr_count[j] == 0) g_hr_start[j] = i;
            else if (g_hr_start[j] + g_hr_count[j] != i) { ESP_LOGW(TAGM, "high res: filter not contiguous"); return false; }
            g_hr_coef[j][g_hr_count[j]++] = (float)we;
        }
    }
    for (int j = 0; j < HR_N; j++) if (g_hr_count[j] == 0) { ESP_LOGW(TAGM, "high res: empty band %d", j); return false; }
    // 3. DO_CCDNORM + DO_CCDNORMAVG: scale so the average CCD weight matches the factory filters'.
    for (int j = 0; j < 36; j++) {
        int sx = buf2int(e + 2488 + j * 4);
        if (j < 34 && sx == buf2int(e + 2488 + (j + 1) * 4)) continue;
        for (int k = 0; k < 16; k++) ccd[0][sx + k] += f32at(e, 2632 + (j * 16 + k) * 4);
    }
    for (int j = 0; j < HR_N; j++) {
        if (j < HR_N - 2 && g_hr_start[j] == g_hr_start[j + 1]) continue;
        for (int k = 0; k < g_hr_count[j]; k++) ccd[1][g_hr_start[j] + k] += g_hr_coef[j][k];
    }
    double avg[2] = {0, 0};
    const double dth[2] = {0.0, 0.007};
    for (int k = 0; k < 2; k++) {
        double mx = 0;
        int mi = 0;
        for (int i = 0; i < NRAW; i++) if (ccd[k][i] > mx) { mx = ccd[k][i]; mi = i; }
        double x[4] = {(double)mi, 0, 0, (double)mi}, y[4];
        for (int i = mi; i >= 0 && ccd[k][i] > 0.8 * mx; i--) x[0] = i;
        for (int i = mi; i < NRAW && ccd[k][i] > 0.8 * mx; i++) x[3] = i;
        x[0] += 2.0;
        x[3] -= 6.0;
        x[1] = floor((2 * x[0] + x[3]) / 3.0);
        x[2] = floor((x[0] + 2 * x[3]) / 3.0);
        for (int i = 0; i < 4; i++) y[i] = ccd[k][(int)x[i]];
        for (int i = 0; i < NRAW; i++) {
            double xw2 = i, yw = 0;
            for (int a = 0; a < 4; a++) {
                double t = y[a];
                for (int b = 0; b < 4; b++) if (b != a) t *= (xw2 - x[b]) / (x[a] - x[b]);
                yw += t;
            }
            // As in C Argyll, coincident points give inf/NaN here and the test below is then simply false.
            if ((xw2 < x[0] || xw2 > x[3]) && yw == yw && yw != 0 && fabs(ccd[k][i] - yw) / yw > dth[k]) ccd[k][i] = yw;
            avg[k] += ccd[k][i];
        }
        avg[k] /= NRAW;
    }
    if (!(avg[1] > 0)) return false;
    const double scale = 10.0 / HR_WIDTH * avg[0] / avg[1];
    for (int j = 0; j < HR_N; j++) for (int k = 0; k < g_hr_count[j]; k++) g_hr_coef[j][k] = (float)(g_hr_coef[j][k] * scale);
    // 4. Stray light (ONEDSTRAYLIGHTUS): near-diagonal entries interpolated from further out (with Argyll's integer
    // division for the weight), resampled linearly, scaled by HR_WIDTH / 10, positive values dropped, and each row's
    // diagonal set so the row sums to 1.
    const double td = u16at(e, 0) >= 4 ? f32at(e, 8020) : 0.001;
    auto s1 = [&](int i, int j) { return td * (int16_t)u16at(e, 5428 + (i * 36 + j) * 2) + (i == j ? 1.0 : 0.0); };
    for (int i = 0; i < 36; i++)
        for (int j = 0; j < 36; j++) {
            double v = s1(i, j);
            if (j == i - 1 || j == i || j == i + 1) {
                int j0, j1;
                if (j == i - 1) { if (j <= 0) j0 = j + 3, j1 = j + 4; else if (j >= 33) j0 = j - 2, j1 = j - 1; else j0 = j - 1, j1 = j + 3; }
                else if (j == i) { if (j <= 1) j0 = j + 2, j1 = j + 3; else if (j >= 34) j0 = j - 3, j1 = j - 2; else j0 = j - 2, j1 = j + 2; }
                else { if (j <= 2) j0 = j + 1, j1 = j + 2; else if (j >= 35) j0 = j - 4, j1 = j - 3; else j0 = j - 3, j1 = j + 1; }
                double w1 = (j - j0) / (j1 - j0);   // int / int, as in Argyll
                v = (1.0 - w1) * s1(i, j0) + w1 * s1(i, j1);
            }
            g_hr_slp[i][j] = (float)v;
        }
    for (int i = 0; i < HR_N; i++) {
        double sum = 0;
        for (int j = 0; j < HR_N; j++) if (j < i - 1 || j > i + 1) sum += hr_stray_off(i, j);
        g_hr_diag[i] = (float)(1.0 - sum);
    }
    g_hr_ok = true;
    ESP_LOGI(TAGM, "high res: %d bands at %.2f nm ready", HR_N, HR_WIDTH);
    return true;
}

// Off-diagonal stray-light weight of input band j on output band i (bilinear in the 36 x 36 table, <= 0).
static double hr_stray_off(int i, int j) {
    double p0 = hr_wl(i), p1 = hr_wl(j);
    if (p0 < HR_MIN_WL) p0 = HR_MIN_WL;
    if (p1 < HR_MIN_WL) p1 = HR_MIN_WL;
    double xx = 35.0 * (p0 - 380.0) / 350.0, yy = 35.0 * (p1 - 380.0) / 350.0;
    int x0 = (int)floor(xx), y0 = (int)floor(yy);
    if (x0 < 0) x0 = 0; else if (x0 > 34) x0 = 34;
    if (y0 < 0) y0 = 0; else if (y0 > 34) y0 = 34;
    double w1 = xx - x0, w0 = 1 - w1, v1 = yy - y0, v0 = 1 - v1;
    double v = w0 * v0 * g_hr_slp[x0][y0] + w0 * v1 * g_hr_slp[x0][y0 + 1] + w1 * v0 * g_hr_slp[x0 + 1][y0] + w1 * v1 * g_hr_slp[x0 + 1][y0 + 1];
    v *= HR_WIDTH / 10.0;
    return v > 0.0 ? 0.0 : v;
}

bool hires_ready() { return g_hr_ok; }
int hires_bands() { return HR_N; }
double hires_wavelength(int j) { return hr_wl(j); }
const float *last_hires() { return g_last_hr; }

// 106-band spectrum (380-730 nm, 3.33 nm steps) from the 128 linearised sensor values, in the same units as the
// standard spectrum for the mode.
bool hires_spectrum(const float *absraw, Mode mode, float *out) {
    if (!g_hr_ok || !g_cal) return false;
    const uint8_t *e = g_cal;
    int cal_off = mode == MODE_AMBIENT ? 5256 : mode == MODE_EMISSION ? 5112 : 8024;
    double cx[36], cy[36];
    for (int j = 0; j < 36; j++) {
        cx[j] = 380.0 + 10.0 * j;
        cy[j] = f32at(e, cal_off + j * 4);
        if (cy[j] != cy[j]) return false;
    }
    static Spline cs;
    cs.fit(cx, cy, 36);
    static double tm[HR_N];
    for (int j = 0; j < HR_N; j++) {
        double o = 0;
        for (int k = 0; k < g_hr_count[j]; k++) o += g_hr_coef[j][k] * absraw[g_hr_start[j] + k];
        tm[j] = o;
    }
    for (int i = 0; i < HR_N; i++) {
        double o = g_hr_diag[i] * tm[i];
        for (int j = 0; j < HR_N; j++) if (j < i - 1 || j > i + 1) o += hr_stray_off(i, j) * tm[j];
        double w = hr_wl(i);
        double cf = cs.at(w < HR_MIN_WL ? HR_MIN_WL : w);
        out[i] = (float)(o * (cf < 0 ? 0 : cf));
    }
    return true;
}


// Argyll munki_optimise_sensor(): from the current integration time / gain and the scale that would bring the
// signal to the target, the integration time and gain to use next. *fill = expected signal as a fraction of the
// target (1 = on target; well below 1 = still dim).
static void optimise(double T, int gain, double scale, double tmin, double *nT, int *ngain, double *fill) {
    double want = T * scale * (gain ? g_highgain : 1.0);   // normal-gain time that would reach the target
    double t = want;
    int g = 0;
    if (t > TARG_MAX_INT) {
        if (TARG_MAX_INT / t > TARG_SCALE2) t = TARG_MAX_INT;   // accept a lower signal to stay at 2 s
        else t *= TARG_SCALE2;                                  // very dim: aim for 15 % of the target
        if (high_gain_ready() && t > MAX_INT) { t /= g_highgain; g = 1; }
    }
    if (t > MAX_INT) t = MAX_INT;
    if (t < tmin) t = tmin;
    *nT = t;
    *ngain = g;
    *fill = want > 0 ? t * (g ? g_highgain : 1.0) / want : 1.0;
}


Result measure_spectrum(Mode mode, Spectrum *out) {
    Params pr;
    if (!g_cal || !get_params(&pr)) return MR_NO_DATA;
    if (!g_dark[0].valid) return MR_NO_DARK;
    if (mode == MODE_PROJECTOR && u16at(g_cal, 0) < 5) return MR_NO_PROJ;
    const double tmin = min_int_time(pr);
    const double target = TARG_SCALE * pr.opt_sval;
    double T = 0.25;          // Argyll's first trial time
    int gain = 0;
    double signal = 0, fill = 1.0;
    Raw r;
    for (int it = 0; it < 5; it++) {
        if (cancelled()) return MR_CANCELLED;
        if (T < tmin) T = tmin;
        if (!measure_raw(false, gain, T, 0, 1, &r)) return MR_COMMS;
        T = r.inttime;
        bool sat = r.max > pr.sat_limit;
        if (sat) {
            ESP_LOGI(TAGM, "trial %d: T %.4f s gain %d saturated (peak %.0f)", it, T, gain, r.max);
            if (gain == 0 && T <= tmin * 1.001) return MR_TOO_BRIGHT;
            T = it == 0 ? tmin : T * 0.25;    // Argyll retries at the minimum time
            gain = 0;
            continue;
        }
        double dk[NRAW + 1];
        dark_at(T, gain, dk);
        double S = 0;
        for (int j = 0; j < NRAW; j++) if (r.band[j] - dk[1 + j] > S) S = r.band[j] - dk[1 + j];
        signal = S;
        double scale = target / (S < 1.0 ? 1.0 : S);
        double nT;
        int ng;
        optimise(T, gain, scale, tmin, &nT, &ng, &fill);
        ESP_LOGI(TAGM, "trial %d: T %.4f s gain %d signal %.0f peak %.0f scale %.3f -> T %.4f gain %d fill %.2f",
                 it, T, gain, S, r.max, scale, nT, ng, fill);
        bool close = ng == gain && nT > T * 0.8 && nT < T * 1.25;
        T = nT;
        gain = ng;
        if (close) break;
    }
    // Normally average READ_TIME worth of integrations; in low light average LOW_READ_TIME worth.
    const bool dim = gain == 1 || T >= TARG_MAX_INT * 0.99;
    int n = (int)((dim ? LOW_READ_TIME : READ_TIME) / T + 0.5);
    if (n < 1) n = 1;
    if (n > 40) n = 40;
    if (cancelled()) return MR_CANCELLED;
    if (!measure_raw(false, gain, T, 0, n, &r)) return MR_COMMS;
    if (r.max > pr.sat_limit) return MR_TOO_BRIGHT;
    memset(out, 0, sizeof *out);
    if (!compute_spectrum(r, mode, out->v)) return mode == MODE_PROJECTOR ? MR_NO_PROJ : MR_NO_DATA;

    out->inttime = r.inttime;
    out->nmeas = n;
    out->gain = r.gain;
    out->hires = hires_spectrum(g_last_absraw, mode, g_last_hr);
    out->signal = signal;
    int pk = 0;
    for (int j = 1; j < 36; j++) if (out->v[j] > out->v[pk]) pk = j;
    out->peak_nm = 380 + pk * 10;
    ESP_LOGI(TAGM, "reading: T %.4f s x %d, gain %d, fill %.2f", r.inttime, n, r.gain, fill);
    // Below about a sixth of the optimum signal the reading is noticeably noisy: flag it.
    return fill < TARG_SCALE2 * 1.01 ? MR_LOW_LIGHT : MR_OK;
}

const char *result_name(Result r) {
    switch (r) {
        case MR_OK: return "OK";
        case MR_NO_DARK: return "DARK CAL FIRST (DIAL TO CAL)";
        case MR_NO_DATA: return "NO CALIBRATION DATA";
        case MR_COMMS: return "INSTRUMENT NOT ANSWERING";
        case MR_TOO_BRIGHT: return "TOO BRIGHT";
        case MR_LOW_LIGHT: return "LOW LIGHT";
        case MR_NO_PROJ: return "NO PROJECTOR DATA";
        case MR_CANCELLED: return "CANCELLED";
    }
    return "?";
}

const char *pos_name(uint8_t s) {
    switch (s) {
        case POS_PROJ: return "PROJECTOR";
        case POS_SURF: return "SURFACE";
        case POS_CALIB: return "CALIBRATION";
        case POS_AMBIENT: return "AMBIENT";
    }
    return "UNKNOWN";
}

}  // namespace munki
