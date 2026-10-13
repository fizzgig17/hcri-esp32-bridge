// SPDX-License-Identifier: AGPL-3.0-or-later
// Ported from ArgyllCMS spectro/munki_imp.c (Copyright Graeme W. Gill, AGPL-3.0). See munki_argyll.h.
#ifdef COLORMUNKI_PROBE
#include "munki_argyll.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAGM = "munki";

namespace munki {

static usb_host_client_handle_t g_client = NULL;
static usb_device_handle_t g_dev = NULL;

struct Wait { volatile bool done; volatile usb_transfer_status_t status; volatile int bytes; };

static void xfer_cb(usb_transfer_t *t) {
    Wait *w = (Wait *)t->context;
    w->status = t->status;
    w->bytes = t->actual_num_bytes;
    w->done = true;
}

static void pump(Wait &w, int timeout_ms) {
    int waited = 0;
    while (!w.done && waited < timeout_ms + 500) {
        usb_host_client_handle_events(g_client, pdMS_TO_TICKS(20));
        waited += 20;
    }
}

// Vendor control transfer on endpoint 0 (the instrument uses these for almost everything).
static bool ctrl(bool in, uint8_t req, uint8_t *buf, int len, int timeout_ms = 2000) {
    if (!g_dev) return false;
    const int mps = 64;
    int data_len = in ? ((len + mps - 1) / mps) * mps : len;
    usb_transfer_t *t = NULL;
    if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + data_len, 0, &t) != ESP_OK) return false;
    usb_setup_packet_t *s = (usb_setup_packet_t *)t->data_buffer;
    s->bmRequestType = (in ? 0x80 : 0x00) | 0x40;   // vendor, device
    s->bRequest = req;
    s->wValue = 0;
    s->wIndex = 0;
    s->wLength = len;
    if (!in && len) memcpy(t->data_buffer + sizeof(usb_setup_packet_t), buf, len);
    Wait w = {false, USB_TRANSFER_STATUS_ERROR, 0};
    t->device_handle = g_dev;
    t->bEndpointAddress = 0;
    t->num_bytes = sizeof(usb_setup_packet_t) + data_len;
    t->callback = xfer_cb;
    t->context = &w;
    t->timeout_ms = timeout_ms;
    bool ok = false;
    if (usb_host_transfer_submit_control(g_client, t) == ESP_OK) {
        pump(w, timeout_ms);
        ok = w.done && w.status == USB_TRANSFER_STATUS_COMPLETED;
        if (ok && in) memcpy(buf, t->data_buffer + sizeof(usb_setup_packet_t), len);
        if (!ok) ESP_LOGW(TAGM, "request 0x%02X failed (status %d)", req, (int)w.status);
    }
    if (w.done || !ok) usb_host_transfer_free(t);
    return ok;
}

// Bulk IN read (instrument data comes back on endpoint 0x81).
static bool bulk_in(uint8_t *buf, int len, int timeout_ms) {
    int off = 0;
    while (off < len) {
        int chunk = len - off;
        if (chunk > 4096) chunk = 4096;
        int alloc = ((chunk + 63) / 64) * 64;
        usb_transfer_t *t = NULL;
        if (usb_host_transfer_alloc(alloc, 0, &t) != ESP_OK) return false;
        Wait w = {false, USB_TRANSFER_STATUS_ERROR, 0};
        t->device_handle = g_dev;
        t->bEndpointAddress = 0x81;
        t->num_bytes = alloc;
        t->callback = xfer_cb;
        t->context = &w;
        t->timeout_ms = timeout_ms;
        bool ok = false;
        if (usb_host_transfer_submit(t) == ESP_OK) {
            pump(w, timeout_ms);
            ok = w.done && w.status == USB_TRANSFER_STATUS_COMPLETED && w.bytes >= chunk;
            if (ok) memcpy(buf + off, t->data_buffer, chunk);
        }
        if (w.done || !ok) usb_host_transfer_free(t);
        if (!ok) return false;
        off += chunk;
    }
    return true;
}

static int buf2int(const uint8_t *b) {   // little endian, signed 32 bit
    return (int)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}
static void int2buf(uint8_t *b, int v) {
    b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}

bool open(usb_host_client_handle_t client, usb_device_handle_t dev) {
    g_client = client;
    g_dev = dev;
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

bool get_firmware(Firmware *f) {
    uint8_t p[24];
    if (!ctrl(true, 0x86, p, 24)) return false;
    f->fw_major = buf2int(&p[0]);
    f->fw_minor = buf2int(&p[4]);
    f->tick_dur = buf2int(&p[8]);
    f->min_int_count = buf2int(&p[12]);
    g_tick_us = f->tick_dur;
    f->ee_blocks = buf2int(&p[16]);
    f->ee_block_size = buf2int(&p[20]);
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
    return bulk_in(buf, size, 6000);
}

static uint8_t *g_cal = NULL;

static uint32_t buf2uint(const uint8_t *b) { return (uint32_t)buf2int(b); }

const uint8_t *calibration_data() { return g_cal; }

// Argyll munki_imp_init(): read the size at offset 4, read that many bytes from offset 0, sum the 32 bit words
// with the stored checksum (at offset 8) zeroed, and compare.
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

bool measure_raw(bool lamp, double inttime_s, int ninvalid, int nmeas, Raw *out) {
    Params pr;
    if (!get_params(&pr) || nmeas < 1 || ninvalid < 0) return false;
    int total = ninvalid + nmeas;
    int intclocks = (int)(inttime_s / pr.tick_s + 0.5);
    uint8_t cmd[12] = {};
    cmd[0] = lamp ? 1 : 0;   // lamp
    cmd[1] = 0;              // scan: off (spot reading)
    cmd[2] = 0;              // gain: normal
    cmd[3] = (uint8_t)pr.hold_temp_duty;
    int2buf(&cmd[4], intclocks);
    int2buf(&cmd[8], total);
    if (!ctrl(false, 0x80, cmd, 12)) return false;
    size_t bytes = (size_t)RDBYTES * total;
    uint8_t *buf = (uint8_t *)malloc(bytes);
    if (!buf) return false;
    int timeout = 2000 + (int)(inttime_s * total * 1000.0);
    bool ok = bulk_in(buf, (int)bytes, timeout);
    if (ok) {
        memset(out, 0, sizeof *out);
        double acc[NRAW] = {};
        for (int i = ninvalid; i < total; i++) {
            const uint8_t *r = buf + (size_t)i * RDBYTES;
            for (int k = 0; k < 4; k++) out->shielded += u16at(r, k * 2);
            for (int j = 0; j < NRAW; j++) acc[j] += u16at(r, (6 + j) * 2);
            out->led += u16at(r, (NSEN - 1) * 2);
        }
        out->n = nmeas;
        out->shielded /= 4.0 * nmeas;
        out->led /= nmeas;
        double sum = 0, mx = 0;
        for (int j = 0; j < NRAW; j++) {
            out->band[j] = (float)(acc[j] / nmeas);
            sum += out->band[j];
            if (out->band[j] > mx) mx = out->band[j];
        }
        out->mean = sum / NRAW;
        out->max = mx;
    }
    free(buf);
    return ok;
}

// ----------------------------------------------------------------------------------------------------------
// Emission / ambient spectrum. Follows Argyll: adaptive dark reference interpolated linearly in integration time
// (munki_interp_dark), shielded-cell drift correction and linearisation (munki_sub_raw_to_absraw), the 36 x 16
// wavelength matrix and stray-light matrix (munki_absraw_to_abswav1), then the mode's calibration factors.
static const double MAX_INT = 4.5, TARG_MAX_INT = 2.0, TARG_SCALE = 0.90, READ_TIME = 1.0;

struct DarkRef {
    bool valid;
    double t0, t1;
    float d0[NRAW + 1], d1[NRAW + 1];   // [0] = shielded cells, [1..128] = bands
};
static DarkRef g_dark = {};

static void put_dark(float *dst, const Raw &r) {
    dst[0] = (float)r.shielded;
    for (int j = 0; j < NRAW; j++) dst[1 + j] = r.band[j];
}

bool dark_valid() { return g_dark.valid; }

bool dark_calibrate() {
    Params pr;
    if (!get_params(&pr)) return false;
    g_dark.valid = false;
    double tmin = pr.tick_s * 224.0;
    {   // minimum integration time from the firmware, as Argyll computes it (tick * min tick count)
        Firmware f;
        if (get_firmware(&f)) tmin = pr.tick_s * f.min_int_count;
    }
    Raw r;
    int n0 = (int)(READ_TIME / tmin + 0.5);
    if (n0 < 1) n0 = 1;
    if (n0 > 140) n0 = 140;
    if (!measure_raw(false, tmin, 0, n0, &r)) return false;
    g_dark.t0 = tmin;
    put_dark(g_dark.d0, r);
    if (!measure_raw(false, 2.0, 0, 1, &r)) return false;
    g_dark.t1 = 2.0;
    put_dark(g_dark.d1, r);
    g_dark.valid = true;
    return true;
}

// Dark level at integration time T: linear in T for every cell (index 0 = shielded cells).
static void dark_at(double T, double *out) {
    for (int j = 0; j <= NRAW; j++) {
        double slope = ((double)g_dark.d1[j] - g_dark.d0[j]) / (g_dark.t1 - g_dark.t0);
        double base = g_dark.d0[j] - slope * g_dark.t0;
        out[j] = base + T * slope;
    }
}

static bool compute_spectrum(const Raw &r, double T, Mode mode, float *out) {
    const uint8_t *e = g_cal;
    int calver = u16at(e, 0);
    double dk[NRAW + 1];
    dark_at(T, dk);
    double sub_sh = dk[0];
    double *sub = dk + 1;
    // zero point for the shielded-cell drift correction
    double avgs = r.shielded, submax = -1e6;
    for (int j = 0; j < NRAW; j++) if (sub[j] > submax) submax = sub[j];
    double zero = 1.08 * 0.5 * (avgs + sub_sh);
    if (zero < 1.005 * avgs) zero = 1.005 * avgs;
    if (zero < 1.005 * sub_sh) zero = 1.005 * sub_sh;
    if (zero < 1.005 * submax) zero = 1.005 * submax;
    double lin[4];   // Argyll stores them reversed (rget_32_doubles)
    for (int i = 0; i < 4; i++) lin[i] = f32at(e, 4936 + (3 - i) * 4);
    double absraw[NRAW];
    for (int j = 0; j < NRAW; j++) {
        double asub = zero - (zero - sub[j]) * (zero - avgs) / (zero - sub_sh);
        double s = r.band[j] - asub;
        double l = lin[3];
        for (int k = 2; k >= 0; k--) l = l * s + lin[k];
        absraw[j] = l / T;
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

Result measure_spectrum(Mode mode, Spectrum *out) {
    Params pr;
    if (!g_cal || !get_params(&pr)) return MR_NO_DATA;
    if (!g_dark.valid) return MR_NO_DARK;
    if (mode == MODE_PROJECTOR && u16at(g_cal, 0) < 5) return MR_NO_PROJ;
    Firmware f;
    if (!get_firmware(&f)) return MR_COMMS;
    double tmin = pr.tick_s * f.min_int_count;
    double T = 0.1;
    double signal = 0;
    bool low = false;
    Raw r;
    for (int it = 0; it < 5; it++) {
        if (T < tmin) T = tmin;
        if (!measure_raw(false, T, 0, 1, &r)) return MR_COMMS;
        double dk[NRAW + 1];
        dark_at(T, dk);
        double S = 0;
        for (int j = 0; j < NRAW; j++) if (r.band[j] - dk[1 + j] > S) S = r.band[j] - dk[1 + j];
        signal = S;
        bool sat = r.max >= pr.sat_limit * 0.98;
        double target = TARG_SCALE * pr.opt_sval;
        double scale = sat ? 0.25 : target / (S < 1.0 ? 1.0 : S);
        ESP_LOGI(TAGM, "trial %d: T %.4f s signal %.0f max %.0f sat %d target %.0f scale %.3f", it, T, S, r.max, sat, target, scale);
        if (sat && T <= tmin * 1.001) return MR_TOO_BRIGHT;
        if (!sat && scale > 0.8 && scale < 1.25) break;
        double nt = T * scale;
        if (nt > TARG_MAX_INT) nt = TARG_MAX_INT;
        if (nt > MAX_INT) nt = MAX_INT;
        if (nt < tmin) nt = tmin;
        if (!sat && fabs(nt - T) < 1e-4) { low = (T >= TARG_MAX_INT * 0.99 && scale > 1.25); break; }
        low = (nt >= TARG_MAX_INT * 0.99 && scale > 1.25);
        T = nt;
    }
    int n = (int)(READ_TIME / T + 0.5);
    if (n < 1) n = 1;
    if (n > 40) n = 40;
    if (!measure_raw(false, T, 0, n, &r)) return MR_COMMS;
    if (r.max >= pr.sat_limit * 0.98) return MR_TOO_BRIGHT;
    memset(out, 0, sizeof *out);
    if (!compute_spectrum(r, T, mode, out->v)) return mode == MODE_PROJECTOR ? MR_NO_PROJ : MR_NO_DATA;
    out->inttime = T;
    out->nmeas = n;
    out->signal = signal;
    int pk = 0;
    for (int j = 1; j < 36; j++) if (out->v[j] > out->v[pk]) pk = j;
    out->peak_nm = 380 + pk * 10;
    return low ? MR_LOW_LIGHT : MR_OK;
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
#endif  // COLORMUNKI_PROBE
