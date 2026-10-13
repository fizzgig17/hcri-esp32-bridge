// SPDX-License-Identifier: AGPL-3.0-or-later
// Ported from ArgyllCMS spectro/munki_imp.c (Copyright Graeme W. Gill, AGPL-3.0). See munki_argyll.h.
#ifdef COLORMUNKI_PROBE
#include "munki_argyll.h"
#include <string.h>
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

bool get_firmware(Firmware *f) {
    uint8_t p[24];
    if (!ctrl(true, 0x86, p, 24)) return false;
    f->fw_major = buf2int(&p[0]);
    f->fw_minor = buf2int(&p[4]);
    f->tick_dur = buf2int(&p[8]);
    f->min_int_count = buf2int(&p[12]);
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

const char *pos_name(uint8_t s) {
    switch (s) {
        case POS_PROJ: return "BETWEEN";
        case POS_SURF: return "SURFACE";
        case POS_CALIB: return "CALIBRATION";
        case POS_AMBIENT: return "AMBIENT";
    }
    return "UNKNOWN";
}

}  // namespace munki
#endif  // COLORMUNKI_PROBE
