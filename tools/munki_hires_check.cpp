// PC-side check of the high-resolution (3.33 nm) path in src/munki_argyll.cpp on real data: a calibration block
// (as tools/munki_diag.py extracts it from an app log) and readings' 128 linearised sensor values, one reading per
// line. Prints each reading's 106 bands, ambient mode, one line per reading.
//   g++ -std=gnu++17 -I tools/sim_stubs -I src -o munki_hires_check tools/munki_hires_check.cpp
//   ./munki_hires_check calibration_1.bin absraw.txt
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "usb/usb_host.h"
int64_t esp_timer_get_time(void) { return 0; }
esp_err_t usb_host_transfer_alloc(size_t, int, usb_transfer_t **) { return 1; }
esp_err_t usb_host_transfer_free(usb_transfer_t *) { return 0; }
esp_err_t usb_host_transfer_submit(usb_transfer_t *) { return 1; }
esp_err_t usb_host_transfer_submit_control(usb_host_client_handle_t, usb_transfer_t *) { return 1; }
esp_err_t usb_host_client_handle_events(usb_host_client_handle_t, uint32_t) { return 0; }
esp_err_t usb_host_interface_claim(usb_host_client_handle_t, usb_device_handle_t, uint8_t, uint8_t) { return 0; }
esp_err_t usb_host_interface_release(usb_host_client_handle_t, usb_device_handle_t, uint8_t) { return 0; }
esp_err_t usb_host_endpoint_halt(usb_device_handle_t, uint8_t) { return 0; }
esp_err_t usb_host_endpoint_flush(usb_device_handle_t, uint8_t) { return 0; }
esp_err_t usb_host_endpoint_clear(usb_device_handle_t, uint8_t) { return 0; }
#include "../src/munki_argyll.cpp"

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s calibration.bin absraw.txt\n", argv[0]); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) return 2;
    std::vector<uint8_t> b(16384 + 4, 0);
    size_t n = fread(b.data(), 1, b.size(), f);
    fclose(f);
    munki::g_cal = (uint8_t *)calloc(n + 4, 1);
    memcpy(munki::g_cal, b.data(), n);
    munki::g_cal_size = (int)n;
    if (!munki::build_hires()) { fprintf(stderr, "high res could not be built\n"); return 1; }
    FILE *r = fopen(argv[2], "r");
    float ab[128], out[128];
    while (r && !feof(r)) {
        int got = 0;
        for (int i = 0; i < 128; i++) got += fscanf(r, "%f", &ab[i]);
        if (got != 128) break;
        if (!munki::hires_spectrum(ab, munki::MODE_AMBIENT, out)) return 1;
        for (int i = 0; i < munki::hires_bands(); i++) printf("%.9g%c", out[i], i + 1 < munki::hires_bands() ? ' ' : '\n');
    }
    return 0;
}
