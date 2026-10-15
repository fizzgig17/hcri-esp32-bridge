// SPDX-License-Identifier: AGPL-3.0-or-later
// ColorMunki (X-Rite, USB 0971:2007) low-level driver, PORTED FROM ArgyllCMS (spectro/munki_imp.c,
// https://github.com/capnm/ArgyllCMS, Copyright Graeme W. Gill, AGPL-3.0). This file and munki_argyll.cpp are
// derived works and carry the AGPL-3.0 licence of the original. Keep ALL Argyll-derived code in these two files
// behind this small interface, so it can be replaced by an independent rewrite later.
#pragma once
#include <stdint.h>
#include "usb/usb_host.h"

namespace munki {

enum SensorPos : uint8_t { POS_PROJ = 0, POS_SURF = 1, POS_CALIB = 2, POS_AMBIENT = 3 };

struct Firmware {
    int fw_major, fw_minor;   // firmware revision
    int tick_dur;             // tick duration
    int min_int_count;        // minimum integration tick count
    int ee_blocks, ee_block_size;
};

// Claims interface 0. Returns true when the instrument can be talked to.
bool open(usb_host_client_handle_t client, usb_device_handle_t dev);
void close();

bool get_firmware(Firmware *f);          // vendor request 0x86 (read once per open(), then cached)
bool get_version(char out[37]);          // 0x85
bool get_chip_id(uint8_t out[8]);        // 0x8A
bool get_status(uint8_t *spos, uint8_t *button);   // 0x87: sensor position, button (1 = pressed)
bool read_eeprom(uint8_t *buf, int addr, int size);  // 0x81 request then bulk IN 0x81

// Reads the instrument's calibration block (size is stored at EEPROM offset 4) and checks its checksum, as Argyll does.
struct Eeprom {
    int size;                 // calibration block length in bytes
    int cal_version;          // calibration version
    int compat_version;       // oldest software version that understands it
    uint32_t sum_stored;      // checksum held in the block
    uint32_t sum_calc;        // checksum worked out from the data
    bool read_ok;             // transfer completed
    bool sum_ok;              // checksums match
};
bool read_calibration(Eeprom *e);
const uint8_t *calibration_data();   // the block read above (nullptr until read), length = Eeprom::size rounded up to 4
int calibration_size();               // its length in bytes (0 until read)

// Measurement settings the instrument stores in its calibration block (offsets as in Argyll's EEPROM parser).
struct Params {
    double min_sval, opt_sval, max_sval, sat_limit;   // sensor target values and saturation limit (raw counts)
    double cal_int_time;     // integration time to use for calibration (seconds)
    double preheat_s;        // LED pre-heat time (seconds)
    double wait_s;           // LED wait time (seconds)
    double invalid_s;        // reflectance readings to discard while the LED settles (seconds)
    int hold_temp_duty;      // LED hold-temperature duty cycle
    double tick_s;           // length of one integration clock tick (seconds)
};
bool get_params(Params *p);   // needs read_calibration() and get_firmware() first

// One averaged raw reading: 128 sensor bands plus the 4 shielded (dark) cells and the LED temperature value.
struct Raw {
    float band[128];
    double shielded;         // mean of the 4 shielded cells
    double mean;             // mean over the 128 bands (averaged readings)
    double max;              // highest band value in any SINGLE reading (for the saturation check)
    double led;              // LED temperature value (raw)
    double inttime;          // integration time actually used (whole clock ticks), seconds
    int gain;                // 0 normal, 1 high
    int n;                   // readings averaged
};
// Triggers a measurement (lamp on or off, normal or high gain), discards `ninvalid` leading readings and averages
// the next `nmeas`. On a failed transfer it discards whatever the instrument is still sending before returning.
bool measure_raw(bool lamp, int gain, double inttime_s, int ninvalid, int nmeas, Raw *out);

// ---- Emission / ambient spectrum (Argyll's adaptive emissive spot measurement, standard resolution) ----
enum Mode { MODE_AMBIENT, MODE_EMISSION, MODE_PROJECTOR };   // dial: ambient / surface / projector
enum Result { MR_OK = 0, MR_NO_DARK, MR_NO_DATA, MR_COMMS, MR_TOO_BRIGHT, MR_LOW_LIGHT, MR_NO_PROJ };
struct Spectrum {
    float v[36];             // 380..730 nm in 10 nm steps, in Argyll's units for the mode
    double inttime;          // integration time used (seconds)
    int nmeas;               // readings averaged
    int gain;                // 0 normal, 1 high gain (only in very dim light)
    double signal;           // peak raw signal above the dark level (counts)
    int peak_nm;             // wavelength of the strongest band
};
// Dark calibration: dial on CALIBRATION. Reads the sensor dark level at two integration times, as Argyll does, for
// normal gain and (if MUNKI_HIGH_GAIN) for high gain.
bool dark_calibrate();
bool dark_valid();
bool high_gain_ready();
// The 128 dark-subtracted, linearised sensor values (per second) behind the last spectrum, for diagnostics.
const float *last_absraw();
// Seconds since the last successful dark calibration, or 0xFFFFFFFF when there is none.
uint32_t dark_age_s();
// Dial must already be on the matching position. MR_LOW_LIGHT still returns a usable spectrum.
Result measure_spectrum(Mode mode, Spectrum *out);
const char *result_name(Result r);
// A short code for the last failure ("REQ 80 ST 3", "BULK TIMEOUT"...), to show on the board when something does not work.
const char *last_error();

const char *pos_name(uint8_t spos);

}  // namespace munki
