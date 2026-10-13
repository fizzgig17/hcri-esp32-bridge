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

bool get_firmware(Firmware *f);          // vendor request 0x86
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
    double mean, max;        // over the 128 bands
    double led;              // LED temperature value (raw)
    int n;                   // readings averaged
};
// Triggers a measurement (lamp on or off), discards `ninvalid` leading readings and averages the next `nmeas`.
bool measure_raw(bool lamp, double inttime_s, int ninvalid, int nmeas, Raw *out);

const char *pos_name(uint8_t spos);

}  // namespace munki
