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

const char *pos_name(uint8_t spos);

}  // namespace munki
