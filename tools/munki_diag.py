#!/usr/bin/env python3
"""Pulls the ColorMunki diagnostics out of an hCRI Companion debug log (Logs tab -> Share).

The bridge sends, after each ColorMunki reading, the 128 dark-subtracted linearised sensor values behind it, and
once per connection the instrument's calibration block. The app writes each as one log line:
    ColorMunki diag 1 (664 bytes): <base64>     (a reading)
    ColorMunki diag 2 (8192 bytes): <base64>    (calibration block)
This writes them to OUT_DIR as calibration_<n>.bin and reading_<n>.json, the inputs for developing and checking the
high-resolution (3.3 nm) mode offline.

    tools/munki_diag.py hcri-log.txt OUT_DIR
"""
import base64, json, re, struct, sys
from pathlib import Path

if len(sys.argv) != 3:
    sys.exit(__doc__)
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
nr = nc = 0
for line in Path(sys.argv[1]).read_text(errors='replace').splitlines():
    m = re.search(r'ColorMunki diag (\d+) \((\d+) bytes\): ([A-Za-z0-9+/=]+)', line)
    if not m:
        continue
    kind, size, data = int(m.group(1)), int(m.group(2)), base64.b64decode(m.group(3))
    if len(data) != size:
        print('skipping a damaged line (%d of %d bytes)' % (len(data), size))
        continue
    if kind == 2:
        nc += 1
        (out / ('calibration_%d.bin' % nc)).write_bytes(data)
    elif kind == 1 and data[0] == 1 and len(data) == 8 + 128 * 4 + 36 * 4:
        nr += 1
        gain, nmeas, inttime = data[1], struct.unpack_from('<H', data, 2)[0], struct.unpack_from('<f', data, 4)[0]
        absraw = list(struct.unpack_from('<128f', data, 8))
        bands = list(struct.unpack_from('<36f', data, 8 + 512))
        (out / ('reading_%d.json' % nr)).write_text(json.dumps({
            'gain': gain, 'readings_averaged': nmeas, 'integration_s': inttime,
            'absraw_128': absraw, 'bands_380_730_10nm': bands}, indent=1))
print('%d reading(s), %d calibration block(s) written to %s' % (nr, nc, out))
