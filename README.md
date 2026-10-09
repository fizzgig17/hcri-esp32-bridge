# Torch Bearer scanner (T-Display S3)

Press the right button (GPIO14) or BOOT: the board asks the Torch Bearer for one
auto-exposure scan over USB (CH340 serial, 115200 8N1) and shows peak wavelength,
peak value, sum, exposure and a spectrum plot. GPLv3 (derived from Torch-Bearer-Tools / tobes-ui).

## Hardware you need

For use with the hCRI Companion Android app:

- **Torch Bearer spectrometer** (the thing being bridged).
- **LilyGO T-Display S3**, the standard non-Pro version with 16 MB flash. It is the bridge: it talks
  USB to the spectrometer and Bluetooth LE to your phone.
- **USB cable/adapter set**: a USB-C male OTG cable or adapter that gives you a female USB-A
  (for the spectrometer) and a female USB-C (for power) at the same time.
- **Power bank** to run the bridge in the field. (A PC USB port works for flashing only.)
- **T-Display-S3 Pro (battery model):** shows the battery percentage in the header (a lightning bolt
  too while outside power is connected). With nothing plugged into its USB-C port it can run the
  Torch Bearer from its own battery: use a plain USB-C male to USB-A female OTG adapter, plug the
  Torch Bearer's USB-A cable into it, and the board's power chip supplies 5 V from the battery.
  While outside power is connected the 5 V output is off and the battery charges as usual; the
  battery does not charge while it is powering the Torch Bearer. This feature is untested on
  hardware (compile it out with `-DPRO_OTG_BOOST=0`).
- **Android phone** with the hCRI Companion app (Android 8.0+, Bluetooth LE).
- Optional: a diffuser in front of the lamp; about 25 cm distance works well.

## Installing it (short version)

1. Easiest: download `hcri_companion_esp32_bridge.py` from the [`prod-latest`](../../releases/tag/prod-latest) release and run it
   (see "Easiest" below) -- it finds the port, identifies your board and flashes the right file. Otherwise:
   download `hcri-esp32-bridge-factory.bin` (T-Display S3) from the same release.
2. Put the board in download mode and flash that file at **0x0** (steps below -- Chrome or Edge is enough).
3. Plug the spectrometer into the board with the OTG cable, power it from the power bank, and open
   the hCRI Companion app: the bridge appears as **Torch Bearer**.

The board allows one Bluetooth connection at a time and stops advertising while connected, so close
other Bluetooth tools (for example nRF Connect) if the app can't see it.

## Getting the firmware

Two ways, same as the hCRI Companion app:

| Branch | Release (fixed link) | Use |
|---|---|---|
| `develop` | [`dev-latest`](../../releases/tag/dev-latest) | newest work, rebuilt on every push (~5-10 min) |
| `master` | [`prod-latest`](../../releases/tag/prod-latest) | the released version |

Each release has `hcri-esp32-bridge-factory.bin` (everything in one file -- flash this at **0x0**),
plus `firmware.bin`, `bootloader.bin`, `partitions.bin` and `SHA256SUMS.txt`.
The GitHub Actions workflows in `.github/workflows/` build it with PlatformIO; you can also
re-run one from the Actions tab (Run workflow).

### Easiest: the hCRI Companion ESP32 Bridge program
**`hcri_companion_esp32_bridge.py`** finds the board's USB port, identifies the board (T-Display S3 or T-Display-S3 Pro),
downloads the right firmware (checksum-verified) and flashes it.

- **Where to get it:** it is attached to every release -- download `hcri_companion_esp32_bridge.py` from the
  [`prod-latest`](../../releases/tag/prod-latest) release (or [`dev-latest`](../../releases/tag/dev-latest) for the newest),
  or find it in this repo at [`tools/hcri_companion_esp32_bridge.py`](tools/hcri_companion_esp32_bridge.py).
- **Run it:** install [Python](https://www.python.org/downloads/), plug the board in, then
  `py hcri_companion_esp32_bridge.py` (Windows) or `python3 hcri_companion_esp32_bridge.py`. It installs esptool if needed.
- Options: `--prod` or `--dev` pick the build without asking, `--no-probe` skips identifying the board.

## Flashing a downloaded build

1. **Put the board in download mode** (needed any time the firmware is running -- it takes over
   the USB port as a host): hold **BOOT**, tap **RST** (or plug the USB-C cable in while holding
   BOOT), then release BOOT. A new serial (COM) port appears on the PC.
2. Flash `hcri-esp32-bridge-factory.bin` at address **0x0**, either:
   - **In the browser** (Chrome or Edge, no install): open <https://espressif.github.io/esptool-js/>,
     click Connect and pick the board's port, set *Flash Address* to `0x0`, choose the `.bin`,
     click Program. Close the tab (or Disconnect) when it says done.
   - **With esptool** (`pip install esptool`):
     `esptool.py --chip esp32s3 --port COM5 --baud 460800 write_flash 0x0 hcri-esp32-bridge-factory.bin`
     (use your own port; on Linux/macOS it looks like `/dev/ttyACM0`). Add
     `erase_flash` as a separate first command only if the board misbehaves after a flash.
3. Unplug the board and power it from a power bank via the OTG cable (spectrometer in the female
   USB-A), or tap **RST**. The screen should show the idle Torch Bearer screen.

## Building it yourself (VS Code + PlatformIO extension)
1. File > Open Folder... > this folder (a path **without spaces**). First build downloads the
   toolchain and the espressif/usb_host_ch34x_vcp + usb_host_vcp components (needs internet, takes a while).
2. Plug the T-Display S3 into the PC with USB-C (download mode, see above). Click the Upload arrow
   (bottom bar), or run `pio run -e tdisplay-s3 -t upload`.
3. Changed `sdkconfig.defaults`? Delete `sdkconfig.tdisplay-s3` and `.pio/build` before building,
   or the old settings stick.
4. Unplug. Power from a power bank into the OTG cable's female USB-C, spectrometer in the female USB-A.

## Bluetooth (BLE) trigger
Advertises as "Torch Bearer". Service 7a1c0001-5b2e-4f0a-9c3d-2e8f6b4a1d00:
- 7a1c0002-... command (write): 01 = scan now, 02 = resend last result
- 7a1c0003-... result (notify/read): summary packet, then the full spectrum in chunks
- 7a1c0004-... status (notify/read): [state u8][try u8][exposure_ms f32]; state 0 ready, 1 scanning, 2 done, 3 error, 4 done-not-locked

Result packets (little-endian):
- 01 | status u8 | peak_nm u16 | exposure_ms f32 | peak_val f32 | sum f32 | npts u16 | start_nm u16
- 02 | npts u16 | start_nm u16 | step_nm u8 | total_bytes u16   (header; float32 spectrum values follow)
- 03 | seq u8 | bytes...   (data chunks, seq from 0)
- 04 | chunks u8           (end marker)

Test with nRF Connect: connect, subscribe to result + status, write 01 to the command characteristic.

## Spectral correction and on-board colorimetry

- `src/corr_table.h` holds a per-nm multiplier (340-1000 nm) fitted against an HPCS-330P on white
  lamps. **The board sends the raw spectrum** (`TB_APPLY_CORRECTION` is 0 in `src/metrics.h`) and
  the hCRI Companion app applies the same curve, on by default, with a Settings toggle ("Torch Bearer
  correction"). Keep the firmware flag at 0 -- if you set it to 1 you must switch the app toggle off,
  or the curve is applied twice. On 5 lamps not used in the fit (1727-4362 K) it cut the CCT RMS
  error against the HPCS from 208 K to 36 K. Shape only: lux/PAR stay uncalibrated.
- `src/metrics.h` computes x, y, CCT, Duv, Ra and R9 on the board (a port of the app's
  `analyzeSpectrum`). Rf/Rg and lux are not computed here. Results matched the app on 8 test
  spectra: x/y/Duv/Ra/R9 identical, CCT within 0.3 K (`tools/host_test.cpp` rebuilds that check on a PC).
- `tools/gen_tables.py` regenerates `src/metrics_tables.h` and `src/corr_table.h`.

## Licence

GPLv3 (see `LICENSE`), derived from Torch-Bearer-Tools / tobes-ui. Copyright fizzgig.
