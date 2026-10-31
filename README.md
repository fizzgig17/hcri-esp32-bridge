# hCRI ESP32 bridge (T-Display S3): Torch Bearer and ColorMunki

Step-by-step setup guide with buying links: <https://www.hcri.io/companion/bridge/>

The same firmware drives a **Torch Bearer** (CH340 serial), an **X-Rite ColorMunki Photo / Design** (USB 0971:2007) or a
**TKLamp TK2303D integrating-sphere tester** (CH340 serial at 9600 baud; live lumen readings) and auto-detects which is
plugged in. The TKLamp is recognised by the frames it streams on its own (about 4.5 a second); anything else on the CH340 is
treated as a Torch Bearer. The ColorMunki is measured in ambient mode only, after a dark calibration (dial in the
calibration position); the app can require recalibration after a set time (Settings, default 1 h).

Press the right button (GPIO14) or BOOT: the board asks the Torch Bearer for one
auto-exposure scan over USB (CH340 serial, 115200 8N1) and shows peak wavelength,
peak value, sum, exposure and a spectrum plot. GPLv3 (derived from Torch-Bearer-Tools / tobes-ui).

## Hardware you need

For use with the hCRI Companion app (Android and iPhone):

- **Torch Bearer spectrometer** or **ColorMunki Photo / Design** (the thing being bridged). ColorMunki Display/Smile are not supported.
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
- With a ColorMunki on the standard T-Display S3, power it from a power bank (it draws more current than the Torch Bearer). The Pro can run it from its battery.
- **Phone** with the hCRI Companion app: Android 7.0+ or iPhone with iOS 15.1+, with Bluetooth LE.
- Optional: a diffuser in front of the lamp; about 25 cm distance works well.

## Installing it (short version)

1. Easiest: download `hcri_companion_esp32_bridge.py` from the [`prod-latest`](../../releases/tag/prod-latest) release and run it
   (see "Easiest" below) -- it finds the port, identifies your board and flashes the right file. Otherwise:
   download `hcri-esp32-bridge-factory.bin` (T-Display S3) from the same release.
2. Put the board in download mode and flash that file at **0x0** (steps below -- Chrome or Edge is enough).
3. Plug the spectrometer into the board with the OTG cable, power it from the power bank, and open
   the hCRI Companion app: the bridge appears as **hCRI Bridge** (older firmware: Torch Bearer). Once connected, the app shows the instrument that is plugged in, Torch Bearer or ColorMunki.

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
  `py hcri_companion_esp32_bridge.py` (Windows) or `python3 hcri_companion_esp32_bridge.py`. It installs esptool if needed (on Arch and other systems that refuse
  system-wide pip installs it makes its own private folder, `~/.hcri-companion-esp32-bridge-venv`, instead).
- Options: `--prod` or `--dev` pick the build without asking, `--no-probe` skips identifying the board.
  `--s3` or `--s3pro` say which board it is (no test program, no menu), and `--yes` answers every question with its
  default, so `py hcri_companion_esp32_bridge.py --s3 --prod --yes` flashes with no prompts. The board still has to be in
  download mode if the bridge firmware is running on it.
- **Already up to date?** Before flashing it reads which build is on the board (commit and build time) and compares it
  with the download. If they are the same build it does nothing (it asks first, unless `--yes`). `--force` flashes anyway.

### Updating from the phone (no cable)
Once a board has the firmware with the two-slot layout (`partitions_ota.csv`), the hCRI Companion app can update it over
Bluetooth: open the bridge's page in the app, tap **Update bridge firmware**, and keep the phone close until the board
restarts (about a minute). The app downloads the right file for the board (`firmware.bin` for the S3,
`firmware-pro.bin` for the Pro), checks its SHA-256 and sends it to the spare slot. If the new firmware resets before it has run
for 10 seconds, the bootloader goes back to the old one by itself.

**One-time step:** boards flashed before this need the full factory image flashed once over USB (the layout changed), exactly as
under *Getting the firmware* above. After that, updates are wireless. (Saved settings, i.e. the screen flip, are reset by that
one USB flash.)

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
   USB-A), or tap **RST**. The screen shows HCRI BRIDGE / NO USB until an instrument is plugged in, then TORCH BEARER or COLORMUNKI.

## Building it yourself (VS Code + PlatformIO extension)
1. File > Open Folder... > this folder (a path **without spaces**). First build downloads the
   toolchain and the espressif/usb_host_ch34x_vcp + usb_host_vcp components (needs internet, takes a while).
2. Plug the T-Display S3 into the PC with USB-C (download mode, see above). Click the Upload arrow
   (bottom bar), or run `pio run -e tdisplay-s3 -t upload`.
3. Changed `sdkconfig.defaults`? Delete `sdkconfig.tdisplay-s3` and `.pio/build` before building,
   or the old settings stick.
4. Unplug. Power from a power bank into the OTG cable's female USB-C, spectrometer in the female USB-A.

## Bluetooth (BLE) trigger
Advertises as "hCRI Bridge" (firmware before October 14, 2026 advertised "Torch Bearer"; the app accepts both). Service 7a1c0001-5b2e-4f0a-9c3d-2e8f6b4a1d00:
- 7a1c0002-... command (write): 01 = scan now, 02 = resend last result, 06 = live (Torch Bearer only: keep scanning and send every locked result until 05), 05 = cancel the reading in progress or stop live,
  07 `<kind>` = developer simulation (see below)
- 7a1c0003-... result (notify/read): summary packet, then the full spectrum in chunks
- 7a1c0004-... status (notify/read): [state u8][try u8][exposure_ms f32]; state 0 ready, 1 scanning, 2 done, 3 error, 4 done-not-locked

Result packets (little-endian):
- 01 | status u8 | peak_nm u16 | exposure_ms f32 | peak_val f32 | sum f32 | npts u16 | start_nm u16
- 02 | npts u16 | start_nm u16 | step_nm u8 | total_bytes u16   (header; float32 spectrum values follow)
- 03 | seq u8 | bytes...   (data chunks, seq from 0)
- 04 | chunks u8           (end marker)
- 05 | id u8 | seq u8 | last u8 | bytes...   (ColorMunki diagnostics after the end marker: id 1 = the reading's raw
  sensor data, id 2 = the calibration block once per connection. Apps that don't know 05 ignore it.)

- 7a1c0005-... update (write, write-no-response, notify, read): firmware update over Bluetooth. Write `01 | size u32 | sha256[32] | board u8`
  (0 = S3, 1 = Pro), then `02 | offset u32 | bytes` in chunks, then `03`; `04` aborts. Notifications: `81 | slot`, `82 | received u32` (every ~4 KB),
  `83` done (restarts), `E1 | expected u32` (send again from there), `E0 | code`. Details in `src/main.cpp`, "Firmware update".
- 7a1c0006-... info (read): `[1][flags: bit0 = can update over Bluetooth][board: 0 S3, 1 Pro][running slot][0]` then the git commit as text.

Test with nRF Connect: connect, subscribe to result + status, write 01 to the command characteristic.

Result packet `08 01` (Torch Bearer): Live was started with the bridge's own button (not by the phone), so the app goes into Live as well; Live ends with the usual ready status. A simulated ColorMunki starts calibrated with the dial on Ambient. During Live the bottom button's label reads STOP.
Command `09 <1 top|2 bottom>` (simulation only): the phone presses a bridge button, handled exactly like the physical one. Command `0A <0|1>` (simulated TKLamp): the app has a run going, so the screen reads RECORDING RUN and the top button is STOP (the level button keeps working). The TKLamp screen draws an arrow and a label at each button like the other testers: START (STOP during a run, with RECORDING RUN in red) on top, NEXT below.

### Simulating an instrument (developers)
With nothing plugged in, write `07 <kind>` to the command characteristic (0 off, 1 Torch Bearer, 2 ColorMunki, 3 TKLamp tester).
The board then advertises and behaves like that instrument, with made-up readings, so its screens can be worked on without the
hardware. It is held in RAM only (a restart or a phone disconnect ends it), a plugged-in instrument is ignored while it is on, and the screen shows `SIM` after
the firmware commit. Physical button presses do what they would on the real instrument and are also sent to the app as result packet
`07 | button u8 (1 top, 2 bottom) | pressed u8 (1 down, 0 up) | ms u32 | held_ms u16`. Buttons in each simulation: Torch Bearer top = READ (one
simulated reading, "SIMULATED READ"), bottom = LIVE (only announced: "SIMULATING LIVE READ, PRESS BUTTON AGAIN TO STOP"; nothing is measured); ColorMunki top = turn the dial (calibration, ambient, surface, projector), bottom = calibrate / measure; TKLamp bottom = next
state (five flashlight-like levels, WAITING FOR DATA, NO DATA), top = start / stop of the run in the app (the screen then reads RECORDING RUN with STOP on top). The hCRI Companion dev builds have a switch for it in Settings.


## Spectral correction and on-board colorimetry

- `src/corr_table.h` holds a per-nm multiplier (340-1000 nm) fitted against an HPCS-330P on white
  lamps. **The board applies it** (`TB_APPLY_CORRECTION` is 1 in `src/metrics.h`) and says so in status byte 7
  bit 6, so the hCRI Companion app does not apply its own copy (it still does for older firmware). On 5 lamps not
  used in the fit (1727-4362 K) it cut the CCT RMS error against the HPCS from 208 K to 36 K. Shape only: lux/PAR
  stay uncalibrated.
- `src/metrics.h` computes x, y, CCT, Duv, Ra and R9 on the board (a port of the app's
  `analyzeSpectrum`). Rf/Rg and lux are not computed here. Results matched the app on 8 test
  spectra: x/y/Duv/Ra/R9 identical, CCT within 0.3 K (`tools/host_test.cpp` rebuilds that check on a PC).
- `tools/gen_tables.py` regenerates `src/metrics_tables.h` and `src/corr_table.h`.

## ColorMunki measurements

- **Integration time and gain** follow ArgyllCMS (`munki_optimise_sensor`): about 90 % of the sensor's optimum
  level, at most 2 s per integration unless the light is very dim, then up to 4.5 s. Only when even 4.5 s is not
  enough does it switch to the sensor's high-gain mode (`MUNKI_HIGH_GAIN` in `src/munki_argyll.cpp`; set it to 0
  to never use it, as stock Argyll does). The dark calibration takes the extra high-gain references for this.
- **Averaging:** normally 1 s worth of integrations per reading (Argyll); in dim light (2 s or more per
  integration) 8 s worth instead, so a dim reading is several integrations, not one.
- **Integration times** are whole instrument clock ticks, and that exact time is used for the dark reference and
  the per-second scaling. Saturation is checked on every single integration, not only their average.
- `tools/munki_sim_test.cpp` runs the driver against a simulated instrument on a PC (bright, dim, very dim,
  saturated, and a USB read that never completes):
  `g++ -std=gnu++17 -I tools/sim_stubs -I src -o munki_sim tools/munki_sim_test.cpp && ./munki_sim`
- **Correction against an HPCS:** `src/munki_corr.h` holds two curves (like the Torch Bearer's, shape only), one for
  each spectrum: `MK_CORR_HR` for high resolution and `MK_CORR` for the standard one (low light). Each is applied on
  the board once fitted (`MK_CORR_HR_FITTED`, `MK_CORR_FITTED`). To fit it, measure several lamps with both the
  ColorMunki and the HPCS in the app, share each reading as CSV from History, and run
  `tools/fit_munki_corr.py --mode hires --pair munki.csv hpcs.csv --pair ... --write` (`--mode standard` for the
  low-light curve). A CSV that says the correction was applied is divided by the current curve first. It prints the CCT/Duv error before and
  after, and a leave-one-out check with three or more lamps. The HPCS is blurred to the ColorMunki's ~10 nm
  bandwidth before fitting (`--bandwidth`), so the curve corrects its colour response and not the resolution it
  cannot recover. First fit (Oct 15 2026, 6 flashlights 2680-6880 K through a parchment diffuser): leave-one-out RMS
  error against the HPCS 40 K -> 20 K CCT, 0.0014 -> 0.0010 Duv.
- **High resolution (3.33 nm):** ArgyllCMS's high-resolution emission mode (`munki_create_hr`: Gaussian filters
  box-integrated over each CCD cell, placed using where the factory 10 nm filters cross over; calibration factors
  and stray light resampled to the new bands), built from the calibration block when the instrument is opened.
  Readings are sent at high resolution, scaled to the standard spectrum's luminance (high resolution comes out a
  constant ~25 % brighter; lux stays on X-Rite's calibration). A reading flagged low light uses the standard 10 nm
  spectrum instead, because the narrower bands are noisier there. Status bit 5 tells the app. Against an HPCS on
  6 flashlights: spectral-shape error 4.9 % -> 2.5 % RMS, blue peak width 25 -> 20 nm (HPCS 19 nm), effective
  bandwidth ~11 nm -> ~4 nm; with each mode's correction, on lamps left out of its fit: CCT 20 K -> 17 K, Duv
  0.0010 -> 0.0008 RMS. `tools/munki_hires_check.cpp` runs it on a PC from a calibration block and readings'
  sensor values (`tools/munki_diag.py` output).
- **Raw data:** after each reading the bridge also sends the 128 linearised sensor values (and once per connection
  the calibration block). The app writes them to its log; `tools/munki_diag.py` extracts them.

## Licence

GPLv3 (see `LICENSE`), derived from Torch-Bearer-Tools / tobes-ui. Copyright fizzgig.

`src/munki_argyll.cpp` and `src/munki_argyll.h` are ported from ArgyllCMS and remain under the **AGPL-3.0**
(<https://www.argyllcms.com>); the combined firmware is distributed under terms compatible with both licences.

## Buttons and sleep

- Torch Bearer: the top button (as seen on the screen) takes one READ (screen: PERFORMING READ), the bottom button starts LIVE (PERFORMING LIVE READ, PRESS BUTTON AGAIN TO STOP); on the S3 Pro an arrow and the word READ (top) and LIVE (bottom) mark the buttons on whichever side they are, and the text steps clear of them. The roles follow the screen when it is flipped. Any press stops Live. A press acts when it is let go.
- Hold either button for 2 seconds to put the board into deep sleep (screen off; on the Pro the instrument power boost is cut). Press a wake button to start it again: both side buttons on the T-Display-S3 Pro, the GPIO 14 button on the T-Display S3 (BOOT is not used to wake it). It reboots, so the phone and instrument reconnect.

_The firmware can be updated from the phone; the update path has been tested end to end on the T-Display S3 Pro._
