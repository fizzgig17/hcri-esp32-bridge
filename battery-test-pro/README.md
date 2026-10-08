# Battery-life tester — LilyGo T-Display-S3 **Pro**

A tiny standalone firmware (not the Torch Bearer bridge) that times how long the
board runs on its battery and keeps the result across the battery dying.

## Using it
1. Fully charge the battery (USB plugged in; the screen shows the charge state).
2. **Unplug USB.** The timer only counts time on battery, and starts at that moment.
3. Leave it until it shuts off.
4. Plug USB back in: the screen lists the last 3 runs (time on battery, start → end voltage, backlight step).

The run is saved every 30 s, so the result is at most 30 s short.

Buttons: **BTN2 (GPIO12)** cycles the backlight step 1–16 (the step is recorded with the run);
**BTN3 (GPIO16)** turns the screen off/on (a screen-off run shows how long the board lasts idle).
**RESET** starts a new run (the unfinished one is kept in the history).

## Notes
- Battery voltage is not valid while USB is plugged in (the power chip reports the input instead), so the screen says so.
- The percentage is a rough single-cell LiPo estimate from the voltage; trust the voltage and the run time.
- Charge current is set to a gentle 320 mA (LilyGo's example value). The charge target voltage is left at the chip's default.
- The run goes until the battery's protection circuit (or the chip) cuts power — use a protected cell.
- USB serial is disabled in this build (it can stall on battery); logging is on the screen only.

## Flashing
Download `battery-test-pro-v1_1.bin` from the `battery-test-pro-latest` release and write it at address **0x0**
(hold BOOT, tap RESET, release BOOT to enter download mode). Use `battery-test-pro-v1_0.bin` only if your board is the older V1.0.
