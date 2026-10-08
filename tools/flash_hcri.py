#!/usr/bin/env python3
"""
Find the ESP32 on USB, show what it is, download the right hCRI firmware and flash it.

    python flash_hcri.py            (Windows: py flash_hcri.py)

Needs Python 3.8+ and esptool (the script installs it with pip if it is missing).
No other setup: it downloads the firmware from this repo's GitHub releases.

What it can and can't detect: the chip (ESP32-S3), flash size and PSRAM come from the board
itself, but a plain T-Display S3 and a T-Display-S3 Pro look identical to esptool, so the
script asks which board you have (and remembers the answer for next time).
"""
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request

REPO = "fizzgig17/hcri-esp32-bridge"
STATE_FILE = os.path.join(os.path.expanduser("~"), ".hcri-flash.json")

# (menu text, release asset name, boards it applies to)
FIRMWARE = [
    ("Torch Bearer bridge - T-Display S3", "hcri-esp32-bridge-factory.bin"),
    ("Torch Bearer bridge - T-Display-S3 Pro", "hcri-esp32-bridge-pro-factory.bin"),
    ("Battery-life tester - T-Display-S3 Pro (V1.1 board)", "battery-test-pro-v1_1.bin"),
    ("Battery-life tester - T-Display-S3 Pro (older V1.0 board)", "battery-test-pro-v1_0.bin"),
]
# Which release tag(s) hold each asset, in the order they are tried.
TAGS = {
    "hcri-esp32-bridge-factory.bin": {"stable": "prod-latest", "dev": "dev-latest"},
    "hcri-esp32-bridge-pro-factory.bin": {"stable": "prod-latest", "dev": "dev-latest"},
    "battery-test-pro-v1_1.bin": {"stable": "battery-test-pro-latest", "dev": "battery-test-pro-latest"},
    "battery-test-pro-v1_0.bin": {"stable": "battery-test-pro-latest", "dev": "battery-test-pro-latest"},
}


def ensure_esptool():
    try:
        import esptool  # noqa: F401
    except ImportError:
        print("Installing esptool (one time)...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", "esptool"])


def load_state():
    try:
        with open(STATE_FILE) as f:
            return json.load(f)
    except Exception:
        return {}


def save_state(st):
    try:
        with open(STATE_FILE, "w") as f:
            json.dump(st, f)
    except Exception:
        pass


def ask(prompt, default=None):
    try:
        v = input(prompt).strip()
    except EOFError:
        v = ""
    return v or (default if default is not None else "")


def find_ports():
    """Serial ports that look like an ESP32 (Espressif USB, or common USB-serial chips), best guesses first."""
    from serial.tools import list_ports
    found = []
    for p in list_ports.comports():
        vid = p.vid or 0
        if vid == 0x303A:
            rank = 0   # Espressif's own USB (ESP32-S3 native USB)
        elif vid in (0x10C4, 0x1A86, 0x0403):
            rank = 1   # CP210x, CH340, FTDI
        else:
            continue
        found.append((rank, p))
    found.sort(key=lambda t: (t[0], t[1].device))
    return [p for _, p in found]


def run_esptool(args, capture=False):
    cmd = [sys.executable, "-m", "esptool"] + args
    if capture:
        r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        return r.returncode, r.stdout
    return subprocess.call(cmd), ""


def probe(port):
    """Ask the chip what it is. Returns a dict of what we could read, or None if it didn't answer."""
    rc, out = run_esptool(["--port", port, "--baud", "115200", "flash_id"], capture=True)
    if rc != 0:
        return None
    info = {"raw": out}
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("Chip is "):
            info["chip"] = line[len("Chip is "):]
        elif line.startswith("Features:"):
            info["features"] = line[len("Features:"):].strip()
        elif line.startswith("MAC:"):
            info["mac"] = line[4:].strip()
        elif "flash size" in line.lower() and ":" in line:
            info["flash"] = line.split(":", 1)[1].strip()
    return info


def http_json(url):
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json", "User-Agent": "hcri-flash"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)


def find_asset(tag, name):
    rel = http_json("https://api.github.com/repos/%s/releases/tags/%s" % (REPO, tag))
    sums = None
    asset = None
    for a in rel.get("assets", []):
        if a["name"] == name:
            asset = a
        if a["name"].startswith("SHA256SUMS"):
            sums = (sums or []) + [a]
    return asset, sums or []


def download(url, dest):
    req = urllib.request.Request(url, headers={"User-Agent": "hcri-flash"})
    with urllib.request.urlopen(req, timeout=120) as r, open(dest, "wb") as f:
        total = int(r.headers.get("Content-Length") or 0)
        got = 0
        while True:
            chunk = r.read(65536)
            if not chunk:
                break
            f.write(chunk)
            got += len(chunk)
            if total:
                print("\r  %d%%" % (got * 100 // total), end="", flush=True)
    print()


def fetch_firmware(asset_name, channel):
    tag = TAGS[asset_name][channel]
    print("Looking for %s in release '%s'..." % (asset_name, tag))
    asset, sums = find_asset(tag, asset_name)
    if not asset:
        return None
    dest = os.path.join(tempfile.gettempdir(), asset_name)
    download(asset["browser_download_url"], dest)
    with open(dest, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    # Verify against the published checksums when they list this file.
    for s in sums:
        sp = dest + "." + s["name"]
        try:
            download(s["browser_download_url"], sp)
            with open(sp) as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2 and os.path.basename(parts[-1].lstrip("*")) == asset_name:
                        if parts[0].lower() != digest:
                            print("Checksum mismatch - not flashing.")
                            return None
                        print("Checksum OK.")
                        return dest
        except Exception:
            pass
    print("(No checksum listed for this file; continuing.)")
    return dest


def pick(title, options, default_index=0):
    print()
    print(title)
    for i, o in enumerate(options, 1):
        print("  %d) %s%s" % (i, o, "   [default]" if i - 1 == default_index else ""))
    while True:
        v = ask("Choose 1-%d: " % len(options), str(default_index + 1))
        if v.isdigit() and 1 <= int(v) <= len(options):
            return int(v) - 1
        print("Please type a number from the list.")


def main():
    ensure_esptool()
    st = load_state()

    print("hCRI firmware flasher")
    print("=====================")
    ports = find_ports()
    if not ports:
        print("\nNo ESP32 found on USB.")
        print("Plug the board in with a DATA USB-C cable. If it is already plugged in and running the")
        print("bridge firmware, put it in download mode: hold BOOT, tap RESET, release BOOT. Then run this again.")
        return 1
    if len(ports) == 1:
        port = ports[0].device
        print("\nFound: %s  (%s)" % (port, ports[0].description))
    else:
        i = pick("More than one candidate port:", ["%s  (%s)" % (p.device, p.description) for p in ports])
        port = ports[i].device

    print("\nAsking the board what it is...")
    info = probe(port)
    if not info:
        print("The board didn't answer. Hold BOOT, tap RESET, release BOOT (download mode) and run this again.")
        return 1
    print("  Chip:     %s" % info.get("chip", "?"))
    print("  Features: %s" % info.get("features", "?"))
    print("  Flash:    %s" % info.get("flash", "?"))
    print("  MAC:      %s" % info.get("mac", "?"))
    if "ESP32-S3" not in info.get("chip", ""):
        print("\nThis is not an ESP32-S3, so none of the hCRI firmware fits it. Nothing was changed.")
        return 1
    print("\nNote: a T-Display S3 and a T-Display-S3 Pro look the same to the chip, so please pick the board.")

    default = FIRMWARE_NAMES.index(st["last"]) if st.get("last") in FIRMWARE_NAMES else 0
    idx = pick("What do you want to put on it?", [t for t, _ in FIRMWARE], default)
    text, asset_name = FIRMWARE[idx]
    ch = pick("Which build?", ["Stable (recommended)", "Latest development build"], 0)
    channel = "stable" if ch == 0 else "dev"

    try:
        path = fetch_firmware(asset_name, channel)
    except (urllib.error.URLError, OSError) as e:
        print("\nCouldn't download: %s" % e)
        return 1
    if not path and channel == "stable":
        print("The stable release doesn't have this file yet.")
        if ask("Try the development build instead? [Y/n] ", "y").lower().startswith("y"):
            try:
                path = fetch_firmware(asset_name, "dev")
            except (urllib.error.URLError, OSError) as e:
                print("\nCouldn't download: %s" % e)
                return 1
    if not path:
        print("That firmware isn't available. Nothing was changed.")
        return 1

    print("\nAbout to flash: %s" % text)
    print("Port: %s   File: %s" % (port, os.path.basename(path)))
    if not ask("Flash it now? [Y/n] ", "y").lower().startswith("y"):
        print("Cancelled. Nothing was changed.")
        return 0
    rc, _ = run_esptool(["--chip", "esp32s3", "--port", port, "--baud", "460800",
                         "write_flash", "0x0", path])
    if rc != 0:
        print("\nFlashing failed. Put the board in download mode (hold BOOT, tap RESET, release BOOT)")
        print("and run this again. If it still fails, try a different USB cable or port.")
        return 1
    st["last"] = asset_name
    save_state(st)
    print("\nDone. Tap RESET on the board (or unplug and replug it) to start the new firmware.")
    return 0


FIRMWARE_NAMES = [n for _, n in FIRMWARE]

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(1)
