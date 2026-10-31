#!/usr/bin/env python3
"""
hCRI Companion ESP32 Bridge -- find the ESP32 on USB, show what it is, download the right firmware and flash it.

    python hcri_companion_esp32_bridge.py            (Windows: py hcri_companion_esp32_bridge.py)
    python hcri_companion_esp32_bridge.py --prod     skip the question: production build only
    python hcri_companion_esp32_bridge.py --dev      skip the question: development build
    python hcri_companion_esp32_bridge.py --no-probe skip identifying the board (you pick the firmware)
    python hcri_companion_esp32_bridge.py --s3       the board is a T-Display S3: no identifying, no choosing
    python hcri_companion_esp32_bridge.py --s3pro    the board is a T-Display-S3 Pro: same (--pro also works)
    python hcri_companion_esp32_bridge.py --yes      answer every question with its default, so it just flashes
    python hcri_companion_esp32_bridge.py --munki    flash the ColorMunki identify tool (T-Display-S3 Pro only)
    python hcri_companion_esp32_bridge.py --force    flash even if the board already has exactly this build

Before flashing it reads which build is on the board (a 256-byte read, about a second) and skips the flash when it is
already the one downloaded. --force flashes anyway.

Hands-off examples:  py hcri_companion_esp32_bridge.py --s3 --prod --yes
                     py hcri_companion_esp32_bridge.py --s3pro --dev --yes
Without --prod or --dev, --yes uses the production build.

Needs Python 3.8+ and esptool (the script installs it with pip if it is missing; on systems that refuse that,
such as Arch or newer Debian/Ubuntu, it makes a private virtual environment in your home folder instead).
No other setup: it downloads the firmware from this repo's GitHub releases.

What it can and can't detect: the chip (ESP32-S3), flash size and PSRAM come from the board
itself, but a plain T-Display S3 and a T-Display-S3 Pro look identical to esptool, so the
script flashes a tiny probe program to find out (the Pro has a power chip on I2C that the
plain board lacks) and remembers each board by its MAC address.
"""
import hashlib
import json
import os
import struct
import zlib
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request

REPO = "fizzgig17/hcri-esp32-bridge"
STATE_FILE = os.path.join(os.path.expanduser("~"), ".hcri-companion-esp32-bridge.json")

# (menu text, release asset name, boards it applies to)
FIRMWARE = [
    ("Torch Bearer bridge - T-Display S3", "hcri-esp32-bridge-factory.bin"),
    ("Torch Bearer bridge - T-Display-S3 Pro", "hcri-esp32-bridge-pro-factory.bin"),
]
# Which release tag(s) hold each asset, in the order they are tried.
MUNKI = ("ColorMunki identify tool - T-Display-S3 Pro", "colormunki-probe-pro.bin")   # only with --munki
PROBE_ASSET = "board-probe.bin"
PROBE_TAG = "board-probe-latest"
BOARD_NAMES = {"pro": "LilyGo T-Display-S3 Pro", "s3": "LilyGo T-Display S3"}
# Which FIRMWARE entries (by asset name) make sense on each board.
FOR_BOARD = {
    "pro": ["hcri-esp32-bridge-pro-factory.bin"],
    "s3": ["hcri-esp32-bridge-factory.bin"],
}
TAGS = {
    "hcri-esp32-bridge-factory.bin": {"stable": "prod-latest", "dev": "dev-latest"},
    "hcri-esp32-bridge-pro-factory.bin": {"stable": "prod-latest", "dev": "dev-latest"},
    MUNKI[1]: {"stable": "colormunki-probe-latest", "dev": "colormunki-probe-latest"},
    PROBE_ASSET: {"stable": PROBE_TAG, "dev": PROBE_TAG},
}


# The Python that has esptool. Normally the one running this script; on systems that refuse system-wide pip installs
# (Arch, newer Debian/Ubuntu/Fedora: "externally-managed-environment") it is a private virtual environment instead.
ESPTOOL_PY = sys.executable
VENV_DIR = os.path.join(os.path.expanduser("~"), ".hcri-companion-esp32-bridge-venv")


def _has_esptool(py):
    return subprocess.call([py, "-c", "import esptool"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def _pip_install(py):
    r = subprocess.run([py, "-m", "pip", "install", "--quiet", "esptool"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    return r.returncode == 0, r.stdout


def _venv_python():
    return os.path.join(VENV_DIR, "Scripts", "python.exe") if os.name == "nt" else os.path.join(VENV_DIR, "bin", "python")


def ensure_esptool():
    global ESPTOOL_PY
    if _has_esptool(sys.executable):
        return
    vpy0 = _venv_python()
    if os.path.exists(vpy0) and _has_esptool(vpy0):   # made on an earlier run
        ESPTOOL_PY = vpy0
        return
    print("Installing esptool (one time)...")
    ok, out = _pip_install(sys.executable)
    if ok and _has_esptool(sys.executable):
        return
    # pip refused (PEP 668 "externally-managed-environment", no permission, or no pip). Use a private virtual
    # environment in the home folder so nothing system-wide is touched.
    vpy = _venv_python()
    if not (os.path.exists(vpy) and _has_esptool(vpy)):
        print("The system Python does not allow installing packages, so esptool goes into its own folder: " + VENV_DIR)
        if not os.path.exists(vpy):
            r = subprocess.run([sys.executable, "-m", "venv", VENV_DIR],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            if r.returncode != 0 or not os.path.exists(vpy):
                _esptool_help(out + "\n" + r.stdout)
        ok, out2 = _pip_install(vpy)
        if not ok or not _has_esptool(vpy):
            _esptool_help(out + "\n" + out2)
    ESPTOOL_PY = vpy


def _esptool_help(detail):
    print("\nCould not install esptool automatically.")
    d = detail.strip()
    if d:
        print("Details:\n" + "\n".join("  " + ln for ln in d.splitlines()[-8:]))
    print("\nInstall it yourself, then run this program again. Any one of these works:")
    print("  Arch / Manjaro:     sudo pacman -S esptool          (or: sudo pacman -S python-pipx && pipx install esptool)")
    print("  Debian / Ubuntu:    sudo apt install python3-venv   (then run this program again), or: sudo apt install esptool")
    print("  Fedora:             sudo dnf install esptool")
    print("  Any system:         pipx install esptool            (or: python3 -m venv ~/esptool-venv && ~/esptool-venv/bin/pip install esptool)")
    print("If the 'esptool' command works in a terminal but this program still fails, tell us which system you are on.")
    sys.exit(1)


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


AUTO_YES = "--yes" in sys.argv or "-y" in sys.argv
FORCE = "--force" in sys.argv
FORCED_KIND = "s3" if "--s3" in sys.argv else ("pro" if ("--s3pro" in sys.argv or "--pro" in sys.argv) else None)


def ask(prompt, default=None):
    if AUTO_YES:
        print("%s%s  (--yes)" % (prompt, default if default is not None else ""))
        return default if default is not None else ""
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
    cmd = [ESPTOOL_PY, "-m", "esptool"] + args
    if capture:
        r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        return r.returncode, r.stdout
    return subprocess.call(cmd), ""


def app_desc(data):
    """Parses an ESP-IDF app description (esp_app_desc_t), which sits 0x20 bytes into an app image: version (the
    firmware stamps its git commit there), build date/time and the SHA-256 of the app's ELF, which is unique per build.
    Returns None if the bytes are not one (e.g. an erased board)."""
    d = data[0x20:0x20 + 256]
    if len(d) < 176 or struct.unpack_from("<I", d, 0)[0] != 0xABCD5432:
        return None
    text = lambda a, b: d[a:b].split(b"\0")[0].decode("ascii", "replace")
    return {"version": text(16, 48), "project": text(48, 80), "time": text(80, 96), "date": text(96, 112),
            "elf_sha256": d[144:176].hex()}


def image_app(path):
    """(app partition offset, app description) of a full flash image written at 0x0, or (None, None)."""
    with open(path, "rb") as f:
        b = f.read()
    for i in range(0x8000, 0x8000 + 0xC00, 32):          # partition table: 32-byte entries, magic AA 50
        e = b[i:i + 32]
        if e[:2] != b"\xaa\x50":
            break
        if e[2] == 0:                                     # first app partition
            off = struct.unpack_from("<I", e, 4)[0]
            return off, app_desc(b[off:off + 0x200])
    return None, None


def read_board_flash(port, off, size):
    """`size` bytes of the board's flash at `off`, or None if it can't be read."""
    dest = os.path.join(tempfile.gettempdir(), "hcri-bridge-installed.bin")
    try:
        os.remove(dest)
    except OSError:
        pass
    rc, _ = run_esptool(["--chip", "esp32s3", "--port", port, "--baud", "460800",
                         "read_flash", hex(off), hex(size), dest], capture=True)
    if rc != 0 or not os.path.exists(dest):
        return None
    with open(dest, "rb") as f:
        return f.read()


def board_app(port, off):
    """App description of what is on the board now (None if it can't be read). `off` is the first app partition of the
    layout the new image uses. A board that can be updated over Bluetooth has two app slots and runs the one its OTA data
    names, so read the board's own partition table and pick the running slot; an older board has the one app at `off`."""
    table = read_board_flash(port, 0x8000, 0xC00)
    slots, otadata = [], None
    for i in range(0, len(table or b""), 32):
        e = table[i:i + 32]
        if e[:2] != b"\xaa\x50":
            break
        ptype, sub = e[2], e[3]
        poff = struct.unpack_from("<I", e, 4)[0]
        if ptype == 0 and 0x10 <= sub <= 0x1F:        # app, OTA slot
            slots.append((sub - 0x10, poff))
        elif ptype == 1 and sub == 0:                 # data, ota (which slot is current)
            otadata = poff
    if len(slots) >= 2 and otadata is not None:
        slots.sort()
        seq = 0
        sel = read_board_flash(port, otadata, 0x2000)
        for k in (0, 0x1000):
            if sel and len(sel) >= k + 32:
                n, crc = struct.unpack_from("<I", sel, k)[0], struct.unpack_from("<I", sel, k + 28)[0]
                if n != 0xFFFFFFFF and crc == (zlib.crc32(sel[k:k + 4]) & 0xFFFFFFFF) and n > seq:
                    seq = n
        slot = (seq - 1) % len(slots) if seq else 0   # no valid entry: the bootloader starts the first slot
        off = slots[slot][1]
    data = read_board_flash(port, off, 0x200)
    return app_desc(data) if data else None


def describe(a):
    return "%s (built %s %s)" % (a["version"], a["date"], a["time"]) if a else "unknown"


def probe(port):
    """Ask the chip what it is. Returns a dict of what we could read, or None if it didn't answer."""
    rc, out = run_esptool(["--port", port, "--baud", "115200", "flash_id"], capture=True)
    if rc != 0:
        return None
    info = {"raw": out}
    for line in out.splitlines():
        line = line.strip()
        # esptool 4 prints "Chip is ESP32-S3 (...)"; esptool 5 prints "Chip type:  ESP32-S3 (...)".
        if line.startswith("Chip is "):
            info["chip"] = line[len("Chip is "):]
        elif line.startswith("Chip type:"):
            info["chip"] = line[len("Chip type:"):].strip()
        elif line.startswith("Features:"):
            info["features"] = line[len("Features:"):].strip()
        elif line.startswith("MAC:"):
            info["mac"] = line[4:].strip()
        elif "flash size" in line.lower() and ":" in line:
            info["flash"] = line.split(":", 1)[1].strip()
    return info


def http_json(url):
    req = urllib.request.Request(url, headers={"Accept": "application/vnd.github+json", "User-Agent": "hcri-companion-esp32-bridge"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.load(r)


LAST_RELEASE = {}


def tag_commit(tag):
    """Commit the release tag points at (short hash), or '?' if it can't be looked up."""
    try:
        ref = http_json("https://api.github.com/repos/%s/git/ref/tags/%s" % (REPO, tag))
        sha = ref["object"]["sha"]
        if ref["object"].get("type") == "tag":   # annotated tag -> follow to the commit
            sha = http_json("https://api.github.com/repos/%s/git/tags/%s" % (REPO, sha))["object"]["sha"]
        try:
            when = http_json("https://api.github.com/repos/%s/commits/%s" % (REPO, sha))["commit"]["committer"]["date"]
            when = when[:19].replace("T", " ") + " UTC"
        except Exception:
            when = "?"
        return sha[:7] + " (" + when + ")"
    except Exception:
        return "?"


def find_asset(tag, name):
    rel = http_json("https://api.github.com/repos/%s/releases/tags/%s" % (REPO, tag))
    LAST_RELEASE.clear()
    LAST_RELEASE.update({"tag": tag, "commit": tag_commit(tag)})
    sums = None
    asset = None
    for a in rel.get("assets", []):
        if a["name"] == name:
            asset = a
        if a["name"].startswith("SHA256SUMS"):
            sums = (sums or []) + [a]
    return asset, sums or []


def download(url, dest):
    req = urllib.request.Request(url, headers={"User-Agent": "hcri-companion-esp32-bridge"})
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
    LAST_RELEASE["size"] = asset.get("size", 0)
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


def parse_probe_line(line):
    """'HCRI-PROBE board=pro pmu=1 ...' -> 'pro' / 'not-pro', or None if it isn't a probe line."""
    if "HCRI-PROBE" not in line:
        return None
    for tok in line.split():
        if tok.startswith("board="):
            return tok[len("board="):]
    return None


def read_probe_answer(timeout_s=30):
    """After the probe program starts, listen on the board's serial port(s) for its answer.
    The port can disappear and come back (possibly under a new COM number) while the board resets."""
    import time
    import serial
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for p in find_ports():
            try:
                ser = serial.Serial()
                ser.port = p.device
                ser.baudrate = 115200
                ser.timeout = 1
                ser.dtr = False   # opening the port must not reset the board
                ser.rts = False
                ser.open()
            except Exception:
                continue
            try:
                end = time.time() + 5
                while time.time() < end:
                    line = ser.readline().decode("utf-8", "replace")
                    answer = parse_probe_line(line)
                    if answer:
                        return answer
            except Exception:
                pass
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
        time.sleep(1)
    return None


def identify_board(port, channel_unused=None):
    """Flash the tiny probe program, read what it says. Returns 'pro', 's3' or None."""
    print("Flashing a tiny test program to find out which board this is (it doesn't touch the screen)...")
    try:
        path = fetch_firmware(PROBE_ASSET, "stable")
    except (urllib.error.URLError, OSError) as e:
        print("Couldn't download the test program: %s" % e)
        return None
    if not path:
        print("The test program isn't published yet.")
        return None
    rc, _ = run_esptool(["--chip", "esp32s3", "--port", port, "--baud", "460800", "write_flash", "0x0", path])
    if rc != 0:
        print("Couldn't flash the test program.")
        return None
    print("Listening for the board's answer...")
    ans = read_probe_answer()
    if ans == "pro":
        return "pro"
    if ans == "not-pro":
        return "s3"
    print("The board didn't answer.")
    return None


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

    print("hCRI Companion ESP32 Bridge")
    print("===========================")
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
    mac = info.get("mac", "")
    boards = st.setdefault("boards", {})
    kind = boards.get(mac)
    if FORCED_KIND:
        kind = FORCED_KIND
        print("\nBoard: %s (from the command line)." % BOARD_NAMES[kind])
    elif kind in BOARD_NAMES:
        print("\nThis board was identified before: %s." % BOARD_NAMES[kind])
    elif "--no-probe" not in sys.argv:
        print("\nA T-Display S3 and a T-Display-S3 Pro look the same to the chip, so the program can")
        print("identify the board by running a tiny test program on it first.")
        if ask("Identify the board now? [Y/n] ", "y").lower().startswith("y"):
            kind = identify_board(port)
            if kind:
                print("\nResult: %s." % BOARD_NAMES[kind])
                if ask("Is that right? [Y/n] ", "y").lower().startswith("y"):
                    boards[mac] = kind
                    save_state(st)
                else:
                    kind = None
            if not kind and AUTO_YES:
                print("Couldn't identify the board, and --yes means nobody is here to choose. Run again with --s3 or --s3pro.")
                return 1
            if not kind:
                print("OK, you'll choose the firmware yourself.")
            # the probe left the port in use / board running: find it again for flashing
            ports = find_ports()
            if ports and all(p.device != port for p in ports):
                port = ports[0].device
                print("(The board is now on %s.)" % port)

    choices = [fw for fw in FIRMWARE if kind is None or fw[1] in FOR_BOARD[kind]]
    if "--munki" in sys.argv:
        if kind == "s3":
            print("\nThe ColorMunki identify tool is for the T-Display-S3 Pro only. Nothing was changed.")
            return 1
        choices = [MUNKI]
    default = 0
    if st.get("last") in [n for _, n in choices]:
        default = [n for _, n in choices].index(st["last"])
    if len(choices) == 1:
        text, asset_name = choices[0]
        print("\nFirmware for this board: %s" % text)
    else:
        idx = pick("What do you want to put on it?", [t for t, _ in choices], default)
        text, asset_name = choices[idx]
    if "--dev" in sys.argv:
        channel = "dev"
    elif "--prod" in sys.argv:
        channel = "stable"
    else:
        ch = pick("Which build?", ["Production (prod-latest) - recommended", "Development (dev-latest) - newest, untested"], 0)
        channel = "stable" if ch == 0 else "dev"
    print("Build: %s" % ("production" if channel == "stable" else "development"))

    try:
        path = fetch_firmware(asset_name, channel)
    except (urllib.error.URLError, OSError) as e:
        print("\nCouldn't download: %s" % e)
        return 1
    if not path and channel == "stable":
        print("The production release doesn't have this file yet.")
        if "--prod" not in sys.argv and ask("Try the development build instead? [Y/n] ", "y").lower().startswith("y"):
            try:
                path = fetch_firmware(asset_name, "dev")
            except (urllib.error.URLError, OSError) as e:
                print("\nCouldn't download: %s" % e)
                return 1
    if not path:
        print("That firmware isn't available. Nothing was changed.")
        return 1

    # Is this exact build already on the board? Compare the ELF fingerprint of the downloaded image with the board's.
    off, new = image_app(path)
    if new and off is not None:
        print("\nChecking which build is on the board...")
        cur = board_app(port, off)
        print("  On the board:  %s" % describe(cur))
        print("  Downloaded:    %s" % describe(new))
        if cur and cur["elf_sha256"] == new["elf_sha256"]:
            if FORCE:
                print("The board already has this build; --force: flashing it again anyway.")
            else:
                print("The board already has exactly this build.")
                if not ask("Flash it again anyway? [y/N] ", "n").lower().startswith("y"):
                    print("Nothing was changed. (Use --force to flash it anyway.)")
                    return 0

    print("\nAbout to flash")
    print("  What:    %s" % text)
    print("  Port:    %s" % port)
    print("  Device:  %s  (MAC %s)" % (info.get("chip", "?"), info.get("mac", "?")))
    print("  File:    %s  (%d bytes)" % (os.path.basename(path), os.path.getsize(path)))
    print("  Build:   %s  [release tag '%s']" % ("production" if channel == "stable" or LAST_RELEASE.get("tag", "").startswith("prod") else "development", LAST_RELEASE.get("tag", "?")))
    print("  Commit:  %s" % LAST_RELEASE.get("commit", "?"))
    if not ask("Flash it now? [Y/n] ", "y").lower().startswith("y"):
        print("Cancelled. Nothing was changed.")
        return 0
    rc, _ = run_esptool(["--chip", "esp32s3", "--port", port, "--baud", "460800",
                         "write_flash", "0x0", path])
    if rc != 0:
        print("\nFlashing failed. Put the board in download mode (hold BOOT, tap RESET, release BOOT)")
        print("and run this again. If it still fails, try a different USB cable or port.")
        return 1
    if asset_name != MUNKI[1]:
        st["last"] = asset_name   # the identify tool never becomes the default
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
