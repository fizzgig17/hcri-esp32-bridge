#!/usr/bin/env python3
"""Prints the compiler flag that stamps the firmware with the git commit it was built from.

Used by platformio.ini (build_flags = !python tools/fw_version.py). The commit id is written without quotes (the firmware
turns it into text), and falls back to "dev" when git isn't available, so a build never fails because of this.
"""
import re
import subprocess

try:
    rev = subprocess.check_output(["git", "rev-parse", "--short=7", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
except Exception:
    rev = ""
if not re.fullmatch(r"[0-9a-f]{7}", rev):
    rev = "dev"
print("-DFW_GIT=" + rev)
