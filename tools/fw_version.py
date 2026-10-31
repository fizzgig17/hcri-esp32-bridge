#!/usr/bin/env python3
"""Prints the compiler flag that stamps the firmware with the git commit it was built from.

Used by platformio.ini (build_flags = !python tools/fw_version.py). The commit id is written without quotes, behind a "g" (the firmware
turns it into text and drops the g), and falls back to "dev" when git isn't available, so a build never fails because of this.
"""
import re
import subprocess

try:
    rev = subprocess.check_output(["git", "rev-parse", "--short=7", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
except Exception:
    rev = ""
if not re.fullmatch(r"[0-9a-f]{7}", rev):
    rev = "dev"
# A commit id made only of digits (0465550) would be read as a number and lose its leading zero, so a letter goes in front
# and the firmware drops it again.
print("-DFW_GIT=g" + rev)
