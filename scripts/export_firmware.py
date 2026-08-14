#!/usr/bin/env python3
import os
import re
import shutil
from pathlib import Path

root = Path(os.environ.get("PROJECT_DIR", os.getcwd())).resolve()

pio_build = root / ".pio" / "build"
env_name = os.environ.get("PIOENV", "esp32s3")
source = pio_build / env_name / "firmware.bin"

header_file = root / "include" / "firmware_version.h"
version = "local"
if header_file.exists():
    text = header_file.read_text(encoding="utf-8")
    match = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', text)
    if match:
        version = match.group(1)

variant = "tank" if env_name == "esp32s3" else "mppt"

target_dir = root / "dist" / "firmware"
target_dir.mkdir(parents=True, exist_ok=True)

if source.exists():
    destination = target_dir / f"{variant}-firmware-{version}.bin"
    shutil.copy2(source, destination)
    print(f"Copied firmware to {destination}")
else:
    print(f"Firmware not found at {source}; build may not have completed")
