#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path

root = Path(os.environ.get("PROJECT_DIR", os.getcwd())).resolve()
out_dir = root / "include"
out_file = out_dir / "firmware_version.h"

# Default variant based on environment if not set explicitly.
variant = os.environ.get("PIOENV", "esp32s3")
if variant == "esp32s3":
    firmware_variant = "tank"
elif variant == "esp32doit":
    firmware_variant = "mppt"
else:
    firmware_variant = variant

try:
    git_sha = subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()
except Exception:
    git_sha = "unknown"

# Prefer explicit tag + build number when available.
version = os.environ.get("FW_VERSION", f"{firmware_variant}-{git_sha}")

header = f'''#pragma once

#define FIRMWARE_VERSION "{version}"
#define FIRMWARE_VARIANT "{firmware_variant}"
#define FIRMWARE_GIT_SHA "{git_sha}"
'''

out_dir.mkdir(exist_ok=True)
out_file.write_text(header, encoding="utf-8")
