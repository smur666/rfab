#!/usr/bin/env python3
"""
Extract Opus audio from WebM files and remux as mono Opus
at 34 kbps and/or 48 kbps. Keeps the original filename,
only changes the extension.
"""

import subprocess
import sys
from pathlib import Path

# ========== CONFIG ==========
INPUT_DIR = Path(".")          # folder with .webm files (or change to a path)
OUTPUT_BITRATES = [34, 48]     # kbps – keep both or remove one
# ============================

def run_ffmpeg(cmd: list[str]) -> bool:
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"  ✗ FFmpeg error:\n{e.stderr.decode(errors='ignore')}")
        return False

def process_webm(webm: Path):
    print(f"\nProcessing: {webm.name}")

    for bitrate in OUTPUT_BITRATES:
        out_file = webm.with_suffix(f".{bitrate}k.opus")

        cmd = [
            "ffmpeg",
            "-y",                          # overwrite
            "-i", str(webm),
            "-vn",                         # no video
            "-ac", "1",                    # force mono
            "-c:a", "libopus",
            "-b:a", f"{bitrate}k",
            "-vbr", "on",                  # variable bitrate (recommended for Opus)
            "-compression_level", "10",    # best quality/size
            "-application", "audio",       # good for music
            str(out_file)
        ]

        print(f"  → {out_file.name}  ({bitrate} kbps mono)")
        if run_ffmpeg(cmd):
            print(f"  ✓ Done")
        else:
            print(f"  ✗ Failed")

def main():
    webms = list(INPUT_DIR.glob("*.webm")) + list(INPUT_DIR.glob("*.WEBM"))

    if not webms:
        print("No .webm files found.")
        sys.exit(1)

    print(f"Found {len(webms)} WebM file(s)")
    for webm in webms:
        process_webm(webm)

    print("\nAll done.")

if __name__ == "__main__":
    main()