#!/usr/bin/env python3
"""
webm_to_rfav.py
Convert a transparent VP8 WebM into RFAV-3 (mask-first, 1-bit alpha).

The Tab5 glass is natively 720x1280 (portrait) and the P4 draw buffer is
720-px rows x 1280 rows, so landscape 1280x720 sources are baked-rotated
90 degrees AT ENCODE TIME (default --rotate cw): the device then writes
HW-JPEG output / mask / background straight into the draw buffer with
cpu-rotation of its own.

Usage:
    python webm_to_rfav.py input.webm output.rfav [--fps 12] [--q 8]
                           [--rotate cw|ccw|none]
"""

import argparse
import os
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

# ---------- Configuration ----------
# This machine has no ffmpeg on PATH — prefer the explicit install, fall
# back to PATH if the file is missing.
def _pick(*candidates: str) -> str:
    for c in candidates:
        if os.path.isfile(c):
            return c
    return candidates[-1]

FFMPEG  = _pick(r"C:\ffmpeg\ffmpeg.exe", "ffmpeg.exe")
FFPROBE = _pick(r"C:\ffmpeg\ffprobe.exe", "ffprobe.exe")

# 90-degree pre-rotation baked into the encode (see module docstring).
#   cw  = transpose=1 (90 deg clockwise:      source top row -> right)
#   ccw = transpose=2 (90 deg counter-clockwise: source top row -> left)
ROTATE = {
    "none": "",
    "cw":   "transpose=1",
    "ccw":  "transpose=2",
}

def run(cmd, check=True):
    print(">", " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if check and r.returncode != 0:
        print(r.stderr)
        raise RuntimeError(f"Command failed: {' '.join(cmd)}")
    return r

def probe_size(path: Path):
    """ffprobe pixel dimensions of the first extracted frame."""
    r = run([
        FFPROBE, "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "csv=p=0",
        str(path),
    ])
    w, h = (int(v) for v in r.stdout.strip().split(",")[:2])
    return w, h

def extract_frames(webm: Path, work: Path, fps: int, q: int, rotate: str):
    """Produce color JPEGs + 1-bit raw masks (rotation baked in)."""
    frames_dir = work / "frames"
    jpeg_dir   = work / "jpeg"
    mask_dir   = work / "mask"
    frames_dir.mkdir()
    jpeg_dir.mkdir()
    mask_dir.mkdir()

    # 1. Extract RGBA PNG sequence at target fps. The 90-degree rotation is
    #    baked in HERE so the color and mask streams always agree
    #    pixel-for-pixel (both inherit the same rotated frames).
    vf = f"fps={fps}"
    if ROTATE[rotate]:
        vf += "," + ROTATE[rotate]
    run([
        FFMPEG, "-y", "-c:v", "libvpx",
        "-i", str(webm),
        "-vf", vf,
        "-pix_fmt", "rgba",
        "-start_number", "0",
        str(frames_dir / "%04d.png")
    ])

    pngs = sorted(frames_dir.glob("*.png"))
    if not pngs:
        raise RuntimeError("No frames extracted from WebM")

    width, height = probe_size(pngs[0])
    if width % 8:
        raise RuntimeError(f"width {width} not a multiple of 8 (mask stride = width/8)")
    if width % 2 or height % 2:
        raise RuntimeError(f"odd dimension {width}x{height} breaks yuvj420p mjpeg")
    mask_bytes = (width * height) // 8
    print(f"frames: {width}x{height} ({rotate}), mask {mask_bytes} B/frame")

    # 2. Color JPEGs (rgb24 → mjpeg q)
    run([
        FFMPEG, "-y",
        "-framerate", str(fps),
        "-i", str(frames_dir / "%04d.png"),
        "-vf", "format=rgb24",
        "-c:v", "mjpeg", "-q:v", str(q),
        "-pix_fmt", "yuvj420p",          # force 4:2:0 baseline-ish
        "-start_number", "0",
        str(jpeg_dir / "%04d.jpg")
    ])

    # 3. Safer mask extraction (one frame at a time is too slow,
    #    so we use a pipe / intermediate approach)

    # First create temporary gray PNGs
    temp_alpha = work / "temp_alpha"
    temp_alpha.mkdir()

    run([
        FFMPEG, "-y",
        "-framerate", str(fps),
        "-i", str(frames_dir / "%04d.png"),
        "-vf", "alphaextract,format=gray,geq=lum='if(gt(lum(X,Y),128),255,0)'",
        "-start_number", "0",
        str(temp_alpha / "%04d.png")
    ])

    # Then convert each gray PNG → 1-bit raw
    for png in sorted(temp_alpha.glob("*.png")):
        out_bin = mask_dir / (png.stem + ".bin")
        run([
            FFMPEG, "-y",
            "-i", str(png),
            "-vf", "format=monob",
            "-f", "rawvideo",
            "-pix_fmt", "monob",
            str(out_bin)
        ], check=True)
        
    jpegs = sorted(jpeg_dir.glob("*.jpg"))
    masks = sorted(mask_dir.glob("*.bin"))

    if len(jpegs) != len(masks):
        raise RuntimeError(f"JPEG/mask count mismatch: {len(jpegs)} vs {len(masks)}")

    # sanity-check mask size (rotated dims: 720x1280 -> 115200 B)
    for m in masks:
        if m.stat().st_size != mask_bytes:
            raise RuntimeError(f"Mask size wrong: {m} is {m.stat().st_size}, expected {mask_bytes}")

    return list(zip(jpegs, masks)), width, height

def pack_rfav(pairs, out_path: Path, fps: int, width: int, height: int,
              loop: bool = True):
    """Write RFAV-3 (mask-first, 1-bit)."""
    n = len(pairs)
    mask_bytes = (width * height) // 8
    header_size = 64
    table_size  = n * 16
    data_offset = header_size + table_size

    # ---- First pass: collect sizes and build table ----
    table = []          # list of (offset, jpeg_len)
    offset = data_offset
    frame_data = bytearray()

    for jpg_path, mask_path in pairs:
        mask = mask_path.read_bytes()
        jpeg = jpg_path.read_bytes()

        if len(mask) != mask_bytes:
            raise RuntimeError("Mask size mismatch during pack")

        # Mask first, then JPEG
        frame_data += mask
        frame_data += jpeg

        table.append((offset, len(jpeg)))
        offset += mask_bytes + len(jpeg)

    frame_data_size = len(frame_data)

    # ---- Header (64 bytes) ----
    flags = 0
    if loop:
        flags |= 1 << 0          # loop
    flags |= 1 << 1              # has_mask
    # bit2 = 0 → 1-bit mask

    header = bytearray(64)
    header[0:4]  = b"RFAV"
    struct.pack_into("<H", header, 4, 3)            # version 3
    struct.pack_into("<H", header, 6, width)
    struct.pack_into("<H", header, 8, height)
    header[10] = fps & 0xFF
    header[11] = flags
    struct.pack_into("<H", header, 12, n)           # frame_count
    struct.pack_into("<H", header, 14, 0)           # reserved
    struct.pack_into("<I", header, 16, header_size) # frame_table_offset
    struct.pack_into("<I", header, 20, data_offset) # frame_data_offset
    struct.pack_into("<I", header, 24, frame_data_size)
    # optional header CRC (first 28 bytes)
    crc = zlib.crc32(header[:28]) & 0xFFFFFFFF
    struct.pack_into("<I", header, 28, crc)
    # rest already zero

    # ---- Frame table (N × 16) ----
    table_blob = bytearray()
    for off, jpeg_len in table:
        # offset, jpeg_len, reserved, frame_crc (0 for now)
        table_blob += struct.pack("<IIII", off, jpeg_len, 0, 0)

    # ---- Write file ----
    with open(out_path, "wb") as f:
        f.write(header)
        f.write(table_blob)
        f.write(frame_data)

    print(f"Wrote {out_path}  ({n} frames, {out_path.stat().st_size/1024/1024:.2f} MB)")

def main():
    ap = argparse.ArgumentParser(description="WebM (VP8+alpha) → RFAV-3")
    ap.add_argument("input", type=Path, help="input .webm")
    ap.add_argument("output", type=Path, help="output .rfav")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--q", type=int, default=8, help="JPEG quality (ffmpeg -q:v)")
    ap.add_argument("--rotate", choices=["none", "cw", "ccw"], default="cw",
                    help="bake 90-degree rotation in (default cw; ccw = the other way; none = as-is)")
    ap.add_argument("--no-loop", action="store_true")
    args = ap.parse_args()

    if not args.input.exists():
        sys.exit(f"Input not found: {args.input}")

    with tempfile.TemporaryDirectory(prefix="rfav_") as tmp:
        work = Path(tmp)
        print("Extracting frames…")
        pairs, width, height = extract_frames(
            args.input, work, args.fps, args.q, args.rotate)
        print(f"Packing {len(pairs)} frames @ {width}x{height}…")
        pack_rfav(pairs, args.output, args.fps, width, height,
                  loop=not args.no_loop)

if __name__ == "__main__":
    main()