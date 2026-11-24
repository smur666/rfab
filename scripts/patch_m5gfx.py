#!/usr/bin/env python3
"""Sync the patched M5GFX Panel_DSI.{cpp,hpp} into this project's libdeps.

The rfab POC needs two modifications to upstream M5GFX (everything else is
stock):

1. s31a/s31b (SkyTab): double-buffer present path. The app draws into a
   dedicated PSRAM buffer; each explicit 4-arg display(0,0,W,H) present
   DMA2D-copies it into the DPI driver's NON-scanned fb and swaps at vsync
   => no scanout tear at any frame rate. (Stock Panel_DSI zero-copies one
   of the driver's two scanout fbs, which is being scanned one frame in two
   => horizontal tear line.)

2. rfab_poc_* extern "C" hooks that src/main.cpp links against:
     rfab_poc_drawbuf() / rfab_poc_drawbuf_ok() / rfab_poc_waitpresent() /
     rfab_poc_color_done()
   including the S101 FIX B exact DMA2D in-flight gate in
   rfab_poc_waitpresent().

PlatformIO re-fetches M5GFX whenever libdeps re-resolve (git dep, no pin),
so this script is the single source of truth: it copies
scripts/m5gfx_patched/Panel_DSI.{cpp,hpp} (the exact files the on-device
firmware was built from) into EVERY M5GFX copy under .pio/libdeps/ and
verifies the markers. Idempotent — copies that already match are skipped.

Fresh-clone build order:
    pio pkg install            (or just let the first `pio run` fetch libs;
                                the first build will fail to link — expected)
    python scripts/patch_m5gfx.py
    pio run -t clean && pio run

After any libdeps re-fetch, re-run this script and do a CLEAN rebuild
(LDF compiles libdeps in place; stale objects otherwise).

Verify on device: boot log line
    I (xxxx) skytab-dsi: drawbuf: ok ptr=...
"""
import hashlib
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
REF_DIR = Path(__file__).resolve().parent / "m5gfx_patched"
FILES = ("Panel_DSI.cpp", "Panel_DSI.hpp")
MARKERS = {
    "Panel_DSI.cpp": (
        "s_skytab_drawbuf_ok",          # s31a/s31b double buffer
        "rfab_poc_drawbuf",             # rfab hook
        "rfab_poc_waitpresent",         # rfab hook
        "rfab_poc_color_done",          # S101 FIX B
    ),
    "Panel_DSI.hpp": (
        "rfabDrawBufRaw",               # rfab hook accessor
        "_draw_buf",                    # s31a
    ),
}


def sha16(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16]


def main() -> int:
    libdeps = REPO / ".pio" / "libdeps"
    if not libdeps.is_dir():
        sys.exit("no .pio/libdeps under repo root — run `pio pkg install` first")
    copies = sorted(
        d
        for d in libdeps.glob("*/M5GFX*")
        if d.is_dir() and (d / "src" / "lgfx" / "v1" / "platforms" / "esp32p4").is_dir()
    )
    if not copies:
        sys.exit(
            "no M5GFX copies found under .pio/libdeps — run `pio pkg install` (or a "
            "first `pio run`) so PlatformIO fetches the libs, then re-run"
        )
    ok = True
    for c in copies:
        dst = c / "src" / "lgfx" / "v1" / "platforms" / "esp32p4"
        for f in FILES:
            ref = (REF_DIR / f).read_bytes()
            cur = dst / f
            if cur.exists() and cur.read_bytes() == ref:
                print(f"  identical (skip): {c.name}/{f}  sha16 {sha16(cur)}")
                continue
            print(f"  PATCHING {c.name}/{f}"
                  + (f"  (was sha16 {sha16(cur)})" if cur.exists() else "  (created)"))
            cur.write_bytes(ref)
            text = cur.read_text(encoding="utf-8", errors="replace")
            missing = [m for m in MARKERS[f] if m not in text]
            if missing:
                ok = False
                print(f"    FAIL markers missing: {missing}")
            else:
                print(f"    ok — markers present, sha16 {sha16(cur)}")
    if not ok:
        return 1
    print("done. If any copy was patched, do a CLEAN rebuild: pio run -t clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
