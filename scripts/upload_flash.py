#!/usr/bin/env python3
"""Upload a file from the host PC to the Tab5 SD card over USB serial.

Protocol (spec: src/file_transfer.md, receiver: src/file_transfer.cpp):

    UPLOAD <sd_path> <byte_size>\n   ->   READY\n   ->   <raw bytes>
                                     ->   OK <bytes> fnv=<fnv1a32 hex>\n

The device spams player telemetry on the same port; this sender skips
non-protocol lines while waiting for the READY / OK handshakes.

Integrity: fnv1a32 (offset 0x811C9DC5, prime 0x01000193 — the project's
mchk hash family) is computed over the file here and over the streamed
bytes on the device; the transfer is only good when they match.

Usage (STOP the serial monitor first — the port is shared):
    python scripts/upload_flash.py --port COM8 --src .\\bg.jpg --dst /avatars/bg.jpg
"""
import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

FNV_OFFSET = 0x811C9DC5
FNV_PRIME = 0x01000193


def fnv_update(data, state):
    """fnv1a32 over a chunk (project hash family — matches the device)."""
    for b in data:
        state ^= b
        state = (state * FNV_PRIME) & 0xFFFFFFFF
    return state


def wait_line(ser, accept, deadline_s, what):
    """Read lines until accept(line) is True. Skips interleaved device
    telemetry (pfs/hof/rfav3/au/ft lines). Raises on timeout or ERR."""
    deadline = time.monotonic() + deadline_s
    while True:
        line = ser.readline()
        if not line:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"timed out waiting for {what}")
            continue
        s = line.strip()
        if s.startswith("ERR"):
            raise RuntimeError(f"device error: {s}")
        if accept(s):
            return s
        # else: interleaved telemetry — skip


def upload(port, src, dst, baud, chunk, ready_timeout, done_timeout):
    if not os.path.isfile(src):
        raise FileNotFoundError(f"local file not found: {src}")
    if not dst.startswith("/"):
        dst = "/" + dst
    size = os.path.getsize(src)

    with serial.Serial(port=port, baudrate=baud, timeout=0.5,
                       write_timeout=10) as ser, open(src, "rb") as f:
        time.sleep(0.2)
        ser.reset_input_buffer()

        fnv = FNV_OFFSET
        ser.write(f"UPLOAD {dst} {size}\n".encode())
        ser.flush()
        resp = wait_line(ser, lambda s: s == "READY", ready_timeout, "READY")
        print(f"device: {resp} — streaming {size} B -> {dst} ...")

        sent = 0
        t0 = time.monotonic()
        last_print = 0.0
        while sent < size:
            data = f.read(chunk)
            if not data:
                break
            fnv = fnv_update(data, fnv)  # integrity in parallel w/ transfer
            ser.write(data)
            sent += len(data)
            now = time.monotonic()
            if now - last_print > 0.5:
                now_dt = max(now - t0, 1e-6)
                print(f"\r  {sent}/{size} B ({100.0*sent/size:.0f}%)  "
                      f"{sent/1024/now_dt/1024:.2f} MB/s   ",
                      end="", flush=True)
        ser.flush()
        print()

        final = wait_line(ser, lambda s: s.startswith("OK "), done_timeout,
                          "OK")
        parts = final.split()
        if len(parts) < 3 or parts[1] != str(size):
            raise RuntimeError(f"unexpected OK line: {final!r}")
        dev_fnv = int(parts[2].split("=", 1)[1], 16)
        if dev_fnv != fnv:
            raise RuntimeError(
                f"INTEGRITY FAIL — device fnv={dev_fnv:08x} host fnv={fnv:08x}")
        dt = time.monotonic() - t0
        print(f"VERIFIED: {size} B -> {dst} (fnv1a32 {fnv:08x} match) "
              f"in {dt:.1f} s")


def main():
    ap = argparse.ArgumentParser(
        description="Upload a file over USB serial to the Tab5 SD card.")
    ap.add_argument("--port", default="COM8", help="COM port (default COM8)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="nominal baud (CDC ignores it; default 115200)")
    ap.add_argument("--src", required=True, help="local file on the host")
    ap.add_argument("--dst", required=True,
                    help="SD path, e.g. /avatars/neko_voss/rfav/x.rfav")
    ap.add_argument("--chunk", type=int, default=65536)
    ap.add_argument("--ready-timeout", type=float, default=30.0)
    ap.add_argument("--done-timeout", type=float, default=120.0)
    a = ap.parse_args()
    try:
        upload(a.port, a.src, a.dst, a.baud, a.chunk, a.ready_timeout,
               a.done_timeout)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
