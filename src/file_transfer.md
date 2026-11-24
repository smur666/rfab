Here is a Markdown spec and code sketch tailored for an **M5Stack Tab5 (ESP32-P4)** running **PlatformIO** (Arduino / ESP-IDF) and writing to **internal Flash** (`LittleFS` or `FATFS` partition—or you can swap the file path prefix if you later target `/sdcard` instead). It uses a lightweight line-header + raw binary chunk protocol over USB Serial (`pyserial`) so your other AI can adapt it directly into your project.

---

# ESP32-P4 (M5Stack Tab5) Serial Flash File Uploader Spec

## Overview

* **Transport:** USB Serial (`115200` or `921600` baud; `921600` is recommended for faster transfers once line stability is verified).
* **Target Storage:** Internal Flash partition (e.g., `LittleFS` or `FFat` / `FATFS` mounted at `/lfs` or `/spiffs` or `/ffat`), **not** `esptool` flash-memory writes.
* **Important:** Close the PlatformIO Serial Monitor before running the Windows Python sender so `COMx` isn't locked.

## Wire Protocol (Host -> Device)

1. Host sends an ASCII command line terminated by `\n`:
`UPLOAD <destination_path> <byte_size>\n`
Example: `UPLOAD /rom/tod_quest.bin 16384\n`
2. Device parses `<destination_path>` and `<byte_size>`, opens/creates the file for writing, and replies with:
`READY\n` (or `ERR <reason>\n` on failure).
3. Host streams exactly `<byte_size>` raw bytes in chunks (e.g., `4096` bytes) with a small write timeout.
4. Once all `<byte_size>` bytes are written and flushed/closed, device replies:
`OK <bytes_written>\n` (or `ERR <reason>\n`).

5. **S135: `SET_WIFI <ssid> [<passkey>]\n`** — live credential update, no reboot, no SD. The device stores the pair in **P4 NVS** (namespace `"skytab"`, keys `wifi_ssid` / `wifi_pass`) and replies:
`WIFI_OK ...\n` (or `WIFI_ERR <reason>\n`). The wifi task applies the new values on its **next retry** (≤60 s after a failed connect) — or re-arms itself if it is waiting for credentials. A bare `SET_WIFI\n` (empty ssid) clears the stored credentials (WiFi down on the next retry). `<passkey>` may contain spaces (everything after the first space). The passkey is never printed to the log.

---

## Windows Python Sender (`upload_flash.py`)

Requires `pyserial` (`pip install pyserial`).

```python
import argparse
import os
import sys
import time
import serial


def upload_file(port: str, baud: int, local_path: str, remote_path: str, chunk_size: int = 4096, timeout: float = 5.0) -> None:
    if not os.path.isfile(local_path):
        raise FileNotFoundError(f"Local file not found: {local_path}")

    if not remote_path.startswith("/"):
        remote_path = "/" + remote_path

    file_size = os.path.getsize(local_path)

    with serial.Serial(port=port, baudrate=baud, timeout=timeout, write_timeout=timeout) as ser:
        # Allow CDC/UART bridge to settle if opened fresh
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        header = f"UPLOAD {remote_path} {file_size}\n".encode("utf-8")
        ser.write(header)
        ser.flush()

        resp = ser.readline().decode("utf-8", errors="replace").strip()
        if resp != "READY":
            raise RuntimeError(f"Device rejected upload: {resp!r}")

        sent = 0
        with open(local_path, "rb") as f:
            while sent < file_size:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                ser.write(chunk)
                sent += len(chunk)

        ser.flush()

        final_resp = ser.readline().decode("utf-8", errors="replace").strip()
        if not final_resp.startswith("OK"):
            raise RuntimeError(f"Upload failed after {sent}/{file_size} bytes. Device response: {final_resp!r}")

        print(f"Uploaded {sent} bytes -> {remote_path} ({final_resp})")


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload a file over USB Serial to ESP32-P4 internal Flash filesystem.")
    parser.add_argument("--port", required=True, help="Windows COM port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    parser.add_argument("--src", required=True, help="Path to local file on Windows")
    parser.add_argument("--dst", required=True, help="Destination path on ESP32 filesystem, e.g. /data/test.bin")
    parser.add_argument("--chunk-size", type=int, default=4096, help="Write chunk size in bytes (default: 4096)")
    args = parser.parse_args()

    try:
        upload_file(args.port, args.baud, args.src, args.dst, args.chunk_size)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

```

### Example Windows Usage

```powershell
python upload_flash.py --port COM7 --baud 921600 --src .\quest.dat --dst /quest.dat

```

---

## Firmware Receiver Sketch (PlatformIO / Arduino Style)

This example uses `LittleFS` on internal Flash. Have your other AI adjust the filesystem object (`LittleFS` vs. `FFat`), USB CDC/UART object (`Serial`), and partition table (`partitions.csv`) to match your `platformio.ini` configuration.

```cpp
#include <Arduino.h>
#include <LittleFS.h>

static constexpr size_t BUF_SIZE = 4096;
static uint8_t rx_buf[BUF_SIZE];

bool ensureParentDirs(const String& path) {
  int last_slash = path.lastIndexOf('/');
  if (last_slash <= 0) {
    return true;  // File is directly in root '/'
  }

  String dir = path.substring(0, last_slash);
  if (LittleFS.exists(dir)) {
    return true;
  }

  // Create intermediate directories if needed
  int start = 1;
  while (start < dir.length()) {
    int idx = dir.indexOf('/', start);
    String sub = (idx == -1) ? dir : dir.substring(0, idx);
    if (!LittleFS.exists(sub) && !LittleFS.mkdir(sub)) {
      return false;
    }
    if (idx == -1) break;
    start = idx + 1;
  }
  return true;
}

size_t readExactBytes(uint8_t* dst, size_t len, uint32_t idle_timeout_ms = 5000) {
  size_t total = 0;
  uint32_t last_byte_ms = millis();

  while (total < len) {
    if (Serial.available() > 0) {
      int n = Serial.readBytes(reinterpret_cast<char*>(dst + total), len - total);
      if (n > 0) {
        total += static_cast<size_t>(n);
        last_byte_ms = millis();
      }
    } else {
      if (millis() - last_byte_ms > idle_timeout_ms) {
        break;  // Host stalled or disconnected mid-transfer
      }
      delay(1);
    }
  }
  return total;
}

void handleUploadCommand(const String& line) {
  // Expected format: UPLOAD <path> <size>
  int first_space = line.indexOf(' ');
  int second_space = (first_space >= 0) ? line.indexOf(' ', first_space + 1) : -1;

  if (first_space < 0 || second_space < 0) {
    Serial.println("ERR bad_header");
    return;
  }

  String path = line.substring(first_space + 1, second_space);
  path.trim();
  size_t expected_size = static_cast<size_t>(line.substring(second_space + 1).toInt());

  if (!path.startsWith("/") || expected_size == 0) {
    Serial.println("ERR invalid_path_or_size");
    return;
  }

  if (!ensureParentDirs(path)) {
    Serial.println("ERR mkdir_failed");
    return;
  }

  File out = LittleFS.open(path, FILE_WRITE);
  if (!out) {
    Serial.println("ERR open_failed");
    return;
  }

  Serial.println("READY");
  Serial.flush();

  size_t remaining = expected_size;
  size_t written_total = 0;

  while (remaining > 0) {
    size_t to_read = (remaining < BUF_SIZE) ? remaining : BUF_SIZE;
    size_t got = readExactBytes(rx_buf, to_read);
    if (got != to_read) {
      out.close();
      LittleFS.remove(path);
      Serial.println("ERR timeout_incomplete");
      return;
    }

    size_t w = out.write(rx_buf, got);
    written_total += w;
    if (w != got) {
      out.close();
      LittleFS.remove(path);
      Serial.println("ERR flash_write_failed");
      return;
    }
    remaining -= got;
  }

  out.flush();
  out.close();

  Serial.print("OK ");
  Serial.println(written_total);
  Serial.flush();
}

void setup() {
  Serial.begin(921600);
  uint32_t start = millis();
  while (!Serial && (millis() - start < 3000)) {
    delay(10);
  }

  if (!LittleFS.begin(true)) {
    Serial.println("ERR littlefs_mount_failed");
  } else {
    Serial.println("READY_IDLE");
  }
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("UPLOAD ")) {
      handleUploadCommand(line);
    }
  }
}

```

## Notes for Your Other AI / Integration

* **Partition table (`partitions.csv`):** Ensure the firmware's partition table defines a `spiffs` / `ffat` / custom data subtype partition that fits inside the Tab5's **16 MB** Flash alongside your app binary and OTA partitions.
* **Background logs:** Keep unrelated `Serial.print` debug chatter out of the transfer once `READY` is sent, because the Python client reads a single line response (`OK ...` or `ERR ...`) after the binary payload finishes.
* **Integrity check (optional upgrade):** If transfers ever glitch at `921600` baud, append a SHA-256 or CRC32 check to the `OK` handshake and verify it before committing the file.