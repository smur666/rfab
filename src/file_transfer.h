// ============================================================================
// S133: Windows -> SD file transfer (spec: src/file_transfer.md).
//
// Protocol (host -> device, USB serial/CDC, any nominal baud):
//
//   UPLOAD <sd_path> <byte_size>\n   ->   READY\n   ->   <raw bytes, exact size>
//                                       ->   OK <bytes_written> fnv=<HEX>\n
//   or  ERR <reason>\n  (partial file removed) at any failure point.
//
//   SET_WIFI <ssid> [<passkey>]\n    ->   WIFI_OK ...\n   (S135: P4 NVS
//   "skytab" credentials — the wifi task applies them on its next retry or
//   re-arms if waiting; bare 'SET_WIFI\n' clears; <passkey> may hold spaces;
//   never printed to the log)
//
// Integrity: fnv1a32 (offset 0x811C9DC5, prime 0x01000193 — the project's
// mchk hash family, S92) is computed over the streamed bytes on the device
// and over the file on the host (scripts/upload_flash.py); the transfer
// only counts as good when they match.
//
// <sd_path> is absolute on the SD volume (1:1 with the app's own paths,
// e.g. "/avatars/neko_voss/rfav/intro.rfav"). Parent dirs are mkdir -p'd.
//
// Where the receiver runs: a FreeRTOS task pinned to P4 CORE 1 (the
// non-render core), priority 1. The C6 options considered and rejected:
//   * the P4's IN-CHIP C6 core is not part of this build's FreeRTOS
//     (CONFIG_FREERTOS_NUMBER_OF_CORES=2 — prebuilt libs sdkconfig), so
//     app tasks cannot be pinned to it;
//   * the Tab5's ON-BOARD ESP32-C6 module is the esp_hosted WiFi coprocessor
//     — it runs its own slave firmware (see wifi_sd.cpp), not app tasks.
// Core 1 at prio 1 sits under the audio tasks (prio 3/4) and is idle between
// audio frames, so a transfer never touches the render path on core 0.
//
// Serial ownership: the rest of the app is TX-only on Serial, so the
// receiver task OWNS Serial RX. The live player's telemetry interleaves on
// TX; the host skips non-protocol lines while waiting for the READY / OK
// handshakes, so the chatter is harmless.
// ============================================================================
#pragma once

void ft_init();  // from setup() — Serial up; SD is optional (checked per upload)
