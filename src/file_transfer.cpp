// ============================================================================
// S133: file transfer receiver — see file_transfer.h for the protocol and the
// core-placement rationale (P4 core 1; the C6 is not app-schedulable here).
// ============================================================================
#include "file_transfer.h"

#include <Arduino.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <string.h>

#include "wifi_sd.h"  // S135 (S134 plan): SET_WIFI -> P4 NVS credential store

namespace {

constexpr size_t kBufSize   = 4096;
constexpr size_t kLineSize  = 256;
constexpr size_t kMaxPath   = 200;
constexpr size_t kMaxSize   = 64UL * 1024UL * 1024UL;
constexpr uint32_t kStallMs = 5000;  // inter-byte gap that aborts a transfer
constexpr uint32_t kLineGapMs = 1000;  // partial-line (garbage) drop timeout

// fnv1a32 — the project's hash family (S92 mchk tables): offset 0x811C9DC5,
// prime 0x01000193, over the raw bytes as streamed.
uint32_t fnv1a32(const uint8_t *data, size_t len, uint32_t state) {
  for (size_t i = 0; i < len; i++) {
    state ^= data[i];
    state *= 0x01000193u;
  }
  return state;
}

uint8_t *s_buf = nullptr;  // 4 KB RX buffer (PSRAM, static fallback)
char s_line[kLineSize];    // command line (internal RAM — tiny)

bool validPath(const char *p) {
  size_t n = strlen(p);
  if (n == 0 || n > kMaxPath) return false;
  if (p[0] != '/') return false;
  if (strstr(p, "..")) return false;          // no traversal
  if (strchr(p, '\\')) return false;          // no backslashes
  for (size_t i = 0; i < n; i++)
    if ((unsigned char)p[i] < 0x20) return false;  // no control chars
  return true;
}

// mkdir -p for the parent chain of `path` (FAT, '/' separators).
bool ensureParentDirs(const char *path) {
  char dir[kMaxPath + 1];
  size_t n = strlen(path);
  if (n == 0 || n >= sizeof(dir)) return false;
  memcpy(dir, path, n + 1);
  char *last_slash = strrchr(dir, '/');
  if (!last_slash || last_slash == dir) return true;  // root-level file
  *last_slash = '\0';
  // mkdir each prefix left-to-right (idempotent via the exists() guard)
  for (char *p = dir + 1; *p; p++) {
    if (*p != '/') continue;
    *p = '\0';
    if (!SD_MMC.exists(dir) && !SD_MMC.mkdir(dir)) return false;
    *p = '/';
  }
  return SD_MMC.exists(dir) || SD_MMC.mkdir(dir);
}

// Read exactly `len` bytes from Serial (CDC). Returns bytes read; < len on a
// host stall (kStallMs inter-byte gap) or a short stream.
size_t readExact(uint8_t *dst, size_t len) {
  size_t got = 0;
  uint32_t last_byte_ms = millis();
  while (got < len) {
    if (Serial.available() > 0) {
      int n = Serial.readBytes(reinterpret_cast<char *>(dst + got), len - got);
      if (n > 0) {
        got += (size_t)n;
        last_byte_ms = millis();
      }
    } else {
      if (millis() - last_byte_ms > kStallMs) break;
      vTaskDelay(2);
    }
  }
  return got;
}

void handleUpload(const char *line) {
  // "UPLOAD <path> <size>"
  const char *p = line + strlen("UPLOAD");
  while (*p == ' ') p++;
  const char *sp = strchr(p, ' ');
  if (!sp) { printf("ERR missing_size\n"); return; }
  size_t plen = (size_t)(sp - p);
  if (plen == 0 || plen > kMaxPath) { printf("ERR bad_path\n"); return; }
  char path[kMaxPath + 1];
  memcpy(path, p, plen);
  path[plen] = '\0';
  char *end = nullptr;
  unsigned long size = strtoul(sp + 1, &end, 10);
  if (end == sp + 1 || size == 0 || size > kMaxSize) {
    printf("ERR bad_size\n");
    return;
  }
  if (!validPath(path)) { printf("ERR invalid_path\n"); return; }
  if (SD_MMC.totalBytes() == 0) { printf("ERR sd_not_ready\n"); return; }
  if (!ensureParentDirs(path)) { printf("ERR mkdir_failed\n"); return; }

  File out = SD_MMC.open(path, "w");
  if (!out) { printf("ERR open_failed\n"); return; }

  printf("ft: UPLOAD %s %lu B — rx core 1\n", path, (unsigned long)size);
  printf("READY\n");  // the host starts the byte stream only after this

  size_t remaining = size;
  size_t written = 0;
  uint32_t fnv = 0x811C9DC5u;
  bool ok = true;
  while (remaining > 0) {
    size_t to = remaining < kBufSize ? remaining : kBufSize;
    size_t got = readExact(s_buf, to);
    if (got != to) {
      ok = false;
      printf("ERR timeout_incomplete\n");
      break;
    }
    fnv = fnv1a32(s_buf, got, fnv);
    int w = out.write(s_buf, got);
    if ((size_t)w != got) {
      ok = false;
      printf("ERR sd_write_failed\n");
      break;
    }
    written += got;
    remaining -= got;
  }
  out.flush();
  out.close();
  if (!ok) {
    SD_MMC.remove(path);  // no partial files left behind
    return;
  }
  printf("OK %lu fnv=%08lX\n", (unsigned long)written, (unsigned long)fnv);
}

// S135 (S134 plan): 'SET_WIFI <ssid> [<pass>]\n' — store the credentials in
// the P4 NVS ("skytab", via wifi_nvs_set) and let the wifi task pick them up
// on its next retry (or re-arm it if it is waiting) — no reboot, no SD.
// A BARE 'SET_WIFI' (empty ssid) clears the stored credentials, which takes
// WiFi down on the next retry. <pass> may contain spaces (everything after
// the first space). The passkey is never printed to the log.
void handleSetWifi(const char *line) {
  const char *p = line + strlen("SET_WIFI");
  while (*p == ' ') p++;
  const char *sp = strchr(p, ' ');
  size_t plen;
  const char *pass_p = nullptr;
  if (!sp) {
    plen = strlen(p);
  } else {
    plen = (size_t)(sp - p);
    pass_p = sp + 1;
    while (*pass_p == ' ') pass_p++;  // allow a leading-space pass
  }
  if (plen == 0) {  // bare SET_WIFI = clear the stored credentials
    if (!wifi_nvs_clear()) { printf("WIFI_ERR nvs_open\n"); return; }
    printf("WIFI_OK cleared — no stored credentials; WiFi down on next "
           "retry (NVS 'skytab' empty)\n");
    return;
  }
  if (plen > 63) { printf("WIFI_ERR ssid_too_long (max 63)\n"); return; }
  char ssid[64];
  memcpy(ssid, p, plen);
  ssid[plen] = '\0';
  for (size_t i = 0; i < plen; i++) {
    if ((unsigned char)ssid[i] < 0x20 || (unsigned char)ssid[i] == 0x7f) {
      printf("WIFI_ERR bad_ssid (control chars)\n");
      return;
    }
  }
  size_t psn = pass_p ? strlen(pass_p) : 0;
  if (psn > 64) { printf("WIFI_ERR pass_too_long (max 64)\n"); return; }
  char pass[65];
  memcpy(pass, pass_p ? pass_p : "", psn);
  pass[psn] = '\0';
  if (!wifi_nvs_set(ssid, pass)) { printf("WIFI_ERR nvs_write\n"); return; }
  printf("WIFI_OK ssid=%s stored in NVS — active on the next wifi retry\n",
         ssid);
}

void ftRxTask(void *) {
  // C6 probe/record: the task reports the core it actually runs on.
  printf("ft: rx task running — core_id=%d (1 = P4 core 1, as pinned)\n",
         xPortGetCoreID());
  vTaskDelay(500);  // let the boot chatter settle before claiming RX
  for (;;) {
    // Wait for the first byte of a line (Serial.setTimeout(20) in ft_init
    // bounds each read; the delay keeps this a cheap poll).
    int c;
    do {
      vTaskDelay(10);
      c = Serial.read();
    } while (c < 0);

    // Collect until '\n' (or a 1 s gap = a partial garbage line -> drop).
    size_t len = 0;
    uint32_t first_ms = millis();
    bool done = false;
    while (!done) {
      c = Serial.read();
      if (c >= 0) {
        if (c == '\n') { done = true; break; }
        if (c == '\r') continue;
        if (len < kLineSize - 1) s_line[len++] = (char)c;
      } else if (millis() - first_ms > kLineGapMs) {
        break;
      }
    }
    s_line[len] = '\0';
    if (len == 0) continue;
    if (strncmp(s_line, "UPLOAD ", 7) == 0) {
      handleUpload(s_line);
    } else if (strncmp(s_line, "SET_WIFI ", 9) == 0) {  // S135 (S134 plan)
      handleSetWifi(s_line);
    } else if (strncmp(s_line, "SET_WIFI", 8) == 0 && s_line[8] == 0) {  // bare = clear
      handleSetWifi(s_line);
    } else if (len <= 48) {
      // Short non-protocol lines are echoed for debugging; long ones are
      // (probably) stray binary garbage — don't mirror that back.
      printf("ft: (line ignored: %s)\n", s_line);
    }
  }
}

}  // namespace

void ft_init() {
  s_buf = (uint8_t *)heap_caps_malloc(kBufSize, MALLOC_CAP_SPIRAM);
  if (!s_buf) {
    printf("ft: PSRAM RX buffer FAIL — static internal-RAM fallback\n");
    // S169: runtime internal alloc (a static fallback cost 4 KB of
    // internal .bss even when PSRAM succeeded — i.e. always).
    s_buf = (uint8_t *)heap_caps_malloc(kBufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_buf) return;
  }
  Serial.setTimeout(20);  // bounds Serial.read() from the rx task
  if (xTaskCreatePinnedToCore(ftRxTask, "ft_rx", 8192, nullptr, 1, nullptr,
                              1) != pdPASS) {
    printf("ft: rx task create FAIL — file transfer disabled\n");
    return;
  }
  printf("ft: file transfer armed — 'UPLOAD <sd_path> <size>' over serial; "
         "host: tools/upload_flash.py\n");
}
