// ============================================================================
// S133: WiFi via the Tab5's on-board ESP32-C6 module — see wifi_sd.h for the
// credential convention (SD root /config.json: "wifi_ssid" / "wifi_pass").
//
// esp_hosted bootstrap is FRAMEWORK-OWNED: the first WiFi.mode()/begin()
// runs wifiLowLevelInit() -> hostedInitWiFi() (cores/esp32/
// esp32-hal-hosted.c), which assigns the SDIO pins (CLK 18, CMD 19, D0-D3
// 14-17, RST 54 — variant esp32p4, matching the Tab5 wiring), then
// esp_hosted_init() + esp_hosted_connect_to_slave() exactly once (guarded
// by hosted_initialized; a second esp_hosted_init is NOT supported
// upstream). This task therefore only drives the WiFi class; it reads the
// C6 identity back via esp_hosted_get_cp_info() (forward-declared exactly
// as esp_hosted.h; symbol verified present in the prebuilt
// libespressif__esp_hosted.a).
//
// CREDENTIALS (S135, S134 plan — see wifi_sd.h for the full flow): P4 NVS
// namespace "skytab" (keys wifi_ssid / wifi_pass) is the PRIMARY store;
// SD root /config.json (skytab convention) is STARTUP-ONLY provisioning
// (read once at boot when NVS is empty, copied into NVS, logged); serial
// 'SET_WIFI' updates NVS live (file_transfer.cpp). After boot the task
// re-reads NVS ONLY on retries — no more 60 s SD re-reads. The passkey is
// never printed to the log.
//
// Task placement: P4 core 1 (the non-render core), priority 1 — the C6
// options considered and rejected are recorded in file_transfer.h.
// ============================================================================
#include "wifi_sd.h"

#include <Arduino.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <ArduinoJson.h>
#include <Preferences.h>       // S135 (S134 plan): NVS (nvs 0x5000 @ 0x9000 in the partition table)
#include <esp_heap_caps.h>     // S135 (S134 plan): internal-RAM telemetry (Q-A)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

extern "C" {
int esp_hosted_get_cp_info(uint32_t *cp_chip_id, char *cp_target_name,
                           size_t cp_target_name_len);
}

namespace {

constexpr size_t kSsidSz = 40;                 // WPA SSID max
constexpr size_t kPassSz = 65;                 // WPA passkey max (64 + NUL)
constexpr uint32_t kConnectWindowMs = 30000;   // begin() -> CONNECTED budget
constexpr uint32_t kLossCheckMs = 30000;       // post-connect health checks
constexpr uint32_t kRetryDelayMs = 60000;      // backoff before re-read + retry

char s_ssid[kSsidSz] = {0};
char s_pass[kPassSz] = {0};

// SD root /config.json (skytab convention) -> only these two keys are read.
// Same static-buffer + StaticJsonDocument pattern as avatar.cpp readConfig
// (S132, built green) and the S127 no-large-stack-allocs rule.
bool readWifiCreds() {
  File f = SD_MMC.open("/config.json");
  if (!f) return false;
  // S169: PSRAM (was 4 KB of internal .bss); allocated once, kept.
  static uint8_t *s_cfg = (uint8_t *)heap_caps_malloc(
      4096, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_cfg) {
    f.close();
    return false;
  }
  const int n = f.read(s_cfg, 4095);
  f.close();
  if (n <= 0) return false;
  s_cfg[n] = '\0';
  static StaticJsonDocument<1024> s_doc;  // function-static (S127 stack rule)
  const DeserializationError err =
      ArduinoJson::deserializeJson(s_doc, (const char *)s_cfg, (size_t)n);
  if (err) {
    printf("wifi: /config.json parse error: %s — ignored\n", err.c_str());
    return false;
  }
  // Unknown/other elements in the file are ignored by design.
  const char *s = s_doc["wifi_ssid"].as<const char *>();
  if (!s || !*s) return false;  // no usable ssid -> no credentials
  const char *p = s_doc["wifi_pass"].as<const char *>();
  strncpy(s_ssid, s, sizeof(s_ssid) - 1);
  s_ssid[sizeof(s_ssid) - 1] = '\0';
  if (p && *p) {
    strncpy(s_pass, p, sizeof(s_pass) - 1);
    s_pass[sizeof(s_pass) - 1] = '\0';
  } else {
    s_pass[0] = '\0';  // open network
  }
  return true;
}

// S135 (S134 plan): credential resolution — NVS primary, then (boot only)
// a ONE-TIME /config.json read that provisions NVS. `sd_allowed` is true
// exactly once, at task start; every later retry is NVS-only (the SD
// re-reads were removed — they are not the only SD traffic, but they were
// a needless part of the Q-SW internal-RAM pressure). On success fills
// s_ssid/s_pass (the live credential cache the WiFi.begin() calls use).
// S169: at boot (sd_allowed), a /config.json with credentials is the
// source of truth: if NVS is empty OR differs (ssid or pass), NVS is
// rewritten from it. No /config.json (or no wifi keys in it) -> NVS as-is
// (the future serial-provisioned-at-flash path). Retries are NVS-only.
bool loadCreds(bool sd_allowed) {
  char nv_ssid[kSsidSz] = {0};
  char nv_pass[kPassSz] = {0};
  const bool have_nvs = wifi_nvs_get(nv_ssid, sizeof(nv_ssid), nv_pass, sizeof(nv_pass));
  if (sd_allowed && readWifiCreds()) {  // fills s_ssid/s_pass from the SD
    const bool same = have_nvs && strcmp(nv_ssid, s_ssid) == 0 &&
                      strcmp(nv_pass, s_pass) == 0;
    if (same) {
      printf("wifi: /config.json matches NVS — ssid=%s\n", s_ssid);
    } else if (wifi_nvs_set(s_ssid, s_pass)) {
      printf("wifi: NVS %s from /config.json — ssid=%s (passkey not printed)\n",
             have_nvs ? "UPDATED" : "provisioned", s_ssid);
    } else {
      printf("wifi: NVS write FAILED — using /config.json creds this boot\n");
    }
    memset(nv_pass, 0, sizeof(nv_pass));
    return true;
  }
  if (have_nvs) {
    strncpy(s_ssid, nv_ssid, sizeof(s_ssid) - 1);
    s_ssid[sizeof(s_ssid) - 1] = '\0';
    strncpy(s_pass, nv_pass, sizeof(s_pass) - 1);
    s_pass[sizeof(s_pass) - 1] = '\0';
    memset(nv_pass, 0, sizeof(nv_pass));
    return true;
  }
  s_ssid[0] = '\0';
  s_pass[0] = '\0';
  return false;
}

void wifiTask(void *) {
  printf("wifi: task running — core_id=%d (1 = P4 core 1, as pinned)\n",
         xPortGetCoreID());

  // S135 (S134 plan): NVS primary; the SD /config.json is read at startup
  // ONLY, once, and provisions NVS. No credentials -> WAIT (10 s polls of
  // NVS) instead of exiting, so a 'SET_WIFI' over serial can arm WiFi
  // without a reboot.
  bool have_creds = loadCreds(true);
  if (!have_creds) {
    printf("wifi: no credentials yet (NVS 'skytab' empty + /config.json "
           "missing/invalid) — waiting\n");
    printf("wifi: provision: 'SET_WIFI <ssid> [<pass>]' over serial (live) "
           "or SD root /config.json {\"wifi_ssid\":\"...\", "
           "\"wifi_pass\":\"...\"} + reboot\n");
  } else {
    printf("wifi: credentials ready — ssid=%s (passkey not printed)\n",
           s_ssid);
  }

  uint32_t chip = 0;
  char tname[32] = {0};
  bool cp_logged = false;
  for (;;) {
    if (!have_creds) {
      vTaskDelay(pdMS_TO_TICKS(10000));  // S135: NVS poll (SET_WIFI writes here)
      have_creds = loadCreds(false);
      if (have_creds) {
        printf("wifi: credentials ready — ssid=%s (passkey not printed)\n",
               s_ssid);
      }
      continue;
    }
    // First call triggers wifiLowLevelInit() -> hostedInitWiFi(): SDIO pin
    // assignment + esp_hosted_init + connect_to_slave (framework-owned,
    // once). A C6 that never answers fails there with the framework's own
    // 'ESP-Hosted: ...' log lines; lowLevelInitDone is reset so each
    // attempt below retries it cleanly.
    WiFi.mode(WIFI_STA);
    if (s_pass[0]) {
      WiFi.begin(s_ssid, s_pass);
    } else {
      WiFi.begin(s_ssid);
    }
    const uint32_t t0 = millis();
    bool connected = false;
    while (millis() - t0 < kConnectWindowMs) {
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        break;
      }
      vTaskDelay(200);
    }
    if (!connected) {
      printf("wifi: connect attempt FAILED (30 s) — retry in 60 s "
             "(re-checks NVS; 'SET_WIFI' over serial updates live)\n");
      vTaskDelay(kRetryDelayMs);
      have_creds = loadCreds(false);  // S135: NVS only (a SET_WIFI may have landed)
      continue;
    }
    if (!cp_logged) {
      // Safe now: the framework's hostedInitWiFi() has completed (or the
      // connect would not have succeeded), so the esp_hosted host state
      // behind this query is live.
      const int ci = esp_hosted_get_cp_info(&chip, tname, sizeof(tname));
      printf("wifi: C6 coprocessor up — chip=0x%08lX target='%s' "
             "(get_cp_info rc=%d)\n",
             (unsigned long)chip, tname, ci);
      cp_logged = true;
    }
    const IPAddress ip = WiFi.localIP();
    printf("wifi: CONNECTED — ip=%s rssi=%d dBm\n", ip.toString().c_str(),
           (int)WiFi.RSSI());
    // S135 (S134 plan): heap telemetry point 2 — POST-WiFi. Q-A: compare
    // with the 'boot:' print (main.cpp, pre-WiFi) and the at-failure print
    // in the prefetch retry line: largest-block-tiny = fragmentation,
    // free-small = total exhaustion -> sizes the Q-B sdkconfig fix.
    {
      const size_t ifree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      const size_t ilarg = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
      printf("wifi: heap post-CONNECT — internal free=%u KB largest=%u KB "
             "min-ever=%u KB psram free=%u KB\n",
             (unsigned)(ifree / 1024), (unsigned)(ilarg / 1024),
             (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    }
    // Health loop: 30 s checks; any loss -> backoff + NVS re-check +
    // full reconnect.
    for (;;) {
      vTaskDelay(kLossCheckMs);
      if (WiFi.status() != WL_CONNECTED) {
        printf("wifi: connection LOST — retry in 60 s (re-checks NVS)\n");
        break;
      }
    }
    vTaskDelay(kRetryDelayMs);
    have_creds = loadCreds(false);  // S135: NVS only
  }
}

}  // namespace

// ---- S135 (S134 plan): P4 NVS credential store (namespace "skytab") ----
// GLOBAL scope (matches the wifi_sd.h declarations — an anonymous-namespace
// definition would give the symbols internal linkage and break the
// file_transfer.cpp SET_WIFI caller). The P4 NVS is the primary credential
// path. The C6's own NVS (RF calibration data) lives on the C6 flash and is
// never touched. NVS is thread-safe (esp-nvs has its own lock) —
// wifi_nvs_set() is called from the file transfer's rx task (SET_WIFI) while
// the wifi task may be mid-retry.
namespace {
constexpr char kNvsNs[]   = "skytab";
constexpr char kNvsSsid[] = "wifi_ssid";
constexpr char kNvsPass[] = "wifi_pass";
}  // namespace

bool wifi_nvs_get(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz) {
  if (!ssid || ssid_sz < 2 || !pass || pass_sz < 2) return false;
  ssid[0] = '\0';
  pass[0] = '\0';
  Preferences p;
  if (!p.begin(kNvsNs, true)) return false;  // RO from the reader side
  if (!p.isKey(kNvsSsid)) {
    p.end();
    return false;  // not provisioned
  }
  const size_t n = p.getString(kNvsSsid, ssid, ssid_sz);
  ssid[ssid_sz - 1] = '\0';  // defensive: NVS strings are not guaranteed NUL-terminated
  if (p.isKey(kNvsPass)) {
    p.getString(kNvsPass, pass, pass_sz);
  }
  pass[pass_sz - 1] = '\0';
  p.end();
  if (n == 0 || n >= ssid_sz) return false;  // empty or truncated -> unusable
  return true;
}

bool wifi_nvs_set(const char *ssid, const char *pass) {
  if (!ssid || !*ssid) return wifi_nvs_clear();
  // putString() returns strlen(value) on success, 0 on failure — note an
  // EMPTY pass legitimately stores 0 bytes, so the pass check falls back to
  // isKey() to distinguish 'stored empty' from 'store failed'.
  Preferences p;
  if (!p.begin(kNvsNs, false)) return false;
  if (p.putString(kNvsSsid, ssid) == 0) {  // ssid is non-empty here (guard above)
    p.end();
    return false;
  }
  if (p.putString(kNvsPass, pass ? pass : "") == 0 && !p.isKey(kNvsPass)) {
    p.end();
    return false;
  }
  p.end();
  return true;
}

bool wifi_nvs_clear() {
  Preferences p;
  if (!p.begin(kNvsNs, false)) return false;
  p.remove(kNvsSsid);  // best effort — a missing key just logs an nvs error
  p.remove(kNvsPass);
  p.end();
  return true;
}

void wifi_sd_init() {
  if (xTaskCreatePinnedToCore(wifiTask, "wifi_sd", 8192, nullptr, 1, nullptr,
                              1) != pdPASS) {
    printf("wifi: task create FAIL — WiFi disabled\n");
    return;
  }
  printf("wifi: started (on-board ESP32-C6 via esp_hosted SDIO; S135: "
         "creds in P4 NVS 'skytab', SD /config.json = startup provisioning)\n");
}
