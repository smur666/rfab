// ============================================================================
// S133: WiFi via the Tab5's on-board ESP32-C6 module (esp_hosted, SDIO 4-bit
// @ 40 MHz — the framework's prebuilt config: CMD 19, CLK 18, D0-D3 14-17,
// reset GPIO 54). The C6 runs the esp_hosted SLAVE firmware (its own image —
// the P4 flash holds no slave partition; CONFIG_SLAVE_IDF_TARGET_ESP32C6);
// the P4 talks to it as a remote WiFi engine (CONFIG_ESP_WIFI_REMOTE).
//
// CREDENTIALS (S135, S134 plan — NVS primary, SD = startup provisioning only):
//   1. P4 NVS namespace "skytab", keys "wifi_ssid" / "wifi_pass" — the live
//      store. The C6's own NVS (RF calibration) lives on the C6 flash and is
//      never touched; the P4 passes creds to the C6 via the esp_wifi API over
//      the esp_hosted SDIO link on each connect.
//   2. SD ROOT /config.json (skytab-app convention) — read ONCE at startup
//      when NVS is empty (one-time provisioning, copied into NVS, logged).
//      After boot the SD is never re-read for creds (the old 60 s re-reads
//      are gone — SD traffic was part of the Q-SW internal-RAM pressure).
//   3. Serial command 'SET_WIFI <ssid> [<pass>]\n' (file_transfer's line
//      dispatch, S135) writes NVS live — applies on the next wifi retry (or
//      re-arms a waiting task), no reboot, no SD. Bare 'SET_WIFI' clears.
// The passkey is never printed to the log.
// ============================================================================
#pragma once

#include <stddef.h>

// S135 (S134 plan): P4 NVS credential store (namespace "skytab").
bool wifi_nvs_get(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz);
bool wifi_nvs_set(const char *ssid, const char *pass);  // empty ssid = clear
bool wifi_nvs_clear();

void wifi_sd_init();  // from setup() — after the SD mount attempt
