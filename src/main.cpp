// ============================================================================
// RFAV-3 POC player — M5Stack Tab5 (ESP32-P4)
// ----------------------------------------------------------------------------
// Plays the ACTIVE AVATAR's clips (RFAV rev3, PORTRAIT 720x1280 — 90°
// baked in at encode, S89; docs/Tab5/rfav2_720p_spec_rev3.md) from
// /avatars/<model>/rfav/ on the SD card at the file's fps (12), looping per
// flags.loop — S122: model + FSM + director come from
// /avatars/<model>/avatar.json (avatar.h; built-in Elena fallback):
//
//   [64 B header][N x 16 B frame table][ (mask 115,200 B + JPEG) x N ]
//
// Per-frame render pipeline (single-writer task, spec rev3 section 7 flow):
//   1. gate: wait for the PREVIOUS present's DMA2D copy to finish
//      (on_frame_buf_complete semaphore from the patched M5GFX panel)
//   2. SD sequential read: mask (115,200 B) + JPEG (jpeg_len)
//   3. P4 HARDWARE JPEG decoder (driver/jpeg_decode.h) -> RGB565 written
//      straight into the 1.75 MB PSRAM draw buffer (S89 portrait-native:
//      no transpose)
//   4. 1-bit mask compositing: transparent pixels restored from the
//      background layer (true alpha; background = /avatars/<bg> per the
//      active avatar, else the legacy copies, else a 32 px checkerboard)
//   5. esp_cache_msync (C2M) so the DMA2D master sees the fresh pixels
//   6. present: 4-arg display(0,0,720,1280) -> DMA2D copy of the WHOLE
//      draw buffer at native panel size into the non-scanned driver fb +
//      vsync swap (patched Panel_DSI, skytab s31a — no scanout tear at
//      any frame rate)
//
// On-screen chip: frame index, real fps, and FRAME TIME vs the per-frame
// budget (1000/fps ms — 83 ms at 12 fps). Per-second telemetry on serial.
//
// Display double buffering: the proven skytab s31a patch — the copied M5GFX
// in .pio/libdeps carries it, plus three rfab_poc_* hooks (Panel_DSI.cpp).
// SD: proven Tab5 SD_MMC pins from the skytab project (43,44,39,40,41,42).
// ============================================================================

#include <M5Unified.h>
#include <SD_MMC.h>
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp32-hal-psram.h>
#include <esp_cache.h>
// esp_private/esp_cache_private.h (esp_mm): the P4 JPEG driver's own output
// buffer alignment check (esp_dma_is_buffer_alignment_satisfied) uses this to
// fetch the L2 cache line size for PSRAM buffers. Declared here so we can
// aligned-allocate fr_fb to it (see S86 note in setup()).
extern "C" esp_err_t esp_cache_get_alignment(uint32_t heap_caps, size_t *out_alignment);
#include <driver/jpeg_decode.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>

#include "rfav3.h"
#include "audio_opus.h"
#include "avatar.h"   // S122: data-file FSM (avatar.json v1.0.0)
#include "file_transfer.h"  // S133: Windows -> SD upload (serial; rx task on core 1)
#include "wifi_sd.h"        // S133: WiFi via on-board ESP32-C6 (esp_hosted SDIO)

// Hooks added to the copied, patched M5GFX Panel_DSI.cpp (rfab POC):
extern "C" uint16_t *rfab_poc_drawbuf(void);    // 720x1280 RGB565 PSRAM draw buffer
extern "C" bool      rfab_poc_drawbuf_ok(void); // s31a double-buffer alloc succeeded?
extern "C" void      rfab_poc_waitpresent(void);// block until previous DMA2D copy done (100 ms cap)
extern "C" uint32_t  rfab_poc_color_done(void); // S101 FIX B: on_color_trans_done callback fires

namespace {
// S89: PORTRAIT-NATIVE. M5GFX runs at rotation 0 (portrait 720x1280 = glass
// native — skytab is a portrait UI, so rot 0 is the ecosystem baseline). The
// SD holds portrait-rotated assets (90° baked in at encode, S78 style) under
// avatar/portrait_rfav/: they decode STRAIGHT into the 720x1280 draw buffer
// (kFW=kBw, kFH=kBh) — no fr_fb, no per-frame transpose (the S85-S88
// landscape path is gone). UI text is drawn in a setRotation(kUiRot) window
// and reverted to 0 (user: "setRotation(1) before the FT text, setRotation(0)
// after" — swap kUiRot 1<->3 if the text reads the wrong way).
constexpr int     kBw      = 720;    // draw buffer (glass-native) width
constexpr int     kBh      = 1280;   // draw buffer height
constexpr int     kFW      = 720;    // frame (asset) width — portrait-native
constexpr int     kFH      = 1280;   // frame height
constexpr size_t  kFbBytes = (size_t)kBw * kBh * 2;  // 1,843,200 B
constexpr size_t  kMaxJpeg = 128 * 1024;  // portrait headroom (S78 portrait max 40,106 B)
constexpr int     kPfBatch = 4;           // S129: prefetch batch-read target (frames per ONE SD transfer)
constexpr size_t  kMask1b  = (size_t)kFW * kFH / 8;  // 115,200 B (90 B rows)
constexpr uint8_t kUiRot   = 1;      // S89: UI-text rotation (swap 1<->3 if wrong)
// S89: bg candidates, tried in order — the user's PORTRAIT reality_fabricator
// bg (720x1280, decodes straight into bg_buf) is primary; the old portrait bg
// and the landscape original (one-shot boot transpose) are fallbacks.
// S122: CLIPS come from the ACTIVE AVATAR (av::active() — the boot-loaded
// /avatars/<model>/avatar.json, or the built-in Elena): clips at
// /avatars/<name>/rfav/<clip>.rfav, and the avatar's state machine + script
// (director rows) replace the S109 compiled playlist. Boot clip = the
// avatar's initial state (intro).
// S122: BG candidates, tried in order — the avatar's OWN file (spec:
// /avatars/<bg>) is primary, then per-model copies, then the legacy S89/S95
// locations (either SD layout works).
constexpr const char *kBgLegacy1 = "/avatar/reality_fabricator_portrait_bg90.jpg";  // S99: user request (720x1280 direct OR 1280x720+transpose)
constexpr const char *kBgLegacy2 = "/avatar/reality_fabricator_pattern_720x1280_rot90.jpg";
constexpr const char *kBgLegacy3 = "/avatar/background.jpg";
constexpr uint32_t kRescanMs     = 5000;
// Frame-time budget shown on screen = one frame period at the file's fps
// (12 fps -> 83 ms). Set in tryOpenRfa() once the header is parsed.
uint32_t kBudgetMs = 83;
}  // namespace

// ------------------------------ state ------------------------------
Rfav3  rf;
File   rf_file;

// S109: CLIP CYCLING state — the PLAYER owns the cycle (it declares "clip
// done" at the end of every clip and advances the playlist); the prefetch
// + handoff tasks RE-ARM IN PLACE on a generation bump (no task restart —
// each is the sole owner of its File/ring, so there is no restart race).
// g_clip_gen bumps on every switch; g_rf_armed_gen is set by the player
// AFTER its rf.open of the new clip PARSED — the prefetch's handshake: it
// never touches the new clip's frame table (rf.readFrame) until the open
// has finished.
char            g_clip_path[96] = "/avatars/elena/rfav/intro.rfav";  // S122: per-model layout
const char     *g_clip_name     = "intro.rfav";
volatile uint32_t g_clip_gen    = 0;   // S109: bumped on every clip switch
volatile uint32_t g_rf_armed_gen = 0;  // S109: = g_clip_gen once rf.open done
volatile bool   g_clip_switch_pending = false;  // S109: rescan — open NOW (no 5 s pace)
uint32_t        g_open_fails    = 0;   // S109: skip a clip after 3 open fails
// S112: AUDIO ANCHOR — esp_timer() time of the current clip's FIRST
// PRESENTED frame (0 = not presented yet). The audio engine starts a clip's
// opus 34 ms after this (user A/V spec); enterClipForState() resets it per
// state entry and the player sets it on the first present of each clip.
volatile int64_t g_clip_first_pres_us = 0;

// S119: TOUCH state (user spec: primary touch = MUTE + VOLUME SLIDER; no
// pause — a future animation might use it). Tap toggles g_muted (feedTask
// writes silence without consuming the ring, so on unmute the audio resumes
// exactly in sync with the video). vswipe = live volume 0..127. S120:
// hswipe = VOLUME too (user: seek "is not a useful control — I actually want
// volume in that direction swipe"); SEEK REMOVED.
volatile bool g_muted = false;  // tap: feed silence (A/V stays locked)
// Touch events: loop() (core 1) classifies gestures -> queue -> the player
// task is the SOLE consumer (single-writer discipline for M5GFX preserved).
enum TouchEvType : uint8_t { TEV_TAP = 0, TEV_SWIPE_H, TEV_SWIPE_V, TEV_LONG };
struct TouchEv {
  uint8_t type;
  int16_t x, y, val;  // x/y = tap position; val = signed swipe delta px
};
static QueueHandle_t s_touch_q = nullptr;  // created in setup() after M5.begin()
// S119: uiFlash — transient second chip line (VOL/mute/seek feedback);
// presentFrame draws it (the only M5GFX writer — single-writer rule kept).
static char     s_flash[40];
static uint32_t s_flash_until_ms = 0;
static void uiFlash(const char *msg) {
  snprintf(s_flash, sizeof(s_flash), "%s", msg);
  s_flash_until_ms = millis() + 1500;
}

uint16_t *fb     = nullptr;   // M5GFX draw buffer (present source, 720x1280)
// S101 FIX A: JDEC SCRATCH — decode never touches the presented fb, so
// un-drained fb rows at DMA2D time show the previous frame's converged
// composite (1-frame ghost, invisible at 12 fps), never JDEC-raw cyan.
// (S89 removed it; S100's cyan-persist verdict put it back.)
uint16_t *fr_fb  = nullptr;
uint16_t *bg_buf = nullptr;   // S89: background layer, BUFFER space (720-px rows)
uint8_t  *mask_buf = nullptr; // per-frame 1-bit mask (reused)
uint8_t  *jpg_in   = nullptr; // JPEG bitstream (DMA-capable)
size_t    jpg_in_alloc = 0;
// S128: per-caller PSRAM scratch for the COMBINED mask+JPEG read (ONE
// contiguous SD transfer, C2M, split in PSRAM — the S127 jank fix on the
// slow Samsung EVO Select U1 card). Each caller owns its own (the read task
// and the player inline path can both call rf.readFrame on the shared
// global rf). kMask1b + kMaxJpeg ~= 246 KB each.
uint8_t  *g_pf_scratch     = nullptr;  // prefetch read task (spawnPrefetch) — S129: two-tier BATCH scratch (985 KB -> 246 KB -> null)
uint8_t   g_pf_batch_max   = 0;        // S129: active batch frames (4 / 1 / 0 — set by the spawnPrefetch tier)
uint8_t  *g_inline_scratch = nullptr;  // player inline path (tryOpenRfa) — S129: unchanged (single-frame S128)

jpeg_decoder_handle_t jdec = nullptr;
jpeg_decode_cfg_t jdec_cfg = {
    .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
    // S79 iter-1 color fix: user saw purple/green cast with RGB order =
    // classic R<->B swap on the P4 JDEC output -> BGR (user confirmed colors
    // correct after this change).
    .rgb_order     = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
    .conv_std      = JPEG_YUV_RGB_CONV_STD_BT601,
};

// ---------------- SD prefetch (S79 iter2 -> S80: 2-frame lookahead) --------
// A core-0 task reads the NEXT frames' mask+JPEG while core 1 decodes /
// composites / presents the current frame. 4-slot ring = 2-frame lookahead:
// a filled slot is not needed for ~166 ms, so even a ~100 ms SD read (the
// prefetch's reads overlap the player's JDEC+composite PSRAM storm every
// frame) lands in time and the player never waits on a slot. S79's 2-slot
// ring had only 83 ms of slack -> the alternating 4-14 ms slot waits seen in
// the log (and their contribution to the visible jitter).
// If the prefetch task dies (SD error) or stalls (player timeout), the player
// transparently falls back to the legacy inline reads (S77 behavior).
constexpr int kSlots = 6;  // S102: 4->6 — 6 frames of headroom (500 ms @12 fps) to ride out the 226-301 ms 1-in-4 SD STALLs; +~0.7 MB PSRAM (15.4 MB free)
struct PSlot {
  uint8_t *mask;
  uint8_t *jpg;
  uint32_t jpg_len;
  uint32_t frame;   // S87: ABSOLUTE frame index this fill holds (tag)
};
PSlot g_slot[kSlots] = {};
SemaphoreHandle_t g_slot_ready[kSlots] = {};  // slot data valid
SemaphoreHandle_t g_slot_free[kSlots]  = {};  // player done w/ slot
// S108: per-slot handoff timestamps (pfHandTask stamps the ready-give
// moment). The player's stale/timeout paths print (now - stamp) = how
// long the sitting tag has been waiting, + the free-credit bank — to pin
// the slot-0 stale chain (every slot-0 take stale, tag i-6, handoffs
// still 12/s, no pfs STALL: the credit phase is the suspect).
static int64_t g_slot_ho_t64[kSlots] = {};

// S83 (decode + alpha fix) — esp_cache_msync directions, settled.
//
// The P4 has NO hardware CPU<->DMA coherence. PSRAM sits behind the unified
// L1/L2 cache; the L2 line is 128 B (S82b's 'not aligned with cache line
// size (0x80)B' error proved it; S82's 64 B rounding logged E-level errors
// on every call). Three masters touch frame memory: the SDMMC DMA (fills
// slots), the JDEC 2D DMA (reads the slot jpg, writes the draw buffer),
// DMA2D (reads the draw buffer at present). esp_cache_msync() at the
// ownership handoffs is the ONLY sync; the direction is the whole game:
//
//  slotWriteback (C2M) — slot mask+jpg, WRITER side only. The SDMMC DMA
//    lands the fresh bytes as DIRTY L2 lines (S82b proof: an M2C invalidate
//    of the jpg range discarded the only copy of the bitstream -> 6,274
//    'JPEG decode FAILED' e=258/259/263, 'EOF/EOI not read', 'invalid
//    DHT'). C2M pushes those lines to the chips so every reader sees fresh
//    bytes; the lines stay valid (a later L2 hit serves the fresh content,
//    because the SDMMC write updated them). The xSemaphoreGive
//    (g_slot_ready) between this and the player's take is the
//    happens-before edge — no reader-side call needed.
//  slotInvalidate (M2C) — the draw buffer, AFTER the decode-success check,
//    BEFORE the composite. The JDEC 2D DMA wrote the frame output to the
//    CHIPS, bypassing L2; L2 still holds VALID-STALE fb lines from the
//    previous frame's composite, and the trailing C2M would flush them
//    over the fresh output (S81: dead-mask alpha regions + blue flashing,
//    4 frames old = the ring period). Invalidate first: the composite
//    repopulates only the lines it writes (fetching fresh chips data on a
//    miss), and the trailing C2M writes back only those.
//
// Ranges are 128 B cache-line aligned (start down / end up). PSRAM heap
// pointers are already line-aligned, so the rounding is exact in practice.
// S99 (model M): chunked esp_cache_msync — a single 1.84 MB range op did not
// reach all the composite's dirty lines (trailing C2M measured 424 µs for
// 1.84 MB = impossible at PSRAM rates). One HW range command per 64 KB.
// (spec: use verbatim)
static esp_err_t syncChunked(const void *p, size_t size, int dir) {
  constexpr size_t kChunk = 64*1024;
  uintptr_t a = (uintptr_t)p & ~(uintptr_t)127;
  const uintptr_t e = ((uintptr_t)p + size + 127) & ~(uintptr_t)127;
  esp_err_t first = ESP_OK;
  for (; a < e; a += kChunk) {
    size_t n = (e-a) < kChunk ? (e-a) : kChunk;
    esp_err_t r = esp_cache_msync((void*)a, n, dir | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
    if (r != ESP_OK && first == ESP_OK) first = r;
  }
  return first;
}
// S99: error accounting — first error + count in globals, printed in the 1 Hz
// telemetry next to c2m (cumulative: any non-OK must stand out).
volatile uint32_t  g_csync_errs  = 0;     // failed syncChunked calls (cumulative)
volatile esp_err_t g_csync_first = ESP_OK; // first failure seen (cumulative)
static inline esp_err_t syncTracked(const void *p, size_t size, int dir) {
  const esp_err_t r = syncChunked(p, size, dir);
  if (r != ESP_OK) {
    g_csync_errs++;
    if (g_csync_first == ESP_OK) g_csync_first = r;
  }
  return r;
}
// S103: per-group bg-skip state — one bit per 8-px mask group (packed bit
// index y * kFW/8 + g; 1280 x 90 bits = 14,400 B BSS). Set = "this group's
// fb pixels already hold the static bg" (bg_buf is never written after
// boot). A fully-transparent group with the bit set is SKIPPED in
// compositeMaskRow — kills the ~1.4-1.8 MB/frame PSRAM write storm on the
// all-transparent runs (thinking.rfav rows 0-488 / 791-1279 skip entirely
// after frame 1). Chip rows safe: presentFrame redraws the whole chip window
// every present; any fb redraw OUTSIDE makeFrame (fatalScreen / wait-state /
// playerTask start) memsets this back to 0 so the next makeFrame writes
// everything. (S100 sentinel/gate state removed S103 — verification done,
// user-confirmed cyan gone S101 + S102.)
static uint8_t g_bgst[(kFH * (kFW / 8) + 7) / 8];
static inline void slotCacheOp(const void *p, size_t size, int dir) {
  if (!p || !size) return;
  syncTracked(p, size, dir);  // S99: 64 KB chunks + error tracking (was single-shot; 128 B rounding is inside syncChunked)
}
static inline void slotWriteback(const void *p, size_t size) {
  slotCacheOp(p, size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}
static inline void slotInvalidate(const void *p, size_t size) {
  slotCacheOp(p, size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
}
volatile bool g_pf_live  = false;   // prefetch task alive and healthy
bool          g_pf_tried = false;   // spawn attempted for this session
volatile uint32_t g_pf_next = 0;    // S87: prefetch's next fill index (abs)
volatile bool   g_pf_dead   = false; // S87: task died — never re-arm
int             g_miss_run  = 0;     // S87: consecutive player slot misses

// Telemetry
uint32_t g_played = 0, g_dropped = 0, g_decode_fail = 0, g_sd_fail = 0;  // S89: g_late -> g_dropped (frame-drop policy)
uint32_t g_last_sd_us = 0, g_last_dec_us = 0, g_last_ft_us = 0;
// S89: per-stage breakdown on the 1 Hz telemetry line, printed in µs:
// iv = fb M2C invalidate, c = composite, c2m = trailing fb C2M. The S87/S88
// 'tp' transpose stage is gone (portrait-native: no transpose).
uint32_t g_last_iv_us = 0, g_last_c_us = 0, g_last_c2m_us = 0;
uint64_t g_ft_sum_us = 0;   // cumulative frame-work time -> avg FT (headroom)
uint32_t g_win_start = 0, g_win_frames = 0, g_win_fps = 0;
bool     g_sd_ok = false;
bool     g_presented = false;
volatile uint32_t g_pf_filled = 0;  // S81: prefetch slot fills (steady-state liveness)
uint32_t g_win_miss = 0;            // S103: frames missed in the last 1 s window (telemetry ms=)
// S128 telemetry (2): inter-present JANK — gaps between consecutive
// presentFrame() completions > 250 ms (a multi-frame stall = a visible
// freeze; the S127 signature was multi-second lovestruck freezes). Windowed
// count + max per 1 s telemetry line as jk=<n>/<max_ms>.
volatile uint32_t g_jk_n = 0, g_jk_max_ms = 0;
static int64_t  g_last_pres_done_us = 0;  // 0 = no present yet
// S128 telemetry (3): SD slow-episode state (owned by the read task) —
// consecutive frames whose read took > 83 ms (the 12 fps budget).
volatile uint32_t g_ep_n = 0, g_ep_max_ms = 0;
volatile uint64_t g_ep_sum_us = 0;

// ------------------------------ helpers ------------------------------

static void bootLog(const char *line) {
  M5GFX &lcd = M5.Display;
  static int y = 90;
  if (y > 680) y = 90;   // S89: UI text lives in the kUiRot window (720 px tall)
  // S89: boot text in the kUiRot window, like the per-frame chip.
  lcd.setRotation(kUiRot);
  lcd.setTextColor(lcd.color565(210, 225, 240));
  lcd.drawString(line, 12, y);
  y += 18;
  lcd.setRotation(0);
  // 4-arg display = present (patched Panel_DSI); without it the line would
  // sit in the draw buffer until the player's first frame.
  lcd.display(0, 0, kBw, kBh);
  printf("%s\n", line);
}

static void fatalScreen(const char *title, const char *detail) {
  M5GFX &lcd = M5.Display;
  memset(g_bgst, 0, sizeof(g_bgst));  // S103: fb redraw outside makeFrame — composite must rewrite all
  lcd.fillScreen(lcd.color565(24, 12, 12));   // full buffer — rotation-proof
  // S89: fatal text in the kUiRot window (readable like the chip).
  lcd.setRotation(kUiRot);
  lcd.setTextColor(lcd.color565(255, 120, 100));
  lcd.drawString(title, 20, 60, 4);
  lcd.setTextColor(lcd.color565(220, 220, 220));
  lcd.drawString(detail, 20, 140, 2);
  lcd.setRotation(0);
  lcd.display(0, 0, kBw, kBh);
  printf("FATAL: %s | %s\n", title, detail);
}

// 1-bit mask row (90 B @720, MSB first, 1 = opaque). S101 FIX A: `src` = this
// frame's JDEC row (fr_fb scratch). Opaque 8-px groups are now MEMCPY'd from
// src (S89 skipped them — JDEC wrote fb in place); fully-transparent groups
// copy the bg row (fb PSRAM holds it from boot seed / last converged
// composite — 0x00 rows are never otherwise written); mixed groups blend
// src over bg per pixel.
// S103: + per-group bg-skip (g_bgst) — a fully-transparent group whose fb
// group already holds the static bg is NOT rewritten.
static inline void compositeMaskRow(const uint8_t *mrow, uint16_t *dst,
                                    const uint16_t *bg, const uint16_t *src,
                                    int width, int y) {
  const int nwords = width / 8;
  size_t bi = (size_t)y * nwords;  // S103: packed g_bgst bit index (y * 90 + r)
  for (int r = 0; r < nwords; r++, bi++) {
    const uint8_t b = mrow[r];
    uint16_t *p = dst + r * 8;
    const uint16_t *s = bg + r * 8;
    const uint16_t *q = src + r * 8;
    uint8_t *stb = &g_bgst[bi >> 3];
    const uint8_t stmask = 1u << (bi & 7);
    if (b == 0xFF) {
      memcpy(p, q, 16);  // fully opaque — this frame's JPEG owns these px
      *stb &= (uint8_t)~stmask;  // S103: group no longer holds the static bg
      continue;
    }
    if (b == 0x00) {
      if (*stb & stmask) continue;  // S103: fb already holds the bg — skip write
      memcpy(p, s, 16);  // fully transparent 8-px group: one 16-byte move
      *stb |= stmask;    // S103: group now holds the bg — skippable next frame
      continue;
    }
    for (int k = 0; k < 8; k++) {
      p[k] = (b & (0x80u >> k)) ? q[k] : s[k];
    }
    *stb &= (uint8_t)~stmask;  // S103: mixed wrote src px — no longer pure bg
  }
}

// S99: composite `rows` consecutive 720-px rows from row-base pointers
// (per-batch C2M loop in makeFrame — flush each batch right after writing it).
// S101: + src base (fr_fb row y0) threaded per row.
// S103: + y0 = first row's ABSOLUTE y (g_bgst bit index).
static void compositeMaskRows(const uint8_t *mask, uint16_t *dst,
                              const uint16_t *bg, const uint16_t *src,
                              int width, int y0, int rows) {
  for (int y = 0; y < rows; y++)
    compositeMaskRow(mask + (size_t)y * (width / 8), dst + (size_t)y * width,
                     bg + (size_t)y * width, src + (size_t)y * width, width,
                     y0 + y);
}

// S89: one-shot BOOT transpose for a landscape background (1280x720 ->
// 720x1280 buffer space). Same 90° CCW mapping the S85-S88 landscape path
// used (dst[r][c] = src[c][1279-r] — the user approved that orientation).
// Runs once at boot — the per-frame path is transpose-free (portrait assets
// decode straight into fb).
static void transposeLandscapeBg(const uint16_t *src, uint16_t *dst) {
  for (int c = 0; c < kBw; c++)
    for (int r = 0; r < kBh; r++)
      dst[(size_t)r * kBw + c] = src[(size_t)c * kBh + (kBh - 1 - r)];
}

static bool initJpegEngine() {
  jpeg_decode_engine_cfg_t eng = {};
  eng.intr_priority = 0;   // driver picks default
  eng.timeout_ms = 200;    // HW 720p decode is a few ms; generous cap
  if (ESP_OK != jpeg_new_decoder_engine(&eng, &jdec) || jdec == nullptr) {
    printf("%s\n", "jpeg: engine init FAILED");
    return false;
  }
  printf("%s\n", "jpeg: P4 hw engine ok");
  return true;
}

// Background layer (S89): three candidates in order — the user's PORTRAIT
// reality_fabricator bg (720x1280, decodes STRAIGHT into bg_buf), the old
// portrait background.jpg, the landscape original (decode + one-shot
// 90° CCW transpose into buffer space). bg_buf is BUFFER space (720-px rows)
// and 128 B aligned (S86 JDEC output requirement). Seeded into fb (plain
// copy now — same space) so the panel shows it before frame 1.
static bool initBackground() {
  M5GFX &lcd = M5.Display;
  size_t bg_align = 128;   // S86: the JDEC PSRAM-output requirement (L2 line)
  size_t qa = 0;
  if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &qa) == ESP_OK && qa > bg_align)
    bg_align = qa;
  bg_buf = (uint16_t *)heap_caps_aligned_alloc(bg_align, kFbBytes,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!bg_buf) {
    printf("%s\n", "bg: PSRAM alloc FAILED");
    return false;
  }

  const uint16_t dark = lcd.color565(24, 28, 36);
  const uint16_t lite = lcd.color565(96, 104, 116);
  bool from_jpg = false;

  // S122/S132: candidates in order — the config.json bg OVERRIDE (if set),
  // the avatar's OWN bg (spec: /avatars/<bg>), per-model copies, then the
  // legacy S89/S95 locations. First that opens wins (an override file that
  // is absent simply falls through to the next candidate).
  const av::AvatarDef *avd0 = av::active();
  char bgc_cfg[64], bgc0[64], bgc1[64], bgc2[64];
  if (av::configBg()[0])
    snprintf(bgc_cfg, sizeof(bgc_cfg), "/avatars/%s", av::configBg());
  snprintf(bgc0, sizeof(bgc0), "/avatars/%s", avd0->bg);
  snprintf(bgc1, sizeof(bgc1), "/avatars/%s/%s", avd0->name, avd0->bg);
  snprintf(bgc2, sizeof(bgc2), "/avatars/%s/background.jpg", avd0->name);
  const char *bgcand[7] = {av::configBg()[0] ? bgc_cfg : nullptr, bgc0, bgc1,
                           bgc2, kBgLegacy1, kBgLegacy2, kBgLegacy3};
  File b;
  const char *bg_used = av::configBg()[0] ? bgc_cfg : bgc0;
  for (int c = 0; c < 7; c++) {
    if (!bgcand[c]) continue;
    b = SD_MMC.open(bgcand[c]);
    if (b) {
      bg_used = bgcand[c];
      break;
    }
  }
  if (b) {
    const uint32_t fsize = (uint32_t)b.size();
    printf("bg: %s found (%lu B)\n", bg_used, (unsigned long)fsize);
    if (fsize <= 1024 * 1024) {
      uint8_t *tmp = (uint8_t *)heap_caps_malloc(fsize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (tmp && b.read(tmp, fsize) == (int)fsize) {
        jpeg_decode_picture_info_t info = {};
        if (jpeg_decoder_get_info(tmp, fsize, &info) == ESP_OK &&
            (size_t)info.width * info.height * 2 <= 4 * 1024 * 1024) {
          const uint32_t w = info.width, h = info.height;
          uint32_t out_size = 0;
          if (w == kBh && h == kBw) {
            // S89: landscape full-frame (1280x720) — aligned temp decode,
            // then the one-shot 90° CCW transpose into buffer space.
            uint16_t *tmpfb = (uint16_t *)heap_caps_aligned_alloc(
                bg_align, (size_t)w * h * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (tmpfb) {
              if (ESP_OK == jpeg_decoder_process(jdec, &jdec_cfg, tmp, fsize,
                                                 (uint8_t *)tmpfb, (uint32_t)w * h * 2,
                                                 &out_size) &&
                  out_size == (uint32_t)w * h * 2) {
                transposeLandscapeBg(tmpfb, bg_buf);
                from_jpg = true;
                printf("bg: %s decoded %ux%u (landscape) -> transposed\n",
                            bg_used, (unsigned)w, (unsigned)h);
              } else {
                printf("%s\n", "bg: decode FAILED");
              }
              heap_caps_free(tmpfb);
            } else {
              printf("%s\n", "bg: temp alloc FAILED");
            }
          } else if (w <= kBw && h <= kBh) {
            // S89: portrait fit — a full-frame portrait jpg decodes STRAIGHT
            // into bg_buf (aligned); smaller ones letterbox from an aligned
            // temp (centered, dark bars).
            uint16_t *out = (w == kBw && h == kBh) ? bg_buf
                : (uint16_t *)heap_caps_aligned_alloc(bg_align, (size_t)w * h * 2,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (out) {
              if (ESP_OK == jpeg_decoder_process(jdec, &jdec_cfg, tmp, fsize,
                                                 (uint8_t *)out, (uint32_t)w * h * 2,
                                                 &out_size) &&
                  out_size == (uint32_t)w * h * 2) {
                if (out == bg_buf) {
                  from_jpg = true;
                  printf("bg: %s decoded %ux%u (portrait) direct\n",
                              bg_used, (unsigned)w, (unsigned)h);
                } else {
                  for (int y = 0; y < kFH; y++)
                    memset((uint16_t *)bg_buf + (size_t)y * kFW,
                           dark * 0x10001u, (size_t)kFW * 2);
                  const int32_t dx0 = ((int32_t)kBw - (int32_t)w) / 2;
                  const int32_t dy0 = ((int32_t)kBh - (int32_t)h) / 2;
                  for (int32_t y = 0; y < (int32_t)h; y++)
                    memcpy(bg_buf + (size_t)(dy0 + y) * kFW + dx0,
                           out + (size_t)y * w, (size_t)w * 2);
                  from_jpg = true;
                  printf("bg: %s decoded %ux%u, placed centered\n",
                              bg_used, (unsigned)w, (unsigned)h);
                }
              } else {
                printf("%s\n", "bg: decode FAILED");
              }
              if (out != bg_buf) heap_caps_free(out);
            } else {
              printf("%s\n", "bg: temp alloc FAILED");
            }
          } else {
            printf("bg: %s size %ux%u unsupported -> checkerboard\n",
                   bg_used, (unsigned)w, (unsigned)h);
          }
        } else {
          printf("%s\n", "bg: bad/oversized jpg header");
        }
        heap_caps_free(tmp);
      } else {
        printf("%s\n", "bg: full read failed");
        if (tmp) heap_caps_free(tmp);
      }
    }
    b.close();
  }

  if (!from_jpg) {
    // 32 px checkerboard — the classic transparency proof.
    printf("%s\n", "bg: checkerboard (no usable bg jpg)");
    for (int y = 0; y < kFH; y++) {
      uint16_t *row = bg_buf + (size_t)y * kFW;
      for (int x = 0; x < kFW; x++) {
        row[x] = (((x >> 5) + (y >> 5)) & 1) ? lite : dark;
      }
    }
  }

  // S89: seed the draw buffer with the background — both are buffer space
  // now, so a plain copy (the S88 rot-3 seed transpose is gone).
  memcpy(fb, bg_buf, kFbBytes);
  esp_cache_msync((void *)fb, kFbBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  return true;
}

// Prefetch worker (core 0) — S106 2-stage pipeline, S107 two-task split.
// readFrame() is seek-based (absolute frame-table offsets), so an
// independent File handle is safe and wraps cleanly at frame 0.
// STAGE 1 (prefetchTask = read task): SD -> PRIVATE ring buffer (g_pb).
// The read task owns these buffers and NEVER touches a player credit: the
// ~41 ms reads run back-to-back, paused only by the ring itself (6 deep),
// which the handoff task frees as credits land.
// STAGE 2 (pfHandTask = handoff task): private ring -> player slot — a
// ~2 ms PSRAM copy + the S83 writer C2M, fired on the first 5 ms poll
// where BOTH the buffered frame for f_hand is ready AND its slot's free
// credit has landed (the player's S105 early-release returns it ~2 ms
// into its take of frame f_hand-6, ~500 ms BEFORE the player needs
// f_hand) -> every handoff lands <= ~10 ms after a player release, with
// >= ~440 ms of runway on every slot.
// S107: WHY THE SPLIT — S106 ran both stages in ONE loop with a BLOCKING
// credit take (portMAX_DELAY). One handoff's credit wait is phase-locked
// to the player's 83 ms take cadence and averaged ~270 ms per lap; while
// it blocked, the NEXT read was paused too, so the read ran at ~11.6/s —
// BELOW the player's 12/s — and the whole handoff chain settled a
// boot-transient offset behind the player grid: exactly one frame per lap
// (slot 0, i%6==0) missed the 50 ms ready window every time (monitor26,
// 12:50-14:25: real=10, ms=2/s, one pfs STALL per lap tk=264-289,
// rd=28-52 clean, zero parks). S107 removes the read<->credit coupling
// entirely: the read pauses only on its own ring; the handoff polls and
// never blocks.
struct PbSlot {
  uint8_t *mask;
  uint8_t *jpg;
  uint32_t jpg_len;
  uint32_t tag;     // frame read into this buffer (valid if in_use)
  uint32_t rd_us;   // S107: SD read time of the buffered frame (pfs rd=) — S129: BATCH-AMORTIZED per frame
  uint32_t rd_end;  // S107: read-complete micros() — the tk deferral base
  uint8_t  batch;   // S129: frames in the batch that filled this entry (pfs b=)
  bool     in_use;
};
static PbSlot g_pb[kSlots];

// ---- STAGE 2 (S107): handoff task — polls, NEVER blocks on a credit ----
static void pfHandTask(void *) {
  // S109: n is read LIVE from rf.frame_count per clip (the handoff only ever
  // mods it for its printfs — the per-clip value is the right one).
  uint32_t my_gen = g_clip_gen;  // S109: re-arm on every clip switch
  int f_hand = 0;  // S107: next frame to hand off to a slot (in order)
  uint32_t ho_wait_ms = 0;  // S108 (d): ms f_hand sat ring-ready w/o credit
  for (;;) {
    // S109: CLIP SWITCH — the new clip's absolute index starts at 0 again
    // (the player and the read task reset the same way, and the ring is
    // poisoned by the read task's gen block, so no old-clip tag can match).
    // The handoff needs no open-handshake — it never reads the frame table
    // (only ring -> slot copies).
    if (g_clip_gen != my_gen) {
      my_gen = g_clip_gen;
      f_hand = 0;  // S120: seek removed — every clip's re-arm starts at frame 0
      ho_wait_ms = 0;
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    g_pf_next = f_hand;  // S87: publish (the resync rewinds to this)
    PbSlot *src = &g_pb[f_hand % kSlots];
    const int slot = f_hand % kSlots;  // S102: % (kSlots-1 mask needs power-of-2)
    if (g_pf_dead) {
      vTaskDelete(nullptr);  // S107: read stage gave up — nothing left to hand
      return;
    }
    if (src->in_use && src->tag == (uint32_t)f_hand &&
        xSemaphoreTake(g_slot_free[slot], 0) == pdPASS) {
      const uint32_t h_t0 = micros();
      memcpy(g_slot[slot].mask, src->mask, kMask1b);
      memcpy(g_slot[slot].jpg, src->jpg, src->jpg_len);
      const uint32_t hlen = src->jpg_len;
      src->in_use = false;  // S107: buffer free — the read task may wrap
      g_slot[slot].jpg_len = hlen;
      g_slot[slot].frame = (uint32_t)f_hand;  // S87: tag (absolute index)
      // S83: writer-side C2M before the ready-give; the give is the
      // happens-before edge to the player.
      slotWriteback(g_slot[slot].mask, kMask1b);
      slotWriteback(g_slot[slot].jpg, hlen);
      g_slot_ho_t64[slot] = esp_timer_get_time();  // S108: tag-landed time
      xSemaphoreGive(g_slot_ready[slot]);
      g_pf_filled++;  // S81: steady-state liveness counter (-> pff= in telemetry)
      const uint32_t pf_ho_us = micros() - h_t0;  // copy + C2M
      // S107 (pfs): rd = SD read (the read task's, stored per buffer),
      // ho = handoff copy+C2M, tk = DEFERRAL = read done -> handoff start
      // (time the frame sat in the ring for its credit). Steady state:
      // the ring is 6 deep and credits land ~500 ms after the previous
      // take, so tk ~= 450 ms is NORMAL — it only STALLs (>800 ms) when
      // the PLAYER goes quiet (no takes -> no credits), the true
      // starvation signal.
      const uint32_t pf_tk_us =
          (h_t0 > src->rd_end) ? (h_t0 - src->rd_end) : 0;
      if (f_hand < 100 || src->rd_us > 80000 || pf_tk_us > 800000)
        printf("pfs f=%d rd=%lu ho=%lu tk=%lu b=%u%s\n", f_hand % (int)rf.frame_count,  // S109: live per clip
               (unsigned long)(src->rd_us / 1000),
               (unsigned long)(pf_ho_us / 1000),
               (unsigned long)(pf_tk_us / 1000),
               (unsigned)src->batch,  // S129: frames in the batch that filled this entry
               (src->rd_us > 80000 || pf_tk_us > 800000) ? " STALL" : "");
      f_hand++;
      ho_wait_ms = 0;  // S108 (d): served
    } else {
      // S108 (d): WHY is f_hand waiting — ring not ready, or no credit?
      // Steady state = the ring is always 6/6 full, so a long wait with
      // the frame present = CREDIT starve (the slot-0 question).
      if (src->in_use && src->tag == (uint32_t)f_hand)
        ho_wait_ms += 5;
      else
        ho_wait_ms = 0;  // read still in flight for this buffer
      if (ho_wait_ms >= 50 && ((ho_wait_ms - 50) % 250) == 0)
        printf("hof f=%d slot=%d wait=%lu ms — no credit (bank=%u)\n",
               f_hand % (int)rf.frame_count, slot, (unsigned long)ho_wait_ms,  // S109: live per clip
               (unsigned)uxSemaphoreGetCount(g_slot_free[slot]));
      vTaskDelay(pdMS_TO_TICKS(5));  // S107: retry next poll (~5 ms away)
    }
  }
}

// ---- STAGE 1 (S107): read task — continuous SD reads, no credits ----
// S109: CLIP CYCLING — the read ring is ALLOC'd once in spawnPrefetch()
// (idempotent); this task inits only the per-frame fields and owns its
// File PER CLIP: boot opens g_clip_path, and every g_clip_gen bump RE-ARMS
// IN PLACE (close, poison the ring, wait for the player's open handshake,
// re-open the new clip). No task restart: this task is the SOLE owner of
// its File + its ring, so there is no restart race.
static void prefetchTask(void *) {
  for (int s = 0; s < kSlots; s++) {  // S106: private read ring (fields only — alloc moved to spawnPrefetch, S109)
    g_pb[s].jpg_len = 0;
    g_pb[s].tag = 0;
    g_pb[s].rd_us = 0;    // S107
    g_pb[s].rd_end = 0;   // S107
    g_pb[s].batch = 0;    // S129
    g_pb[s].in_use = false;
  }
  File      pf     = File();         // S109: opened per clip (boot path + re-arm)
  uint32_t  my_gen = g_clip_gen;     // S109: the gen this task last re-armed on
  uint32_t  n      = rf.frame_count; // S109: per-clip frame count (was const)
  int       f_read = 0;              // S106: next frame to read (per-clip; the ring bounds it to f_hand+6)
  for (;;) {
    // S109: CLIP SWITCH — re-arm IN PLACE (top of loop, before any read).
    if (g_clip_gen != my_gen) {
      my_gen = g_clip_gen;
      // HANDSHAKE: wait until the player's rf.open of the NEW clip
      // FINISHED (it set g_rf_armed_gen after parsing the header) —
      // rf.readFrame below must never touch a half-parsed table.
      // Gen-aware: if the player SKIPS past this clip (3 open fails) the
      // gen bump breaks the wait and the top of the loop re-enters with
      // the new gen (g_rf_armed_gen never goes back to a skipped gen).
      while (g_rf_armed_gen != my_gen && g_clip_gen == my_gen)
        vTaskDelay(pdMS_TO_TICKS(10));
      if (g_clip_gen != my_gen) continue;
      if (pf) { pf.close(); pf = File(); }
      for (int s = 0; s < kSlots; s++) {
        g_pb[s].tag = 0xFFFFFFFFu;  // poison — the handoff never matches
        g_pb[s].in_use = false;
        g_pb[s].jpg_len = 0;
      }
      f_read = 0;  // S120: seek removed — every clip's re-arm starts at frame 0
      g_ep_n = 0; g_ep_sum_us = 0; g_ep_max_ms = 0;  // S128: no cross-clip episodes
      g_pf_live = false;  // the player runs inline until the reopen lands
      pf = SD_MMC.open(g_clip_path);
      if (!pf) {
        printf("prefetch: reopen(%s) failed — inline until next clip\n",
               g_clip_path);
        // the player owns the SD for its rescan — don't race it
        while (g_clip_gen == my_gen) vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      n = rf.frame_count;  // valid: the handshake guarantees the open parsed
      g_pf_live = true;
      printf("prefetch: re-armed %s (%u frames)\n", g_clip_name, n);
    }
    // S109: BOOT open (first iteration only — after any re-arm pf is open).
    if (!pf) {
      while (g_rf_armed_gen != my_gen && g_clip_gen == my_gen)
        vTaskDelay(pdMS_TO_TICKS(100));
      pf = SD_MMC.open(g_clip_path);
      if (!pf) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      n = rf.frame_count;
      g_pf_live = true;
      printf("prefetch: live (core 0, %d-slot ring, S107 two-task)\n", kSlots);
    }
    // ---- S129 BATCH READ: f_read + up to `want` frames into the ring ----
    // want = min(g_pf_batch_max, frames left in the clip, longest FREE ring
    // prefix starting at f_read): SHRINK, never waits longer than S128 (a
    // full ring = the S128 5 ms poll-wait below, want already clamped to
    // 1). The batch never crosses the clip end. A contiguity GAP inside the
    // batch is handled by readFramesBatch itself (table check, no SD access:
    // it shrinks to the longest contiguous prefix — a 1-frame batch is
    // exactly the S128 combined-read shape).
    const int f_rel = f_read % n;
    int want = (int)g_pf_batch_max;
    if (want > kPfBatch) want = kPfBatch;  // defensive (g_pf_batch_max is 4/1/0)
    const int left_n = (int)n - f_rel;
    if (want > left_n) want = left_n;
    if (want > 1) {
      int free_run = 0;
      for (int k = 0; k < want; k++) {
        if (g_pb[(f_read + k) % kSlots].in_use) break;
        free_run++;
      }
      if (free_run < want) want = free_run;  // 0 = first slot busy -> the wait below
    }
    if (want < 1) want = 1;
    const int pb = f_read % kSlots;
    while (g_pb[pb].in_use && g_clip_gen == my_gen)
      vTaskDelay(pdMS_TO_TICKS(5));  // ring full: the handoff task frees it
    if (g_clip_gen != my_gen) continue;  // S109: a switch raced the ring-full wait
    const uint32_t r_t0 = micros();
    uint32_t gots[kPfBatch] = {0};
    uint8_t *m_bufs[kPfBatch] = {nullptr};
    uint8_t *j_bufs[kPfBatch] = {nullptr};
    for (int k = 0; k < want; k++) {
      m_bufs[k] = g_pb[(f_read + k) % kSlots].mask;
      j_bufs[k] = g_pb[(f_read + k) % kSlots].jpg;
    }
    int gotn = 0;  // S129: frames actually read (a contiguity SHRINK is not an error)
    for (int attempt = 0; attempt < 10 && gotn < 1; attempt++) {  // S87: retry (0 = seek/short-read/C2M fail)
      if (g_clip_gen != my_gen) break;  // S109: switch raced the retry — top of loop re-arms
      if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(250));  // S135 (S134 plan): let transient WiFi-stack internal-SRAM allocs settle between retries
      gotn = rf.readFramesBatch(pf, f_rel, want, m_bufs, j_bufs, gots,
                                g_pf_scratch);  // S129: batch (null scratch = legacy 2-seek x1)
      for (int k = 0; k < gotn && k < want; k++)
        if (gots[k] > kMaxJpeg) { gotn = 0; break; }  // per-frame sanity
    }
    if (g_clip_gen != my_gen) continue;  // S109: don't store a switch-raced read
    const uint32_t pf_rd_us = micros() - r_t0;  // S106: SD read only
    // S129: amortize the ONE batch transfer over its frames — the per-frame
    // number drives the slow-episode detector + the per-ring rd_us (pfs rd=
    // stays a per-frame ms).
    const uint32_t pf_rd_ff = (gotn > 0) ? pf_rd_us / (uint32_t)gotn : pf_rd_us;
    // S128 telemetry (3): SD slow-episode detector — consecutive frames
    // with rd > 83 ms (the 12 fps budget) = one episode; print when it ends
    // (first fast frame).
    if (pf_rd_ff > 83000) {
      for (int k = 0; k < gotn; k++) {  // S129: the batch's amortized frames
        g_ep_n++;
        g_ep_sum_us += pf_rd_ff;
      }
      if (pf_rd_ff / 1000 > g_ep_max_ms) g_ep_max_ms = (uint32_t)(pf_rd_ff / 1000);
    } else if (g_ep_n) {
      const uint32_t ms = g_ep_sum_us ? (uint32_t)(g_ep_sum_us / 1000) : 1;
      printf("pfs: SD slow episode — %lu fr rd avg=%lu max=%lu ms (~%lu KB/s)\n",
             (unsigned long)g_ep_n, (unsigned long)(g_ep_sum_us / 1000 / g_ep_n),
             (unsigned long)g_ep_max_ms,
             (unsigned long)((uint64_t)g_ep_n * (kMask1b + kMaxJpeg) / 1024 / ms));
      g_ep_n = 0; g_ep_sum_us = 0; g_ep_max_ms = 0;
    }
    if (gotn < 1) {
      // S109: a read fail no longer KILLS the task (S87's vTaskDelete) —
      // the clip may be fine next cycle: close, go inline, and wait for the
      // NEXT clip switch (the player's end-of-clip or skip bumps the gen),
      // then re-arm at the top of the loop. The prefetch SURVIVES across
      // clips (g_pf_dead is now only settable via the spawnPrefetch alloc
      // fail).
      // S135 (S134 plan): heap telemetry point 3 — AT FAILURE (the Q-SW
      // moment: internal-SRAM state when the SD driver's bounce alloc
      // exhausts). Q-A: tiny largest = fragmentation; small free = total.
      printf("prefetch: frames %d+%d read FAILED after 10 tries (%s) "
             "[heap internal free=%u KB largest=%u KB] — waiting for next clip\n",
             f_rel, want, rf.err,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
      pf.close();
      pf = File();
      g_pf_live = false;
      while (g_clip_gen == my_gen) vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    const uint32_t rd_end_now = micros();
    for (int k = 0; k < gotn; k++) {
      const int sl = (f_read + k) % kSlots;
      // S83: writer C2M before the handoff / any DMA read of the buffer.
      slotWriteback(g_pb[sl].mask, kMask1b);
      slotWriteback(g_pb[sl].jpg, gots[k]);
      g_pb[sl].jpg_len = gots[k];
      g_pb[sl].tag = (uint32_t)(f_read + k);
      g_pb[sl].rd_us = pf_rd_ff;       // S129: amortized (the handoff prints it, pfs rd=)
      g_pb[sl].batch = (uint8_t)gotn;  // S129: the pfs b= field
      g_pb[sl].in_use = true;
      g_pb[sl].rd_end = rd_end_now;    // S107: data-ready time (the tk deferral base)
    }
    f_read += gotn;  // S129: the whole batch (a shrink never loses frames — the next lap starts where this one stopped)
  }
}

static bool spawnPrefetch() {
  for (int s = 0; s < kSlots; s++) {
    if (!g_slot[s].mask)
      g_slot[s].mask = (uint8_t *)heap_caps_malloc(kMask1b,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_slot[s].jpg)
      g_slot[s].jpg = (uint8_t *)heap_caps_malloc(kMaxJpeg,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // S109: the prefetch's PRIVATE read ring is allocated HERE (idempotent)
    // — moved out of prefetchTask so the read task can re-arm in place
    // across clips without re-allocating (its File is per-clip; the ring
    // memory is for the life of the device).
    if (!g_pb[s].mask)
      g_pb[s].mask = (uint8_t *)heap_caps_malloc(kMask1b,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_pb[s].jpg)
      g_pb[s].jpg = (uint8_t *)heap_caps_malloc(kMaxJpeg,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // S129: the read task's BATCH-read scratch (once — not per slot),
    // TWO-TIER. Non-fatal: if null, readFramesBatch() falls back to the
    // legacy 2-seek single-frame path (g_pf_batch_max = 0).
    //   tier 1: kPfBatch frames = 985,088 B -> g_pf_batch_max = 4
    //   tier 2: 1 frame         = 246,272 B -> g_pf_batch_max = 1 (= S128 shape)
    if (s == 0 && !g_pf_scratch) {
      g_pf_scratch = (uint8_t *)heap_caps_malloc(kMask1b + (size_t)kPfBatch * kMaxJpeg,
                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (g_pf_scratch) {
        g_pf_batch_max = (uint8_t)kPfBatch;
      } else {
        g_pf_scratch = (uint8_t *)heap_caps_malloc(kMask1b + kMaxJpeg,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        g_pf_batch_max = g_pf_scratch ? 1 : 0;
      }
      printf("prefetch: S129 batch scratch %s — max %u frame(s)/transfer\n",
             g_pf_scratch ? "ok" : "FAILED (legacy 2-seek)", (unsigned)g_pf_batch_max);
    }
    if (!g_slot_ready[s]) g_slot_ready[s] = xSemaphoreCreateCounting(1, 0);
    // S92 lock fix (S90 NEXT #2, one line): (1,1) -> (2,1). With max=1 a
    // slot holds exactly ONE free credit; any path where the player does
    // not hand it straight back (the 50 ms ready TIMEOUT detour reads
    // inline WITHOUT giving g_slot_free) loses that credit and the ring
    // locks up to 4 frames behind the player — the 1-in-4 "stale slot
    // (tag i-4) — inline" pattern (S89/S90 logs, monitor15). max=2 banks
    // a spare credit so the prefetch keeps its 4 frames of headroom.
    if (!g_slot_free[s])  g_slot_free[s]  = xSemaphoreCreateCounting(2, 1);
  }
  for (int s = 0; s < kSlots; s++) {
    if (!g_slot[s].mask || !g_slot[s].jpg ||
        !g_pb[s].mask || !g_pb[s].jpg ||  // S109: the private ring too
        !g_slot_ready[s] || !g_slot_free[s]) {
      printf("prefetch: alloc FAILED — inline fallback\n");
      return false;
    }
  }
  g_pf_live = false;
  // S81: back to prio 1 (S79's proven value). S80's prio 0 = idle priority:
  // the task was created but NEVER scheduled on core 0 (no "prefetch: live"
  // print, g_pf_live never set, player stuck on inline reads: sd 21-27 ms
  // every frame, avg FT back to the pre-prefetch ~67 ms). The player's
  // critical path is on core 1, so a core-0 prefetch at prio 1 cannot
  // outrun it — the S80 contention concern is handled by the 4-slot ring.
  // S107: TWO tasks — the handoff task first so the first ring fill is
  // picked up on its next 5 ms poll, then the read task. Both prio 1
  // core 0 (unchanged scheduling class; total CPU is the same as S106).
  if (xTaskCreatePinnedToCore(pfHandTask, "rfav_pfhand", 8192, nullptr, 1,
                              nullptr, 0) != pdPASS ||
      xTaskCreatePinnedToCore(prefetchTask, "rfav_pfetch", 8192, nullptr, 1,
                              nullptr, 0) != pdPASS) {
    printf("prefetch: task create FAILED — inline fallback\n");
    return false;
  }
  return true;
}

// ============================================================================
// S94 E3 — SCREEN CAPTURE (ALL capture code; compile out: S94_CAPTURE = 0)
// ----------------------------------------------------------------------------
// Captures the draw buffer at two moments of each TRIGGER frame (trigger =
// one per 4-slot ring position, spaced 9 frames for settling — S94 user idea
// — plus a boot transient; capped at 40 frames/boot):
//   RAW  = right after the fb M2C invalidate, BEFORE the composite. Post-M2C
//          the CPU read is a MIXED reference (L1-resident bottom ~22 rows =
//          previous composite; the rest = fresh JDEC from chips).
//   COMP = right after the trailing fb C2M = the fully composited frame,
//          EXACTLY what the panel presents (what the user sees).
// Per moment: full-res raw RGB565 .cap (1,843,200 B) + a self-contained
// 180x320 PNG (4x box downsample, stored-deflate + adler32, NO zlib dep).
// COMP also writes the 1-bit mask expanded to an 8-bit gray BMP. Staged on
// core 1 (synchronous, tearing-free, same task as the composite; ~10-20 ms
// per trigger frame), drained by two core-0 tasks:
//   capTask    — PNG encode + SD writes to /rfab_cap (~300-500 ms/COMP)
//   capSerTask — COMP png -> base64 over serial (CAPB/CAPD/CAPE), SEPARATE so
//                the ~20 s b64 drain never blocks new captures.
// The SD is NOT PC-readable in this rig, so the serial b64 PNG is the primary
// path to the PC / a vision LLM (tools\s94_capcollect.py).
// S96: compiled out per user (evidence already extracted — see handoff S96:
// frame recovered from the one complete capture, composite verified, encoder
// CRC bug documented). Set to 1 to re-enable (fix the IDAT-CRC offset first).
#define S94_CAPTURE 0
#if S94_CAPTURE  // S129: #if (NOT #ifdef — #ifdef is true even for 0: the "disable" that never worked)
namespace s94cap {

constexpr int    kDsw      = 180;            // 720 / 4
constexpr int    kDsh      = 320;            // 1280 / 4
constexpr size_t kDsBytes  = (size_t)kDsw * kDsh * 3;   // 172,800 RGB24
constexpr size_t kMask8    = (size_t)kFW * kFH;          // 921,600 (BMP px)
constexpr size_t kPngCap   = 210000;         // measured PNG ~206,198 B
constexpr int    kStages   = 2;              // double-buffered staging
constexpr int    kSerBufs  = 4;              // capSerTask's own staging
constexpr int    kSerQueue = 16;             // max pending serial PNGs
constexpr int    kMaxFrames = 40;            // trigger frames / boot
constexpr int    kB64Chunk  = 1500;          // b64 chars per CAPD line

// ---------------- PNG (self-contained, no zlib dependency) ----------------
static uint32_t s_crctab[256];
static bool     s_crc_ready = false;
static void crcTab() {
  if (s_crc_ready) return;
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    s_crctab[i] = c;
  }
  s_crc_ready = true;
}
static inline uint32_t crcUpd(uint32_t c, const uint8_t *p, size_t n) {
  for (size_t k = 0; k < n; k++) c = s_crctab[(c ^ p[k]) & 0xFF] ^ (c >> 8);
  return c;
}
static inline uint32_t crcOf(const uint8_t *p, size_t n) {
  return crcUpd(0xFFFFFFFFu, p, n) ^ 0xFFFFFFFFu;
}
static void chunk(uint8_t *dst, size_t *off, const char *type,
                  const uint8_t *data, uint32_t len) {
  uint8_t hdr[8];
  hdr[0] = (uint8_t)(len >> 24); hdr[1] = (uint8_t)(len >> 16);
  hdr[2] = (uint8_t)(len >> 8);  hdr[3] = (uint8_t)len;
  memcpy(hdr + 4, type, 4);
  memcpy(dst + *off, hdr, 8); *off += 8;
  if (len) { memcpy(dst + *off, data, len); *off += len; }
  uint32_t c = crcOf(hdr + 4, 4);
  if (len) c = crcUpd(c, data, len);
  c ^= 0xFFFFFFFFu;
  uint8_t cb[4] = {(uint8_t)(c >> 24), (uint8_t)(c >> 16),
                   (uint8_t)(c >> 8), (uint8_t)c};
  memcpy(dst + *off, cb, 4); *off += 4;
}
// Encode ds (RGB24, kDsw x kDsh) into a self-contained PNG in `out` (>=
// kPngCap). zlib(0x78 0x01) + stored deflate blocks (BTYPE=00) + adler32.
static size_t pngEncode(const uint8_t *ds, uint8_t *out) {
  crcTab();
  size_t off = 0;
  const uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  memcpy(out, sig, 8); off = 8;
  uint8_t ihdr[13] = {};
  ihdr[0] = (uint8_t)(kDsw >> 24); ihdr[1] = (uint8_t)(kDsw >> 16);
  ihdr[2] = (uint8_t)(kDsw >> 8);  ihdr[3] = (uint8_t)kDsw;
  ihdr[4] = (uint8_t)(kDsh >> 24); ihdr[5] = (uint8_t)(kDsh >> 16);
  ihdr[6] = (uint8_t)(kDsh >> 8);  ihdr[7] = (uint8_t)kDsh;
  ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
  chunk(out, &off, "IHDR", ihdr, 13);
  const size_t rawsize = (size_t)(kDsw * 3 + 1) * kDsh;   // 173,120
  const size_t blkmax  = 65535;
  const size_t nblk    = (rawsize + blkmax - 1) / blkmax; // 3
  const size_t idatlen = 2 + nblk * 5 + rawsize + 4;      // 173,141
  uint8_t idh[8];
  idh[0] = (uint8_t)(idatlen >> 24); idh[1] = (uint8_t)(idatlen >> 16);
  idh[2] = (uint8_t)(idatlen >> 8);  idh[3] = (uint8_t)idatlen;
  memcpy(idh + 4, "IDAT", 4);
  memcpy(out + off, idh, 8); off += 8;
  uint8_t *z = out + off;
  z[0] = 0x78; z[1] = 0x01;
  size_t p = 2;
  uint32_t ad_a = 1, ad_b = 0;
  const uint32_t AMOD = 65521;
  int row = 0, rpos = 0;
  auto nextRaw = [&]() -> uint8_t {
    if (rpos == 0) { rpos = 1; return 0x00; }           // per-row filter byte
    const size_t idx = (size_t)row * kDsw * 3 + (rpos - 1);
    const uint8_t v = ds[idx];
    rpos++;
    if (rpos == kDsw * 3 + 1) { rpos = 0; row++; }      // next row
    return v;
  };
  auto emitRaw = [&](uint8_t b) {
    z[p++] = b;
    ad_a += b;    if (ad_a >= AMOD) ad_a -= AMOD;
    ad_b += ad_a; if (ad_b >= AMOD) ad_b -= AMOD;
  };
  for (size_t b = 0; b < nblk; b++) {
    size_t blen = rawsize - b * blkmax;
    if (blen > blkmax) blen = blkmax;
    z[p++] = (b == nblk - 1) ? 0x01 : 0x00;
    z[p++] = (uint8_t)(blen & 0xFF);
    z[p++] = (uint8_t)((blen >> 8) & 0xFF);
    const uint16_t nlen = (uint16_t)(~(uint32_t)blen & 0xFFFFu);
    z[p++] = (uint8_t)(nlen & 0xFF);
    z[p++] = (uint8_t)((nlen >> 8) & 0xFF);
    for (size_t k = 0; k < blen; k++) emitRaw(nextRaw());
  }
  const uint32_t adler = (ad_b << 16) | ad_a;
  z[p++] = (uint8_t)(adler & 0xFF);
  z[p++] = (uint8_t)((adler >> 8) & 0xFF);
  z[p++] = (uint8_t)((adler >> 16) & 0xFF);
  z[p++] = (uint8_t)((adler >> 24) & 0xFF);
  uint32_t c = crcOf(idh + 4, 4);
  c = crcUpd(c, z, idatlen);
  c ^= 0xFFFFFFFFu;
  uint8_t cb[4] = {(uint8_t)(c >> 24), (uint8_t)(c >> 16),
                   (uint8_t)(c >> 8), (uint8_t)c};
  memcpy(out + off, cb, 4);
  off += idatlen + 4;
  chunk(out, &off, "IEND", nullptr, 0);
  return off;
}

// ---------------- base64 ----------------
static const char kB64Tab[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static void b64enc(const uint8_t *in, size_t n, char *out) {
  size_t o = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t x = (uint32_t)in[i] << 16;
    if (i + 1 < n) x |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < n) x |= (uint32_t)in[i + 2];
    out[o++] = kB64Tab[(x >> 18) & 63];
    out[o++] = kB64Tab[(x >> 12) & 63];
    out[o++] = (i + 1 < n) ? kB64Tab[(x >> 6) & 63] : '=';
    out[o++] = (i + 2 < n) ? kB64Tab[x & 63] : '=';
  }
  out[o] = 0;
}

// ---------------- image helpers ----------------
// 1-bit mask (1bpp MSB-first, 90 B/row) -> 8-bit gray (720 B/row, 255/0).
static void maskExpand(const uint8_t *m1b, uint8_t *m8) {
  for (int y = 0; y < kFH; y++) {
    const uint8_t *mr = m1b + (size_t)y * (kFW / 8);
    uint8_t *o = m8 + (size_t)y * kFW;
    for (int x = 0; x < kFW; x++)
      o[x] = (mr[x >> 3] & (0x80u >> (x & 7))) ? 255 : 0;
  }
}
// 4x box downsample of the RGB565 fb -> kDsw x kDsh RGB24.
static void downsample(const uint16_t *src, uint8_t *dst) {
  for (int y = 0; y < kDsh; y++)
    for (int x = 0; x < kDsw; x++) {
      uint32_t r = 0, g = 0, b = 0;
      for (int dy = 0; dy < 4; dy++)
        for (int dx = 0; dx < 4; dx++) {
          const uint16_t px = src[(size_t)(y * 4 + dy) * kFW + (x * 4 + dx)];
          r += (px >> 11) & 31; g += (px >> 5) & 63; b += px & 31;
        }
      const uint32_t r5 = r >> 4, g6 = g >> 4, b5 = b >> 4;  // /16
      uint8_t *o = dst + ((size_t)y * kDsw + x) * 3;
      o[0] = (uint8_t)((r5 << 3) | (r5 >> 2));
      o[1] = (uint8_t)((g6 << 2) | (g6 >> 4));
      o[2] = (uint8_t)((b5 << 3) | (b5 >> 2));
    }
}
// S95: write in 32 KB chunks — the littlefs/SDMMC lock is released between
// chunks so the core-1 player / core-0 prefetch can interleave their SD
// reads (S94 freeze: single 1.84 MB writes stalled the whole player).
static size_t sdWriteChunked(File &f, const uint8_t *p, size_t n) {
  constexpr size_t kCh = 32 * 1024;
  size_t done = 0;
  while (done < n) {
    const size_t c = (n - done > kCh) ? kCh : (n - done);
    const size_t w = f.write(p + done, c);
    if (w != c) break;
    done += w;
  }
  return done;
}
// 8-bit gray BMP (720x1280, 54 B std header, no row pad), bottom-up rows.
static void writeMaskBmp(const uint8_t *m8, uint32_t frame) {
  const uint32_t px = (uint32_t)kMask8;
  const uint32_t off = 54, size = off + px;
  uint8_t hdr[54] = {};
  hdr[0] = 'B'; hdr[1] = 'M';
  hdr[2] = size & 0xFF; hdr[3] = (size >> 8) & 0xFF;
  hdr[4] = (size >> 16) & 0xFF; hdr[5] = (size >> 24) & 0xFF;
  hdr[10] = off & 0xFF; hdr[11] = (off >> 8) & 0xFF;
  hdr[12] = (off >> 16) & 0xFF; hdr[13] = (off >> 24) & 0xFF;
  hdr[14] = 40;                       // DIB header size
  hdr[18] = kFW & 0xFF; hdr[19] = (kFW >> 8) & 0xFF;
  hdr[20] = kFH & 0xFF; hdr[21] = (kFH >> 8) & 0xFF;
  hdr[26] = 1;                        // planes
  hdr[28] = 8;                        // bpp
  hdr[34] = px & 0xFF; hdr[35] = (px >> 8) & 0xFF;
  hdr[36] = (px >> 16) & 0xFF; hdr[37] = (px >> 24) & 0xFF;
  char name[24];
  snprintf(name, sizeof(name), "/rfab_cap/c%03u_comp_mask.bmp", frame % 1000);
  File f = SD_MMC.open(name, "w");
  if (!f) return;
  f.write(hdr, 54);
  // S95: 64-row batches (46,080 B) instead of 1,280 single-row writes.
  for (int y = kFH - 1; y >= 0; y -= 64) {
    const int rows = (y + 1 >= 64) ? 64 : (y + 1);
    f.write(m8 + (size_t)(y - rows + 1) * kFW, (size_t)rows * kFW);
  }
  f.close();
}

// ---------------- staging + serial queue ----------------
struct Stage {
  uint8_t *full  = nullptr;  // kFbBytes raw RGB565
  uint8_t *ds    = nullptr;  // kDsBytes RGB24
  uint8_t *mask8 = nullptr;  // kMask8 (COMP only)
  uint8_t *png   = nullptr;  // kPngCap
  volatile bool pending = false;
  volatile bool hasComp = false;
  uint32_t frame = 0;
  bool hit = false;
};
Stage g_st[kStages];
volatile int  g_capcount = 0;    // trigger frames captured this boot
volatile bool g_capok    = false;

uint8_t     *g_ser[kSerBufs] = {};
volatile bool g_serbusy[kSerBufs] = {};
struct SerItem { int buf; uint32_t frame; bool hit; uint32_t len; };
SerItem      g_serq[kSerQueue];
volatile int g_serq_head = 0, g_serq_tail = 0;
SemaphoreHandle_t g_serq_mtx = nullptr;
volatile uint32_t g_serid = 0;

static bool serEnqueue(const uint8_t *png, size_t len, uint32_t frame, bool hit) {
  if (!g_serq_mtx || len > kPngCap) return false;
  xSemaphoreTake(g_serq_mtx, portMAX_DELAY);
  const int used = (g_serq_tail - g_serq_head + kSerQueue) % kSerQueue;
  if (used >= kSerQueue) { xSemaphoreGive(g_serq_mtx); return false; }
  int buf = -1;
  for (int k = 0; k < kSerBufs; k++) if (!g_serbusy[k]) { buf = k; break; }
  if (buf < 0) { xSemaphoreGive(g_serq_mtx); return false; }
  memcpy(g_ser[buf], png, len);
  g_serbusy[buf] = true;
  SerItem *it = &g_serq[g_serq_tail % kSerQueue];
  it->buf = buf; it->frame = frame; it->hit = hit; it->len = (uint32_t)len;
  g_serq_tail++;
  xSemaphoreGive(g_serq_mtx);
  return true;
}

// Core-0 drain: PNG encode + SD writes for each staged capture.
static void capTask(void *) {
  for (;;) {
    int s = -1;
    for (int k = 0; k < kStages; k++) if (g_st[k].pending) { s = k; break; }
    if (s < 0) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
    Stage &st = g_st[s];
    if (st.hasComp) {
      char name[24];
      snprintf(name, sizeof(name), "/rfab_cap/c%03u_comp.cap", st.frame % 1000);
      File f = SD_MMC.open(name, "w");
      if (f) { sdWriteChunked(f, st.full, kFbBytes); f.close(); }  // S95
      const size_t pngsz = pngEncode(st.ds, st.png);
      snprintf(name, sizeof(name), "/rfab_cap/c%03u_comp.png", st.frame % 1000);
      File p = SD_MMC.open(name, "w");
      if (p) { sdWriteChunked(p, st.png, pngsz); p.close(); }  // S95
      writeMaskBmp(st.mask8, st.frame);
      serEnqueue(st.png, pngsz, st.frame, st.hit);
    } else {
      char name[24];
      snprintf(name, sizeof(name), "/rfab_cap/c%03u_raw.cap", st.frame % 1000);
      File f = SD_MMC.open(name, "w");
      if (f) { sdWriteChunked(f, st.full, kFbBytes); f.close(); }  // S95
    }
    st.pending = false;
  }
}

// Core-0 serial: COMP png -> base64 (CAPB/CAPD/CAPE). Separate task so the
// ~20 s drain never blocks capTask / new captures.
static void capSerTask(void *) {
  char b64[kB64Chunk + 16];
  for (;;) {
    SerItem it; bool have = false;
    if (g_serq_mtx) {
      xSemaphoreTake(g_serq_mtx, pdMS_TO_TICKS(50));
      if (g_serq_head != g_serq_tail) {
        it = g_serq[g_serq_head % kSerQueue];
        g_serq_head++;
        have = true;
      }
      xSemaphoreGive(g_serq_mtx);
    }
    if (!have) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
    const uint8_t *png = g_ser[it.buf];
    const size_t len = it.len;
    const uint32_t id = ++g_serid;
    const size_t bytesPerChunk = (size_t)(kB64Chunk / 4) * 3;   // 1125 (x3)
    printf("CAPB id=%u i=%u t=%c w=%d h=%d n=%u\n", (unsigned)id,
           (unsigned)it.frame, it.hit ? 'H' : 'I', kDsw, kDsh, (unsigned)len);
    size_t sent = 0; uint32_t seq = 0;
    while (sent < len) {
      size_t cb = len - sent;
      if (cb > bytesPerChunk) cb = bytesPerChunk;
      b64enc(png + sent, cb, b64);
      printf("CAPD %u %s\n", (unsigned)seq, b64);
      sent += cb; seq++;
    }
    printf("CAPE id=%u\n", (unsigned)id);
    g_serbusy[it.buf] = false;
  }
}

// Core 1 (called from makeFrame): stage the current fb (+ mask for COMP).
static void capStage(int i, bool is_hit, bool is_comp, const uint8_t *m_buf) {
  int s = -1;
  for (int k = 0; k < kStages; k++) if (!g_st[k].pending) { s = k; break; }
  if (s < 0) return;                          // both busy — skip (diag ok)
  Stage &st = g_st[s];
  memcpy(st.full, fb, kFbBytes);
  downsample((const uint16_t *)fb, st.ds);
  st.hasComp = is_comp;
  if (is_comp && m_buf) maskExpand(m_buf, st.mask8);
  st.frame = (uint32_t)i;
  st.hit = is_hit;
  st.pending = true;
}

// Called once from setup() after the player task starts.
static void capInit() {
  g_serq_mtx = xSemaphoreCreateMutex();
  if (!g_sd_ok) { printf("S94 capture: SD not mounted\n"); return; }
  if (!SD_MMC.exists("/rfab_cap")) SD_MMC.mkdir("/rfab_cap");
  for (int s = 0; s < kStages; s++) {
    g_st[s].full  = (uint8_t *)heap_caps_malloc(kFbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_st[s].ds    = (uint8_t *)heap_caps_malloc(kDsBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_st[s].mask8 = (uint8_t *)heap_caps_malloc(kMask8,   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_st[s].png   = (uint8_t *)heap_caps_malloc(kPngCap,  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_st[s].full || !g_st[s].ds || !g_st[s].mask8 || !g_st[s].png) {
      printf("S94 capture: INIT FAIL (staging alloc)\n"); return;
    }
  }
  for (int b = 0; b < kSerBufs; b++) {
    g_ser[b] = (uint8_t *)heap_caps_malloc(kPngCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_ser[b]) { printf("S94 capture: INIT FAIL (ser alloc)\n"); return; }
  }
  // S95 fix: prio 1 -> 0 (IDLE) — the S94 freeze: capTask's multi-MB SD
  // writes + PNG encodes time-sliced with the prefetch (prio 1, core 0) and
  // held the littlefs lock -> every fill STALLed (pfs tr 126-184 ms) and
  // the player's inline reads slowed to 74-381 ms -> 0-1 fps. Prio 0 lets
  // the player (core 1) and prefetch (core 0, prio 1) always preempt.
  if (xTaskCreatePinnedToCore(capTask, "rfab_cap", 8192, nullptr, 0, nullptr, 0) != pdPASS ||
      xTaskCreatePinnedToCore(capSerTask, "rfab_capser", 12288, nullptr, 0, nullptr, 0) != pdPASS) {
    printf("S94 capture: INIT FAIL (task)\n"); return;
  }
  g_capok = true;
  printf("S94 capture: ON (2-stage x[full+ds+mask8+png], 4 ser, /rfab_cap)\n");
}

}  // namespace s94cap
#endif  // S94_CAPTURE

// Read + decode + composite frame i into the draw buffer.
// False on SD/decode failure — the panel keeps the last good frame.
static bool makeFrame(int i) {
  const uint32_t t0 = micros();
  uint8_t  *m_buf = mask_buf;
  uint8_t  *j_buf = jpg_in;
  uint32_t  jpg_got = 0;
  bool      used_slot = false;
  bool      slot_given  = false;  // S105: slot credit already returned early
#if S94_CAPTURE
  // S94 E3: capture trigger for this frame (S129: decl moved INSIDE the gate
  // — with #if 0 the whole block is compiled out, no unused var). One per
  // 4-slot ring position, spaced 9 frames for settling (S94 user idea) +
  // boot transient; capped at 40 frames/boot.
  bool s94trig = (s94cap::g_capok && s94cap::g_capcount < s94cap::kMaxFrames) &&
                 ((i < 8 && i % 2 == 0) || i % 32 == 0 || i % 32 == 9 ||
                  i % 32 == 18 || i % 32 == 27);
  if (s94trig) s94cap::g_capcount++;
#endif

  if (g_pf_live) {
    const int slot = i % kSlots;  // S102: % (kSlots-1 mask needs power-of-2; kSlots=6)
    // S103: 300 -> 50 ms + FAST DROP — a miss drops the frame (S89 policy)
    // instead of waiting 300 ms (that stretched every stall into a 300+ ms
    // frame — still over budget, 250 ms slower recovery). The inline SD read
    // below now runs ONLY when !g_pf_live (prefetch dead) — the player never
    // holds the littlefs lock while the prefetch is alive (breaks the S96/S101
    // self-sustaining stall loop).
    if (xSemaphoreTake(g_slot_ready[slot], pdMS_TO_TICKS(50)) == pdPASS) {
      if (g_slot[slot].frame == (uint32_t)i) {
        // S105: pull the slot's data into the PRIVATE buffers (jpg_in /
        // mask_buf — the same ones the inline path uses; ~2 ms PSRAM copy)
        // and hand the slot credit BACK IMMEDIATELY. Holding the slot
        // through the ~30 ms decode+composite delayed every refill credit
        // (the prefetch's next fill of this slot starts only on that give)
        // — with a 1-frame headroom that pushed the fill to land on/after
        // the player's next need = the 1-in-3 stale race (tag i-6). Early
        // release gives every refill ~480 ms of runway. S83 cache note
        // intact: the prefetch C2M'd the slot before the ready-give, and
        // the memcpy->decode/composite chain is CPU->CPU (L2-coherent).
        jpg_got = g_slot[slot].jpg_len;
        memcpy(jpg_in, g_slot[slot].jpg, jpg_got);
        memcpy(mask_buf, g_slot[slot].mask, kMask1b);
        xSemaphoreGive(g_slot_free[slot]);  // S105: early release
        slot_given = true;
        m_buf = mask_buf;
        j_buf = jpg_in;
        used_slot = true;
        g_miss_run = 0;
      } else {
        // S103: STALE fill (≥500 ms time-jump) — S89 policy: DROP. Release
        // the slot so the ring keeps turning.
        const uint32_t bank_pre =
            uxSemaphoreGetCount(g_slot_free[slot]);  // S108: BEFORE release
        xSemaphoreGive(g_slot_free[slot]);
        // S108 FIX (phase nudge): monitor28 proved the slot-0 handoff chain
        // runs ONE RELEASE BEHIND the player grid — every H(6k) is served
        // by R(6k), the release of the very take it missed (stale age
        // 481-502 ms = 500 - ~15 ms landing; bank_pre 0; hof wait 300-550 ms
        // per lap): a self-perpetuating phase set by the boot transient.
        // Bank one EXTRA credit on the stale: H(6k) takes R(6k) (<=5 ms),
        // then H(6k+6) takes the nudge immediately (the ring is full — the
        // frame is already there), so tag 6k+6 lands ~15 ms AFTER the stale
        // = ~485 ms BEFORE take(6k+6) -> HIT, and the chain settles in-sync
        // (each later H consumes the release of the take 6 frames earlier,
        // 490 ms ahead). Bank max=2 absorbs the nudge; the tag check keeps
        // every handoff correct. Self-healing: any future transient (boot,
        // park/rewind) that re-slips the phase is corrected by the next
        // stale.
        xSemaphoreGive(g_slot_free[slot]);
        g_miss_run++;
        // S108: age = how long the sitting tag has been ready-stamped.
        // ~485 ms = the handoff chain runs one release BEHIND the grid
        // (each slot-0 handoff consumes the credit of the take it missed);
        // ~985 ms = two; ~0-20 ms = a handoff just landed and the next is
        // in flight (see 'hof' for its wait). bank_pre 0 = all credits
        // already spent; 1 = a credit was sitting (handoff waited on the
        // ring, not the credit).
        {
          const int64_t age_us =
              g_slot_ho_t64[slot]
                  ? (esp_timer_get_time() - g_slot_ho_t64[slot])
                  : -1;
          printf("frame %d: stale slot (tag %u) — drop S108 age=%lu ms "
                 "bank=%u\n",
                 i, (unsigned)g_slot[slot].frame,
                 (unsigned long)(age_us < 0 ? 999999 : age_us / 1000),
                 bank_pre);
        }
        return false;
      }
    } else {
      // S105: timeout — DROP. Return the slot credit UNCONDITIONALLY. With
      // the S105 early release the player holds no slot data past the
      // makeFrame copy, so an at-most-1 extra banked credit (bank max 2,
      // S92) only lets the prefetch run one fill deeper; the tag check in
      // the ready path keeps every handoff correct. The old count>0 gate
      // stranded the credit whenever the prefetch owned it mid-fill.
      const uint32_t bank_pre =
          uxSemaphoreGetCount(g_slot_free[slot]);  // S108: BEFORE release
      xSemaphoreGive(g_slot_free[slot]);
      g_miss_run++;
      // S108: ready sem was EMPTY — the handoff for THIS frame has not
      // landed at all; age = when the slot was last stamped.
      {
        const int64_t age_us =
            g_slot_ho_t64[slot]
                ? (esp_timer_get_time() - g_slot_ho_t64[slot])
                : -1;
        printf("frame %d: prefetch timeout — drop S108 tag=%u age=%lu ms "
               "bank=%u\n",
               i, (unsigned)g_slot[slot].frame,
               (unsigned long)(age_us < 0 ? 999999 : age_us / 1000),
               bank_pre);
      }
      return false;
    }
  }
  if (!used_slot) {
    if (!rf.readFrame(rf_file, i % (int)rf.frame_count, mask_buf, jpg_in,
                      &jpg_got, g_inline_scratch)) {  // S128: combined read (nullptr = legacy)
      g_sd_fail++;
      printf("frame %d: SD read FAILED (%s)\n", i, rf.err);
      return false;
    }
  }
  const uint32_t sd_us = micros() - t0;   // ~0 when prefetched (slot wait only)

  const uint32_t t1 = micros();
  uint32_t out_size = 0;
  // S101 FIX A: portrait-native decode into the SCRATCH fr_fb — never into
  // the presented fb (un-drained fb rows then show last-frame composite,
  // not JDEC-raw cyan). rfab_poc_waitpresent() in playerTask gates the
  // composite rewrite against the previous present's DMA2D copy.
  const esp_err_t e = jpeg_decoder_process(jdec, &jdec_cfg, j_buf, jpg_got,
                                           (uint8_t *)fr_fb, (uint32_t)kFbBytes,
                                           &out_size);
  const uint32_t dec_us = micros() - t1;
  if (e != ESP_OK || out_size != kFbBytes) {
    g_decode_fail++;
    printf("frame %d: JPEG decode FAILED e=%d out=%lu (want %u)\n",
                  i, (int)e, (unsigned long)out_size, (unsigned)kFbBytes);
    if (used_slot && !slot_given) xSemaphoreGive(g_slot_free[i % kSlots]);  // S105
    return false;
  }

  // S101 FIX A (S83 discipline): M2C fr_fb — kill its stale L2 lines BEFORE
  // the composite CPU-reads the decoded rows (JDEC wrote PSRAM-direct,
  // bypassing L2). Then the standing fb invalidate: kill VALID-STALE fb
  // lines (previous frame's composite + chip) so no stale L2 line survives
  // into the frame — S81's ghosting / alpha holes / blue flash. The
  // composite repopulates only the lines it writes (fetching fresh chips
  // data on a miss), and the trailing C2M below writes back exactly those.
  const uint32_t s_iv = micros();
  slotInvalidate(fr_fb, kFbBytes);
  slotInvalidate(fb, kFbBytes);
#if S94_CAPTURE
  if (s94trig) s94cap::capStage(i, used_slot, false, m_buf);  // RAW (pre-composite)
#endif
  const uint32_t s_c = micros();
  // S100 (model M): PER-BATCH C2M, NO OVERLAP — composite 32 rows
  // (46,080 B = 128 B x 360, 128 B-aligned), then C2M exactly that batch.
  // S99's 32-row overlap re-queued ~1.8 MB/frame of duplicate drain; the
  // trailing full C2M below (chunked) stays as the safety net (S103: the
  // convergence gate after it was REMOVED — verification done, cyan gone).
  for (int b = 0; b < kFH / 32; b++) {
    const int y0 = b * 32;
    compositeMaskRows(m_buf + (size_t)y0 * (kFW / 8), fb + (size_t)y0 * kFW,
                      bg_buf + (size_t)y0 * kFW, fr_fb + (size_t)y0 * kFW,
                      kFW, y0, 32);
    syncTracked(fb + (size_t)y0 * kFW, (size_t)32 * kFW * 2,
                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  }
  const uint32_t s_cend = micros();
  // S103: mchk + convergence gate REMOVED (verification done — user confirmed
  // cyan gone S101 + S102; saves ~5 ms/frame of L1 churn + row readbacks).
  if (used_slot && !slot_given) xSemaphoreGive(g_slot_free[i % kSlots]);  // S105: skip if released early
  const uint32_t s_c2m = micros();

  // CPU writes -> PSRAM cache: make them visible to the DMA2D master before
  // the present (same C2M discipline M5GFX applies to its own writes).
  // S99: chunked (was a single 1.84 MB range op — the coverage suspect).
  syncTracked(fb, kFbBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  g_last_c2m_us = micros() - s_c2m;  // S103: trailing C2M only (gate removed)
#if S94_CAPTURE
  if (s94trig) s94cap::capStage(i, used_slot, true, m_buf);  // COMP (= presented)
#endif

  // S89: per-stage audit fields (µs) — the 'tp' stage is gone (portrait).
  g_last_sd_us  = sd_us;
  g_last_dec_us = dec_us;
  g_last_iv_us  = s_c - s_iv;
  g_last_c_us   = s_cend - s_c;  // S103: composite + per-batch C2M only (mchk+gate removed)
  return true;
}

static char s_status[192];

// Diagnostic chip (top-left) + present. The 4-arg display(0,0,W,H) is the
// unambiguous "present" for the patched panel (DMA2D -> non-scanned fb + swap).
// FT = this frame's work time; AVG FT = mean frame-work time since playback
// start (how much headroom is left against the per-frame budget — the app
// also runs other work, so the AVERAGE is the planning number).
static void presentFrame(int idx) {
  M5GFX &lcd = M5.Display;
  const uint32_t ft_ms = g_last_ft_us / 1000;
  const uint32_t avg_ft_ms =
      g_played ? (uint32_t)(g_ft_sum_us / 1000 / g_played) : 0;
  // S109: the clip name is on the chip (first field); dropped the "ms"
  // suffixes so the line stays < 720 px at font 1.
  snprintf(s_status, sizeof(s_status),
           "rfav3 %s f %u/%u %u fps FT %lu/%lu avg %lu/%lu (%lu left) drop %u",
           g_clip_name,
           idx + 1, rf.frame_count, g_win_fps,
           (unsigned long)ft_ms, (unsigned long)kBudgetMs,
           (unsigned long)avg_ft_ms, (unsigned long)kBudgetMs,
           (unsigned long)(kBudgetMs > avg_ft_ms ? kBudgetMs - avg_ft_ms : 0),
           g_dropped);
  // S89: the FT chip is the "UI" — drawn in the kUiRot window, then back to
  // 0 (user: setRotation(1) before the text, setRotation(0) after; swap
  // kUiRot 1<->3 if the orientation reads wrong). M5GFX's display() auto-
  // C2M's the dirty region before the DMA2D blit, so the text is visible.
  lcd.setRotation(kUiRot);
  // S119: taller bar (2 lines) — the uiFlash line at y=40 lives inside it,
  // so an expired flash never leaves stale line-2 text behind.
  lcd.fillRect(4, 4, 700, 56, lcd.color565(10, 12, 18));
  lcd.setTextColor(lcd.color565(160, 220, 170));
  lcd.drawString(s_status, 12, 12);
  if (s_flash_until_ms > millis()) {
    lcd.setTextColor(lcd.color565(255, 210, 120));
    lcd.drawString(s_flash, 12, 40);
  }
  lcd.setRotation(0);
  lcd.display(0, 0, kBw, kBh);
}

// ------------------------------ player ------------------------------

// ---------------- S122: DATA-FILE ANIMATION FSM + DIRECTOR -----------------
// The FSM + director are DATA: the active avatar (av::active()) =
// /avatars/<model>/avatar.json (v1.0.0, parsed + validated at boot by
// avatar.cpp) or the built-in Elena (the S121 tables verbatim). intro plays
// once per session, idle loops, one-shot gestures interrupt idle EARLY via
// the '*' transitions, a gaze runs its atomic in/hold/out chain, and the
// terminal state (outro) ends the session -> the show restarts at the
// avatar's initial state (a new session).
//
// Events are SCRIPTED (the avatar's `script` rows, fired by dirTick) — and
// the same strings can be posted EXTERNALLY (the WiFi queue, later) via
// rfav_fsm_queueEvent(); external events are drained first, ahead of the
// script. One entry point: fsmEvent() walks the avatar's transition table in
// FILE ORDER (first match wins — the JSON row order is authoritative, e.g.
// the _in.finished still_held row precedes the released row).
//
// Row due rules (S121 semantics, data-driven):
//   - row WITHOUT after_state: due after `dwell_ms` spent in IDLE (the
//     early interrupt).
//   - row WITH after_state = A: due `dwell_ms` after the engine ENTERED A
//     (g_row_anchor_ms, stamped in enterClipForState) — the release rows
//     fire once the press->release span elapses (S121's
//     lookReleaseDueNow() semantics), from whichever chain state accepts
//     them; speech/think.end fire from their looping anchor states. A
//     rejected row retries next tick and only advances when the FSM
//     ACCEPTS the event.

// ---- FSM state (owned by the player task — single writer) ----
// uint8_t = state index into av::active()->states (av::kStateN = none;
// setup() stamps the avatar's initial state before the player task starts).
uint8_t  g_fsm_state    = av::kStateN;
uint32_t g_fsm_enter_ms = 0;

// ---- director (the active avatar's script rows) ----
uint32_t g_dir_session   = 0;   // show runs complete (boot = session 0)
uint32_t g_dir_step      = 0;   // script index (advances when an event FIRES)
uint32_t g_gaze_press_ms = 0;   // millis() of the FIRED ".pressed" (0 = not held)
uint32_t g_row_anchor_ms = 0;   // millis() the engine entered the ACTIVE ROW's
                                // after_state (0 = not yet) — the due clock for
                                // after_state rows (replaces S112's
                                // g_look_press_ms/g_speech_start_ms/g_think_start_ms)

// External event queue — the WiFi hook. Any task may post an event string
// (the avatar.json vocabulary: "play:gun", "look_left.pressed", "speech.end",
// "outro", "cancel", ...); dirTick() drains it FIRST, ahead of the script,
// so an external source can preempt the show.
static QueueHandle_t s_evq = nullptr;
bool rfav_fsm_queueEvent(const char *ev) {
  if (!s_evq) s_evq = xQueueCreate(8, 32);
  if (!s_evq || !ev) return false;
  char buf[32] = {0};
  snprintf(buf, sizeof(buf), "%s", ev);
  return xQueueSend(s_evq, buf, 0) == pdTRUE;
}

static bool endsWith(const char *s, const char *suf) {
  const size_t ls = strlen(s), lf = strlen(suf);
  return ls >= lf && strcmp(s + ls - lf, suf) == 0;
}

// held = a ".pressed" event has fired and no ".released" has since (the
// _in.finished still_held/released guards).
static inline bool gazeHeld() { return g_gaze_press_ms != 0; }

// <name>.finished at clip end for auto_return one-shots + the initial state
// (on_finish) — null = looping states / event-exit states / terminal.
// (S114 carry: the gaze _in stages DO finish — held ? hold : _out.)
static const char *stateFinishedEvent(const av::AvatarDef *avd, uint8_t s,
                                      char *out, size_t cap) {
  const av::State &st = avd->states[s];
  if (!st.auto_return && !st.on_finish[0]) return nullptr;
  snprintf(out, cap, "%s.finished", st.name);
  return out;
}

// The transition engine (avatar.json `transitions`, walked in FILE ORDER —
// first match wins). Guards: "" = none; "clip_end" = the current clip is
// exhausted; "not_terminal" = the current state is not terminal; a guard
// ending in ".still_held"/".released" = the gaze is (no longer) held;
// anything else = never fires. "*" = any state except the initial, the
// terminal, and the transition's own target.
static bool fsmEvent(uint8_t cur, const char *ev, bool clip_end, bool held,
                     uint8_t &nxt) {
  nxt = cur;
  if (!ev) return false;
  const av::AvatarDef *avd = av::active();
  for (uint8_t t = 0; t < avd->nTrans; t++) {
    const av::Trans &tr = avd->trans[t];
    if (tr.from != cur &&
        !(tr.from == av::kAnyFrom && cur != avd->initial &&
          !avd->states[cur].terminal && tr.to != cur))
      continue;
    if (strcmp(tr.event, ev) != 0) continue;
    if (tr.guard[0]) {
      if (strcmp(tr.guard, "clip_end") == 0) {
        if (!clip_end) continue;
      } else if (strcmp(tr.guard, "not_terminal") == 0) {
        if (avd->states[cur].terminal) continue;
      } else if (endsWith(tr.guard, ".still_held")) {
        if (!held) continue;
      } else if (endsWith(tr.guard, ".released")) {
        if (held) continue;
      } else {
        continue;  // unknown guard — never fires
      }
    }
    nxt = tr.to;
    return true;
  }
  return false;
}

// ---- clip switching (player-task context only) ----
// S122: the clip path is DERIVED from the active avatar's name (spec layout
// /avatars/<name>/rfav/<clip>.rfav). g_clip_name points into the avatar's
// state table (static lifetime — the built-in def or the boot-loaded SD def).
static void setClipByName(const char *rfav) {
  // S124: spec clip values are STEMs — the firmware appends .rfav (the same
  // rule as avatar.cpp checkClipCoverage's "/avatars/<name>/rfav/<clip>.rfav").
  // A value already ending in .rfav is used as-is, so a hand-edited JSON with
  // full names keeps working.
  const size_t n = strlen(rfav);
  const bool hasExt = n >= 5 && strcmp(rfav + n - 5, ".rfav") == 0;
  snprintf(g_clip_path, sizeof(g_clip_path), "/avatars/%s/rfav/%s%s",
           av::activeName(), rfav, hasExt ? "" : ".rfav");
  g_clip_name = rfav;
  g_open_fails = 0;
}

// The single "the FSM changed state" entry: point the player (and the audio
// engine, via g_clip_gen) at the new state's clip. The rescan branch opens
// it immediately (g_clip_switch_pending).
static void enterClipForState(uint8_t s) {
  const av::AvatarDef *avd = av::active();
  g_fsm_state = s;
  g_fsm_enter_ms = millis();
  // S122: stamp the ACTIVE ROW's anchor entry — the due clock for its
  // after_state dwell (the press tick for the gaze release rows; the entry
  // into speaking/thinking for the .end rows).
  if (g_dir_step < avd->nRows) {
    const av::Row &ar = avd->rows[g_dir_step];
    if (ar.after != av::kNoState && ar.after == (int8_t)s) g_row_anchor_ms = millis();
  }
  setClipByName(avd->states[s].clip);
  if (rf_file) { rf_file.close(); rf_file = File(); }
  rf.ok = false;
  g_clip_gen++;               // re-arm prefetch/handoff (S109) + the audio engine
  g_clip_switch_pending = true;
  g_clip_first_pres_us = 0;   // audio starts 34 ms after this clip's first present
  printf("fsm: -> %s (gen %lu)\n", avd->states[s].clip, (unsigned long)g_clip_gen);
}

// S122: bookkeeping for a FIRED event (script AND ext paths): gaze press
// tracking (replaces S112's g_look_press_ms; the speech/think start
// timestamps are gone — the anchor rule is timestamp-based).
static void eventFiredBookkeeping(const char *ev) {
  if (endsWith(ev, ".pressed"))       g_gaze_press_ms = millis();
  else if (endsWith(ev, ".released")) g_gaze_press_ms = 0;
}

// ---- director tick (runs in the player task, ~12 Hz) ----
// Returns the new state index when an event FIRES (av::kStateN = nothing to do).
static uint8_t dirTick() {
  const av::AvatarDef *avd = av::active();
  // external events first (the WiFi hook) — they bypass the script dwell
  if (s_evq) {
    char ev[32] = {0};
    if (xQueueReceive(s_evq, ev, 0) == pdTRUE) {
      uint8_t nxt = g_fsm_state;
      if (fsmEvent(g_fsm_state, ev, /*clip_end=*/false, gazeHeld(), nxt)) {
        eventFiredBookkeeping(ev);
        printf("fsm: [ext] --[%s]--> %s\n", ev, avd->states[nxt].clip);
        return nxt;
      }
      printf("fsm: [ext] event '%s' rejected in %s\n", ev, avd->states[g_fsm_state].clip);
    }
  }
  if (g_dir_step >= avd->nRows) return av::kStateN;  // script done; the terminal clip-end restarts
  const av::Row &r = avd->rows[g_dir_step];
  const uint32_t now = millis();
  bool due;
  if (r.after != av::kNoState) {
    // due `dwell_ms` after ENTERING the row's anchor — the release rows fire
    // once the press->release span elapses (S121's lookReleaseDueNow()
    // semantics), from whichever chain state accepts them
    due = g_row_anchor_ms != 0 && (int32_t)(now - g_row_anchor_ms) >= (int32_t)r.dwell_ms;
  } else {
    // no after_state — min dwell in IDLE (the early interrupt)
    due = g_fsm_state == avd->idle && (int32_t)(now - g_fsm_enter_ms) >= (int32_t)r.dwell_ms;
  }
  if (!due) return av::kStateN;
  uint8_t nxt = g_fsm_state;
  if (!fsmEvent(g_fsm_state, r.event, /*clip_end=*/false, gazeHeld(), nxt))
    return av::kStateN;  // FSM says no right now — retry next tick
  g_dir_step++;  // advance only when the transition actually fires
  eventFiredBookkeeping(r.event);
  printf("fsm: --[%s]--> %s (dir %lu/%u, session %lu)\n", r.event,
         avd->states[nxt].clip, (unsigned long)g_dir_step, (unsigned)avd->nRows,
         (unsigned long)g_dir_session);
  return nxt;
}

static bool ensureSd() {
  if (g_sd_ok) return true;
  g_sd_ok = SD_MMC.setPins(43, 44, 39, 40, 41, 42) && SD_MMC.begin();
  printf("sd: %s\n", g_sd_ok ? "ok" : "NOT AVAILABLE");
  return g_sd_ok;
}

static bool tryOpenRfa() {
  if (rf_file) { rf_file.close(); rf_file = File(); }
  rf_file = SD_MMC.open(g_clip_path);  // S109: the current playlist clip
  if (!rf_file) {
    snprintf(rf.err, sizeof(rf.err), "open(%s) failed", g_clip_path);
    g_open_fails++;  // S109: the rescan branch skips the clip after 3 fails
    return false;
  }
  if (!rf.open(rf_file)) {
    rf_file.close(); rf_file = File();
    g_open_fails++;  // S109: a bad header counts as a fail too
    return false;
  }
  // (Re)allocate per-file buffers from the parsed header.
  const size_t mb = rf.maskBytes();
  if (!mask_buf || mb != kMask1b) {
    if (mask_buf) heap_caps_free(mask_buf);
    mask_buf = (uint8_t *)heap_caps_malloc(mb, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!jpg_in) {
    jpeg_decode_memory_alloc_cfg_t in_cfg = {};
    in_cfg.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER;
    jpg_in = (uint8_t *)jpeg_alloc_decoder_mem(kMaxJpeg, &in_cfg, &jpg_in_alloc);
  }
  if (!g_inline_scratch) {  // S128: combined-read scratch (player inline path)
    g_inline_scratch = (uint8_t *)heap_caps_malloc(kMask1b + kMaxJpeg,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (!mask_buf || !jpg_in) {
    snprintf(rf.err, sizeof(rf.err), "buffer alloc failed");
    return false;
  }

  kBudgetMs = 1000UL / (rf.fps ? rf.fps : 12);   // 12 fps -> 83 ms budget
  // S89: the portrait-native path assumes 720x1280 assets — a landscape file
  // has the SAME pixel count and would decode "fine" but rotated; warn.
  if (rf.width != kFW || rf.height != kBh)
    printf("rfav3: WARNING — %s is %ux%u; S89 expects %dx%d portrait\n",
           g_clip_path, rf.width, rf.height, kFW, kBh);
  printf("rfav3: %s ok — %u frames @%u fps %dx%d flags=0x%02x (loop=%d) budget %lu ms\n",
                g_clip_path, rf.frame_count, rf.fps, rf.width, rf.height, rf.flags,
                (int)rf.loopFlag(), (unsigned long)kBudgetMs);
  g_open_fails = 0;            // S109: this clip opens fine
  g_rf_armed_gen = g_clip_gen;  // S109 HANDSHAKE: the frame table is valid for
                                // this clip — the prefetch may re-arm on it
  return true;
}

static void playerTask(void *) {
  M5GFX &lcd = M5.Display;
  memset(g_bgst, 0, sizeof(g_bgst));  // S103: fb redraw outside makeFrame
  lcd.fillScreen(lcd.color565(10, 12, 18));
  lcd.display(0, 0, kBw, kBh);

  // S80 jitter fix — FIXED-GRID PRESENT KICK.
  // ideal[i] = t0 + i*frame_us (64-bit µs clock, 12.000 fps exactly — no
  // integer-ms drift like 1000/12=83) is the moment frame i SHOULD become
  // visible. Work (gate+SD+decode+composite+msync) runs as early as possible
  // after the previous present; its ±10 ms variance is absorbed by the slack
  // before the kick. The present is kicked at ideal[i] - kPreUs so the DMA2D
  // copy completes a few ms BEFORE ideal[i]; the panel then picks it up at
  // its next vsync. The Tab5 glass refreshes at 65.98 Hz (15.14 ms; 83.33 ms
  // = 5.5038 refreshes), so visible frames land on the deterministic, evenly
  // distributed 5/6-refresh beat — the smoothest 12 fps can look at 66 Hz.
  // (S79 kicked right after the work -> visible time = grid + work variance
  //  = random vsync choice each frame = the user-reported "super jittery".)
  constexpr int64_t kPreUs = 12000;  // text+sync+DMA2D copy, swap at next vsync
  const int64_t frame_us = 1000000LL / (rf.ok && rf.fps ? rf.fps : 12);
  int64_t  ideal_us = 0;
  bool     grid_live = false;
  uint32_t win_maxft_us = 0;   // per-window worst FT (jitter detector)
  uint32_t last_pres_us = 0;   // last presentFrame() duration (copy time)
  uint32_t last_scan = 0;
  uint32_t last_err_log = 0;
  int i = 0;
  g_win_start = millis();

  for (;;) {
    // S119: TOUCH DRAIN — loop()'s gesture engine posts events; the player
    // is the sole consumer (and the sole M5GFX writer). Primary = tap MUTE
    // + vswipe VOLUME SLIDER (user spec); S120: hswipe = volume too. NO pause.
    TouchEv ev = {};
    while (s_touch_q && xQueueReceive(s_touch_q, &ev, 0) == pdTRUE) {
      switch (ev.type) {
        case TEV_TAP: {
          g_muted = !g_muted;
          printf("touch: tap — %s\n", g_muted ? "MUTED" : "unmuted");
          if (g_presented) rfab_poc_waitpresent();  // gate the last present's DMA2D
          lcd.setRotation(kUiRot);
          lcd.fillRect(4, 36, 700, 24, lcd.color565(10, 12, 18));
          lcd.setTextColor(lcd.color565(255, 210, 120));
          lcd.drawString(g_muted ? "MUTED — tap to unmute" : "unmuted", 12, 40);
          lcd.setRotation(0);
          lcd.display(0, 0, kBw, kBh);
          break;
        }
        case TEV_SWIPE_V: {
          // VOLUME SLIDER — swipe UP (dy < 0) = louder; 24 px/step, ±32
          // clamp per swipe, 0..127 scale (25% = 32 = the boot default).
          int32_t dv = -(int32_t)ev.val / 24;
          if (dv > 32) dv = 32;
          if (dv < -32) dv = -32;
          int32_t nv = (int32_t)rfav_audio::getVolume() + dv;
          if (nv < 0) nv = 0;
          if (nv > 127) nv = 127;
          rfav_audio::setVolume((uint32_t)nv);
          char m[40];
          snprintf(m, sizeof(m), "VOL %d (%d%%)", (int)nv, (int)(nv * 100 / 127));
          uiFlash(m);
          printf("touch: vswipe — vol %d (%d%%)\n", (int)nv, (int)(nv * 100 / 127));
          break;
        }
        case TEV_SWIPE_H: {
          // S120: hswipe = VOLUME (user: seek "is not a useful control — I
          // actually want volume in that direction swipe"). RIGHT (dx > 0) =
          // louder; 24 px/step, ±32 clamp, 0..127 (same math as vswipe).
          if (ev.val == 0) break;
          int32_t dv = ev.val / 24;
          if (dv > 32) dv = 32;
          if (dv < -32) dv = -32;
          int32_t nv = (int32_t)rfav_audio::getVolume() + dv;
          if (nv < 0) nv = 0;
          if (nv > 127) nv = 127;
          rfav_audio::setVolume((uint32_t)nv);
          char m[40];
          snprintf(m, sizeof(m), "VOL %d (%d%%)", (int)nv, (int)(nv * 100 / 127));
          uiFlash(m);
          printf("touch: hswipe — vol %d (%d%%)\n", (int)nv, (int)(nv * 100 / 127));
          break;
        }
        case TEV_LONG:
          printf("touch: long-press — (reserved)\n");
          break;
      }
    }
    // S122: DIRECTOR tick — the active avatar's scripted events (the external
    // WiFi source posts the same strings via rfav_fsm_queueEvent()).
    const uint8_t dnext = dirTick();
    if (dnext != av::kStateN) {
      enterClipForState(dnext);
      i = 0;              // the new clip's index space starts at 0
      grid_live = false;  // re-anchor the grid on the new clip's first frame
      g_miss_run = 0;
      ideal_us = esp_timer_get_time();
      continue;
    }
    if (!rf.ok) {
      // ---- open/rescan state (file missing, SD absent, bad header) ----
      grid_live = false;        // re-anchor the grid when playback resumes
      // S109: a CLIP SWITCH opens NOW (the end-of-clip branch already closed
      // the file and bumped g_clip_gen) — only a boot-time missing file or
      // SD-absent paces on kRescanMs.
      if (g_clip_switch_pending)
        g_clip_switch_pending = false;
      else if (millis() - last_scan < kRescanMs) {
        delay(100);
        continue;
      }
      last_scan = millis();
      if (!ensureSd()) {
        snprintf(rf.err, sizeof(rf.err), "SD card not available — insert and retry");
      } else if (!tryOpenRfa()) {
        // rf.err already set
        // S109: SKIP-ON-3-FAILS — a missing/corrupt clip must not wedge the
        // cycle: advance the playlist (the gen bump re-arms the prefetch +
        // handoff; i resets with the new clip's index space).
        if (g_open_fails >= 3) {
          printf("rfav3: clip %s unopenable (3 tries) — skipping\n",
                 g_clip_name);
          enterClipForState(av::active()->idle);  // S122: avatar idle (gen bump + pending inside)
          i = 0;
        }
      }
      // S114: the wait card is drawn ONLY when the open really failed. A
      // planned clip switch passes through this branch exactly once with
      // g_clip_switch_pending=true (fast-path open, no throttle); drawing the
      // card unconditionally there flashed a full-screen text frame between
      // every clip — and with real=0 (S113 video regression) nothing ever
      // presented over it, so it looked like a permanent text screen.
      if (rf.ok) {
        // S119 FIX1: credit/token reset at switch — drop any STALE ready
        // tokens (the player would consume them for the WRONG frame) and top
        // the credit banks back to 1 (max=2 caps over-gives; a token a racing
        // handoff gives AFTER our clear just becomes a benign stale-drop —
        // the tag check keeps every handoff correct). The BOOT clip never
        // enters here (setup's own tryOpenRfa opens it), so this is a
        // null-guarded no-op at boot.
        for (int s = 0; s < kSlots; s++) {
          if (g_slot_ready[s]) xSemaphoreTake(g_slot_ready[s], 0);
          if (g_slot_free[s]) xSemaphoreGive(g_slot_free[s]);
        }
        // open succeeded — fall through to playback, no text.
      } else {
        if (millis() - last_err_log > 1000) {
          last_err_log = millis();
          printf("rfav3 waiting: %s\n", rf.err);
        }
        // draw the wait state over the background (fb still holds it) —
        // S89: text in the kUiRot window, like the chip.
        memset(g_bgst, 0, sizeof(g_bgst));  // S103: fb redraw outside makeFrame
        lcd.fillScreen(lcd.color565(10, 12, 18));
        lcd.setRotation(kUiRot);
        lcd.setTextColor(lcd.color565(255, 200, 120));
        lcd.drawString("RFAV-3 POC — waiting for SD card / file", 20, 200, 3);
        lcd.setTextColor(lcd.color565(200, 200, 200));
        lcd.drawString(rf.err, 20, 280, 2);
        char scanline[64];
        snprintf(scanline, sizeof(scanline), "rescan every 5 s — clips: /avatars/%s/rfav/",
                 av::activeName());  // S122
        lcd.drawString(scanline, 20, 320, 2);
        lcd.setRotation(0);
        lcd.display(0, 0, kBw, kBh);
        continue;
      }
    }

    // ---- spawn the SD prefetch exactly once, on the first playable frame ----
    if (!g_pf_tried) {
      g_pf_tried = true;
      spawnPrefetch();   // on any failure the player falls back to inline reads
    }

    // ---- S87: PARK + RE-ARM after a run of slot misses. S103: with
    // fast-drop the player holds no SD lock while the prefetch is alive, so
    // 8 consecutive misses = the prefetch is ≥1 ring away or dead. Park 100
    // ms (was 1 s — the fast-drop player doesn't need the long cooldown),
    // then rewind the absolute index to just before the prefetch's next fill
    // so the tags re-converge within ≤ ring depth frames — the bounded
    // rewind IS the resync. g_pf_dead = the task deleted itself (SD retries
    // exhausted): stay inline for good. ----
    if (g_pf_live && g_miss_run >= 8) {
      g_miss_run = 0;
      if (g_pf_dead) {
        printf("rfav3: prefetch dead — inline for good\n");
        g_pf_live = false;
      } else {
        printf("rfav3: prefetch stalled (8 misses) — park 100 ms, resync\n");
        g_pf_live = false;
        delay(100);
        i = (int)g_pf_next - 1;  // bounded rewind to the prefetch's position
        if (i < 0) i = 0;  // S109: g_pf_next can be 0 on a fresh clip (i=-1 would index g_slot[-1])
        grid_live = false;      // re-anchor the present grid with the new i
        g_pf_live = true;
      }
    }

    // ---- grid anchor: first playable frame after (re)open starts the
    // ideal-time grid ~50 ms out (gives the prefetch its first fills). ----
    if (!grid_live) {
      // S119 FIX2: H(0) runway gate — kill the 6-frame startup transient
      // (the first takes all miss: the prefetch has filled nothing yet =
      // ~400-615 ms before the first present, which truncates the per-clip
      // audio tail). Wait (bounded 500 ms) for the handoff to LAND frame 0
      // — g_slot_ho_t64[0] is stamped on the fill — then anchor. Timeout =
      // the old behavior. At boot g_slot_ho_t64 is all-zero, so the boot
      // clip is gated too (waits the bounded window until the first fill
      // lands, ~60-150 ms warm — correct). Park/rewind re-anchors with
      // i != 0 -> no gate (resync path unchanged).
      if (i == 0 && g_pf_tried) {
        const int64_t gate_t0 = esp_timer_get_time();
        while (g_slot_ho_t64[0] <= gate_t0 &&
               esp_timer_get_time() - gate_t0 < 500000)
          vTaskDelay(pdMS_TO_TICKS(5));
      }
      grid_live = true;
      ideal_us = esp_timer_get_time() + kPreUs + 50000;
    }

    // ---- gate: previous present's DMA2D copy must be done before we
    // rewrite the draw buffer (binary sem given by on_frame_buf_complete) ----
    const int64_t ft_start = esp_timer_get_time();
    if (g_presented) rfab_poc_waitpresent();

    // S87/S109: i is ABSOLUTE WITHIN THE CURRENT CLIP — the player's i and
    // the prefetch's read/handoff indices share ONE per-clip index space
    // that resets to 0 on every clip switch (every index resets together,
    // so the tag checks stay valid across clips).
    // S122: CLIP END = FSM INPUT (data-driven). Looping states replay in
    // place; the terminal state (outro) ends the session and the show
    // restarts at the avatar's initial state; every other state fires
    // <clip>.finished (clip_end guard) -> the FSM transition.
    if (i >= rf.frame_count) {
      const av::AvatarDef *avd = av::active();
      const uint8_t cur = g_fsm_state;
      if (avd->states[cur].loop) {
        i = 0;  // loop in place — no event, no gen bump
        continue;
      }
      if (avd->states[cur].terminal) {
        g_dir_session++;
        g_dir_step = 0;
        g_gaze_press_ms = 0;
        printf("fsm: %s done — session %lu complete, show restarts at %s\n",
               avd->states[cur].name, (unsigned long)g_dir_session,
               avd->states[avd->initial].name);
        enterClipForState(avd->initial);
      } else {
        char fin[48];
        const char *f = stateFinishedEvent(avd, cur, fin, sizeof(fin));
        const bool held = gazeHeld();  // the _in.finished still_held guard
        uint8_t nxt = cur;
        if (f && fsmEvent(cur, f, /*clip_end=*/true, held, nxt)) {
          printf("rfav3: clip %s done -> [%s] -> %s\n", g_clip_name, f,
                 avd->states[nxt].clip);
          enterClipForState(nxt);
        } else {
          // FSM invariant says this can't happen (every non-terminal
          // non-looping state has a path back to idle) — loop to avoid wedging.
          printf("fsm: BUG — no transition on %s in %s; looping clip\n",
                 f ? f : "?", avd->states[cur].clip);
          i = 0;
          continue;
        }
      }
      // state changed (or session restart) — the new clip opens via rescan
      i = 0;              // the new clip's index space starts at 0
      grid_live = false;  // re-anchor the grid on the new clip's first frame
      g_miss_run = 0;
      ideal_us = esp_timer_get_time();
      continue;
    }

    bool ok = makeFrame(i);
    const uint32_t ft_us = (uint32_t)(esp_timer_get_time() - ft_start);
    g_last_ft_us = ft_us;   // measure even on failure (it still cost time)
    g_ft_sum_us += ft_us;   // -> avg FT (headroom) shown on screen + serial
    if (ft_us > win_maxft_us) win_maxft_us = ft_us;  // per-window worst FT

    if (ok) {
      // ---- S80 fixed-grid present kick: hold until ideal[i] - kPreUs so
      // the copy completes just before ideal[i] and the vsync swap lands on
      // the deterministic 5/6-refresh beat (see playerTask prologue). ----
      // S89 FRAME DROP (user: "if we get stale and exceed the frame budget
      // we should drop the frame being worked on"): work overran the ideal
      // slot -> skip the present (the panel holds the last good frame); the
      // grid re-anchors in the tail below and the drop is counted.
      const int64_t now_us = esp_timer_get_time();
      if (now_us < ideal_us) {
        const int64_t kick_us = ideal_us - kPreUs;
        if (now_us < kick_us)
          delay((unsigned long)((kick_us - now_us) / 1000));
        const uint32_t pres_t0 = micros();
        presentFrame(i % (int)rf.frame_count);
        last_pres_us = micros() - pres_t0;
        // S128 telemetry (2): inter-present gap > 250 ms = a visible freeze.
        // Dropped/missed frames skip the present, so a stall shows up as ONE
        // big gap to the next present — windowed count + max, printed at 1 Hz.
        {
          const int64_t pres_done_us = esp_timer_get_time();
          if (g_last_pres_done_us) {
            const uint64_t gap_us =
                (uint64_t)(pres_done_us - g_last_pres_done_us);
            if (gap_us > 250000) {
              g_jk_n++;
              if (gap_us / 1000 > g_jk_max_ms)
                g_jk_max_ms = (uint32_t)(gap_us / 1000);
            }
          }
          g_last_pres_done_us = pres_done_us;
        }
        g_presented = true;
        if (g_clip_first_pres_us == 0)
          g_clip_first_pres_us = esp_timer_get_time();  // S112: audio starts 34 ms from here
        g_played++;
        g_win_frames++;
      } else {
        g_dropped++;
        if (g_dropped == 1 || (g_dropped % 50) == 0)
          printf("rfav3: frame %u DROPPED (FT overran the %lu ms budget) — %lu drops\n",
                 (unsigned)(i % rf.frame_count), (unsigned long)kBudgetMs,
                 (unsigned long)g_dropped);
      }
    } else {
      // S103 DROP PACE: makeFrame-false = a miss (stale/timeout drop) — sleep
      // to the end of this frame's ideal slot instead of looping fast (that
      // raced i past the prefetch -> stale-everything -> park/rewind cycles).
      g_win_miss++;
      const int64_t slot_end = ideal_us;
      if (esp_timer_get_time() < slot_end)
        delay((unsigned long)((slot_end - esp_timer_get_time()) / 1000));
    }

    ideal_us += frame_us;
    if (esp_timer_get_time() > ideal_us)  // fell >1 slot behind: re-anchor
      ideal_us = esp_timer_get_time() + frame_us;

    // ---- 1 Hz serial telemetry ----
    if (millis() - g_win_start >= 1000) {
      g_win_fps = g_win_frames;
      g_win_frames = 0;
      g_win_start = millis();
      printf(
          "rfav3: f=%u/%u real=%u fps FT=%lu ms max=%lu ms pres=%lu ms avg=%lu ms (budget %lums) sd=%lu ms dec=%lu iv=%lu c=%lu c2m=%lu us ce=%lu cf=%d jk=%u/%lu cd=%lu drop=%u ms=%u pf=%d pff=%u psram_free=%lu KB ifree=%lu KB ilarg=%lu KB\n",  // S143 FIX-4: internal-SRAM telemetry (root-cause monitor)
          (unsigned)(i % rf.frame_count), (unsigned)rf.frame_count,
          (unsigned)g_win_fps,
          (unsigned long)(ft_us / 1000),
          (unsigned long)(win_maxft_us / 1000),  // S81: read BEFORE the reset below
          (unsigned long)(last_pres_us / 1000),
          (unsigned long)(g_played ? (g_ft_sum_us / 1000 / g_played) : 0),
          (unsigned long)kBudgetMs,
          (unsigned long)(g_last_sd_us / 1000), (unsigned long)g_last_dec_us,
          (unsigned long)g_last_iv_us, (unsigned long)g_last_c_us,
          (unsigned long)g_last_c2m_us,   // S89: stage fields in µs, no 'tp'
          (unsigned long)g_csync_errs,    // S99: syncChunked error count (cumulative)
          (int)g_csync_first,             // S99: first sync error (0 = none)
          (unsigned)g_jk_n,               // S128: inter-present gaps > 250 ms this window
          (unsigned long)g_jk_max_ms,     // S128: worst such gap (ms)
          (unsigned long)rfab_poc_color_done(),  // S101 FIX B: on_color_trans_done fires (callback liveness)
          (unsigned)g_dropped,
          (unsigned)g_win_miss,           // S103: windowed misses (stale/timeout drops)
          (int)g_pf_live, (unsigned)g_pf_filled,
          (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
          (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),        // S143 FIX-4: internal free KB
          (unsigned long)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));  // S143 FIX-4: internal largest block KB
      win_maxft_us = 0;   // S81: was reset before the printf -> max= always 0
      g_win_miss = 0;     // S103: windowed miss count
      g_jk_n = 0;         // S128: windowed jank count
      g_jk_max_ms = 0;
    }

    i++;
  }
}

// ------------------------------ setup ------------------------------

void setup() {
  // skytab s13 lesson: force the one-time newlib stdout-lock creation at boot
  // (clean heap) so later driver printfs only TAKE the lock.
  (void)printf("rfav3-poc: boot\n");
  fflush(stdout);

  M5.begin();
  // S119: TOUCH — M5GFX auto-detected the controller at M5.begin() (boot
  // prints "M5Tab5 GT911 touch detected" or "M5Tab5 ST touch FW version NN");
  // loop() polls it via M5.update() and posts gestures here; the player task
  // is the sole consumer. (M5.begin() owns the internal I2C — no Wire.begin.)
  s_touch_q = xQueueCreate(8, sizeof(TouchEv));
  bootLog("S122: data-file FSM (avatar.json v1.0.0, /avatars/<model>/); touch on (tap=mute, hswipe+vswipe=vol) — vol 32/127 (25%)");
  M5GFX &lcd = M5.Display;
  // S89: PORTRAIT-NATIVE — rotation 0 (glass-native 720x1280; skytab is a
  // portrait UI so rot 0 is the baseline). Portrait assets decode straight
  // into fb; all UI text uses the kUiRot window (see the S89 config note).
  lcd.setRotation(0);
  lcd.fillScreen(lcd.color565(10, 14, 20));
  lcd.setRotation(kUiRot);
  lcd.setTextColor(lcd.color565(120, 220, 140));
  lcd.drawString("RFAV-3 POC player — M5Stack Tab5", 12, 20, 3);
  lcd.setRotation(0);
  lcd.display(0, 0, kBw, kBh);

  bootLog(psramFound() ? "PSRAM: ok" : "PSRAM: MISSING!");
  bootLog(rfab_poc_drawbuf_ok() ? "dsi: double-buffer patch OK"
                                : "dsi: drawbuf FAIL (zero-copy fallback)");

  fb = rfab_poc_drawbuf();
  if (!fb) {
    fatalScreen("DRAW BUFFER FAIL", "M5GFX rfab_poc_drawbuf() NULL — libdeps patch missing?");
    return;
  }
  // S101 FIX A: fr_fb is BACK — portrait assets decode into the 1.75 MB
  // PSRAM scratch allocated just below; the presented fb is never the JDEC
  // target. The S86 driver requirement (JDEC output must be PSRAM + aligned
  // to the L2 line, 128 B; size a multiple of it) is verified once at boot
  // so a regression is a clean fatal, not a per-frame e=258.
  {
    const uintptr_t fba = (uintptr_t)fb;
    const size_t fb_alloc = heap_caps_get_allocated_size(fb);
    bootLog(("fb: " + String((fba & 127u) ? "MISALIGNED!" : "JDEC-out ok") +
             " (fb=0x" + String((unsigned long)fba, 16) + " heap_alloc=" +
             String((unsigned long)fb_alloc) + " B — S101: decodes go to fr_fb)").c_str());
    if ((fba & 127u) || (kFbBytes % 128u)) {
      fatalScreen("FB ALIGN FAIL", "draw buffer must be 128 B aligned (JDEC PSRAM output)");
      return;
    }
  }

  // S101 FIX A: JDEC scratch fr_fb — PSRAM + 128 B aligned (S86), same
  // pattern as bg_buf. After decode+M2C the fb PSRAM still holds the
  // previous frame's fully-converged composite, so anything the trailing
  // C2M has not drained by DMA2D read time is last-frame content, never
  // JDEC-raw light blue. psram_free was 17.2 MB at S100 boot -> fine.
  {
    size_t ff_align = 128;
    size_t qa = 0;
    if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &qa) == ESP_OK && qa > ff_align)
      ff_align = qa;
    fr_fb = (uint16_t *)heap_caps_aligned_alloc(ff_align, kFbBytes,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fr_fb) {
      fatalScreen("FR_FB ALLOC FAIL", "1.75 MB PSRAM JDEC scratch (S101 FIX A)");
      return;
    }
    const uintptr_t ffa = (uintptr_t)fr_fb;
    if ((ffa & 127u) || (kFbBytes % 128u)) {
      fatalScreen("FR_FB ALIGN FAIL", "JDEC scratch must be 128 B aligned (S86)");
      return;
    }
    bootLog(("fr_fb: ok (0x" + String((unsigned long)ffa, 16) + " heap_alloc=" +
             String((unsigned long)heap_caps_get_allocated_size(fr_fb)) +
             " B — S101 FIX A decode scratch, psram_free=" +
             String((unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024)) +
             " KB)").c_str());
  }

  if (!initJpegEngine()) {
    fatalScreen("JPEG ENGINE FAIL", "jpeg_new_decoder_engine() failed");
    return;
  }

  // S79: mount SD BEFORE initBackground() — background.jpg comes off the card,
  // and the unmounted-VFS open() is what logged "File system is not mounted"
  // (bg silently fell back to the checkerboard before).
  if (ensureSd()) {
    // S128 telemetry (1): SD card info at boot — card class + size + the
    // host clock cap. (The negotiated REAL clock is not exposed by this
    // framework's SDMMCFS; default cap = BOARD_MAX_SDMMC_FREQ =
    // SDMMC_FREQ_HIGHSPEED = 40 MHz — the A/B rd distribution is the
    // empirical answer to 'why 2 MB/s'.)
    char sdb[112];
    snprintf(sdb, sizeof(sdb), "sd: mounted — %s %lu MB, host max %lu kHz 4-bit",
             SD_MMC.cardType() == CARD_SDHC ? "SDHC/SDXC" : "SD",
             (unsigned long)(SD_MMC.cardSize() / (1024UL * 1024UL)),
             (unsigned long)BOARD_MAX_SDMMC_FREQ);
    bootLog(sdb);
  } else {
    bootLog("sd: NOT MOUNTED — checkerboard bg for now; player rescans every 5 s");
  }

  // S122: load the active avatar (/avatars/<model>/avatar.json — first valid
  // candidate wins; built-in Elena on any failure) — AFTER the SD mount,
  // BEFORE initBackground() (the bg candidate list is built from it).
  av::load();

  // S133: file transfer (serial 'UPLOAD <path> <size>' -> SD; rx task on
  // core 1) + WiFi via the on-board ESP32-C6 module (S135: credentials from
  // P4 NVS "skytab" — SD /config.json provisions it once at startup; never
  // in source).
  ft_init();
  wifi_sd_init();

  // S135 (S134 plan): heap telemetry point 1 — POST-BOOT, PRE-WIFI (the
  // Q-SW baseline). Q-A: compare with the wifi task's post-CONNECT print
  // and the at-failure print in the prefetch retry line below — the delta
  // is the WiFi/esp_hosted internal-SRAM load that starves the SD driver's
  // per-transfer internal-RAM DMA bounce (allocate_dma_buf, 0x101/0x102).
  printf("boot: heap internal free=%u KB largest=%u KB psram free=%u KB\n",
         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
         (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
         (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

  if (!initBackground()) {
    fatalScreen("BACKGROUND FAIL", "bg PSRAM alloc failed");
    return;
  }
  bootLog("background: ready (checkerboard or background.jpg)");

  // S122: ANIMATION FSM (the active avatar's data) — initial state = the
  // avatar's `initial` (intro); the director (dirTick in the player task)
  // fires the avatar's script rows and clip ends fire <clip>.finished. The
  // audio engine arms on this first gen bump (its decode task starts at
  // last_gen = 0xFFFFFFFF, so the current clip is picked up as a change).
  {
    const av::AvatarDef *avd = av::active();
    g_fsm_state = avd->initial;
    g_fsm_enter_ms = millis();
    setClipByName(avd->states[avd->initial].clip);
    g_clip_gen++;
    g_clip_switch_pending = true;
    bootLog(("avatar: " + String(avd->name) + " (" + (av::fromSd() ? "sd" : "built-in fallback") +
             ") — " + String((int)avd->nStates) + " states / " + String((int)avd->nTrans) +
             " trans / " + String((int)avd->nRows) + " rows; initial=" +
             String(avd->states[avd->initial].clip) + "; audio=34k +34ms delay").c_str());
  }

  // First attempt at the clip (player rescans if this fails).
  if (ensureSd()) {
    if (tryOpenRfa()) {
      bootLog(("clip: " + String(g_clip_path) + " — " + String(rf.frame_count) +
               " frames @" + String(rf.fps) + " fps").c_str());
    } else {
      bootLog(("clip: WAIT — " + String(rf.err)).c_str());
    }
  } else {
    bootLog("clip: WAIT — SD card not available");
  }

  // S112: AUDIO ENGINE — ES8388 + I2S0 + decode/feed tasks. ALL steps
  // non-fatal: any failure degrades to silent, video keeps running (S111).
  rfav_audio::audioInit();

  // S102: player core 1 -> 0 — kill the 87-127 ms core-1 descheduling chunks
  // (S101 item 4: unexplained gaps inside ft while SD/SDMMC activity ran on
  // core 1). FIX A is core-independent, so model M stays dead. Prefetch stays
  // where it is (core 0, prio 1).
  xTaskCreatePinnedToCore(playerTask, "rfav3", 12288, nullptr, 1, nullptr, 0);
  bootLog("player task started (core 0) — S102");
#if S94_CAPTURE
  s94cap::capInit();   // S94 E3: screen capture (staging + core-0 drain tasks)
#endif
}

void loop() {
  // S119: TOUCH ENGINE (core 1, ~100 Hz) — M5.update() is the ONLY poll of
  // the touch controller (it was never polled before S119 — touch was dead).
  // Gestures are classified here and posted to s_touch_q; the PLAYER task is
  // the sole consumer (nothing M5GFX-related is touched here — the player
  // owns the display). Rotation is 0 (portrait-native 720x1280) so td.x/td.y
  // ARE glass-native — no hand-transform. M5GFX throttles the touch I2C read
  // to >= 4 ms internally, so delay(10) is enough for ~100 Hz.
  M5.update();
  static bool     t_in = false;
  static int16_t  t_x0 = 0, t_y0 = 0, t_x1 = 0, t_y1 = 0;
  static uint32_t t_t0 = 0;
  if (M5.Touch.getCount() > 0) {
    const auto &td = M5.Touch.getDetail();
    if (!t_in) {
      t_in = true;
      t_x0 = t_x1 = td.x;
      t_y0 = t_y1 = td.y;
      t_t0 = millis();
    } else {
      t_x1 = td.x;
      t_y1 = td.y;
    }
  } else if (t_in) {
    t_in = false;
    const int dx = (int)t_x1 - (int)t_x0;
    const int dy = (int)t_y1 - (int)t_y0;
    const int ax = dx < 0 ? -dx : dx;
    const int ay = dy < 0 ? -dy : dy;
    const int dist = ax > ay ? ax : ay;
    const uint32_t dur = millis() - t_t0;
    TouchEv ev = {};
    ev.x = t_x1;
    ev.y = t_y1;
    bool post = false;
    if (dist < 25) {
      ev.type = (dur >= 400) ? TEV_LONG : TEV_TAP;  // < 25 px = not a swipe
      ev.val = 0;
      post = true;
    } else if (ax * 10 >= ay * 15) {
      ev.type = TEV_SWIPE_H;
      ev.val = (int16_t)dx;
      post = true;
    } else if (ay * 10 >= ax * 15) {
      ev.type = TEV_SWIPE_V;
      ev.val = (int16_t)dy;
      post = true;
    }
    // diagonal-ish: ignore
    if (post && s_touch_q) xQueueSend(s_touch_q, &ev, 0);
  }
  delay(10);
}
