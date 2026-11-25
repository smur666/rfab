// S112: Opus audio pipeline — RFAV clip voice on the Tab5 ES8388 speaker.
//
// Replaces the S110 link probe with the full engine (design locked in S111
// STATE item 4): hand-rolled OGG/Opus demux (our files are ffmpeg-generated
// single-stream OggOpus) -> libopus decode (48 kHz mono) -> mono->stereo +
// 70% software gain (codec vol reg = 0x00 = 0 dB, M5 convention) -> PSRAM
// 12-frame ring (46,080 B, 240 ms) -> I2S0 feed.
//
// Sources on SD (user-loaded; see assets/tree.txt):
//   /avatars/<model>/34k_opus/<clip>.34k.opus   34 kbps mono — THE set we play (spec)
//   /avatars/<model>/48k_opus/<clip>.48k.opus   48 kbps mono — A/B spare
// Clips with no .opus play silent (open fails -> logged once, no retry).
//
// A/V: a clip's audio starts kAuStartDelayUs (34 ms — user spec) after that
// clip's FIRST PRESENTED video frame (g_clip_first_pres_us, main.cpp).
//
// Hardware (S111 recon, M5Unified master): I2S0 MCK 30 / BCK 27 / WS 29 /
// DOUT 26 (I2S_NUM_0, P4 = master); ES8388 @ I2C 0x10 on the internal I2C
// (SCL 32 / SDA 31, up after M5.begin()); speaker amp = PI4IOE5V6408 @0x43,
// reg 0x05, bit 0x02.
//
// Tasks (all core 1 — video is core 0): decode prio 3 / 8 KB, feed prio 4 /
// 4 KB. audioInit() is all-non-fatal: video must keep running.

#include <Arduino.h>
#include <M5Unified.h>
#include <SD_MMC.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <opus.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_opus.h"
#include "avatar.h"   // S122: per-model audio paths (/avatars/<model>/34k_opus/)
#include "task_util.h" // S169: PSRAM task stacks

// main.cpp globals (the player owns them; this module only reads)
extern const char *g_clip_name;
extern volatile uint32_t g_clip_gen;
extern volatile int64_t g_clip_first_pres_us;
extern volatile bool g_muted;  // S119: tap-mute — feedTask writes silence while set

// E3: not in the public esp_cache.h here — same forward-declaration pattern as
// main.cpp (decl lives in esp_mm/include/esp_private/esp_cache_private.h).
extern "C" esp_err_t esp_cache_get_alignment(uint32_t heap_caps, size_t *out_alignment);

namespace rfav_audio {

// ------------------------------ config ------------------------------------
static constexpr char     kAudioSuffix[]  = ".34k.opus";  // S122: dir is per-model (/avatars/<name>/34k_opus/)
static constexpr int64_t  kAuStartDelayUs = 34000;   // 34 ms start delay (spec)
static constexpr uint32_t kAuAnchorTimeoutUs = 2000000;  // 2 s: give up waiting
static constexpr uint32_t kFs         = 48000;
static constexpr int      kFrame      = 960;   // 20 ms @48 kHz = one Opus packet
static constexpr int      kRingFrames = 12;    // 240 ms of headroom
static constexpr int      kWatermark  = 8;     // decode pacing (frames)
static constexpr uint32_t kRingBytes  = (uint32_t)kRingFrames * kFrame * 2 * 2;  // 46,080
static constexpr int      kChunk      = kFrame * 2 * 2;   // 3,840 B = 960 stereo frames
static constexpr uint32_t kCodecAddr  = 0x10;  // ES8388 (NOT the 0x18 guess)
static constexpr uint32_t kAmpAddr    = 0x43;  // PI4IOE5V6408
static constexpr uint32_t kAmpReg     = 0x05;  // OUTPUT data
static constexpr uint32_t kAmpBit     = 0x02;  // AMP enable
// S119: live volume on a 0..127 scale (was the fixed 90/7 = 70% constant).
// Default 32 = 25% — "low but knowable it's working" (user spec).
static volatile uint32_t g_vol = 32;
uint32_t getVolume() { return g_vol; }
void setVolume(uint32_t v) { if (v > 127) v = 127; g_vol = v; }

// ES8388 init — VERBATIM M5Unified master _speaker_enabled_cb_tab5 (28
// pairs, same order; bulk format [datalen=2][reg][val] decoded from the
// raw inl, S116). S116 FIX: the S112 copy had shifted entries 2-12 by one
// register (official writes reg 0x00 FOUR times [0x80,0x00,0x00,0x0E], then
// 0x01-0x08; S112 had 0x00-0x0B) — so CHIPPOWER 0x02 was 0x00 and DACPOWER
// 0x04 was 0x00: codec DAC unpowered -> digital pipeline perfect, silence.
// Official decodes: 0x00=CHIPCONTROL0 (0x80=reset pulse; net 0x0E) ·
// 0x02=CHIPPOWER (0x0A=power up all) · 0x03=ADCPOWER (0xFF=down) ·
// 0x04=DACPOWER (0x3C=up + LOUT1/ROUT1/LOUT2/ROUT2 enable) · 0x05/06=
// ChipLowPower · 0x07=VSEL (0x7C) · 0x08=I2S slave mode · 0x09-0x16=ADC
// (defaults) · 0x17=I2S format (16-bit) · 0x18=MCLK ratio (0x00; the 256xFS
// I2S MCLK with this register = the shipping combo, do NOT "correct") ·
// 0x19=DAC unmute (0x20) · 0x1A/0x1B=DAC volume (0x00=0 dB -> software
// gain) · 0x1C=click-free · 0x26=DAC CTRL16 · 0x27/0x2A=ch mix · 0x2B=ADC/
// DAC separate · 0x2D=VREF (1.5k) · 0x2E-0x31=0x21.
static constexpr uint8_t kEs8388Regs[][2] = {
  {0x00, 0x80},  // RESET/ CSM POWER ON
  {0x00, 0x00},
  {0x00, 0x00},
  {0x00, 0x0E},
  {0x01, 0x00},
  {0x02, 0x0A},  // CHIP POWER: power up all
  {0x03, 0xFF},  // ADC POWER: power down all
  {0x04, 0x3C},  // DAC POWER: up + LOUT1/ROUT1/LOUT2/ROUT2 enable
  {0x05, 0x00},  // ChipLowPower1
  {0x06, 0x00},  // ChipLowPower2
  {0x07, 0x7C},  // VSEL
  {0x08, 0x00},  // set I2S slave mode
  // 0x09-0x16 == adc (leave defaults)
  {0x17, 0x18},  // I2S format (16bit)
  {0x18, 0x00},  // I2S MCLK ratio (128) — shipping combo with mclk_mult 256
  {0x19, 0x20},  // DAC unmute
  {0x1A, 0x00},  // LDACVOL 0x00~0xC0
  {0x1B, 0x00},  // RDACVOL 0x00~0xC0
  {0x1C, 0x08},  // digital click free
  {0x1D, 0x00},
  {0x26, 0x00},  // DAC CTRL16
  {0x27, 0xB8},  // LEFT Ch MIX
  {0x2A, 0xB8},  // RIGHT Ch MIX
  {0x2B, 0x08},  // ADC and DAC separate
  {0x2D, 0x00},  // VREF (1.5k)
  {0x2E, 0x21},
  {0x2F, 0x21},
  {0x30, 0x21},
  {0x31, 0x21},
};

// --------------------------- PSRAM frame ring ------------------------------
// Single producer (decode task) / single consumer (feed task), both core 1,
// frame-aligned (3,840 B chunks) with ABSOLUTE byte counters — no lock, the
// frame count is the only shared truth. S83 cache discipline: M2C once after
// alloc, C2M after every CPU fill.
static uint8_t *s_ring = nullptr;
static volatile uint32_t s_ring_head = 0;  // bytes written (decode task)
static volatile uint32_t s_ring_tail = 0;  // bytes read (feed task)
static volatile uint32_t s_underruns = 0;  // feed: ring-empty chunks (silence)
static volatile uint32_t s_glitches = 0;   // feed: failed i2s writes
static volatile uint32_t s_pkts = 0;       // decode: packets decoded (cum)
static volatile uint32_t s_drops = 0;      // decode: ring-full drops (cum)
static volatile uint32_t s_c2m_errs = 0;   // decode: failed C2M (cum)

static int ringFrames() {
  // S120 ring-race fix (part 1): SIGNED diff. If decode's ringClear() (clip
  // switch, head=tail=0) lands inside the feed's check->advance window, a
  // stale-absolute tail can sit ahead of the reset head and the unsigned
  // diff wraps toward 2^32 (S119 runaway: tail 167 chunks ahead -> ring
  // =1,118,314; feed looping the stale 240 ms of audio, decode
  // false-full-dropping everything). Clamp to 0: the feed self-recovers via
  // the underrun re-anchor (part 3) and decode stops false-dropping.
  const int32_t d = (int32_t)(s_ring_head - s_ring_tail);
  return d > 0 ? d / (int32_t)kChunk : 0;
}

// Same 128 B-aligned chunked msync as main.cpp's syncChunked (S83/S99).
static void auSync(const void *p, size_t size, int dir) {
  constexpr size_t kChunkSz = 64 * 1024;
  uintptr_t a = (uintptr_t)p & ~(uintptr_t)127;
  const uintptr_t e = ((uintptr_t)p + size + 127) & ~(uintptr_t)127;
  for (; a < e; a += kChunkSz) {
    size_t n = (e - a) < kChunkSz ? (e - a) : kChunkSz;
    if (esp_cache_msync((void *)a, n, dir | ESP_CACHE_MSYNC_FLAG_TYPE_DATA) != ESP_OK)
      s_c2m_errs++;
  }
}

static void ringClear() { s_ring_head = 0; s_ring_tail = 0; }

static void ringWrite(const uint8_t *chunk) {
  const uint32_t off = s_ring_head % kRingBytes;
  if (off + (uint32_t)kChunk <= kRingBytes) {
    memcpy(s_ring + off, chunk, (size_t)kChunk);
    auSync(s_ring + off, (size_t)kChunk, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  } else {
    const uint32_t a = kRingBytes - off;  // ring wraps: 46,080 % 3,840 == 0, so a > 0
    memcpy(s_ring + off, chunk, a);
    memcpy(s_ring, chunk + a, (size_t)kChunk - a);
    auSync(s_ring + off, a, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    auSync(s_ring, (size_t)kChunk - a, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  }
  s_ring_head += (uint32_t)kChunk;
}

static void ringRead(uint8_t *chunk) {
  const uint32_t off = s_ring_tail % kRingBytes;
  if (off + (uint32_t)kChunk <= kRingBytes) {
    memcpy(chunk, s_ring + off, (size_t)kChunk);
  } else {
    const uint32_t a = kRingBytes - off;
    memcpy(chunk, s_ring + off, a);
    memcpy(chunk + a, s_ring, (size_t)kChunk - a);
  }
  s_ring_tail += (uint32_t)kChunk;
}

// ----------------------------- OGG demux bits ------------------------------
// OggOpus page: 'OggS' ver htype granule(8 LE) serial(4) seq(4) crc(4)
// nseg(1) lacing[nseg] body. A packet = a run of lacing segments that ENDS
// at the first value < 255 (255 = 255 bytes + continue). Ogg CRC-32:
// poly 0x04C11DB7, NON-reflected, init 0, no xorout — table-based, computed
// over the whole page with the stored-CRC field zeroed.
static uint32_t s_ogg_tab[256];
static bool s_ogg_tab_ok = false;

static void oggCrcTab() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i << 24;
    for (int j = 0; j < 8; j++) c = (c & 0x80000000u) ? (c << 1) ^ 0x04C11DB7u : (c << 1);
    s_ogg_tab[i] = c;
  }
  s_ogg_tab_ok = true;
}

static inline uint32_t oggCrcUpd(uint32_t c, const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) c = (c << 8) ^ s_ogg_tab[((c >> 24) ^ p[i]) & 0xFFu];
  return c;
}

static inline uint16_t rdLE16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t rdLE32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

// ------------------------------- feed task ---------------------------------
// Core 1, prio 4: pull one 960-frame chunk per ~70 ms; i2s_channel_write
// blocks on the DMA (the natural pacing). Ring empty -> silence + underrun.
static i2s_chan_handle_t s_tx = nullptr;  // E1: real type in this IDF (i2s_common.h)

static void feedTask(void *) {
  static uint8_t buf[kChunk];  // SRAM: CPU writes, I2S DMA reads
  uint32_t last_tel = 0;
  for (;;) {
    // S115: one chunk in the ring = one 960-sample frame to feed. (Was
    // `>= kFrame` = >= 960 CHUNKS — the ring holds 12 — so this was never
    // true: the feed wrote silence forever, the decode stalled at the
    // 8-frame watermark, and the user heard nothing.)
    bool zc = false;  // S119 FIX3: this chunk fed zero-copy from the ring
    if (ringFrames() >= 1) {
      if (g_muted) {
        // S119: MUTE — feed silence WITHOUT consuming the ring (s_ring_tail
        // stays put): the decode blocks at the watermark (ring full), so on
        // unmute the audio continues EXACTLY where the video is (A/V locked).
        memset(buf, 0, sizeof(buf));
      } else {
        // S119 FIX3: ZERO-COPY — feed the ring directly when the chunk is
        // un-wrapped. i2s_std memcpys the user buffer INTO the driver's own
        // DMA descriptor buffers during the call (P4 default config =
        // 6 x 240 frames x 4 B = 30 ms of driver headroom; the 240 ms ring
        // adds more) — the ring data is only live for the call's duration.
        // The wrap case (1 of 12 chunks) stitches via buf.
        const uint32_t off = s_ring_tail % kRingBytes;
        if (off + (uint32_t)kChunk <= kRingBytes) {
          // S120 ring-race fix (part 2): CLAIM the chunk (advance tail)
          // BEFORE the 20-100 ms blocking write — S119's post-write advance
          // gave decode's ringClear() a 20-100 ms window to zero the head
          // first, leaving the stale-absolute tail up to 167 chunks ahead.
          // Now a clear landing during the write only costs one chunk of
          // stale audio (inaudible); the counters stay consistent.
          s_ring_tail += (uint32_t)kChunk;
          zc = true;
          size_t zw = 0;
          if (i2s_channel_write(s_tx, s_ring + off, (size_t)kChunk, &zw, 100) != ESP_OK)
            s_glitches++;
        } else {
          ringRead(buf);
        }
      }
    } else {
      // S120 ring-race fix (part 3): re-anchor any residual tail > head
      // (a ringClear that still landed between check and advance) within
      // one 20 ms chunk instead of a 167-chunk (~3.3 s) silence catch-up.
      if (s_ring_tail > s_ring_head) s_ring_tail = s_ring_head;
      memset(buf, 0, sizeof(buf));
      if (s_underruns % 10000 == 0)
        printf("au: underrun silence (cum %lu)\n", (unsigned long)s_underruns);
      s_underruns++;
    }
    if (!zc) {
      size_t w = 0;
      if (i2s_channel_write(s_tx, buf, sizeof(buf), &w, 100) != ESP_OK)
        s_glitches++;
    }
    if (millis() - last_tel >= 1000) {
      last_tel = millis();
      printf("au: ring=%d/%d und=%lu glitch=%lu pkt=%lu c2me=%lu\n",
             ringFrames(), kRingFrames, (unsigned long)s_underruns,
             (unsigned long)s_glitches, (unsigned long)s_pkts,
             (unsigned long)s_c2m_errs);
    }
  }
}

// ------------------------------- decode task --------------------------------
// One clip = open /avatars/<model>/34k_opus/<stem>.34k.opus (if it exists), wait the
// 34 ms start delay (from the video clip's first present), then demux +
// decode into the ring until EOF or a generation bump (a clip switch can
// interrupt mid-clip — the gen check at every page AND after the watermark
// wait). Watermark pacing: never run >= 8 frames (160 ms) ahead of the feed.
static void playOneClip(File &f, const char *name, uint32_t gen) {
  static int16_t pcm[kFrame];   // mono decode out (decode task is single)
  static uint8_t chunk[kChunk]; // stereo 16-bit (decode task is single)

  uint8_t *page = nullptr;      // heap page buffer (grows to need, cap 32 KB)
  uint32_t page_cap = 0;
  auto ensurePage = [&](uint32_t need) -> bool {
    if (need <= page_cap) return true;
    uint32_t nc = 4096;
    while (nc < need) nc *= 2;
    if (nc > 32768) nc = 32768;
    free(page);
    page = nullptr;
    page_cap = 0;
    page = (uint8_t *)malloc(nc);
    if (!page) return false;
    page_cap = nc;
    return true;
  };

  OpusDecoder *dec = nullptr;
  bool head_seen = false;
  uint32_t preskip_left = 0;    // OpusHead preskip: decoded samples to discard
  uint32_t pages = 0, crc_bad = 0, pkts = 0, dec_errs = 0, drops = 0;

  // S115: whole file -> aligned PSRAM buffer -> demux from memory. The
  // per-page f.read() walk died with a silent FR_DISK_ERR (f.read -> 0) at
  // ~2.3 KB into a 30.8 KB file (3 pages then "clean EOF", EVERY voiced
  // clip, monitor31) — big-chunk reads + memory demux remove all small
  // reads from the decode path (and the 3rd reader's VFS churn off the SD
  // bus). The fbuf probe reports got<want if the SD path itself truncates
  // the file (=> card/file re-copy, not a firmware problem).
  static uint8_t *fbuf = nullptr;
  static uint32_t fbuf_cap = 0;
  uint32_t pos = 0;    // file cursor (memory mode)
  bool     from_mem = false;
  uint32_t gsz = 0;    // bytes actually read
  {
    const uint32_t want = (uint32_t)f.size();
    if (want >= 27 && want < 512 * 1024) {
      if (want > fbuf_cap) {
        free(fbuf); fbuf = nullptr; fbuf_cap = 0;
        size_t al = 128;
        if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &al) == ESP_OK && al > 128)
          al = al;
        fbuf = (uint8_t *)heap_caps_aligned_alloc(al, want,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        fbuf_cap = fbuf ? want : 0;
      }
      if (fbuf) {
        uint32_t got = 0;
        while (got < want) {
          const uint32_t nleft = want - got;
          const int n = f.read(fbuf + got, nleft > 4096 ? 4096 : nleft);
          if (n <= 0) break;
          got += (uint32_t)n;
        }
        printf("audio: fbuf %lu/%lu B%s\n", (unsigned long)got, (unsigned long)want,
               (got < want) ? "  <-- SD READ TRUNCATION" : "");
        if (got >= 27) { from_mem = true; gsz = got; }
      }
    }
  }

  printf("audio: playing %s (gen %lu)\n", name, (unsigned long)gen);

  for (;;) {
    if (g_clip_gen != gen) break;  // interrupted by a clip switch

    // ---- read one OGG page (27 B header + lacing + body) ----
    uint8_t h[27];
    int hn;
    if (from_mem) {
      hn = (pos + 27 <= gsz) ? 27 : (int)(gsz - pos);
      if (hn > 0) memcpy(h, fbuf + pos, (size_t)hn);
      pos += (uint32_t)hn;
    } else {
      hn = f.read(h, 27);
    }
    if (hn == 0) break;  // clean EOF
    if (hn != 27 || memcmp(h, "OggS", 4) != 0) {
      printf("audio: bad page (n=%d) — aborting clip\n", hn);
      break;
    }
    if (h[4] != 1 && h[4] != 0) printf("audio: page ver=%u (want 1)\n", (unsigned)h[4]);
    const uint32_t nseg = h[26];

    uint8_t lacing[255];
    if (from_mem) {
      if (pos + nseg > gsz) {
        printf("audio: lacing read fail (%u segs)\n", (unsigned)nseg);
        break;
      }
      memcpy(lacing, fbuf + pos, nseg);
      pos += nseg;
    } else if (f.read(lacing, nseg) != (int)nseg) {
      printf("audio: lacing read fail (%u segs)\n", (unsigned)nseg);
      break;
    }
    uint32_t body = 0;
    for (uint32_t s = 0; s < nseg; s++) body += (lacing[s] == 255) ? 255 : lacing[s];
    const uint32_t plen = 27 + nseg + body;
    if (!ensurePage(plen)) {
      printf("audio: page too big (%u B)\n", (unsigned)plen);
      break;
    }
    memcpy(page, h, 27);
    memcpy(page + 27, lacing, nseg);
    if (body) {
      if (from_mem) {
        if (pos + body > gsz) {
          printf("audio: body read fail\n");
          break;
        }
        memcpy(page + 27 + nseg, fbuf + pos, body);
        pos += body;
      } else if (f.read(page + 27 + nseg, body) != (int)body) {
        printf("audio: body read fail\n");
        break;
      }
    }
    // CRC check (stored field zeroed): count mismatches, never hard-fail.
    const uint32_t stored = rdLE32(page + 22);
    const uint8_t saved[4] = {page[22], page[23], page[24], page[25]};
    page[22] = page[23] = page[24] = page[25] = 0;
    const uint32_t calc = oggCrcUpd(0, page, plen);
    page[22] = saved[0]; page[23] = saved[1];
    page[24] = saved[2]; page[25] = saved[3];
    if (calc != stored) crc_bad++;
    pages++;

    // ---- packets in the page ----
    uint32_t off = 0;
    for (uint32_t s = 0; s < nseg;) {
      const uint32_t start = off;
      do {
        off += (lacing[s] == 255) ? 255 : lacing[s];
        s++;
      } while (s < nseg && lacing[s - 1] == 255);
      const uint32_t len = off - start;
      const uint8_t *pkt = page + 27 + nseg + start;

      if (!head_seen) {
        if (len >= 16 && memcmp(pkt, "OpusHead", 8) == 0) {
          const uint8_t ver = pkt[8], ch = pkt[9];
          preskip_left = rdLE16(pkt + 10);
          const uint32_t inrate = rdLE32(pkt + 12);
          int err = 0;
          dec = opus_decoder_create((int)kFs, ch ? ch : 1, &err);
          if (!dec) printf("audio: opus_decoder_create failed (%d)\n", err);
          printf("audio: OpusHead ver=%u ch=%u preskip=%lu input_rate=%lu\n",
                 (unsigned)ver, (unsigned)ch, (unsigned long)preskip_left,
                 (unsigned long)inrate);
          head_seen = true;
        } else if (len >= 8 && memcmp(pkt, "OpusTags", 8) == 0) {
          // comment map — nothing to do
        } else {
          printf("audio: unexpected first packet (len=%u)\n", (unsigned)len);
        }
        continue;
      }

      if (!dec) { dec_errs++; continue; }
      const int n = opus_decode(dec, pkt, (opus_int32)len, pcm, kFrame, 0);
      if (n < 0) { dec_errs++; continue; }

      // mono -> stereo (L=R), live volume (S119: hoisted once per 960-sample
      // frame), honor preskip (first preskip decoded samples are the
      // encoder's pre-roll — spec-correct A/V alignment).
      const uint32_t vol = g_vol;
      int16_t *st = (int16_t *)chunk;
      for (int k = 0; k < kFrame; k++) {
        int32_t v = 0;
        if (k < n) {
          if (preskip_left) preskip_left--;
          else v = pcm[k];
        }
        const int16_t s16 = (int16_t)(((int32_t)v * (int32_t)vol) >> 7);
        st[2 * k] = s16;
        st[2 * k + 1] = s16;
      }

      // watermark pacing, then ring write (drop if the ring is full)
      while (ringFrames() >= kWatermark && g_clip_gen == gen)
        vTaskDelay(pdMS_TO_TICKS(2));
      if (g_clip_gen != gen) break;
      if (ringFrames() >= kRingFrames) { drops++; continue; }
      ringWrite(chunk);
      pkts++;
      s_pkts++;
    }
  }

  if (dec) opus_decoder_destroy(dec);
  free(page);
  if (f) f.close();
  printf("audio: %s done — pages=%lu crc_bad=%lu pkt=%lu dec_err=%lu drop=%lu (gen %lu)\n",
         name, (unsigned long)pages, (unsigned long)crc_bad, (unsigned long)pkts,
         (unsigned long)dec_errs, (unsigned long)drops, (unsigned long)gen);
}

static void decodeTask(void *) {
  if (!s_ogg_tab_ok) oggCrcTab();
  uint32_t last_gen = 0xFFFFFFFFu;  // so the CURRENT clip (gen 0/1) is handled
  for (;;) {
    const uint32_t gen = g_clip_gen;
    if (gen == last_gen) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }
    last_gen = gen;

    // Capture the clip name (the player owns g_clip_name — re-check the gen
    // across the read so a mid-read switch is caught).
    char name[64] = {0};
    for (int tries = 0; tries < 100; tries++) {
      const uint32_t g0 = g_clip_gen;
      snprintf(name, sizeof(name), "%s", g_clip_name);
      if (g_clip_gen == g0) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    // stem: "intro.rfav" -> "intro"
    char stem[48] = {0};
    strncpy(stem, name, sizeof(stem) - 1);
    char *dot = strrchr(stem, '.');
    if (dot) *dot = 0;

    ringClear();  // stale ring data from the previous clip must not play

    char path[128];
    snprintf(path, sizeof(path), "/avatars/%s/34k_opus/%s%s", av::activeName(), stem, kAudioSuffix);  // S122
    File f = SD_MMC.open(path);
    if (!f) {
      printf("audio: no file %s (silent clip: %s)\n", path, name);
      vTaskDelay(pdMS_TO_TICKS(50));  // don't hammer VFS on rapid gen bumps
      continue;
    }
    // S114 diagnostic (no-audio investigation C): SD stub (~0.3-2.3 KB, the
    // 3-page/8-pkt files) vs the real set (~28-49 KB / 13 pages). Size first,
    // then demux verdicts the rest.
    printf("audio: open %s size=%lu B\n", path, (unsigned long)f.size());

    // ---- 34 ms start delay from the video clip's FIRST PRESENTED frame ----
    const int64_t w0 = esp_timer_get_time();
    bool anchored = false;
    while (g_clip_gen == gen) {
      if (g_clip_first_pres_us > 0) {
        anchored = true;
        if (esp_timer_get_time() >= g_clip_first_pres_us + kAuStartDelayUs) break;
      }
      if (esp_timer_get_time() - w0 > (int64_t)kAuAnchorTimeoutUs) break;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (!anchored)
      printf("audio: start anchor never came — playing late (gen %lu)\n",
             (unsigned long)gen);
    if (g_clip_gen != gen) {
      if (f) f.close();
      continue;
    }

    playOneClip(f, name, gen);
  }
}

// ------------------------------- init ---------------------------------------
bool audioInit() {
  // 1. ES8388 — the official 28-register sequence (M5Unified master), all via
  //    the internal I2C that M5.begin() already brought up. Non-fatal.
  int fails = 0;
  for (const auto &r : kEs8388Regs)
    if (!M5.In_I2C.writeRegister8(kCodecAddr, r[0], r[1], 400000)) fails++;
  const uint8_t rb19 = M5.In_I2C.readRegister8(kCodecAddr, 0x19, 400000);
  printf("audio: ES8388 @0x%02X — 28 regs, fails=%d, reg0x19 readback=0x%02X (wrote 0x20; bit-1 status ok)\n",
         kCodecAddr, fails, rb19);

  // 2. I2S0 master 48 kHz — mirrors M5Unified master _setup_i2s verbatim
  //    (driver-managed clock; 48000*512 = 24.6 MHz < either P4 source, so
  //    the 256xFS multiple stands on both the 160 MHz PLL and 40 MHz XTAL).
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  cc.auto_clear = true;
  if (i2s_new_channel(&cc, &s_tx, nullptr) != ESP_OK) {
    printf("audio: I2S new_channel FAIL — audio off\n");
    return false;
  }
  i2s_std_config_t sc;
  memset(&sc, 0, sizeof(sc));
  sc.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
  sc.clk_cfg.sample_rate_hz = (int)kFs;
  sc.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
#if defined(I2S_LL_DEFAULT_CLK_FREQ)
  if ((uint64_t)kFs * 512 > I2S_LL_DEFAULT_CLK_FREQ)
    sc.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_128;
#endif
  sc.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
  sc.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
  sc.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;  // we fill L=R ourselves
  sc.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  sc.slot_cfg.ws_width = 16;
  sc.slot_cfg.ws_pol = false;
  sc.slot_cfg.bit_shift = true;
#if defined(SOC_I2S_HW_VERSION_1)
  sc.slot_cfg.msb_right = false;
#else
  sc.slot_cfg.left_align = true;    // P4 = HW v2 fields
  sc.slot_cfg.big_endian = false;
  sc.slot_cfg.bit_order_lsb = false;
#endif
  sc.gpio_cfg.mclk = GPIO_NUM_30;
  sc.gpio_cfg.bclk = GPIO_NUM_27;
  sc.gpio_cfg.ws = GPIO_NUM_29;
  sc.gpio_cfg.dout = GPIO_NUM_26;
  sc.gpio_cfg.din = (gpio_num_t)I2S_PIN_NO_CHANGE;  // E2: (-1) is int; cast like M5 master
  if (i2s_channel_init_std_mode(s_tx, &sc) != ESP_OK) {
    printf("audio: I2S init_std FAIL — audio off\n");
    i2s_del_channel(s_tx);
    s_tx = nullptr;
    return false;
  }
  if (i2s_channel_enable(s_tx) != ESP_OK) {
    printf("audio: I2S enable FAIL — audio off\n");
    i2s_del_channel(s_tx);
    s_tx = nullptr;
    return false;
  }
  printf("audio: I2S0 master %u kHz up (MCK30 BCK27 WS29 DOUT26, mclk_mult=%ld)\n",
         (unsigned)(kFs / 1000), (long)sc.clk_cfg.mclk_multiple);

  // 3. speaker amp — PI4IOE expander (M5.begin() init'd it for Tab5)
  const bool amp = M5.In_I2C.bitOn(kAmpAddr, kAmpReg, kAmpBit, 400000);
  printf("audio: amp %s (PI4IOE 0x%02X reg 0x%02X bit 0x%02X)\n",
         amp ? "ON" : "on-FAIL", kAmpAddr, kAmpReg, kAmpBit);

  // 4. PSRAM ring (46,080 B = 360 x 128 B — alignment-exact, so the msync
  //    region never spills past the allocation) + M2C to drop stale lines.
  size_t align = 128, qa = 0;
  if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &qa) == ESP_OK && qa > align)
    align = qa;
  s_ring = (uint8_t *)heap_caps_aligned_alloc(align, kRingBytes,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!s_ring) {
    printf("audio: ring alloc FAIL (%u B) — audio off\n", (unsigned)kRingBytes);
    i2s_channel_disable(s_tx);
    i2s_del_channel(s_tx);
    s_tx = nullptr;
    return false;
  }
  auSync(s_ring, kRingBytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  // 5. tasks — both core 1 (video owns core 0)
  // S169: PSRAM stacks (decode = SD + opus, feed = ring -> I2S copy; no flash/NVS).
  createTaskPsram(decodeTask, "au_dec", 8192, nullptr, 3, nullptr, 1);
  createTaskPsram(feedTask, "au_feed", 4096, nullptr, 4, nullptr, 1);

  printf("audio: READY — opus %s, decoder probe %s, ring %u B (%d frames x %d), vol %u/127\n",
         opusVersion(), probeDecoder() ? "ok" : "FAIL", (unsigned)kRingBytes,
         kRingFrames, kFrame, (unsigned)g_vol);
  return true;
}

// --------------------------- probes (S110, kept) -----------------------------
const char* opusVersion() { return opus_get_version_string(); }

bool probeDecoder() {
  int err = 0;
  OpusDecoder *d = opus_decoder_create(48000, 1, &err);
  if (!d) return false;
  opus_decoder_destroy(d);
  return true;
}

}  // namespace rfav_audio
