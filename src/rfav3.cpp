// RFAV-3 container reader — impl. (spec: docs/Tab5/rfav2_720p_spec_rev3.md)
#include "rfav3.h"
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

namespace {
// Read exactly `len` bytes (the SD layer chunks underneath).
bool readAll(File &f, uint8_t *dst, size_t len) {
  size_t got = 0;
  while (got < len) {
    int n = f.read(dst + got, len - got);
    if (n <= 0) return false;
    got += (size_t)n;
  }
  return true;
}

// S128: S83 cache discipline for the combined-read scratch. The SDMMC DMA
// lands the fresh bytes in the scratch, so C2M the scratch range BEFORE any
// CPU read of it (the split memcpys below). Same shape as main.cpp's
// syncChunked() (S99: one HW range op per 64 KB; 128 B L2-line rounding) —
// the P4 has no CPU<->DMA coherence.
static bool scratchC2M(const uint8_t *p, size_t size) {
  constexpr size_t kChunk = 64*1024;
  uintptr_t a = (uintptr_t)p & ~(uintptr_t)127;
  const uintptr_t e = ((uintptr_t)p + size + 127) & ~(uintptr_t)127;
  for (; a < e; a += kChunk) {
    size_t n = (e-a) < kChunk ? (e-a) : kChunk;
    if (esp_cache_msync((void*)a, n, ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                        ESP_CACHE_MSYNC_FLAG_TYPE_DATA) != ESP_OK)
      return false;
  }
  return true;
}
// S169: sector-aligned raw read. Reads [start, start+len) by fetching the
// enclosing 512-B-aligned span [a0, a1) with pread() straight into the
// 128-B-aligned PSRAM scratch. Every disk_read FATFS issues then targets an
// aligned DMA-capable buffer, so sdmmc never allocates its internal bounce.
// Cache: C2M+INVALIDATE before the DMA (no dirty line can later clobber the
// DMA'd bytes), M2C after (CPU sees the fresh bytes) — P4 has no CPU<->DMA
// coherence (S83 discipline).
// Returns 1 = ok (*out = first payload byte), 0 = I/O error (caller retries),
// -1 = not applicable (caller uses the legacy File path).
constexpr size_t kSec = 512;
int s_raw_errno = 0;  // S169: last raw-path failure cause (errno, or -1000 cache / -2000 short)
int alignedRead(int fd, uint32_t start, size_t len, uint8_t *scratch,
                size_t cap, uint8_t **out) {
  if (fd < 0 || !scratch || ((uintptr_t)scratch & 127)) return -1;
  const uint32_t a0 = start & ~(uint32_t)(kSec - 1);
  const size_t head = start - a0;
  const size_t n = (head + len + kSec - 1) & ~(kSec - 1);
  if (n > cap) return -1;
  if (esp_cache_msync(scratch, n, ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                      ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                      ESP_CACHE_MSYNC_FLAG_TYPE_DATA) != ESP_OK) {
    s_raw_errno = -1000;
    return 0;
  }
  size_t got = 0;
  while (got < n) {
    const ssize_t r = pread(fd, scratch + got, n - got, (off_t)(a0 + got));
    if (r < 0) { s_raw_errno = errno; return 0; }
    if (r == 0) break;  // EOF inside the rounded tail is fine
    got += (size_t)r;
  }
  if (got < head + len) { s_raw_errno = -2000; return 0; }
  // S169b: NOT a plain M2C. At EOF FatFs reads the final PARTIAL sector
  // into its FIL buffer and CPU-memcpys it into scratch — those bytes can
  // still be dirty in cache, and a plain invalidate discarded them (the
  // last-frame-of-clip 'JPEG decode e=259/263'). C2M|INVALIDATE writes back
  // dirty (= CPU-written, authoritative) lines, then drops clean (possibly
  // stale) lines so the CPU re-reads the DMA'd bytes from PSRAM.
  if (esp_cache_msync(scratch, n, ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                      ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                      ESP_CACHE_MSYNC_FLAG_TYPE_DATA) != ESP_OK) {
    s_raw_errno = -1001;
    return 0;
  }
  *out = scratch + head;
  return 1;
}
}  // namespace

int Rfav3::openRaw(const char *sd_path) {
  if (!sd_path || !*sd_path) return -1;
  char p[192];
  snprintf(p, sizeof(p), "/sdcard%s", sd_path);
  return ::open(p, O_RDONLY);
}

int Rfav3::readAligned(int fd, uint32_t off, size_t len, uint8_t *scratch,
                       size_t cap, uint8_t **out) {
  return alignedRead(fd, off, len, scratch, cap, out);
}

void Rfav3::closeRaw(int &fd) {
  if (fd >= 0) ::close(fd);
  fd = -1;
}

bool Rfav3::open(File &f) {
  ok = false;
  err[0] = 0;
  if (!off || !jpeg_len) {  // S169: PSRAM tables, once per instance
    off = (uint32_t *)heap_caps_calloc(MAX_FRAMES, sizeof(uint32_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    jpeg_len = (uint32_t *)heap_caps_calloc(MAX_FRAMES, sizeof(uint32_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!off || !jpeg_len) {
      snprintf(err, sizeof(err), "frame table PSRAM alloc failed");
      return false;
    }
  }

  uint8_t h[64];
  if (!readAll(f, h, sizeof(h))) {
    snprintf(err, sizeof(err), "header: short read");
    return false;
  }
  if (memcmp(h, "RFAV", 4) != 0) {
    snprintf(err, sizeof(err), "bad magic (want \"RFAV\")");
    return false;
  }
  memcpy(&magic,      h + 0, 4);
  memcpy(&version,    h + 4, 2);
  memcpy(&width,      h + 6, 2);
  memcpy(&height,     h + 8, 2);
  fps   = h[10];
  flags = h[11];
  memcpy(&frame_count,        h + 12, 2);
  memcpy(&frame_table_offset, h + 16, 4);
  memcpy(&frame_data_offset,  h + 20, 4);
  memcpy(&frame_data_size,    h + 24, 4);
  memcpy(&hdr_crc,            h + 28, 4);

  if (version != 3) {
    snprintf(err, sizeof(err), "unsupported version %u (want 3)", version);
    return false;
  }
  if (width == 0 || height == 0) {
    snprintf(err, sizeof(err), "zero width/height");
    return false;
  }
  if (fps == 0 || fps > 60) {
    snprintf(err, sizeof(err), "fps %u out of range", fps);
    return false;
  }
  if (!hasMask()) {
    snprintf(err, sizeof(err), "flags: has_mask clear (rev3 profile needs mask)");
    return false;
  }
  if (mask4Bit()) {
    snprintf(err, sizeof(err), "4-bit mask flag set (POC supports 1-bit only)");
    return false;
  }
  if (frame_count == 0 || frame_count > MAX_FRAMES) {
    snprintf(err, sizeof(err), "frame_count %u out of range", frame_count);
    return false;
  }
  if (frame_table_offset < 64) {
    snprintf(err, sizeof(err), "frame_table_offset %lu too small",
             (unsigned long)frame_table_offset);
    return false;
  }

  // Frame table (N x 16 B) with ABSOLUTE offsets.
  if (!f.seek(frame_table_offset)) {
    snprintf(err, sizeof(err), "seek to table failed");
    return false;
  }
  for (uint16_t i = 0; i < frame_count; i++) {
    uint8_t e[16];
    if (!readAll(f, e, sizeof(e))) {
      snprintf(err, sizeof(err), "table: short read at entry %u", i);
      return false;
    }
    uint32_t o, j, res, c;
    memcpy(&o,   e + 0, 4);
    memcpy(&j,   e + 4, 4);
    memcpy(&res, e + 8, 4);
    memcpy(&c,   e + 12, 4);
    (void)res; (void)c;  // reserved / optional per-frame CRC: ignored by POC
    if (o < frame_data_offset || o >= frame_data_offset + frame_data_size) {
      snprintf(err, sizeof(err), "table: frame %u offset %lu outside data region",
               i, (unsigned long)o);
      return false;
    }
    if (j == 0) {
      snprintf(err, sizeof(err), "table: frame %u jpeg_len 0", i);
      return false;
    }
    off[i] = o;
    jpeg_len[i] = j;
  }

  ok = true;
  return true;
}

bool Rfav3::readFrame(File &f, int i, uint8_t *mask_buf, uint8_t *jpg_buf,
                      uint32_t *jpg_got, uint8_t *scratch, int fd,
                      size_t scratch_cap) {
  if (i < 0 || (uint32_t)i >= frame_count) {
    snprintf(err, sizeof(err), "readFrame: bad index %d", i);
    return false;
  }
  if (jpg_got) *jpg_got = 0;
  const size_t mb = maskBytes();
  const uint32_t base = off[i];
  const uint32_t jl = jpeg_len[i];

  if (scratch && fd >= 0) {
    // S169: sector-aligned raw read (no sdmmc internal bounce).
    uint8_t *p = nullptr;
    const int ar = alignedRead(fd, base, mb + jl, scratch, scratch_cap, &p);
    if (ar == 0) {
      snprintf(err, sizeof(err), "readFrame: raw aligned read failed (errno %d)", s_raw_errno);
      return false;
    }
    if (ar == 1) {
      memcpy(mask_buf, p, mb);
      memcpy(jpg_buf, p + mb, jl);
      if (jpg_got) *jpg_got = jl;
      return true;
    }
    // ar == -1: not applicable -> legacy File paths below
  }

  if (scratch) {
    // S128 COMBINED READ: frame i is [mask mb][JPEG jl] back-to-back in the
    // file (rev3 spec) -> ONE seek + ONE contiguous SD transfer, split in
    // PSRAM. Halves the per-frame seeks/commands — the lever on the slow
    // Samsung EVO Select U1 card (S127 jank: sustained 83-95 ms/frame).
    if (!f.seek(base)) {
      snprintf(err, sizeof(err), "readFrame: seek frame failed");
      return false;
    }
    if (!readAll(f, scratch, mb + jl)) {
      snprintf(err, sizeof(err), "readFrame: frame short read");
      return false;
    }
    if (!scratchC2M(scratch, mb + jl)) {
      snprintf(err, sizeof(err), "readFrame: scratch C2M failed");
      return false;
    }
    memcpy(mask_buf, scratch, mb);
    memcpy(jpg_buf, scratch + mb, jl);
    if (jpg_got) *jpg_got = jl;
    return true;
  }

  if (!f.seek(base)) {
    snprintf(err, sizeof(err), "readFrame: seek mask failed");
    return false;
  }
  if (!readAll(f, mask_buf, mb)) {
    snprintf(err, sizeof(err), "readFrame: mask short read");
    return false;
  }
  if (!f.seek(base + mb)) {
    snprintf(err, sizeof(err), "readFrame: seek jpeg failed");
    return false;
  }
  if (!readAll(f, jpg_buf, jl)) {
    snprintf(err, sizeof(err), "readFrame: jpeg short read");
    return false;
  }
  if (jpg_got) *jpg_got = jl;
  return true;
}

int Rfav3::readFramesBatch(File &f, int i0, int count,
                           uint8_t *const *mask_bufs,
                           uint8_t *const *jpg_bufs, uint32_t *jpg_gots,
                           uint8_t *scratch, int fd, size_t scratch_cap) {
  if (i0 < 0 || (uint32_t)i0 >= frame_count) {
    snprintf(err, sizeof(err), "readFramesBatch: bad index %d", i0);
    return 0;
  }
  if (count < 1) count = 1;
  const uint32_t left = frame_count - (uint32_t)i0;
  if ((uint32_t)count > left) count = (int)left;
  if (!scratch) {
    // Defensive: no scratch -> ONE frame via the legacy 2-seek path.
    if (count > 1) count = 1;
    return readFrame(f, i0, mask_bufs[0], jpg_bufs[0], jpg_gots, nullptr) ? 1 : 0;
  }
  const size_t mb = maskBytes();
  // Contiguity from the TABLE (no SD access): frame k+1 must start exactly
  // where frame k's [mask][jpeg] ends. A gap shrinks the batch to the
  // longest contiguous prefix (a single frame is always "contiguous").
  for (int k = 0; k + 1 < count; k++) {
    if (off[i0 + k + 1] != off[i0 + k] + mb + jpeg_len[i0 + k]) {
      count = k + 1;
      break;
    }
  }
  // S169: never overrun the scratch — shrink until the (aligned) span fits.
  if (scratch_cap) {
    while (count > 1) {
      const size_t span = (size_t)(off[i0 + count - 1] + mb +
                                   jpeg_len[i0 + count - 1] - off[i0]);
      if (span + kAlignSlack <= scratch_cap) break;
      count--;
    }
  }
  const uint32_t start = off[i0];
  const uint32_t end = off[i0 + count - 1] + mb + jpeg_len[i0 + count - 1];
  const size_t total = (size_t)(end - start);
  if (fd >= 0) {
    // S169: sector-aligned raw read (no sdmmc internal bounce).
    uint8_t *p = nullptr;
    const int ar = alignedRead(fd, start, total, scratch, scratch_cap, &p);
    if (ar == 0) {
      snprintf(err, sizeof(err), "readFramesBatch: raw aligned read failed (%d frames, errno %d)", count, s_raw_errno);
      return 0;
    }
    if (ar == 1) {
      size_t cursor = 0;
      for (int k = 0; k < count; k++) {
        const uint32_t jl = jpeg_len[i0 + k];
        memcpy(mask_bufs[k], p + cursor, mb);
        memcpy(jpg_bufs[k], p + cursor + mb, jl);
        if (jpg_gots) jpg_gots[k] = jl;
        cursor += mb + jl;
      }
      return count;
    }
    // ar == -1 -> legacy File path below
  }
  if (!f.seek(start)) {
    snprintf(err, sizeof(err), "readFramesBatch: seek failed");
    return 0;
  }
  if (!readAll(f, scratch, total)) {
    snprintf(err, sizeof(err), "readFramesBatch: short read (%d frames)", count);
    return 0;
  }
  if (!scratchC2M(scratch, total)) {
    snprintf(err, sizeof(err), "readFramesBatch: scratch C2M failed");
    return 0;
  }
  size_t cursor = 0;
  for (int k = 0; k < count; k++) {
    const uint32_t jl = jpeg_len[i0 + k];
    memcpy(mask_bufs[k], scratch + cursor, mb);
    memcpy(jpg_bufs[k], scratch + cursor + mb, jl);
    if (jpg_gots) jpg_gots[k] = jl;
    cursor += mb + jl;
  }
  return count;
}
