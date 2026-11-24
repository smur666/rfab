// RFAV-3 container reader — impl. (spec: docs/Tab5/rfav2_720p_spec_rev3.md)
#include "rfav3.h"
#include <esp_cache.h>
#include <string.h>

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
}  // namespace

bool Rfav3::open(File &f) {
  ok = false;
  err[0] = 0;

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
                      uint32_t *jpg_got, uint8_t *scratch) {
  if (i < 0 || (uint32_t)i >= frame_count) {
    snprintf(err, sizeof(err), "readFrame: bad index %d", i);
    return false;
  }
  if (jpg_got) *jpg_got = 0;
  const size_t mb = maskBytes();
  const uint32_t base = off[i];
  const uint32_t jl = jpeg_len[i];

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
                           uint8_t *scratch) {
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
  const uint32_t start = off[i0];
  const uint32_t end = off[i0 + count - 1] + mb + jpeg_len[i0 + count - 1];
  const size_t total = (size_t)(end - start);
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
