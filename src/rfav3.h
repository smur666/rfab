// RFAV-3 (SD-optimized) container reader — spec: docs/Tab5/rfav2_720p_spec_rev3.md
//
//   [ Header 64 B ][ Frame table N x 16 B ][ (mask + JPEG) x N ]
//
// Header (little-endian):
//   0   u32 magic "RFAV"            12  u16 frame_count
//   4   u16 version (3)             14  u16 reserved
//   6   u16 width (720)             16  u32 frame_table_offset
//   8   u16 height (1280)           20  u32 frame_data_offset
//   10  u8  fps                     24  u32 frame_data_size
//   11  u8  flags (b0 loop | b1 has_mask | b2 mask_4bit)
//   28  u32 crc32_header
//
// Frame table entry (16 B): u32 offset (ABSOLUTE file offset of the mask)
//                           u32 jpeg_len | u32 reserved | u32 frame_crc32
//
// Frame i: [ mask maskBytes() ][ JPEG jpeg_len[i] ]   (strictly interleaved,
// mask FIRST — the rev3 sequential-read optimization)
//
// 1-bit mask: 1 bit/pixel, MSB first, row stride = width/8 (90 B @720),
// 1 = opaque, 0 = transparent, no header/padding.
#pragma once

#include <Arduino.h>
#include <SD_MMC.h>

struct Rfav3 {
  static constexpr uint16_t MAX_FRAMES = 4096;

  bool ok = false;
  char err[160] = {0};

  // ---- parsed header ----
  uint32_t magic = 0;
  uint16_t version = 0;
  uint16_t width = 0;
  uint16_t height = 0;
  uint8_t  fps = 0;
  uint8_t  flags = 0;
  uint16_t frame_count = 0;
  uint32_t frame_table_offset = 0;
  uint32_t frame_data_offset = 0;
  uint32_t frame_data_size = 0;
  uint32_t hdr_crc = 0;

  // ---- parsed frame table ----
  // S169: the two MAX_FRAMES tables (32 KB) live in PSRAM, allocated on the
  // first open() — as plain arrays they cost 32 KB of INTERNAL .bss per
  // Rfav3 instance (rf + avatar.cpp's coverage probe = 64 KB).
  uint32_t *off      = nullptr;  // absolute mask offset
  uint32_t *jpeg_len = nullptr;

  bool   loopFlag()  const { return (flags & 0x01) != 0; }
  bool   hasMask()   const { return (flags & 0x02) != 0; }
  bool   mask4Bit()  const { return (flags & 0x04) != 0; }
  size_t maskBytes() const {
    return (size_t)width * height * (mask4Bit() ? 4 : 1) / 8;
  }

  // Parse header + frame table. Returns false with err set on any violation.
  // `f` is left positioned just past the table (readFrame seeks per frame).
  bool open(File &f);

  // Seek to frame i and read mask then JPEG, strictly sequential.
  //   mask_buf: capacity >= maskBytes()
  //   jpg_buf:  capacity >= jpeg_len[i]
  //   *jpg_got: bytes of JPEG read (out, optional)
  //   scratch:  S128 — optional PSRAM scratch >= maskBytes()+jpeg_len[i].
  //             When given, mask+JPEG (contiguous in the file per the rev3
  //             spec) are fetched in ONE seek + ONE contiguous SD transfer
  //             into scratch, C2M'd (S83 discipline), then split into
  //             mask_buf/jpg_buf in PSRAM. nullptr = legacy 2-seek/2-read
  //             path (fallback). Each CALLER owns its own scratch — the
  //             read task and the player inline path can both call this on
  //             the shared Rfav3 (state is read-only after open).
  //   fd/scratch_cap: S169 — optional raw VFS fd (openRaw) + scratch capacity.
  //             When fd >= 0, scratch is 128-B aligned and scratch_cap is
  //             big enough, the read is SECTOR-ALIGNED (512 B start/length)
  //             via pread() straight into scratch: the SDMMC driver DMAs
  //             directly to PSRAM and never needs its INTERNAL-RAM bounce
  //             buffer (allocate_dma_buf) — the S133-S168 SD starvation
  //             under WiFi. Anything else -> the File path above (unchanged).
  bool readFrame(File &f, int i, uint8_t *mask_buf, uint8_t *jpg_buf,
                 uint32_t *jpg_got = nullptr, uint8_t *scratch = nullptr,
                 int fd = -1, size_t scratch_cap = 0);

  // S129 BATCH READ: up to `count` frames starting at i0 in ONE contiguous
  // SD transfer. rev3 frames are [mask][jpeg] back-to-back, so a run of
  // PHYSICALLY CONTIGUOUS frames (checked against the parsed table — no SD
  // access) is fetched with ONE seek + ONE readAll into `scratch`, C2M'd
  // (S83 discipline), then split into the per-frame buffers.
  // The batch SHRINKS to the longest contiguous prefix (returns 1 = the
  // S128 combined-read shape); it only FAILS (returns 0, err set) on a
  // seek/short-read/cache error, which the caller retries.
  //   mask_bufs[j]/jpg_bufs[j]: frame i0+j destinations (j < count)
  //   jpg_gots[j]:  out, JPEG bytes (= the table's exact jpeg_len[i0+j])
  //   scratch:      PSRAM, capacity >= count*(maskBytes()+max jpeg_len);
  //                 nullptr falls back to the legacy 2-seek path for ONE
  //                 frame (count is clamped to 1).
  // Returns the number of frames read (0 = error).
  //   fd/scratch_cap: S169 raw aligned path (see readFrame). With a known
  //                 scratch_cap the batch also SHRINKS to fit the scratch
  //                 (pre-S169 the batch was only bounded by kMaxJpeg sizing).
  int readFramesBatch(File &f, int i0, int count, uint8_t *const *mask_bufs,
                      uint8_t *const *jpg_bufs, uint32_t *jpg_gots,
                      uint8_t *scratch, int fd = -1, size_t scratch_cap = 0);

  // S169: open/close a raw read-only VFS fd for `sd_path` (an SD_MMC path
  // like "/avatars/x/rfav/y.rfav"; the "/sdcard" mount prefix is added).
  // Each caller owns its own fd (pread is offset-explicit, so no shared
  // file position). Returns -1 on failure (callers fall back to File).
  static int openRaw(const char *sd_path);
  static void closeRaw(int &fd);
  // S169: generic no-bounce read of [off, off+len) from a raw fd into a
  // 128-B-aligned PSRAM scratch (cap >= round512(len + off%512)).
  // 1 = ok (*out = first byte), 0 = I/O error, -1 = not applicable.
  static int readAligned(int fd, uint32_t off, size_t len, uint8_t *scratch,
                         size_t cap, uint8_t **out);
  // Extra scratch bytes the aligned path needs beyond the payload.
  static constexpr size_t kAlignSlack = 1024;
};
