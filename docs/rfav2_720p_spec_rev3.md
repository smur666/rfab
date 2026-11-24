# RFAV-3 (SD-Optimized) — 720×1280 (Pre-rotated 90° CW) MJPEG + 1-bit Alpha Mask

**Target:** M5Stack Tab5 / ESP32-P4  
**Primary use case:** Streaming from SD card at 12 fps  
**Design goals:** Maximum sequential-read efficiency, constant-size mask, minimal seeking, simple decoder, **no on-device rotation**.

> **Revision note (ground-truth update).** This revision matches the shipped POC player (`rfav3.h`, `rfav3.cpp`, `main.cpp`):
>
> - The stored orientation is the Tab5's **native portrait, 720 wide × 1280 tall**. The JPEG frames, the 1-bit masks and the background image are all **pre-rotated 90° CW at encode time**. Earlier drafts described 1280×720 with a 160 B mask stride. The reader uses a 90 B stride.
> - Rotating on the device (transposing each frame into the native portrait buffer) cost about 70–90 ms per frame and was removed from the design.
> - After compositing, the UI layer is drawn in a `setRotation(1)` window, then rotation returns to 0 (section 1.1).
> - Audio is a sidecar Opus file at about 34 kbit/s, remuxed from the WebM source (section 5).
> - The reference reader's validation limits and ignored fields are documented in sections 2 and 3.
> - The reference player has grown since (S122): a per-avatar SD layout, decode into a scratch buffer, a two-stage SD prefetch ring, group-level background skipping, a fixed-grid present kick and an Opus A/V anchor. These are described, as informative text, in sections 5, 7 and 10.
> - **SD read path (S128/S129):** the reader now issues **one combined mask+JPEG transfer per frame** (S128) and the prefetch read task issues a **4-frame batch read** — up to `4 × (115,200 + jpeg_len)` bytes in a single contiguous SD transfer, split in a PSRAM scratch (S129). Both were the fix for sustained jank on the field unit's **U1-class card (Samsung EVO Select U1 32 GB)**, which sustains only ~2–3 MB/s and takes 83–95 ms per ~165 KB frame with the old 2-seek/2-read path. Section 6 items 9–10 and section 7 describe them.
> - **Memory/heap (S127, S132, S135–S143):** every frame buffer lives in PSRAM (section 11), the avatar state machine is data-driven from `/avatars/<name>/avatar.json` with an optional top-level `/avatars/config.json` control file (S132; described in `avatar_json_spec.md`), and the internal-SRAM budget is protected against the WiFi/esp_hosted stack and the SD driver's bounce allocations (section 11).

---

### 1. Design Decisions (vs RFAV-2)

| Decision                    | Choice                          | Reason |
|----------------------------|----------------------------------|------|
| Orientation                | **Pre-rotated 90° CW, stored as native portrait 720×1280** | On-device rotation cost about 70–90 ms per frame. Decoding into a native-orientation buffer costs nothing |
| Mask order                 | **Mask first**, then JPEG       | Constant JPEG offset, better sequential reads |
| Mask depth                 | **1-bit primary**               | 115.2 KB fixed, fastest, good enough for hard/soft silhouette |
| 4-bit support              | Optional (flag)                 | Kept for future high-quality short clips (the POC reader rejects it) |
| Frame table location       | Near the beginning              | Fast open, no seek-to-end |
| Audio                      | **External** Opus sidecar (not inside .rfav) | Simpler, remuxed from the WebM source with no re-encode, loops independently |
| Interleaving               | Strict Mask + JPEG pairs        | One sequential read per frame possible |

#### 1.1 Orientation and Rotation

**Native buffer.** The Tab5's 5" MIPI-DSI panel is natively portrait, 720×1280. M5GFX rotation 0 is that native orientation. The player's draw buffer is one native-size PSRAM buffer: 720 × 1280 × 2 B (RGB565) = 1,843,200 B, which is 1440 B per row. The present step copies the **whole** buffer (`display(0, 0, 720, 1280)`) by DMA2D into the driver's non-scanned panel buffer and swaps on vsync.

**Pre-rotation at encode.** Source clips are authored landscape (1280×720 WebM with alpha). The encoder rotates every frame **90° clockwise before anything else** (`webm_to_rfav.py --rotate cw`, the default; ffmpeg `transpose=1`, "source top row → right column"). The rotation is applied once at the PNG-extraction step, so the color JPEG and the 1-bit mask inherit the same rotated pixels and cannot disagree. For a source pixel (x, y), x = 0..1279 left→right and y = 0..719 top→bottom, the stored pixel is:

```text
u = 719 - y      // column within the 720-px row   (source top  -> stored right)
v = x            // row index 0..1279              (source left -> stored top)
```

| Landscape source / view | Stored (native) image |
|---|---|
| Left margin | Top rows |
| Right margin | Bottom rows |
| Horizontal position `x` | Row index `v = x` |
| Top of picture (head) | Right columns |
| Bottom of picture (feet) | Left columns |

With the tablet turned to landscape, the native top edge is on the viewer's left and the original landscape picture is seen upright. The encode rotation and the way the tablet is held must agree. The `--rotate ccw` option exists for the opposite pairing.

**Background.** The background image is rotated CW 90° as well. It is a 720×1280 JPEG that the player decodes **straight into its background buffer**, which is in the same native buffer space (720-px rows) as the draw buffer. A landscape background is not part of the format. The POC player can accept one through a one-shot transpose at boot, as a fallback that is never on the per-frame path.

**Why the rotation is baked in.** An on-device landscape path (render in landscape, then CPU-transpose each frame into the native buffer) cost about 70–90 ms per frame and gave about 8 fps. The shipped design decodes into a native-orientation scratch buffer, composites against the native background into the native draw buffer and pushes that buffer as is, so no frame is rotated on the device.

**Render order and the UI layer.** Video, mask and background all live in native buffer space at rotation 0. Each frame:

1. Decode into a scratch buffer (never into the presented draw buffer) and composite into the draw buffer, at rotation 0.
2. After the composite is written back, call `setRotation(1)` and draw the UI layer: a 700×56 status bar in the rotated window, with the status line and a transient second line for touch feedback.
3. Call `setRotation(0)` as the frame cycle completes and **before** the whole-buffer present, so the next frame's decode, composite and push run in native space.

M5GFX applies the rotation only as a coordinate mapping. The UI text lands in native buffer space and is pushed with the rest of the frame in the same present. If the text reads backwards, use rotation 3 instead of 1 (this depends on how the tablet is held).

---

### 2. File Layout

```text
[ Header          ]  64 bytes
[ Frame Table     ]  N × 16 bytes
[ Frame Data      ]  (Mask₀ + JPEG₀) (Mask₁ + JPEG₁) ... 
[ Footer          ]  32 bytes (optional, for CRC / future)
```

#### 2.1 Header (64 bytes)

| Offset | Size | Field              | Notes |
|--------|------|--------------------|-------|
| 0      | 4    | magic              | `"RFAV"` |
| 4      | 2    | version            | `3` |
| 6      | 2    | width              | 720 (stored, native portrait) |
| 8      | 2    | height             | 1280 (stored, native portrait) |
| 10     | 1    | fps                | 12 typical (the reader accepts 1–60) |
| 11     | 1    | flags              | see below |
| 12     | 2    | frame_count        | N (the reader accepts 1–4096) |
| 14     | 2    | reserved           | 0 |
| 16     | 4    | frame_table_offset | usually 64 (the reader requires ≥ 64) |
| 20     | 4    | frame_data_offset  | 64 + N×16 |
| 24     | 4    | frame_data_size    | total size of all masks + JPEGs |
| 28     | 4    | crc32_header       | CRC of first 28 bytes (optional). The encoder writes it. The POC reader parses it but does not verify it |
| 32     | 32   | reserved           | 0 (future: title, bg color, etc.) |

**Flags (1 byte):**
- bit 0 : loop (1 = loop). The reference player only prints this bit at open. Whether a clip loops or advances is decided by the avatar state machine (section 7.1)
- bit 1 : has_mask (must be 1 for this profile; the reader rejects 0)
- bit 2 : mask_4bit (0 = 1-bit, 1 = 4-bit; the POC reader rejects 1)
- bits 3–7 : reserved (0)

**Reader checks (reference `Rfav3::open`):** magic `"RFAV"`; version 3; width and height non-zero; fps 1–60; `has_mask` set; `mask_4bit` clear; `frame_count` 1–4096; `frame_table_offset` ≥ 64; every frame offset inside `[frame_data_offset, frame_data_offset + frame_data_size)`; every `jpeg_len` non-zero.

The reader does not require 720×1280. The player logs a warning for any other size. A 1280×720 file has the same pixel count and would decode rotated.

#### 2.2 Frame Table (N × 16 bytes)

Each entry:

```text
u32 offset          // absolute file offset of this frame’s mask
u32 jpeg_len        // length of the JPEG that follows the mask
u32 reserved        // 0 (or future per-frame flags / crc)
u32 frame_crc32     // optional CRC of (mask + jpeg)
```

The POC reader ignores `reserved` and `frame_crc32`.

Because the mask is fixed size:
- 1-bit mask size = `115200` bytes (exactly)
- JPEG always starts at `offset + 115200`

#### 2.3 Frame Data Layout (Mask-first)

```text
Frame i:
  [0x00000]  1-bit mask     115200 bytes   (fixed)
  [0x1C200]  JPEG data      jpeg_len bytes (variable)
```

This is the key optimization.

---

### 3. 1-bit Mask Format (Primary)

- Stored resolution: **720 × 1280** (pre-rotated CW, see 1.1)
- 1 bit per pixel, **MSB first**
- Row stride = **90 bytes** (720 / 8)
- Total size = **115200 bytes** exactly
- 1 = opaque, 0 = transparent
- No header, no padding
- Mask row *r* corresponds to JPEG row *r* and to row *r* of the native draw buffer.

A row is exactly 720 bits, so the bit index equals the pixel index and the decoder can treat the mask as a pure bitstream.

---

### 4. Optional 4-bit Mask (Secondary)

If `flags.bit2 == 1`:

- 4 bits per pixel, two pixels per byte (high nibble = left pixel)
- Size = **460800 bytes**
- Levels 0–15 → alpha 0, 17, 34 … 255 (same mapping as before)

This is intended only for short, high-quality clips that can be staged in PSRAM.  
For normal SD streaming, **use 1-bit**. The POC reader rejects this flag.

---

### 5. Audio Handling (External)

Audio is **not** stored inside the `.rfav` file. The sidecar is the WebM's own Opus stream, **remuxed (stream-copied, no re-encode) at about 34 kbit/s** into a standalone Opus file, so there is no generation loss. For example:

```text
ffmpeg -i idle.webm -vn -c:a copy idle.opus
```

The audio file sits next to the video with the same base name:

```text
idle.rfav
idle.opus
```

Player responsibilities (reference player):

- Load the `.rfav` and its Opus file when the clip is entered.
- **A/V anchor:** record the time the clip's first frame is *presented*, and start the Opus 34 ms after that moment. Do not start audio when decoding begins, because the first present trails the first decode by the prefetch and the fixed-grid present kick.
- Duration is `frame_count / fps` from the header.
- A clip switch re-arms the audio engine. Whether an in-place loop restarts the audio is the audio engine's concern (not reviewed for this revision).
- Mute: while muted the player feeds silence without consuming the audio ring, so unmuting resumes in sync with the video.
- The location and naming of the Opus file are defined by the audio engine (`audio_opus.cpp`): the current player streams `/avatars/<model>/34k_opus/<stem>.34k.opus` (34 kbit/s mono, same stem as the clip), remuxed from the WebM source with `ffmpeg -vn -c:a copy`. Missing file = silent clip, logged once.

This keeps the video container simple and lets the audio be handled independently.

---

### 6. SD Streaming Optimizations Included

1. **Mask-first + fixed size** → constant JPEG offset
2. Frame table near the front → fast open
3. Strict sequential Mask+JPEG pairs → ideal for SD sequential reads
4. 1-bit primary → ~115 KB mask instead of 450 KB
5. Optional per-frame CRC
6. No variable-size headers inside the frame data
7. External audio → no container complexity
8. **Pre-rotated content → no on-device transpose, the JPEG decodes into a native-orientation buffer**
9. **Combined mask+JPEG read (S128).** Because `[mask][JPEG]` is back-to-back in the file, the reader issues **one seek + one contiguous transfer of `115200 + jpeg_len` bytes** into a per-caller PSRAM scratch (kMask1b + kMaxJpeg ≈ 246 KB), C2M-syncs it (S83 discipline: the SDMMC DMA lands the bytes behind the cache), then splits it into the mask and JPEG buffers with two memcpys. Halves the per-frame seeks and SDMMC command restarts.
10. **Multi-frame batch read (S129).** The prefetch read task reads **up to 4 frames per SD transfer**: a table-only contiguity check (frame k+1 must start exactly where frame k's `[mask][JPEG]` ends; any gap shrinks the batch to the longest contiguous prefix, never fails), one seek, one `readAll(total)`, one C2M, then a per-frame split. The batch never crosses the clip end and shrinks to fit the free ring prefix. The 64 KB-chunked C2M and the 500 ms slot runway ride out the slowest transfers. On the U1 card this amortizes the per-transfer command overhead and keeps the card's flash pipeline primed; per-frame `rd=` in the telemetry is the batch time amortized over its frames.

---

### 7. Recommended Player Flow (Tab5)

This describes the reference player (S122). It is informative. Only sections 2–4 define the container.

**Boot**

1. Mount SD, load the active avatar (`/avatars/<model>/avatar.json`, with a built-in fallback) and open its boot clip. Read the 64-byte header + frame table into RAM and validate (section 2.1).
2. Decode the pre-rotated 720×1280 background JPEG straight into a 128 B-aligned, native-orientation background buffer (candidate paths in 7.1).
3. Seed the draw buffer from the background and start the two SD tasks.

**SD tasks (core 0)**

- **Read task.** Reads the clip's frames in order into a private 6-deep ring (mask + JPEG per buffer). It takes no player credit and pauses only when the ring is full.
- **Handoff task.** Polls every 5 ms. When frame *f*'s ring buffer is ready **and** the free-credit of slot `f % 6` has landed, it copies ring → slot, writes the slot back (C2M) and gives the slot-ready semaphore. It never blocks on a credit.
- The ring and slots exist to ride out SD latency spikes: stalls of 226–301 ms on about 1 frame in 4 were observed (S102), and 6 frames is 500 ms of runway at 12 fps. If the SD tasks are not alive, the render task reads inline instead.

**Render task** (single writer of the draw buffer and of M5GFX; budget = 1000 / fps = 83 ms at 12 fps)

```text
a. Gate: wait (100 ms cap) for the previous present's DMA2D copy to finish
b. Take the slot-ready semaphore for frame i (50 ms timeout) and check the slot's frame tag.
   Copy slot -> private buffers (~2 ms) and return the slot credit at once (early release).
   A timeout or a stale tag DROPS the frame: no render, no present
c. Hardware JPEG decode -> RGB565 into the SCRATCH buffer (never the draw buffer);
   the output must be exactly 1,843,200 B
d. M2C-invalidate the scratch buffer (the decoder wrote PSRAM behind the cache) and the draw buffer
e. Composite in 32-row batches (46,080 B, 128 B-aligned); one mask byte = one 8-pixel group:
     0xFF  -> copy 16 B from scratch
     0x00  -> copy 16 B of background, unless the group's skip bit says the draw buffer already holds it
     mixed -> per pixel: scratch where the mask bit is 1, background where it is 0
   A skip bit is set when a group is filled with the background and cleared when it receives anything
   else. Any other draw into the draw buffer (boot text, fatal screen) clears every skip bit
f. C2M-sync each batch right after writing it (64 KB chunked esp_cache_msync, errors counted),
   then a trailing full-buffer C2M as a safety net
g. If the work finished before the frame's ideal time: wait until (ideal - 12 ms) if that is still ahead
   (the fixed-grid kick) and present. Otherwise DROP: skip the present (the panel keeps the last good frame)
h. UI layer: setRotation(1) -> fill the 700x56 bar, draw the status line (+ a transient feedback line)
   -> setRotation(0)
i. Present: display(0, 0, 720, 1280) -> DMA2D copy of the whole buffer -> vsync swap.
   On the first present of a clip, stamp the A/V anchor (section 5)
j. Advance the ideal clock by 1,000,000 / fps microseconds (exactly 12.000 fps); re-anchor if
   more than one slot behind
```

A frame whose slot is stale or late (step b) sleeps to the end of its slot, so the player does not race ahead of the prefetch. A dropped frame (step g) was still composited into the draw buffer. The next present copies the whole buffer, so nothing is lost.

**Fixed-grid present kick.** `ideal[i] = t0 + i × (1,000,000 / fps)` µs is when frame *i* should become visible. The Tab5 glass refreshes at about 65.98 Hz (15.14 ms), so 83.33 ms is 5.50 refreshes. Kicking the present at `ideal − 12 ms` makes visible frames land on a deterministic beat instead of a random vsync each frame.

**SD read shape (current, S128/S129).** The reader (`rfav3.cpp`) has three read paths, all table-driven: (1) the legacy two-seek path (kept as fallback — one seek + read for the mask, one seek + read for the JPEG); (2) the **combined read** — one seek to `off[i]`, one contiguous transfer of `115200 + jpeg_len` bytes into a caller-owned PSRAM scratch (246,272 B = kMask1b + kMaxJpeg), a 64 KB-chunked C2M of the scratch, then two memcpys splitting it into the mask and JPEG buffers; (3) the **4-frame batch read** (`readFramesBatch`, S129) used by the prefetch read task — a table check that frames `i0..i0+count-1` are physically back-to-back (shrinking to the longest contiguous prefix on any gap, no SD access), then one seek, one transfer of the whole range (up to ~985 KB), one C2M and the per-frame split. The player's inline fallback path uses the combined single-frame shape. All three verify short reads and report them; a failed batch is retried (10 attempts, 250 ms apart) before the prefetch parks and the player falls back to inline reads.

#### 7.1 Storage Layout (reference player)

```text
/avatars/<model>/avatar.json        state machine, clips and script (avatar.json v1.0.0, defined elsewhere)
/avatars/<model>/rfav/<clip>.rfav   clips; avatar.json gives clip stems and the player appends ".rfav"
```

Background candidates, tried in order (the first that opens wins; a config.json override that is absent simply falls through):

```text
/avatars/<config.json "bg">         the S132 config.json bg override (first, if set)
/avatars/<bg>                       the avatar's own `bg` value
/avatars/<model>/<bg>
/avatars/<model>/background.jpg
/avatar/reality_fabricator_portrait_bg90.jpg
/avatar/reality_fabricator_pattern_720x1280_rot90.jpg
/avatar/background.jpg
(none opens)                        a 32 px checkerboard
```

A 720×1280 background decodes directly into the background buffer. A 1280×720 background is accepted through the one-shot boot transpose described in 1.1.

Clip lifecycle (loop, advance, interrupt) is decided by the avatar state machine, not by `flags.loop`. The `avatar.json` format itself is outside this spec and was not reviewed for this revision.
---

### 8. Size Estimates (1-bit, 12 fps)

| Item                  | Size          |
|-----------------------|---------------|
| 1-bit mask            | 112.5 KB      |
| JPEG (q=8, typical)   | 40–70 KB      |
| **Per frame**         | **~155–185 KB** |
| 10 second clip        | ~18–22 MB     |
| 19 second clip        | ~34–40 MB     |
| Opus audio (about 34 kbit/s) | ≈ 4.25 KB/s (10 s ≈ 43 KB, 19 s ≈ 81 KB) |

**Measured:** `make_sandwich.rfav` is 112 frames at 12 fps in 16,503,619 B, about 147 KB per frame, with a maximum JPEG of 40,106 B.

This is very comfortable for SD card streaming.

---

### 9. Encoder Notes (reference pipeline)

- Command: `python webm_to_rfav.py <webm> <out.rfav>`. The default is `--rotate cw`. `ccw` and `none` also exist.
- Per frame: extract the PNG with the rotation applied, build the 1-bit mask from the rotated alpha (MSB first, 90 B per row, 1 = opaque), JPEG-encode the rotated color, and write mask then JPEG.
- **Verification:** the baked mask for `make_sandwich.rfav` is a 100.0000% bit-match against the source frame-0 alpha rotated 90° CW (145,798 opaque pixels in both). A CCW rotation matches only 89.9%, which confirmed the direction.
- Audio: remux the WebM's Opus stream to a standalone Opus file (section 5). The video frames are not involved.
- The background is a 720×1280 JPEG rotated CW to match.

---

### 10. Reference Telemetry (1 Hz serial line)

```text
rfav3: f=<i>/<N> real=<fps> FT=<ms> max=<ms> pres=<ms> avg=<ms> (budget <ms>) sd=<ms> dec=<us> iv=<us> c=<us> c2m=<us> ce=<n> cf=<code> cd=<n> drop=<n> ms=<n> pf=<0|1> pff=<n> psram_free=<KB>
```

| Field | Meaning |
|---|---|
| `real` | frames presented in the last second |
| `FT`, `max`, `avg` | this frame's work time, the window's worst, and the mean since playback start (the planning number). The gate wait is included |
| `pres` | duration of the last present call |
| `sd` | slot wait for the frame (about 0 when prefetched) |
| `dec` | hardware JPEG decode, µs |
| `iv` | the two M2C invalidates, µs |
| `c` | composite plus per-batch C2M, µs |
| `c2m` | trailing full-buffer C2M, µs |
| `ce` / `cf` | chunked-sync failures (cumulative) / first error code (0 = none) |
| `cd` | `on_color_trans_done` callback count (liveness) |
| `drop` | frames dropped for overrunning their slot |
| `ms` | frames missed in the window (stale or late slot) |
| `pf` / `pff` | SD tasks alive / slot fills so far |

**Fields added since the S122 snapshot:** `jk=<n>/<max_ms>` — inter-present gaps over 250 ms in the window (the S127 jank detector; a multi-second freeze shows up as one large `max`); the per-fill `pfs` line gained `b=<n>` (frames in the S129 batch that filled the ring entry) and the read-task slow-episode detector prints `pfs: SD slow episode — N fr rd avg=… max=… ms (~KB/s)` when consecutive amortized per-frame reads exceed the 83 ms budget; the line now also ends with `ifree=<KB> ilarg=<KB>` (internal-SRAM free / largest free block — the S135+ heap-telemetry pair that catches SD-bounce starvation before it freezes playback, section 11). Boot prints the SD card class/size and the host's max SDMMC clock (`host max 40000 kHz 4-bit` — already the SD legacy maximum, so on a slow card the clock is not a lever).

---

### 11. Memory and Heap (PSRAM + internal SRAM)

**PSRAM carries all frame memory.** The P4's internal SRAM is only ~320 KB of heap (two DRAM regions); everything multi-megabyte is allocated from the 32 MB Octal PSRAM at boot and never re-allocated:

| Buffer | Size | Where |
|---|---|---|
| Draw buffer `fb` (M5GFX `_draw_buf`) | 1,843,200 B | PSRAM, 128 B aligned |
| Decode scratch `fr_fb` | 1,843,200 B | PSRAM, 128 B aligned (JDEC output must be 128 B aligned — the L2 line size) |
| Background `bg_buf` | 1,843,200 B | PSRAM, 128 B aligned |
| Player slots `g_slot[6]` | 6 × (115,200 + 131,072) B | PSRAM |
| Prefetch read ring `g_pb[6]` | 6 × (115,200 + 131,072) B | PSRAM, allocated once in `spawnPrefetch` (idempotent) |
| S129 batch scratch `g_pf_scratch` | 985,088 B (two-tier: falls back to 246,272 B, then to the legacy null path) | PSRAM |
| S128 inline scratch `g_inline_scratch` | 246,272 B | PSRAM |
| Audio PCM ring | 46,080 B (240 ms) | PSRAM, 128 B aligned |

All of these use `heap_caps_(aligned_)alloc(..., MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)`; alignment is raised to `esp_cache_get_alignment(MALLOC_CAP_SPIRAM)` when it exceeds 128 B. The 128 B alignment is what keeps every `esp_cache_msync` range exact (the L2 line is 128 B; 64 B rounding logs E-level errors). A successful boot with everything allocated leaves **~19.4 MB PSRAM free** (steady-state telemetry `psram_free=19430 KB`, flat across sessions — a rising/falling trend would indicate a leak). `kSlots = 6` (6 frames ≈ 500 ms of runway at 12 fps) was raised from 4 specifically to ride out 226–301 ms SD stalls on the U1 card; the ring/slot math uses `% kSlots`, never a power-of-two mask.

**Internal SRAM is the scarce resource, not PSRAM.** Two subsystems eat it:

1. **The SDMMC driver's per-transfer bounce buffer.** When a DMA destination cannot be written directly, `sdmmc_cmd` allocates a bounce buffer from **internal** DMA-capable heap (`heap_caps_malloc(chunk, MALLOC_CAP_DMA)` — on this build the SPIRAM region does not carry the DMA cap, so the alloc is internal-only even with 19 MB PSRAM free), halving until it fits, giving up at the 512 B floor with `err=0x101` (`allocate_dma_buf: not enough mem`). If the largest free internal block drops to ~21 KB the driver's minimum does not fit and **every SD read fails** — the player then starves and freezes with `psram_free` perfectly healthy. The fix is to keep the internal heap's largest block above the bounce floor at all times.
2. **The always-on WiFi stack on the C6 co-processor (esp_hosted).** Measured on device: the stack holds **~201 KB free / ~122 KB largest** of internal SRAM after connect, and the esp_hosted SDIO transport pre-allocates its streaming-RX buffers from the same heap at init (1536 B × C6-reported queue size × 2 slots ≈ 60 KB).

**The heap optimizations (all in the current build):**

- **Trim the WiFi buffer pools** via `custom_sdkconfig` in `platformio.ini`: `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=4`, `DYNAMIC_RX=8`, `DYNAMIC_TX=8`, `MGMT_SBUF=8` (framework defaults 10/32/32/32). This forces a one-time from-source recompile of the IDF+Arduino core libs (the `esp32p4_es` variant) — the first build is 10–30 min, incrementals ~6 min.
- **`CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y`** so the esp_hosted host-side aligned allocations try PSRAM first and only fall back to internal SRAM.
- **esp_hosted `hosted_malloc_align` patch:** a plain `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT` attempt between the SPIRAM|DMA attempt and the internal fallback (PSRAM blocks may lack the DMA cap; plain SPIRAM is still DMA-reachable on the P4 through the cache).
- **esp_hosted SDIO RX-buffer halve-and-retry:** `bus_sdio_set_rx_buffer_size` halves the streaming-RX allocation (30,720 → 15,360 → 7,680 …, floor 1536 B) instead of aborting the boot when the full size cannot land.
- **App-side rules (S127):** no large object on any task stack — the `Rfav3` reader (two 4096-entry tables ≈ 33 KB) is the one global; the `avatar.json` parse uses a function-static 12,288 B buffer + `StaticJsonDocument`, and `config.json` a function-static 512 B buffer (an 8 KB task stack cannot hold a 33 KB table — that was a boot crash loop). The JSON documents themselves are the reason avatar files are capped at 12 KB.
- **Telemetry for regression:** the 1 Hz line's `ifree=`/`ilarg=` pair, the `boot: heap internal free=… largest=… psram free=…` boot print, a post-CONNECT heap print in the WiFi task, and the internal-heap state appended to every `read FAILED after 10 tries` line. If `allocate_dma_buf: not enough mem` lines appear with `psram_free` healthy, the diagnosis is internal-SRAM starvation, not the card.

**Observed budget (device, S151 firmware):** boot print `internal free ≈ 220–236 KB / largest ≈ 111–151 KB` before WiFi init; ~26–35 KB free / 21–29 KB largest after the WiFi init window; steady-state playback 12 fps with zero bounce failures. The U3 card on order (S127 follow-up) is the remaining lever for the raw per-frame read floor; the read-path levers (combined read, batch read, 6-deep ring) are exhausted.
