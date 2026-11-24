# RFAV-4 (Cropped-Band, SD-Optimized) — 720×1280 MJPEG + 1-bit Alpha Mask

**Status:** Draft, unshipped. Supersedes `rfav2_720p_spec_rev3.md`. No backwards-compatibility requirement.
**Target:** M5Stack Tab5 / ESP32-P4, streaming from SD at 12 fps (up to 24).
**Goals:**

1. Never read, decode, composite or cache-sync pixels that are fully transparent across the whole frame.
2. Never do more per-frame work than rev3 in the worst case (a full-height band).
3. Keep the container and the decoder simple and strictly sequential.
4. Make SD reads, cache maintenance and the JPEG engine operate on aligned ranges.


> **Revision note (aligned to the reference player, S151 firmware / S158 state).** The reference player decodes into a **scratch buffer** and composites from scratch + background into the draw buffer (S101), uses chunked per-batch cache syncs with error counters (S99), skips rewriting background groups that already hold the background (S103), runs a 6-slot two-stage prefetch (S102/S107), and reads SD frames with the **S128 combined read** (one mask+JPEG transfer per frame) and the **S129 4-frame batch read** (rev3 spec, sections 6 and 11 — the U1-class-SD levers, with per-caller PSRAM scratch and the internal-SRAM heap protections). Rev4's `readRecord` (5.3) is the natural generalization of what the reader already does: the S129 batch shape *is* rev4's single contiguous record read for the common back-to-back case. The player-side detail (SD ring, present kick, UI layer, storage layout, heap budget) is in rev3 sections 7 and 11.

---

## 0. Conventions and Orientation

### 0.1 Conventions

- All fields little-endian.
- **Stored orientation** is the Tab5 panel's native portrait orientation (see 0.2 and 0.3): canvas **W = 720** wide, **H = 1280** tall. A *row* is one 720-pixel line of the stored image.
- Runtime pixels are RGB565, so one row is **1440 B** (`PITCH`). One mask row is **90 B** (`STRIDE = W/8`).
- A *band* is a contiguous run of rows. `A = band_align = 16` rows (the JPEG MCU height for 4:2:0).
- *Background* is supplied by the player (720×1280, same format as the draw buffer). It is not part of the container.

> **Note on rev3:** the rev3 prose describes 1280×720 with a 160 B mask stride, but the reference reader and player use 720 wide × 1280 tall with a 90 B stride. Rev4 defines the stored orientation normatively as W=720, H=1280.

### 0.2 Tab5 native buffers

- The Tab5's 5" panel is a MIPI-DSI video-mode display whose **native** orientation is portrait, **720 × 1280**. M5GFX at rotation 0 draws directly in this native orientation (the same baseline the skytab portrait UI uses). A landscape view (`setRotation(1)` or `(3)`) is only a coordinate mapping inside M5GFX. It does not change the memory layout.
- The player's draw buffer is one native-size PSRAM buffer: 720 × 1280 × 2 B = **1,843,200 B**, 128 B aligned. The patched `Panel_DSI::display()` pushes the *whole* buffer (`esp_lcd_panel_draw_bitmap` at native `panel_width × panel_height`) by DMA2D into the driver's non-scanned panel buffer, then swaps on vsync. App-side width/height arguments only gate the push.
- Everything the compositor touches lines up **row for row in native space**: the hardware JPEG decoder's output rows (720 px), the mask rows (90 B), the background rows and the draw-buffer rows. A row range is therefore one contiguous range of memory in all four. That alignment is what makes a row crop cheap.
- Only UI text (status chip, boot log, fatal text) is drawn in a rotated window: the player calls `setRotation(kUiRot)` (1; swap with 3 if the text reads backwards), draws, and returns to rotation 0. All video, mask and background data stays in native buffer space.
- The rev3 status bar is `fillRect(4, 4, 700, 56)` in the `kUiRot = 1` window (a status line plus a transient feedback line). In native space (mapping in 0.3) it covers rows 4–703 and columns 660–715.
- Decode never targets the presented buffer. The reference player decodes into a scratch buffer of the same size, then composites into the draw buffer, so the draw buffer never holds raw decoder output.

### 0.3 Pre-rotation: how landscape left/right becomes native top/bottom

Source clips are authored landscape (1280 × 720, for example a WebM with alpha). Rev3 does **not** rotate on the device. The encoder rotates every frame **90° clockwise before anything else** (`webm_to_rfav.py --rotate cw`, the default; ffmpeg `transpose=1`, "source top row → right column"). The rotation is applied once, at the PNG-extraction step, so the color JPEG and the 1-bit mask are derived from the same rotated pixels and cannot disagree.

For a landscape source pixel (x, y), with x = 0..1279 left→right and y = 0..719 top→bottom, the stored (native) pixel is:

```text
u = 719 - y        // column within the 720-px row   (landscape top    -> native right)
v = x              // row index 0..1279              (landscape left   -> native top)
```

When the tablet is turned to landscape so that the `kUiRot = 1` text window reads correctly, the display geometry is `native(u = 719 - Y, v = X)` for display coordinates (X, Y) (confirmed from an extracted device frame). That is the exact inverse of the encode rotation, so the viewer sees the original landscape picture. Native row 0 is the display's left edge, and native column 719 is its top edge.

| Landscape view (what the viewer sees) | Native 720×1280 buffer |
|---|---|
| Left margin, `x < x0` | **Top** rows, `v < x0` |
| Right margin, `x ≥ x1` | **Bottom** rows, `v ≥ x1` |
| Horizontal position `x` | **Row index** `v = x` |
| Top margin / head room (small `y`) | Right columns (large `u`) |
| Bottom margin / feet (large `y`) | Left columns (small `u`) |

A character standing centered in the landscape frame has wide transparent *left/right* margins and spans nearly the full height. In native space that is a **horizontal band of rows** around the character, with transparent rows above and below it. Rev4 crops exactly those head and tail rows. Cropping the other axis (native columns) is rejected: it breaks contiguity, and the gain is small (see the measured example below).

**Why the rotation is baked into the file:** an on-device landscape path (`setRotation(3)` plus a per-frame CPU 90° transpose) cost about 70–90 ms per frame and was the cause of the 8 fps result. The shipped design decodes into a native-orientation scratch buffer and composites into the native draw buffer, with no device-side rotation.

**Direction:** the encode rotation and the way the tablet is held must agree. Rev3 shipped CW with `kUiRot = 1`. The direction was verified: the baked mask is a 100.0000% bit-match against the source alpha rotated CW, against 89.9% for CCW. A CCW pairing would use `v = 1279 - x` and `u = y`. The container format is identical either way, so header byte 15 (`stored_orientation`) records which was used (N = 1 for CW, N = 3 for CCW). Other mountings and portrait avatars are covered in Appendix A.

**Measured example** (`thinking.rfav` frame 0, 15.6% opaque): opaque bounding box rows 489–790, columns 0–646. Opaque percentage per 1/16-height band, top to bottom: 0, 0, 0, 0, 0, 0, 35, 88, 85, 42, 0, 0, 0, 0, 0, 0. After 16-row rounding the band is rows 480–800 (320 rows, landscape x = 480–800). The columns span 0–646 of 720, so a column crop would save at most about 10%.

---

## 1. Design Decisions (vs RFAV-3)

| Decision | Choice | Reason |
|---|---|---|
| Frame extent | **Row-cropped band** per frame (`first_row`, `row_count`) | Skip decode, SD bytes, compositing and sync for fully transparent head/tail rows |
| Crop axis | Rows only (full 720 width) | Native rows = landscape X (see 0.3); keeps JPEG output, mask and framebuffer contiguous; no stride logic |
| Alignment | `first_row` and `row_count` multiples of **16** | JPEG MCU boundary; band edges land on 128 B cache lines (16 rows = 23,040 B = 180 lines) |
| Mask | 1-bit, **variable length** (`row_count × 90 B`), band-local | Mask shrinks with the band |
| Record alignment | Records start on **512 B** sectors; mask and JPEG each padded to 512 B | Aligned SD reads; JPEG lands 128 B-aligned in RAM |
| Read pattern | **One contiguous read** per frame (mask + JPEG) | rev3 did seek, read mask, seek, read JPEG |
| Static holds | `REPEAT` frame flag (no record) | Zero decode, zero SD, optionally zero present |
| Blank frames | `row_count = 0` allowed (background only) | Rev3 rejected `jpeg_len == 0` |
| Buffer sizing | Header carries max band rows and max record size | Fixed slot allocation, validated once |
| Audio | External (unchanged) | Unchanged from rev3 |

---

## 2. File Layout

```text
[ Header        ]  64 bytes
[ Frame Table   ]  N × 16 bytes
[ zero pad      ]  to frame_data_offset (multiple of 512)
[ Records       ]  Record₀ Record₁ ... (each starts on a 512 B boundary; REPEAT frames have no record)
[ Footer        ]  32 bytes (optional)
```

### 2.1 Header (64 bytes)

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 4 | magic | `"RFAV"` |
| 4 | 2 | version | `4` |
| 6 | 2 | canvas_width | 720; MUST be a multiple of 16 |
| 8 | 2 | canvas_height | 1280; MUST be a multiple of `band_align` |
| 10 | 1 | fps | 1–60 |
| 11 | 1 | flags | see below |
| 12 | 2 | frame_count | N (1..4096) |
| 14 | 1 | band_align | rows; 16 in this profile |
| 15 | 1 | stored_orientation | informative, players never rotate. bits 1:0 = N: the stored image is the upright picture rotated N×90° CW (rev3: N = 1; N = 3 is CCW). bit 2 = flip_v, bit 3 = flip_h (applied after the rotation). bits 4–7 = 0. See Appendix A |
| 16 | 4 | frame_table_offset | 64 |
| 20 | 4 | frame_data_offset | `align_up(64 + N×16, 512)` |
| 24 | 4 | frame_data_size | bytes from `frame_data_offset` to end of last record |
| 28 | 4 | crc32_header | CRC32 of the 64 header bytes with this field zeroed; 0 = absent |
| 32 | 2 | max_band_rows | largest `row_count` of any frame |
| 34 | 2 | reserved | 0 |
| 36 | 4 | max_mask_stored | largest `mask_stored` (see 2.3) |
| 40 | 4 | max_jpeg_stored | largest `jpeg_stored` |
| 44 | 4 | max_record_len | largest `record_len` |
| 48 | 16 | reserved | 0 |

**Flags:**

- bit 0: loop
- bit 1: has_mask (MUST be 1)
- bit 2: mask_4bit (0 = 1-bit; POC players MAY reject 4-bit)
- bits 3–7: reserved (0)

### 2.2 Frame Table Entry (16 bytes)

```text
u32 sector       // record start = sector × 512 (absolute file offset). 0 for REPEAT frames.
u16 first_row    // first canvas row covered; multiple of band_align
u16 row_count    // rows covered; multiple of band_align; 0 = no new content
u32 len_flags    // bits 23:0  jpeg_len in bytes (0 if row_count == 0)
                 // bits 31:24 eflags: bit0 REPEAT, bits 1–7 reserved (0)
u32 frame_crc32  // optional CRC32 of mask bytes (mask_len) then JPEG bytes (jpeg_len), padding excluded; 0 = absent
```

Rules:

- `first_row + row_count ≤ canvas_height`.
- **REPEAT:** `sector`, `first_row`, `row_count` and `jpeg_len` MUST all be 0. The frame is identical to the nearest preceding non-REPEAT frame (its *anchor*). Frame 0 MUST NOT be REPEAT.
- **Blank frame** (not REPEAT): `row_count == 0`, `jpeg_len == 0`. Shows the background only.
- **Content frame:** `row_count > 0`, `jpeg_len > 0`.

### 2.3 Record Layout

```text
Record i (starts at sector × 512):
  [ mask       mask_len = row_count × 90 B ][ zero pad to 512 ]   → mask_stored
  [ JPEG       jpeg_len bytes              ][ zero pad to 512 ]   → jpeg_stored
record_len = mask_stored + jpeg_stored        (always a multiple of 512)
```

Derived values:

```text
stride       = canvas_width / 8                      // 90
mask_len     = row_count * stride
mask_stored  = align_up(mask_len, 512)
jpeg_stored  = align_up(jpeg_len, 512)
record_len   = mask_stored + jpeg_stored
```

Writers SHOULD place records back-to-back in frame order, so `record[i+1].offset == record[i].offset + record_len(i)` for consecutive non-REPEAT frames. This allows batched read-ahead (see 8.2).

### 2.4 Footer (optional, 32 bytes)

Unchanged from rev3 (reserved for CRC or future use).

---

## 3. Mask Format

- 1 bit per pixel, **MSB first**, 1 = opaque, 0 = transparent, no header.
- **Band-local:** mask row 0 corresponds to canvas row `first_row`. Row stride is 90 B.
- The band MUST contain every opaque pixel of the frame. Pixels outside the band are transparent by definition.
- **Flat-array property:** a row is exactly 720 bits, so bit index equals pixel index within the band. Since `row_count` is a multiple of 16, `mask_len` is a multiple of 1440 B (180 `u64` words per 16 rows), so the band mask is a flat, 8-byte-aligned array of `u64` words. Word *k* covers band pixels `[64k, 64k+64)`.
- 4-bit mask (flag): stride 360 B, `mask_len = row_count × 360`. Retained for future use; compositing needs a blend and is not covered here.

---

## 4. JPEG Payload

- Baseline JPEG, **dimensions exactly `canvas_width × row_count`**. The reader MUST reject a mismatch.
- Same color space, subsampling and quality constraints as rev3 (whatever the P4 hardware decoder accepts).
- `row_count == 0` means no JPEG.
- **Encoder guidance for transparent pixels inside the band** (they are never shown, but they affect size and edge quality):
  - Near mask edges (roughly 8–16 px), extend the nearest opaque colors outward to avoid chroma bleed from 4:2:0 and 8×8 DCT.
  - Far from edges, use one flat constant color so those MCUs compress to almost nothing.

---

## 5. Player Model

### 5.1 State and invariant

```c
const W = 720, H = 1280, PITCH = 1440, STRIDE = 90, A = 16;
const STRIPE_ROWS = 64;          // 92,160 B = 720 cache lines of 128 B

fb : RGB565[H][W]    // draw buffer, PSRAM, 128 B aligned
bg : RGB565[H][W]    // background, PSRAM, 128 B aligned, read-only after load
scratch : RGB565[H][W]  // JPEG decode target, PSRAM, aligned to esp_cache_get_alignment (128 B on current builds); only the current band is meaningful
skip[] : 1 bit per 8-px group (14,400 B)   // optional: 1 = this group of fb already holds the background

// INVARIANT: every row of fb outside [dirty0, dirty1) is bit-identical to bg.
// Boot: fb := bg;  dirty0 = dirty1 = 0;  rendered_anchor = -1;  fb_unpresented = false;  skip[] = 0
```

Because every band edge is a multiple of 16 rows (23,040 B = 180 lines), the band, the dirty interval and every restore range are cache-line aligned. They never share a line with each other.

The status overlay (the rev3 status bar: native rows 4–703, columns 660–715, see 0.2) is **excluded** from the invariant. It is redrawn after each composite and before the present, and its CPU writes are followed by their own C2M (M5GFX `display()` does this). Where the band covers the chip's rows, the composite overwrites the chip region and the redraw puts it back.

### 5.2 Validation at open

```c
bool open(File f):
    read header; check magic, version == 4, W%16 == 0, H%band_align == 0,
         band_align >= 16, fps in 1..60, has_mask set, !mask_4bit (POC), frame_count in 1..4096
    if hdr.crc32_header: verify
    read table (N × 16 B) into RAM
    for i in 0..N-1:
        e = table[i]
        if e.repeat:
            require i > 0 and e.sector == 0 and e.first_row == 0 and e.row_count == 0 and e.jpeg_len == 0
            anchor[i] = anchor[i-1]
            continue
        anchor[i] = i
        require e.first_row % A == 0 and e.row_count % A == 0 and e.first_row + e.row_count <= H
        require (e.row_count == 0) == (e.jpeg_len == 0)
        if e.row_count > 0:
            require e.sector*512 >= hdr.frame_data_offset
            require e.sector*512 + record_len(e) <= hdr.frame_data_offset + hdr.frame_data_size
            require e.row_count <= hdr.max_band_rows and record_len(e) <= hdr.max_record_len
    return true
```

### 5.3 Reading a record (SD task)

Replaces rev3 `readFrame()`: one request, no seek during sequential playback, no fixed-size mask assumption. In the reference player this runs in the SD read task (core 0) into a 6-deep private ring. A handoff task copies each record into a slot, and the render task copies it again into private buffers before releasing the slot early (rev3 section 7). The record layout is the same at every stage. Ground truth for the rev3 reader it replaces: `Rfav3::readFrame` with a PSRAM scratch (S128, one `115200 + jpeg_len` transfer) and `Rfav3::readFramesBatch` (S129, up to 4 frames per transfer with a table-based contiguity check) — so the single-request, no-mid-frame-seek, C2M-before-CPU-read shape is already proven on device; only the fixed-size-mask assumption and the two-stage copy differ.

```c
bool readRecord(File f, int i, Slot &s):
    e = table[i]
    s.index = i;  s.repeat = e.repeat;  s.first = e.first_row;  s.rows = e.row_count
    if e.repeat or e.row_count == 0:  return true            // no SD traffic

    len = record_len(e)                                       // multiple of 512, <= max_record_len
    pos = e.sector * 512
    if f.position() != pos: f.seek(pos)                       // skipped when playing in order
    if !readAll(f, s.buf, len): return false                  // ONE read: mask + JPEG

    s.mask     = s.buf                                        // row_count * 90 bytes valid
    s.jpeg     = s.buf + mask_stored(e)                       // 512-aligned => 128 B-aligned in RAM
    s.jpeg_len = e.jpeg_len
    publishSlot(s.buf, len)                                   // same coherence policy as the rev3 player,
                                                              // in chunks <= 64 KB, return codes checked
    return true
```

`s.buf` MUST be 128 B-aligned with capacity `hdr.max_record_len`.

### 5.4 Rendering a frame (decode/composite task)

```c
RenderResult renderFrame(Slot &s):
    if s.repeat: return handleRepeat(s)

    b0 = s.first;  b1 = s.first + s.rows

    if s.rows == 0:                                   // blank frame
        restoreRows(dirty0, dirty1);  dirty0 = dirty1 = 0;  fb_unpresented = true;  return OK

    // 1. Restore only what the previous frame covered and this one will not.
    restoreRows(dirty0,          min(dirty1, b0))     // rows above the new band
    restoreRows(max(dirty0, b1), dirty1)              // rows below the new band
    dirty0 = b0;  dirty1 = b1                         // set BEFORE the band is touched

    // 2. Decode the band into the SCRATCH buffer at the same row offset (never into fb).
    dst = (uint8_t*)scratch + b0 * PITCH              // aligned because b0 % 16 == 0
    if !jpeg_decode(s.jpeg, s.jpeg_len, dst, s.rows * PITCH, W, s.rows):
        return DECODE_FAILED                          // fb was not written by the decode

    // 3. Composite stripe by stripe: scratch + background -> fb.
    for (y = b0; y < b1; y += STRIPE_ROWS):           // reference player: 32-row batches
        n = min(STRIPE_ROWS, b1 - y)
        sync(scratch + y*W, n * PITCH, M2C)           // the JPEG engine wrote PSRAM behind the cache
        compositeBand(s.mask + (y - b0) * STRIDE, fb + y*W, bg + y*W, scratch + y*W,
                      n, first_group = y * STRIDE)
        sync(fb + y*W, n * PITCH, C2M)                // DMA2D present reads PSRAM, not cache
    rendered_anchor = s.index;  fb_unpresented = true
    return OK

void restoreRows(r0, r1):                             // [r0, r1), all multiples of 16 rows
    for (y = r0; y < r1; y += STRIPE_ROWS):
        n = min(STRIPE_ROWS, r1 - y)
        memcpy(fb + y*W, bg + y*W, n * PITCH)
        sync(fb + y*W, n * PITCH, C2M)
```

`sync()` is `esp_cache_msync()` with the return code counted (the reference player's `syncTracked`). Ranges are small and 128 B-aligned by construction: 32 rows is 46,080 B = 360 lines, and band edges are multiples of 16 rows. Only `scratch` needs an M2C, because the JPEG engine wrote it behind the cache. `fb` is written only by the CPU and M5GFX, each followed by a C2M, so this flow does not invalidate `fb`. The reference player still does (see 8.2, item 10).

**Composite** (replaces the rev3 in-place patch). The decoder no longer writes `fb`, so every pixel of the band comes from one of two sources:

```c
void compositeBand(const u8 *mask, u16 *dst, const u16 *bgp, const u16 *src,
                   size_t n_rows, size_t first_group):
    for (idx = 0; idx < n_rows * STRIDE; idx++):      // one mask byte = one 8-px group
        b = mask[idx];  p = 8 * idx;  g = first_group + idx
        if b == 0xFF:                                  // opaque: this frame's pixels
            memcpy(dst + p, src + p, 16);  skip[g] = 0
        elif b == 0x00:                                // transparent: background
            if !skip[g]: memcpy(dst + p, bgp + p, 16);  skip[g] = 1
        else:                                          // mixed: per pixel
            for k in 0..7: dst[p+k] = (b & (0x80 >> k)) ? src[p+k] : bgp[p+k]
            skip[g] = 0
```

`skip[]` is the reference player's per-group bit array: "this group of `fb` already holds the static background". It is optional. Without it, every transparent group is rewritten each frame. A set bit must always mean `fb` equals the background for that group. `restoreRows` leaves the bits alone (restored rows equal the background, so a stale 0 only costs one rewrite), and any other draw into `fb` outside the composite clears all bits.

### 5.5 REPEAT, blank and dropped frames

```c
RenderResult handleRepeat(Slot &s):
    a = anchor[s.index]
    if rendered_anchor == a:
        return fb_unpresented ? PRESENT_ONLY : NO_CHANGE   // nothing to render
    // Pacing dropped the anchor, so "no change" would show the wrong picture.
    readRecord(f, a, tmp);  return renderFrame(tmp)        // render the anchor instead
```

- `dirty0/dirty1` always describe what is actually in `fb`, not what was presented.
- The reference player has two kinds of drop. A **miss** (the slot is stale, or late past a 50 ms wait) renders nothing and presents nothing. An **overrun** (the work finished after the frame's ideal time) renders the frame into `fb` but skips the present. Every present copies the whole `fb`, so the next present shows everything drawn since, including the overrun frame's band and restores. `fb_unpresented` records that `fb` holds content the panel has not shown, so a `REPEAT` frame still gets presented.
- Loop wrap and clip switches need no special case: the first frame's band is restored and decoded like any other. A clip switch resets the frame index to 0.
- After a failed decode the player SHOULD NOT present. The restore step has already cleared the rows outside the new band, so the panel would show a clipped copy of the previous frame.

### 5.6 UI layer and present

The reference player holds the present until `ideal − 12 ms` (the fixed-grid kick, rev3 section 7). It then draws the UI layer (`setRotation(kUiRot)` → status bar → `setRotation(0)`) and calls `display(0, 0, 720, 1280)`: a DMA2D copy of the whole draw buffer into the non-scanned panel buffer, then a vsync swap. M5GFX follows the bar's writes with its own C2M. A partial-rectangle present is a candidate optimization (8.3).

### 5.7 Buffer sizing

The reference pipeline allocates a 6-deep read ring, 6 slots and one private copy: 13 buffers of `align_up(hdr.max_record_len, 128)`, allocated once at open. For a full-height rev3-sized record (115,200 B mask + about 40 KB JPEG) that is about 2 MB. Rev4's `max_record_len` is smaller for cropped clips, so the buffers shrink with it. The scratch, draw and background buffers are 1.84 MB each.

---

## 6. Encoder Algorithm

```python
# 0. Sources are landscape 1280x720 RGBA. Rotate color AND alpha identically, once, first
#    (ffmpeg transpose=1, 90 deg CW): landscape (x, y) -> native (u = 719 - y, v = x).
#    Everything below works in the rotated space: 1280 rows x 720 columns.
frames = [(rotate90_cw(rgb), rotate90_cw(alpha)) for (rgb, alpha) in landscape_frames]

for i, (rgb, alpha) in enumerate(frames):                 # rgb: [1280][720] RGB, alpha: [1280][720] bool
    if i > 0 and rgb_equal_under_mask(rgb, alpha, prev) :  # identical opaque content and mask
        table[i] = REPEAT; continue

    rows_any = alpha.any(axis=1)
    if not rows_any.any():
        table[i] = blank(); continue                       # row_count = 0, jpeg_len = 0

    r0 = first_true(rows_any);  r1 = last_true(rows_any) + 1
    first = (r0 // 16) * 16
    end   = ((r1 + 15) // 16) * 16
    rows  = end - first                                    # multiple of 16

    mask = np.packbits(alpha[first:end], axis=1).tobytes() # MSB first, 90 B per row
    img  = fill_transparent(rgb[first:end], alpha[first:end])   # edge-extend, then flat fill (see section 4)
    jpg  = encode_jpeg(img, width=720, height=rows)        # SOF dims must equal 720 x rows

    record = pad512(mask) + pad512(jpg)
    offset = align_up(file_pos, 512)
    write_at(offset, record)
    table[i] = (offset // 512, first, rows, len(jpg), crc32(mask + jpg) or 0)

header.stored_orientation = 1                       # N = 1: upright picture rotated 90 deg CW; informative
header.max_band_rows   = max(rows);  header.max_mask_stored = max(...)
header.max_jpeg_stored = max(...);   header.max_record_len  = max(...)
```

---

## 7. Performance Model and Worst-Case Guarantee

Let *f = row_count / 1280*. Costs per frame in the reference player (S122) and how each scales:

| Cost | rev3 (full frame) | rev4 |
|---|---|---|
| SD bytes, mask | 115,200 | `row_count × 90` |
| SD bytes, JPEG | full frame | roughly ∝ band area; flat fill compresses to near zero |
| Copies after the SD read (ring → slot → private) | 2 × ~155 KB | 2 × `record_len` |
| Mask scan in the composite | 115,200 B | `row_count × 90` B |
| JPEG decode (hardware) | 1280 rows | `row_count` rows |
| M2C invalidates | scratch + draw buffer, 3.7 MB | the band in scratch (the draw buffer's band only if kept, see 8.2) |
| Composite writes | transparent groups mostly skipped by the skip bits after the first frame; opaque and mixed groups written | band only, with the same skip bits |
| C2M | 40 batches of 32 rows plus a trailing full-buffer sync | the band's batches plus a trailing band sync |
| Background restore | none | rows in the previous band but not the new one |

Mask bytes at the example sizes: 1280 rows = 115,200 B, 720 rows = 64,800 B, 320 rows = 28,800 B.

**Where the gain is.** The reference player already skips rewriting transparent groups that hold the background, so its composite *writes* are small after frame 1 and rev4 will not cut them much further. Rev4's gain is in everything that still walks or moves a full frame: decode, the two 1.84 MB invalidates, the per-batch and trailing syncs, the mask scan, the two record copies and the SD read. These all scale with *f*. Use the telemetry fields in rev3 section 10 (`dec`, `iv`, `c`, `c2m`, `sd`) to size it on your clips. The composite-time estimates in earlier drafts used a pre-S103 figure and no longer apply.

Fixed costs that do not shrink: the full-frame DMA2D present (about 5–9 ms), per-request SD latency (including the 226–301 ms stalls), JPEG engine setup, the UI bar, and the gate wait.

**Worst-case non-regression:**

- Restored rows and decoded rows are disjoint subsets of the canvas, so rows written per frame never exceed 1280. Every cost term is monotone in `row_count`.
- For a full-height band (`first_row = 0`, `row_count = 1280`) the restore set is empty and the work equals the reference player's: the same 40 batches and the same trailing sync. The extra work is one table-entry parse and under 1 KB of padding on the SD read. **Verify the overhead is under 1 ms on target.**
- **Baseline:** compare against the current reference player (chunked, per-batch syncs with error counters from S99, scratch decode from S101, skip bits from S103). The earlier whole-buffer sync was found not to reach all dirty lines (a 1.84 MB sync measured 424 µs), so FT figures taken before that change, including "under 45 ms", may not include the full write-back cost. Re-measure rev3 on the current build before comparing.

---

## 8. Additional Optimizations

### 8.1 Built into the format (no downside expected)

1. **Row-cropped band with 16-row alignment:** less decode, mask, SD, composite and sync work. Alignment keeps all ranges on cache lines.
2. **512 B record alignment with padded mask and JPEG:** aligned, single-request reads. Padding costs about 0.5 KB per frame (under 1% of a typical frame).
3. **REPEAT frames:** idle holds cost no SD, decode, composite or present.
4. **Blank frames:** a fully transparent frame is a pure background restore.
5. **Header maxima:** slots are sized once and no per-frame size check can overflow them.
6. **Explicit orientation:** removes the rev3 spec/reader mismatch.

### 8.2 Safe implementation optimizations

1. **Decode only the band, at the band offset:** the decoder writes `row_count` rows at `scratch + first_row × PITCH`, so no head or tail rows are decoded or copied.
2. **Restore only the delta:** the background is written only where the previous frame left content.
3. **Word-wise composite (optional):** the reference player works per mask byte (8 px). Testing one `u64` (64 px) at a time for fully transparent or opaque words can cut the loop count further. Measure it against the skip bits, which already avoid most transparent writes.
4. **Single read per record, no mid-frame seek:** the rev3 reader already does this for the per-frame case (S128 combined read: one `mask + jpeg` transfer) and for up to 4 frames at once (S129 batch); rev4 keeps it with the record's 512 B padding instead of the fixed mask size. Remaining win vs S129: no PSRAM scratch copy stage and smaller per-frame bytes.
5. **Chunked, checked cache maintenance:** ≤ 64 KB per call with `esp_err_t` logged. A silent failure becomes visible.
6. **Skip present for NO_CHANGE and unchanged blank frames:** the panel keeps the last frame.
7. **Batched read-ahead:** because records are back-to-back, two consecutive records can be fetched in one request when two slots are free. This reduces SD command overhead; keep it optional.
8. **Frame table kept in RAM:** it is at most 64 KB for 4096 frames.
9. **Skip bits inside the band:** the reference player's per-8-px "`fb` already holds the background" bits keep working unchanged in rev4. The only rule is that a set bit describes `fb`, never scratch.
10. **Drop the draw-buffer invalidate (verify first):** decode now goes to scratch, so only the CPU and M5GFX write `fb`, each followed by a C2M. The reference player still M2C-invalidates all of `fb` every frame, a 1.84 MB walk. Remove it on a test build and watch for ghosting or alpha holes.

### 8.3 Measure first (possible trade-offs)

1. **Partial-rectangle present.** The DMA2D present copies 1.84 MB every frame. Presenting only a rectangle needs the union of the last *two* frames' dirty bands, because the driver alternates two panel buffers. It may touch the patched M5GFX path and the overlay's rows must be included (the rev3 status bar is native rows 4–703, columns 660–715). Potential saving up to `(1 − f) × (5–9 ms)`.
2. **Ping-pong draw buffers.** JPEG decode is a separate engine, so decode(i+1) can overlap composite(i). It costs one more framebuffer in PSRAM (~1.75 MB), per-buffer dirty tracking and a present hook that takes a buffer pointer. Only worth it if FT is still too high.
3. **Raw sector reads bypassing FatFS** using a cached cluster map. Only if profiling shows SD is the bottleneck. Higher complexity and fragility.
4. **Vector (PIE) composite.** The P4's SIMD extensions could speed up the mixed-word path, but toolchain support in the Arduino build is unverified.
5. **Pre-allocated, contiguous clip file** (unfragmented) for the best FatFS direct-read behavior.

### 8.4 Considered and rejected

1. **Column cropping:** breaks contiguous decode output and stride-free memcpy, for little gain on a full-width character.
2. **RLE or compressed masks:** the band mask is already small (28.8 KB at 320 rows) and decompression costs CPU.
3. **Baked-background encoding (no mask):** gives up the runtime-swappable background.
4. **Per-frame CRC checking at playback:** the CRC field stays optional and is not checked in the hot path.

---

## 9. Reader Must Reject

- Bad magic, version ≠ 4, `canvas_width % 16 ≠ 0`, `canvas_height % band_align ≠ 0`, `band_align < 16`.
- `has_mask` clear; `frame_count` 0 or > 4096; `fps` 0 or > 60.
- Any table entry violating section 2.2 or the checks in section 5.2.
- JPEG header dimensions ≠ `canvas_width × row_count` for the frame.
- Record outside the data region, or `record_len` larger than `max_record_len`.
- Frame 0 marked REPEAT.

---

## 10. Telemetry to Add

Extend the reference 1 Hz line (`FT max pres avg sd dec iv c c2m ce cf cd drop ms pf pff`, defined in rev3 section 10) with `first_row`, `row_count` and `restored_rows` for the last frame. Keep `ce` and `cf` (chunked-sync error count and first error code) in every comparison run. Report p50, p95 and max over the same clip and background for rev3 and rev4.

---

## 11. Worked Example

Frame *k* (landscape x = 259..975, which is native rows 259..975): opaque rows 259..975 (exclusive end 976).

- `first = floor(259/16)·16 = 256`, `end = ceil(976/16)·16 = 976`, `row_count = 720`.
- `mask_len = 720 × 90 = 64,800`; `mask_stored = 65,024` (127 × 512).
- Decode target offset `256 × 1440 = 368,640 B` (= 2,880 cache lines).

Frame *k+1*: opaque rows 247..959 (exclusive end 960).

- `first = 240`, `end = 960`, `row_count = 720`.
- Dirty interval before = `[256, 976)`. Restore = `[256, min(976, 240))` ∪ `[max(256, 960), 976)` = ∅ ∪ `[960, 976)` → **16 rows** (23,040 B memcpy from the background).
- Decode into `[240, 960)` (offset 345,600 B = 2,700 lines), composite that band only, set `dirty = [240, 960)`.

### Example B — measured clip (`thinking.rfav`, frame 0)

Opaque bounding box rows 489–790 (landscape x = 489–790, a 302-px-wide strip), columns 0–646.

- `first = floor(489/16)·16 = 480`, `end = ceil(791/16)·16 = 800`, `row_count = 320`, so *f* = 0.25.
- `mask_len = 320 × 90 = 28,800 B` (rev3: 115,200 B). The other 960 rows are not read, decoded, composited or synced.
- In steady state those 960 rows already equal the background (invariant in 5.1). Only the few rows between consecutive frames' bands are restored.

---

## 12. To Verify on Hardware

1. The P4 JPEG decoder accepts a different image height on each call and writes to an offset inside the **scratch** buffer. The driver's output-alignment check uses the L2 line size (`esp_cache_get_alignment`). Band offsets are multiples of 23,040 B, which is a multiple of 128, 256 and 512, so any line size up to 512 B passes. The reference player's `out_size != 1,843,200` check must become `row_count × PITCH`.
2. The per-stripe sync cost for a full-height band (it should match the reference player's 40 batches).
3. The reference player counts sync errors (`ce`, `cf` in the telemetry). Confirm `ce = 0` and `cf = 0` over a long run.
4. FatFS direct multi-sector reads for sector-aligned, 128 B-aligned destinations (a performance benefit, not a correctness requirement).
5. The shipped orientation pairing: CW pre-rotation (`stored_orientation` N = 1) with the tablet held so the `kUiRot = 1` text window reads correctly. If it is held the other way, re-encode with `--rotate ccw` (N = 3). Swapping `kUiRot` alone only flips the text, not the video.
6. Whether the draw-buffer M2C is still needed now that decode goes to scratch (8.2, item 10): remove it on a test build and watch for ghosting.

---

## Appendix A — Portrait Avatars, Mounting and Authoring (non-normative)

Rev3 ships one pairing: a landscape-authored avatar, rotated CW at encode time, shown with the tablet turned to landscape. This appendix covers other mountings (including an upright portrait avatar), how crop efficiency changes with them, and options for authoring the WebM clips.

### A.1 Terms

- **Upright picture:** the avatar as the viewer should see it.
- **Stored image:** the 720×1280 native image in the file.
- **`stored_orientation` N** (header byte 15, bits 1:0): the stored image is the upright picture rotated N×90° CW. It describes the stored pixels, not how they were produced. Whether the rotation happened at encode time or the generator drew the character sideways, the value is the same.

### A.2 Mountings

| N | Stored image | Tablet mounted | Native top edge ends up | Natural `kUiRot` |
|---|---|---|---|---|
| 0 | upright | portrait, native top up | up | 0 |
| 1 | upright rotated 90° CW (**rev3**) | landscape, turned 90° CCW from portrait | viewer's left | 1 |
| 2 | upright rotated 180° | portrait, upside-down | down | 2 |
| 3 | upright rotated 90° CCW | landscape, turned 90° CW from portrait | viewer's right | 3 |

`kUiRot = N` is the natural pairing for the status text. It is verified only for N = 1 (rev3) and the known 1↔3 swap. Check 0, 2 and 3 on the device.

### A.3 Which crop helps

The row-band crop helps when the avatar's long axis lies along the stored *columns* (u), because the transparent margins are then whole rows.

| Authored avatar | Mounting | Opaque rows span | Row-band crop |
|---|---|---|---|
| Standing character, narrow in the landscape view | N = 1 or 3 | a narrow strip (`thinking.rfav`: rows 489–790, f ≈ 0.25) | effective |
| Full-body character in a portrait canvas | N = 0 or 2 | most of the 1280 rows (expected f ≈ 0.7–1.0; measure it) | little gain |

- A portrait avatar is **supported from day one**. With f → 1 rev4 does the same work as rev3, and rev3 was reported under the 83 ms budget (FT < 45 ms; re-measure on the current build, see section 7). The crop is headroom, not feasibility.
- Column margins are what an upright portrait avatar has. A rect crop (A.6) is the only way to use them, and it is not in the rev4 baseline.

### A.4 Authoring options (Reality Fabricator clips)

**Option 1 — landscape authoring, rotate at encode (rev3).** Keep the current prompt. The 90° transposition is lossless, and the generator draws an upright character, which is what video models handle best. Encode with `--rotate cw` (N = 1) or `ccw` (N = 3).

**Option 2 — sideways portrait authoring.** Make the reference character image lie on its side and prompt for a 9:16 video, so the generated WebM is already in stored layout and the encode rotation step disappears. N is still 1 (CW). Example prompt, based on the existing one:

```text
Vertical 9:16 frame. The character is rotated 90° clockwise: lying on its side, head toward the right edge of the frame, feet toward the left edge. Locked-off position, static camera, 2d anime game animation style. Perfect seamless loop: the character ends the clip in EXACTLY the same pose it started in. Character visibly angry, furrowed brows, puffed cheeks, small frustrated shakes. Keep the character on the same solid background.
```

Trade-offs:

- There is no resolution gain. The character's long axis is 720 px in both layouts.
- Video models are trained mostly on upright subjects, so sideways generation may degrade gravity-driven motion (hair, cloth), face consistency and loop closure. This is a risk to test, not a known result.
- A/B one clip against Option 1 before changing the pipeline.

**Option 3 — true portrait authoring (N = 0 or 2).** Natural for 9:16 generators and for a device standing in portrait. Row-crop gains are small (A.3).

### A.5 Flip versus rotate, and mounting for stability

- As you described it, the Tab5's power is at the native bottom and the heavier battery toward the native top. I have not verified this on the unit. For a **portrait** stand, native-top-up is then top-heavy. Mounting native-top-down is **N = 2**, which is a 180° rotation of the stored image (rows and columns both reversed).
- A vertical flip alone is not the same thing. With the device turned upside-down, a `flip_v` image shows a **left/right mirror image**. Use the `flip_v` and `flip_h` bits only for intentionally mirrored content.
- For a **landscape** stand (N = 1 or 3) the native-top edge is a side edge. N = 1 versus N = 3 chooses which side carries the heavy edge, so choose by the stand design and cable exit.
- Extend the encoder CLI from `--rotate cw|ccw|none` to `--turns 0|1|2|3`.

### A.6 Optional: rect crop for upright portrait avatars (not in rev4)

For N = 0 or 2 clips with a tall opaque band but narrow columns, a later revision could add per-frame `first_col` and `col_count`:

- Column start and width in multiples of 64 px, so each row segment is a whole 128 B line (JPEG MCU is 16 px).
- Decode into a contiguous scratch buffer (rows × cols), then composite scratch + background into the framebuffer. Unlike the row band, the scratch rect is not row-aligned with `fb`, so the composite must copy every row segment of the rect.
- Track a dirty **rect**. Restore the old rect minus the new rect (up to 4 rects). The mask stride becomes `col_count / 8`.
- The current 16 B table entry has no spare room, so this needs a 24 B entry or dropping the CRC field.

**Decision rule:** implement it only if the clip set shows f_r ≥ 0.8 and f_c ≤ 0.5 and FT headroom is actually needed.

### A.7 Desktop avatar use

A desktop anime avatar is an always-on companion display with long idle loops and many held frames. For that product, `REPEAT` frames and skipped presents matter more than peak FT. They should also lower average power, since held frames need no decode, composite or present. Measure it.

### A.8 Encoder reporting and sanity checks

Per clip, report the opaque bounding box in stored space, f_r (row fraction), f_c (column fraction), the percentage of `REPEAT` frames and the maximum band. For odd N (landscape mounting), the avatar's opaque extent should be wider in columns than in rows (`thinking.rfav`: 647 columns against 302 rows). If it is not, warn that the clip may be mis-rotated.
