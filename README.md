# RFab Tab5 — Reality Fabricator companion avatar player

Proof-of-concept re-implementation of the **Reality Fabricator (RFab)** companion
avatar playback engine on the **M5Stack Tab5** (ESP32-P4, 720×1280 MIPI-DSI
display, 32 MB PSRAM): **12 fps portrait MJPEG flipbooks with true alpha,
streamed from a microSD card, with per-clip voice and fully data-driven avatar
behavior.**

[![Reality Fabricator m5stack tab5 featuring Neko Voss avatar animation | esp32p4 mjpeg](https://i.ytimg.com/vi/s-WHDjJM208/hqdefault.jpg)](https://youtu.be/s-WHDjJM208)

*Demo: the `neko_voss` avatar running on the Tab5 — [watch on YouTube](https://youtu.be/s-WHDjJM208).*

## Concept

A Reality Fabricator companion is a small always-on avatar device. A cloud
service (`api.rfab.ai`, WebSocket) tells the device which avatar **state** to
show — `intro`, `idle`, `speaking`, `happy`, `look_left`, `make_sandwich`,
`heart`, … — and for each state the device plays:

- a short looping **animation**: a flipbook of standalone JPEG frames (MJPEG,
  no inter-frame prediction) in the `.rfav` container, with per-frame alpha;
- an optional **voice line**: raw G.711 μ-law audio (`.ulaw`, 8 kHz mono,
  1 byte/sample) on the shipping device — on this Tab5 POC the voice lines are
  **OggOpus** (`.34k.opus`, 34 kbps mono) files, decoded on-device with libopus
  and played through the Tab5's ES8388 codec (Audio section below).

The shipping device (ESP32-S3 "CoreS3" board) runs exactly this in production:
it downloads `.rfav` + `.ulaw` clips from a cloud manifest into flash and plays
them back-to-back as the companion's "face". This repo is the Tab5 POC of that
same playback pipeline, at 4.5× the pixel count (720×1280 vs 320×240).

### Why MJPEG

The ESP32-P4 has no general-purpose video-decode hardware suitable for this
workload, but it *does* have a **hardware JPEG decoder (JDEC)** that can write
RGB565 pixels straight into a caller-chosen PSRAM buffer. So instead of a real
video codec, each frame is an independent baseline JPEG plus a 1-bit alpha
mask. Properties this buys:

- zero codec state / inter-frame dependency — any frame can be decoded alone,
  dropped, or re-seeked;
- constant-size mask per frame → constant JPEG offset within each frame → the
  whole file is a run of `(mask + JPEG)` pairs ideal for **sequential SD
  reads** (the container, RFAV-3, is built around exactly this);
- trivial encoder (ffmpeg) and trivial decoder (one hardware call + a mask
  composite pass);
- 12 fps × ~150 KB/frame ≈ 1.8 MB/s sustained SD traffic — comfortable for SD.

The cost: files are much bigger than VP9/H.264 (~17 MB for a 16 s clip vs a few
hundred KB) — irrelevant on an SD card, the point is the decode path.

## Feature summary

| | |
|---|---|
| Video | 720×1280 @ 12–13 fps, hardware JDEC into a 1.75 MB RGB565 draw buffer, ~19 MB PSRAM headroom, tear-free double-buffered present |
| Alpha | 1-bit per-frame mask, composited over a per-avatar background layer |
| Audio | per-clip OggOpus (34 kbps mono) → libopus → ES8388/I2S, 34 ms A/V offset, live volume + mute |
| Avatars | per-model `avatar.json` FSM + director script; model swap / background override via one `/avatars/config.json` — no recompile |
| WiFi | Tab5's on-board ESP32-C6 as remote engine (esp_hosted), NVS-stored credentials, live `SET_WIFI` over serial |
| Tooling | WebM → RFAV-3 + OggOpus encoders, host → SD file upload over USB serial with FNV-1a-32 end-to-end integrity |

## Container: RFAV-3

Spec: [`docs/rfav2_720p_spec_rev3.md`](docs/rfav2_720p_spec_rev3.md) ·
Reader: [`src/rfav3.{h,cpp}`](src/rfav3.cpp) · (a forward-looking "cropped-band"
RFAV-4 spec lives at [`docs/rfav4_720p_spec_rev4.md`](docs/rfav4_720p_spec_rev4.md)
— draft, not implemented)

```
[ 64 B header ][ N × 16 B frame table ][ (mask 115,200 B + JPEG) × N ]

header : "RFAV" | ver 3 | 720×1280 | fps | flags(loop|has_mask|4bit)
         | frame_count | table/data offsets | hdr CRC32
entry  : u32 absolute mask offset | u32 jpeg_len | u32 reserved | u32 crc
mask   : 1 bit/px, MSB-first, 90 B rows, 1=opaque   (4-bit is spec'd, unused in POC)
```

**Audio is deliberately external** (spec §5): `<clip>.rfav` + a sibling audio
file, started together. On the POC the sibling is `<clip>.34k.opus`
(OggOpus, 34 kbps mono) and it starts 34 ms after the clip's first presented
frame — see the Audio section below.

## Player pipeline (`src/main.cpp`)

Two-core FreeRTOS split with a 6-slot PSRAM prefetch ring (each slot =
115,200 B mask + up to 128 KB JPEG, credit-gated by semaphores):

1. **prefetch task (core 0)** reads `mask + JPEG` off the SD into free slots —
   batched: up to 4 physically contiguous frames in one seek + one SD
   transfer — so the player never blocks on SD;
2. **player task (core 1)**, once per frame period:
   - gate on the previous present's DMA2D completion,
   - take the ready slot,
   - **P4 hardware JPEG decode** → RGB565 straight into the 1.75 MB draw buffer,
   - **1-bit mask composite**: transparent pixels restored from the background
     layer (the avatar's own background image, or a generated checkerboard —
     `scripts/make_background.py`);
   - `esp_cache_msync` (the P4 has **no CPU↔DMA cache coherence** — this is the
     single most important discipline in the file; see the comment block in
     `main.cpp`),
   - **present**: explicit 4-arg `display(0,0,720,1280)` → the patched M5GFX
     `Panel_DSI` DMA2D-copies the whole draw buffer into the DPI driver's
     non-scanned fb and swaps it at vsync (double-buffer present path — no
     scanout tear at any frame rate).

On-screen chip: clip name, frame index, real fps, frame time vs the 83 ms
budget. 1 Hz telemetry on the 115200-baud USB CDC port.

M5GFX is modified (double-buffer present path + `rfab_poc_*` hooks). Upstream
M5GFX is pulled fresh by PlatformIO, so the exact patched
`Panel_DSI.{cpp,hpp}` are vendored in [`scripts/m5gfx_patched/`](scripts/m5gfx_patched/)
and synced in by **`scripts/patch_m5gfx.py`** (idempotent; see Setup below).

## Audio (`src/audio_opus.{h,cpp}`)

Per-clip voice. The full engine is implemented and integrated; it is currently
being stabilized (see Status & known issues).

- **Sources on SD:** `/avatars/<model>/34k_opus/<clip>.34k.opus` — OggOpus,
  34 kbps mono, 20 ms packets (a 48 kbps A/B set sits in `48k_opus/`,
  generated by `scripts/webm_to_opus_mono.py`). Clips with no `.opus` file
  play silent.
- **Pipeline:** hand-rolled OggOpus demux (page CRC + lacing) → **libopus**
  decode at 48 kHz mono → mono→stereo + live software gain → PSRAM 12-frame
  ring (46,080 B = 240 ms) → I2S feed task.
- **Hardware:** ES8388 codec on the internal I2C (0x10), Tab5 I2S0
  (MCK 30 / BCK 27 / WS 29 / DOUT 26, 48 kHz stereo), speaker amp
  PI4IOE5V6408 @0x43.
- **A/V sync:** each clip's audio starts **34 ms after that clip's first
  presented video frame** (measured from the present, not a wall clock);
  decode and feed tasks run on core 1, watermark-paced so the decoder never
  runs more than 160 ms ahead of the speaker.
- **Controls:** live volume 0..127 (boot default 32 ≈ 25%), tap = mute,
  h/v swipe = volume (touch UI).
- Every init step is non-fatal: any failure degrades to silence and the
  video keeps running.

## Avatars: data-driven (`src/avatar.{h,cpp}`)

All avatar behavior is **data, not code**: one `avatar.json` per model on the
SD (schema v1.0.0 — full spec in [`docs/avatar_json_spec.md`](docs/avatar_json_spec.md))
describes an embedded FSM (`companion_anim_fsm v1.0.0`: states, transitions,
guards, validated invariants) plus an ordered **director script** (dwell rows
that fire scripted events such as `play:gun`, `speech.end`,
`look_left.pressed`). The firmware parses and validates it at boot
(ArduinoJson, function-static buffers), and the player drives *everything* from
the active `AvatarDef` — states are indexed (no compile-time enum),
transitions and script rows are data, events are the JSON strings.

**Control plane — `/avatars/config.json`** (optional, ≤ 512 B):

```json
{ "version": "1.0.0", "active": "neko_voss", "bg": "cyber_neon_bg90.jpg" }
```

- `"active"` selects the model; `"bg"` (optional) overrides the background
  (resolved as `/avatars/<bg>`, first candidate; a missing file falls through).
- Boot load order (first success wins): `config.json` "active" → candidate
  list (`elena`, `neko_voss`) → **built-in Elena** (`src/avatar_elena.h`) —
  the device never wedges.
- **Swap avatars:** edit `config.json` + reboot (delete the file to fall back
  to the candidate list). **Add a new avatar:** drop
  `/avatars/<name>/{avatar.json, rfav/, 34k_opus/}` + set
  `"active":"<name>"` — no code change, no recompile.

## WiFi (on-board ESP32-C6, `src/wifi_sd.{h,cpp}`)

The Tab5's on-board **ESP32-C6** runs as a remote WiFi engine (esp_hosted over
4-bit SDIO; its slave firmware lives on the C6's own flash — the P4 holds no
slave partition). A dedicated task on P4 core 1 (low priority, never touches
the render path) handles STA connect with 60 s backoff retries, post-connect
health checks, and internal-heap telemetry.

**Credentials** — the permanent store is P4 NVS, namespace `skytab` (keys
`wifi_ssid` / `wifi_pass`):

- **Live update (the permanent path):** `SET_WIFI <ssid> [<pass>]` over USB
  serial — written to NVS, applied by the wifi task on its next retry (or
  re-arms a waiting task); a bare `SET_WIFI` clears the credentials. No
  reboot, no SD. The passkey is never printed to the log.
- **Interim provisioning:** an SD-root `/config.json` —
  `{"wifi_ssid":"...","wifi_pass":"..."}` — is read **once at boot when NVS is
  empty** and copied into NVS. This is a stopgap for getting credentials onto
  the device without a more permanent flashing tool; after that boot the SD is
  never re-read for credentials.

Today the WiFi link is used for provisioning and connectivity; the avatar
event queue already defines the hook for posting the same event strings from
an external (cloud) source once the link is stable.

## Host → SD file transfer (`src/file_transfer.{h,cpp}`)

A small line-protocol receiver over USB CDC (dedicated task on P4 core 1,
below the audio tasks' priority) pushes files straight onto the SD card while
the device runs — no card removal:

```
UPLOAD <sd_path> <byte_size>\n  →  READY\n  →  <raw bytes, exact size>
                                 →  OK <bytes> fnv=<HEX>\n    (or ERR <reason>\n — partial file removed)

SET_WIFI <ssid> [<pass>]\n      →  WIFI_OK ...\n
```

Integrity: **FNV-1a-32** (offset `0x811C9DC5`, prime `0x01000193`) is computed
on the device over the streamed bytes and on the host over the file; the
transfer only counts as good when they match. Parent directories are created
on demand; `<sd_path>` is 1:1 with the app's own paths (e.g.
`/avatars/neko_voss/rfav/intro.rfav`).

Host sender: [`scripts/upload_flash.py`](scripts/upload_flash.py) (Python,
`pip install pyserial`):

```powershell
python scripts/upload_flash.py --port COM8 --src .\bg.jpg --dst /avatars/bg.jpg
```

Stop the serial monitor first — the port is shared.

## Encode pipeline (`scripts/`)

Source material is a **VP8 + alpha WebM** (the 90° rotation to portrait is
baked in at encode time, so the device writes decoded frames straight into the
portrait draw buffer):

```
input.webm --ffmpeg 12 fps + transpose=1 (rgba)-->
   color:  baseline JPEG, yuvj420p, -q:v 8
   alpha:  geq threshold 128 → 1-bit mask (115,200 B)
--> packed RFAV-3
```

```powershell
python scripts/webm_to_rfav_portrait.py input.webm out.rfav [--fps 12] [--q 8] [--rotate cw|ccw|none] [--no-loop]
python scripts/webm_to_opus_mono.py input.webm out    # → .34k.opus (+ .48k.opus A/B set)
```

`scripts/probe_bg.py` / `probe_gun.py` verify encoded files (header, frame
table, JPEG scan); `scripts/make_background.py` generates the 720×1280
checkerboard background. Requires `ffmpeg`/`ffprobe` on PATH (or
`C:\ffmpeg\ffmpeg.exe`, which the scripts prefer if present).

## Repository layout

```
rfab/
├── README.md                  ← you are here
├── .gitattributes  .gitignore
├── platformio.ini             pioarduino 54.03.21, esp32-p4-evboard, 3 MB app slot, WiFi pool trims
├── app3M_fat9M_16MB.csv       16 MB partition table (dual 3 MB app + 9.9 MB FAT)
├── idf_component.yml          pins espressif/esp_hosted ==2.13.1
├── lib/opus/                  vendored libopus v1.4 (decode-only)
├── sd_stage/                  ← reference SD card content (~685 MB, Git LFS) — copy to the microSD
├── src/                       firmware (PlatformIO src dir)
│   ├── main.cpp               player (pipeline, prefetch ring, cache discipline, UI, telemetry)
│   ├── rfav3.h / rfav3.cpp    RFAV-3 container reader (combined + batched SD reads)
│   ├── avatar.h / avatar.cpp  data-driven avatars (avatar.json FSM + director, config.json)
│   ├── avatar_elena.h         built-in Elena fallback avatar
│   ├── audio_opus.h / .cpp    Opus audio engine (OggOpus → libopus → ES8388/I2S)
│   ├── wifi_sd.h / .cpp       C6 WiFi (esp_hosted), NVS credentials, /config.json provisioning
│   ├── file_transfer.h / .cpp serial UPLOAD / SET_WIFI receiver
│   └── reality_fabricator_portrait_bg.jpg   fallback background (91 KB)
├── scripts/                   host-side Python
│   ├── webm_to_rfav_portrait.py    ← THE encoder: VP8+alpha webm → packed RFAV-3 (portrait)
│   ├── webm_to_opus_mono.py        webm audio → mono OggOpus .34k.opus
│   ├── upload_flash.py             host sender for the SD upload protocol (pyserial)
│   ├── patch_m5gfx.py         sync patched M5GFX Panel_DSI into .pio/libdeps (idempotent)
│   ├── m5gfx_patched/         exact Panel_DSI.{cpp,hpp} the firmware was built from
│   ├── make_background.py     generate the 720×1280 checkerboard background.jpg
│   └── probe_bg.py  probe_gun.py   encoded-file verifiers
└── docs/
    ├── rfav2_720p_spec_rev3.md    ← current RFAV-3 spec
    ├── rfav4_720p_spec_rev4.md    ← RFAV-4 (cropped-band) spec — draft, not implemented
    ├── avatar_json_spec.md        ← avatar.json / config.json schema
    └── Tab5/                      earlier spec revisions + probes (history)
```

The tree above is the **committed set**. Local working sets stay out of git
(see `.gitignore`): `assets/` (the pre-encode source webms + encodes behind
`sd_stage/`), `decoded/` (shipping-companion flash extract: 16 `.rfav` clips,
17 `.ulaw` voice clips, manifest, format notes — the reference for the μ-law
voice format), and dev/session scratch under `tools/`.

## Setup & build (fresh clone)

Prereqs:

- an **M5Stack Tab5** + a **microSD** (FAT32, U1 minimum / U3 preferred — see
  [SD card](#sd-card-requirements-and-layout)) carrying the reference layout
- [PlatformIO Core](https://platformio.org/install/cli) (`pio`)
- [Git LFS](https://git-lfs.com) (the `sd_stage/` reference content is LFS-tracked)
- Python 3 + `pip install pyserial` (host tooling: uploader, encoders)
- `ffmpeg` / `ffprobe` (only if you author new clips)

```powershell
git clone <this repo> && cd <this repo>
# copy the reference SD content to the card (sd_stage/ tree -> card root), then
pio pkg install
python scripts/patch_m5gfx.py      # vendor the patched M5GFX into .pio/libdeps
pio run                            # first build compiles the IDF/Arduino core from source (see notes)
pio run -t upload --upload-port COM8
pio device monitor --port COM8     # 115200 baud
```

Notes:

- **`sd_stage/` is Git LFS** (~685 MB of clips + voice). Install Git LFS
  *before* cloning, or after a plain clone run `git lfs install && git lfs
  pull` — otherwise the clip files are small pointer stubs and the device
  will report bad headers.

- **The libopus include path in `platformio.ini` is absolute** (an `-I` flag in
  `build_flags` pointing at `<clone>\rfab\lib\opus\src\include`). After cloning
  elsewhere, edit that line to your checkout's path — or make it portable:
  `-I${PROJECT_DIR}/lib/opus/src/include`.
- The **first** `pio run` before patching M5GFX will fail to link
  (`rfab_poc_*` undefined) — that's the expected signal to run the patch
  script, then `pio run -t clean && pio run`.
- If PlatformIO ever re-fetches M5GFX (lib re-resolution), re-run
  `scripts/patch_m5gfx.py` + clean rebuild.
- **First build is slow (10–30 min):** the `custom_sdkconfig` block (trimmed
  WiFi buffer pools — the always-on C6 WiFi stack would otherwise claim the
  internal SRAM the SD driver needs) forces a one-time recompile of the IDF +
  Arduino core from source.
- The build pins **pioarduino 54.03.21** (`platformio.ini`) and
  **esp_hosted ==2.13.1** (`idf_component.yml`, with
  `IDF_COMPONENT_OVERWRITE_MANAGED_COMPONENTS = 0`). Bumping either re-resolves
  the C6 host stack — re-verify WiFi bring-up before adopting.
- Flashing and the serial console share the Tab5's single USB port. Stop the
  monitor before uploading; after a successful upload (`Hash of data
  verified`), power-cycle the Tab5 (the DTR/RTS reset toggle is a no-op on
  this board).

### SD card requirements and layout

**Required.** All avatar content (clips, voice, backgrounds, config) lives on
a microSD card (SD_MMC, 4-bit), formatted **FAT32**, mounted at boot:

- **Speed class: U1 minimum, U3 (V30) preferred.** The player sustains
  ~1.8 MB/s of sequential reads (12 fps × ~150 KB/frame), with short bursts
  as the 6-slot prefetch ring re-fills after a stall. U1's guaranteed
  10 MB/s covers that with headroom; U3's guaranteed 30 MB/s is a
  comfortable margin and re-fills the ring faster.
- **Capacity: 16 GB or more** recommended — the reference content is
  ~685 MB, so a larger card leaves room to add avatars.
- **Reference content:** the exact layout below ships in this repo under
  [`sd_stage/`](sd_stage/) — copy its tree to the card root.

Content is per-model, so multiple avatars coexist on one card:

```
/config.json                              OPTIONAL, interim: {"wifi_ssid":"...","wifi_pass":"..."}
                                          (read once at boot when NVS is empty → copied to NVS)
/avatars/config.json                      OPTIONAL (≤512 B): {"version":"1.0.0","active":"<model>","bg":"<file>.jpg"}
/avatars/<bg>.jpg                         avatar backgrounds live at the avatars/ root (720×1280)
/avatars/<model>/avatar.json              the model's FSM + director (docs/avatar_json_spec.md)
/avatars/<model>/rfav/<clip>.rfav         clips (RFAV-3, 720×1280, 12 fps, 1-bit mask)
/avatars/<model>/34k_opus/<stem>.34k.opus voice lines (OggOpus 34 kbps mono; missing = silent)
```

The reference card ships with two avatars — `elena` (20 clips, 9 voice lines)
and `neko_voss` (20 clips, 6 voice lines) — plus each one's background
(`reality_fabricator_portrait_bg90.jpg` / `cyber_neon_bg90.jpg`). Which avatar
plays — and which clips and voice lines it uses — is decided entirely by these
data files: `config.json` picks the model, `avatar.json` drives the rest.

## Controls & telemetry

- **Touch:** tap = mute toggle · h/v swipe = volume (0..127, boot default
  32 ≈ 25%)
- **On-screen chip:** clip name, frame index, real fps, frame time vs the
  83 ms budget
- **Serial:** 1 Hz telemetry lines on the 115200-baud USB CDC port (player
  `pfs`, prefetch `hof`, audio `au`, transfer `ft`); the boot banner reports
  the active avatar, background, PSRAM free, and audio state

## Status & known issues

- **Video (stable):** 12–13 fps presented, no crashes, PSRAM flat (~19 MB
  free), background-alpha compositing correct, colors correct, no scanout tear.
- **Audio (being stabilized):** the full OggOpus → libopus → ES8388 path is
  implemented and integrated; decode underruns can still occur under load as
  of this writing and the voice path is being chased to zero.
- **WiFi (in progress):** credential provisioning (NVS + serial `SET_WIFI` +
  SD bootstrap) and the retry/health-check task are in place; a stable STA
  connect through the C6 is still being chased.
- **RFAV-4** (cropped-band container) is a written spec
  ([`docs/rfav4_720p_spec_rev4.md`](docs/rfav4_720p_spec_rev4.md)) but not
  implemented; everything on the card today is RFAV-3.

## Reference: the shipping companion (reverse-engineered)

The Tab5 POC is modeled on the production RFab companion (ESP32-S3 CoreS3),
whose flash was dumped and fully unpacked: 16 `.rfav` flipbooks (320×240,
12 fps, RFAV rev1 footer format) + 17 `.ulaw` voice clips + a JSON state
manifest, all byte-verified against the cloud S3 URLs. Format details and the
naming hash scheme (FNV-1a-32 of the manifest URL) are in the local
`decoded/` reference set (gitignored).

## License

MIT — see [`LICENSE`](LICENSE). The vendored
[`lib/opus`](lib/opus) is BSD-3-Clause (Xiph.Org; see
[`lib/opus/src/COPYING`](lib/opus/src/COPYING)).
