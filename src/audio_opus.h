// S112: Opus audio pipeline — public surface (engine in audio_opus.cpp).
//
// Plays /avatars/<model>/34k_opus/<clip>.34k.opus for the clips that have one, with a
// 34 ms start delay measured from the video clip's first presented frame
// (g_clip_first_pres_us, set by main.cpp). Clips without an .opus file play
// silent.
#pragma once

namespace rfav_audio {
// ES8388 (I2C 0x10) + Tab5 I2S0 (MCK 30 / BCK 27 / WS 29 / DOUT 26) + PI4IOE
// amp (0x43 reg 0x05 bit 0x02) + decode/feed tasks (core 1). ALL steps
// non-fatal: any failure degrades to silent, video keeps running.
// Call from setup() after M5.begin() and the first ensureSd().
bool audioInit();
// S119: live volume, 0..127 (127 = full scale); boot default 32 (25%).
uint32_t getVolume();
void setVolume(uint32_t v);
// Boot-log probes (kept from the S110 link probe).
const char* opusVersion();
bool probeDecoder();
}  // namespace rfav_audio
