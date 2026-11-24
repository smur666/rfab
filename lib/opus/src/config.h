/* config.h — hand-rolled for the rfab ESP32-P4 (M5Stack Tab5) opus port.
 *
 * Replaces the autotools/CMake-generated config for the vendored
 * xiph/opus v1.4 sources (see lib/opus/library.json for the file list,
 * taken verbatim from the repo's *_sources.mk).
 *
 * RULE (learned the hard way): opus gates features with #ifdef in many
 * places, so ONLY DEFINE macros that should be ENABLED. Defining a macro
 * to 0 still trips #ifdef (e.g. OPUS_ARM_INLINE_ASM 0 pulled ARM asm
 * onto RISC-V).
 *
 * Decode-only use, float DSP (P4 RISC-V has an FPU), no SIMD/asm paths.
 * Build flags (library.json): -DHAVE_CONFIG_H -DOPUS_BUILD
 *   -DNONTHREADSAFE_PSEUDOSTACK -DHAVE_LRINTF
 */
#ifndef RFAB_OPUS_CONFIG_H
#define RFAB_OPUS_CONFIG_H 1

/* --- package ---------------------------------------------------------- */
#define OPUS_VERSION_STRING "1.4"
#define OPUS_VERSION 0x010400

/* --- enabled C99 features (define = on; disabled features stay undef) -- */
#define HAVE_STDINT_H 1
#define HAVE_STDBOOL_H 1
#define OPUS_HAVE_INLINE 1

#endif /* RFAB_OPUS_CONFIG_H */
