// ============================================================================
// S122: AVATAR — per-model DATA-DRIVEN FSM + director (avatar.json v1.0.0)
// ----------------------------------------------------------------------------
// One avatar.json per model on the SD:  /avatars/<name>/avatar.json
//   { version, name, bg, state_machine (companion_anim_fsm v1.0.0),
//     script (director rows), tuning }
// Paths are DERIVED from name (fixed layout, spec: docs/Tab5/avatar_json_spec.md):
//   clips:  /avatars/<name>/rfav/<clip>.rfav
//   audio:  /avatars/<name>/34k_opus/<stem>.34k.opus
//   bg:     /avatars/<bg>   (avatar's own file; fallbacks stay in firmware)
//
// S132: load() first reads /avatars/config.json ({"active":"<model>",
// "bg":"<file>.jpg"}) — the data-driven control plane: swap avatars (or
// override the background) by editing that one SD file + reboot; no renames,
// no recompile, new models addable without code changes. Missing/bad config
// -> the candidate list (elena; neko_voss, pre-S132 behavior) -> built-in
// Elena (avatar_elena.h, the S121 tables verbatim) on ANY failure, so the
// device never wedges. One avatar per boot (swap = config.json + reboot).
//
// The player (main.cpp) drives EVERYTHING from the active AvatarDef:
// states are INDEXED (no compile-time enum), transitions/script rows are
// data, events are the JSON strings ("play:gun", "look_left.pressed", ...).
// ============================================================================
#pragma once

#include <stdint.h>

namespace av {

constexpr int8_t  kAnyFrom = -1;     // transition "from": "*"
constexpr int8_t  kNoState = -1;     // script "after": absent -> anchor = idle
constexpr uint8_t kStateN  = 0xFF;   // "no state" sentinel (dirTick return)

struct State {
  char     name[24];      // state name (FSM identity)
  char     clip[24];      // .rfav stem (no extension)
  bool     loop;          // clip exhausted -> replay in place (no event)
  bool     auto_return;   // fires <name>.finished at clip end -> back to idle
  bool     terminal;      // absorbing (session end)
  char     on_finish[24]; // initial state: <name>.finished target ("" = none)
  int8_t   prio;          // (informational)
};

struct Trans {
  int8_t   from;          // state index or kAnyFrom ("*")
  uint8_t  to;            // state index
  char     event[24];     // "play:<clip>" | "speech.end" | "<x>.finished" | ...
  char     guard[24];     // "" | "clip_end" | "<n>.still_held" | "<n>.released"
                          // | "not_terminal"
};

struct Row {
  char     event[24];     // the scripted FSM event
  uint32_t dwell_ms;      // due = in anchor state for >= dwell_ms
  int8_t   after;         // anchor state index, or kNoState -> idle
};

struct AvatarDef {
  char     name[24];
  char     bg[48];        // file name, resolved as /avatars/<bg>
  uint8_t  nStates;
  uint8_t  nTrans;
  uint8_t  nRows;
  uint8_t  initial;       // state index (intro)
  uint8_t  idle;          // state index (default_state)
  uint32_t idle_min_ms;   // tuning (row dwells are authoritative)
  uint32_t look_hold_ms;  // tuning
  State    states[24];
  Trans    trans[48];
  Row      rows[32];
};

// ---- active avatar (set once at boot by load()) ----
const AvatarDef *active();      // never null (built-in fallback guaranteed)
const char      *activeName();  // active()->name
bool             fromSd();      // true = an SD avatar.json is active
const char       *configBg();   // S132: config.json "bg" override ("" = none)

// S132: config.json "active" first, then the candidate list; first VALID one
// wins and becomes active. SD absent / all invalid -> built-in Elena.
// Prints "avatar: <name> (sd[, config])" or "avatar: elena (built-in
// fallback)". Safe to call with the SD not mounted (degrades to built-in).
void load();

}  // namespace av
