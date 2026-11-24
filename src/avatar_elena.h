// ============================================================================
// S122: BUILT-IN ELENA — the S121 tables verbatim (avatar_elena.json v1.0.0)
// ----------------------------------------------------------------------------
// FALLBACK avatar: active when the SD has no valid /avatars/<name>/avatar.json
// (missing, corrupt, or any validation failure). 20 states / 38 transitions /
// 18 script rows — byte-for-byte the S121 behavior (kFsmDef + the compiled
// transition logic + kDirScript with the S121 dwells: idle 4000, gaze span
// 5000, speech 4000, think 10000).
// ============================================================================
#pragma once

#include "avatar.h"

namespace av {

static constexpr AvatarDef kBuiltinElena = {
  .name  = "elena",
  .bg    = "reality_fabricator_portrait_bg90.jpg",
  .nStates = 20,  // S127: was 21 — a phantom zeroed slot; the list has 20
  .nTrans  = 38,
  .nRows   = 18,
  .initial = 0,   // intro
  .idle    = 1,   // idle
  .idle_min_ms  = 4000,
  .look_hold_ms = 5000,
  .states = {
    {"intro",         "intro",         false, false, false, "idle",    0},
    {"idle",          "idle",          true,  false, false, "",        0},
    {"look_left_in",  "look_left_in",  false, true,  false, "",        50},
    {"look_left",     "look_left",     true,  false, false, "",        50},
    {"look_left_out", "look_left_out", false, true,  false, "",        50},
    {"look_right_in", "look_right_in", false, true,  false, "",        50},
    {"look_right",    "look_right",    true,  false, false, "",        50},
    {"look_right_out","look_right_out",false, true,  false, "",        50},
    {"ahegao",        "ahegao",        false, true,  false, "",        20},
    {"ar15",          "ar15",          false, true,  false, "",        20},
    {"gun",           "gun",           false, true,  false, "",        20},
    {"happy",         "happy",         false, true,  false, "",        20},
    {"heart",         "heart",         false, true,  false, "",        20},
    {"lovestruck",    "lovestruck",    false, true,  false, "",        20},
    {"make_sandwich", "make_sandwich", false, true,  false, "",        20},
    {"speaking",      "speaking",      true,  false, false, "",        30},
    {"surprised",     "surprised",     false, true,  false, "",        20},
    {"thinking",      "thinking",      true,  false, false, "",        25},
    {"yandere",       "yandere",       false, true,  false, "",        20},
    {"outro",         "outro",         false, false, true,  "",        100},
  },
  .trans = {
    // intro
    {0,  1,  "intro.finished",      "clip_end"},
    // gaze chains (atomic in->hold->out; held/released guards at _in end)
    {1,  2,  "look_left.pressed",   ""},
    {1,  5,  "look_right.pressed",  ""},
    {2,  3,  "look_left_in.finished", "look_left.still_held"},
    {2,  4,  "look_left_in.finished", "look_left.released"},
    {2,  1,  "cancel",              ""},
    {3,  4,  "look_left.released",  ""},
    {3,  1,  "cancel",              ""},
    {4,  1,  "look_left_out.finished", "clip_end"},
    {5,  6,  "look_right_in.finished", "look_right.still_held"},
    {5,  7,  "look_right_in.finished", "look_right.released"},
    {5,  1,  "cancel",              ""},
    {6,  7,  "look_right.released", ""},
    {6,  1,  "cancel",              ""},
    {7,  1,  "look_right_out.finished", "clip_end"},
    // one-shot gestures from "*"
    {kAnyFrom,  8,  "play:ahegao",       ""},
    {kAnyFrom,  9,  "play:ar15",         ""},
    {kAnyFrom,  10, "play:gun",          ""},
    {kAnyFrom,  11, "play:happy",        ""},
    {kAnyFrom,  12, "play:heart",        ""},
    {kAnyFrom,  13, "play:lovestruck",   ""},
    {kAnyFrom,  14, "play:make_sandwich",""},
    {kAnyFrom,  16, "play:surprised",    ""},
    {kAnyFrom,  18, "play:yandere",      ""},
    {kAnyFrom,  15, "speech.start",      ""},
    {kAnyFrom,  17, "think.start",       ""},
    // auto_return .finished (clip_end)
    {8,  1,  "ahegao.finished",       "clip_end"},
    {9,  1,  "ar15.finished",         "clip_end"},
    {10, 1,  "gun.finished",          "clip_end"},
    {11, 1,  "happy.finished",        "clip_end"},
    {12, 1,  "heart.finished",        "clip_end"},
    {13, 1,  "lovestruck.finished",   "clip_end"},
    {14, 1,  "make_sandwich.finished","clip_end"},
    {16, 1,  "surprised.finished",    "clip_end"},
    {18, 1,  "yandere.finished",      "clip_end"},
    // speaking / thinking exits
    {15, 1,  "speech.end",            ""},
    {17, 1,  "think.end",             ""},
    // terminal
    {kAnyFrom, 19, "outro",           "not_terminal"},
  },
  .rows = {
    {"play:ahegao",           4000, kNoState},
    {"play:ar15",             4000, kNoState},
    {"play:gun",              4000, kNoState},
    {"play:happy",            4000, kNoState},
    {"play:heart",            4000, kNoState},
    {"look_left.pressed",     4000, kNoState},
    {"look_left.released",    5000, 2},   // anchor look_left_in
    {"look_right.pressed",    4000, kNoState},
    {"look_right.released",   5000, 5},   // anchor look_right_in
    {"play:lovestruck",       4000, kNoState},
    {"play:make_sandwich",    4000, kNoState},
    {"speech.start",          4000, kNoState},
    {"speech.end",            4000, 15},  // anchor speaking
    {"play:surprised",        4000, kNoState},
    {"think.start",           4000, kNoState},
    {"think.end",             10000, 17}, // anchor thinking (S121)
    {"play:yandere",          4000, kNoState},
    {"outro",                 4000, kNoState},
  },
};

}  // namespace av
