# avatar.json — v1.0.0 (SD per-avatar state machine + director script)

One file per avatar model: `D:\avatars\<name>\avatar.json` (FAT, case-insensitive).
The firmware loads it at boot (ArduinoJson), validates it, and drives the
animation FSM + director + paths from it. Missing/invalid file -> built-in
fallback avatar (device never wedges). Target size < 12 KB (StaticJsonDocument ~10-16 KB).

## Top level

```jsonc
{
  "version": "1.0.0",                       // schema version (exact match required)
  "name":  "elena",                         // model id == SD folder name
  "bg":    "reality_fabricator_portrait_bg90.jpg",  // resolved as /avatars/<bg>
  "state_machine": { ... },                 // embedded companion_anim_fsm v1.0.0 object
  "script":      [ ... ],                   // director rows (ordered)
  "tuning":      { "idle_min_ms": 4000, "look_hold_ms": 5000 }
}
```

- **Paths are derived from `name`** (fixed layout, not per-file):
  - clips:  `/avatars/<name>/rfav/<clip>.rfav`   (RFAV-3, 720x1280, 12 fps, 1-bit mask)
  - audio:  `/avatars/<name>/34k_opus/<stem>.34k.opus` (34 kbps mono; missing = silent, logged once)
  - bg:     `/avatars/<bg>` (720x1280 direct; 1280x720 -> one-shot boot transpose; fallbacks stay in firmware)
- **`tuning`** = conventional pacing defaults. Row-level `dwell_ms` is
  authoritative; the loader may use tuning as sanity bounds.

## config.json — top-level control file (S132, data-driven avatar swap)

`/avatars/config.json` (FAT, **<= 512 B**, optional) selects the active model
and can override the background — no renames, no recompile, no code change:

```jsonc
{
  "version": "1.0.0",
  "active": "neko_voss",                              // model id == /avatars/<name> folder
  "bg":     "reality_fabricator_portrait_bg90.jpg"    // OPTIONAL: bg file at /avatars/<bg>
}
```

Boot-time load order (first success wins, `avatar.cpp`):

1. `config.json` present and `"active"` names a model whose `avatar.json`
   parses + validates -> that model is active (boot line
   `avatar: <name> (sd, config) — N states / …`).
2. Otherwise the candidate list, in order: `elena`, `neko_voss` (the
   pre-S132 default — with `config.json` absent behavior is unchanged).
3. Otherwise the built-in Elena (`avatar_elena.h`) — the device never wedges.

- `"bg"` (optional) becomes the FIRST background candidate
   (`/avatars/<value>`), ahead of the avatar's own `bg` value; a missing
   override file simply falls through to the next candidate (rev3 spec 7.1).
- Value safety: `active`/`bg` are used to build an `/avatars/<value>` path,
  so the loader rejects `/`, `\` and leading `.` and caps the lengths
  (active 32 B, bg 48 B).
- Read with a function-static 512 B buffer + `StaticJsonDocument` (never on
  the 8 KB task stack — the S127 stack rule); a missing/unreadable/unparseable
  file is ignored silently (pre-S132 behavior).
- **Swap to another model:** set `"active"` to the model's folder name (or
  delete `config.json` to fall back to the elena-first candidate list).
  **Add a new model:** drop `/avatars/<name>/{avatar.json, rfav/, 34k_opus/}`
  and set `"active":"<name>` — no code, no recompile.

## state_machine (embedded object, companion_anim_fsm v1.0.0)

Same schema as `assets/state_machine.json` (elena's file embeds it VERBATIM):

- `initial`, `terminal[]`, `default_state` — all must name existing states.
- `states`: `{ <name>: { clip, loop, interruptible, auto_return,
  priority?, gaze?, role?, on_finish?, terminal? } }`
  - `clip` = file stem (no `.rfav`).
  - `loop: true` -> clip exhausted replays in place (no event).
  - `auto_return: true` -> one-shot; fires `<name>.finished` at clip end and
    must transition back to idle.
  - `on_finish` (intro only) -> state entered when the (non-looping) initial
    clip ends, via `<initial>.finished` + guard `clip_end`.
- `transitions`: `[{ from, to, event, guard? }]`
  - `from: "*"` = any state except the initial, the terminal, and the state itself.
  - `guard`: `clip_end` (event only honored at clip exhaustion) |
    `<name>.still_held` | `<name>.released` (gaze hold-state checks) |
    `not_terminal` (outro edge).
- `invariants` (validated):
  1. `intro` never repeats within a session (session restart re-enters it only).
  2. every non-terminal, non-looping state has a path back to idle.
  3. `outro` is absorbing (no transitions out).

### Event vocabulary

`play:<clip>` (one-shots, from `*`), `speech.start`/`speech.end`,
`think.start`/`think.end`, `look_<l|r>.pressed`/`.released` (+ the gaze
`_in.finished`/`_out.finished` + `cancel` chain where the avatar has gaze
clips), `<clip>.finished` (internal, at clip end for auto_return states),
`outro` (to terminal), `cancel` (gaze aborts -> idle).
External sources (WiFi, later) post the SAME strings via the event queue —
drained ahead of the script.

## script (director rows)

Ordered list; the engine walks it top to bottom, one active row at a time.
Rows fire SCRIPTED events (the S112 director, data-driven).

```jsonc
{ "event": "play:happy", "dwell_ms": 4000 }                        // anchor = idle
{ "event": "speech.end", "dwell_ms": 4000, "after_state": "speaking" }
{ "event": "look_left.released", "dwell_ms": 5000, "after_state": "look_left_in" }
```

- **anchor** = `after_state` if present, else `"idle"`.
- **Due rule:** the engine has been in the anchor state for >= `dwell_ms`.
  (The engine stamps an enter timestamp on EVERY state entry —
  `enterClipForState`/`g_fsm_enter_ms` — so "in the anchor for dwell_ms" is
  exact. A row is only *checked* while the engine sits in the anchor, so
  external interruptions can't fire rows early; on rejection the row retries
  next tick and only advances when the FSM ACCEPTS the event — identical to
  firmware `dirTick`.)
- Row kinds and their anchors, matching the firmware 1:1:
  | row | anchor | firmware equivalent |
  |---|---|---|
  | `play:<x>`, presses, `speech.start`, `think.start`, `outro` | `idle` | dirTick default branch (`kIdleMinMs` rows) |
  | `look_l/r.released` | `look_l/r_in` (entered at the press tick) | `lookReleaseDueNow()` — span = press->release = dwell (`kLookHoldMs`); the `_in` clip (2.17/2.67 s) plays inside the span, hold gets the rest |
  | `speech.end` | `speaking` | `g_speech_start_ms` + `kSpeechMs` (entry into Speaking == start tick) |
  | `think.end` | `thinking` | `g_think_start_ms` + `kThinkMs` (S121: 10000 = full 9.9 s clip) |
- **Session wrap:** the last row is `outro`; when the outro clip ends the
  engine bumps the session counter and re-enters `initial` (script index
  resets) — the show restarts forever.

## Validation (loader -> fall back to built-in avatar on ANY failure)

1. `version == "1.0.0"`; `name` non-empty, valid SD-dir chars.
2. `state_machine` present; `initial`/`terminal[]`/`default_state` exist.
3. every `from`/`to`/`on_finish`/`after_state` (incl. `"*"` expansion) names a state;
   every `play:<x>` row's `<x>` is a state; every `clip` non-empty.
4. no state is both `auto_return` and `loop`; `terminal` states have no outgoing
   transitions; every non-terminal, non-looping state can reach idle.
5. `script` non-empty, ends in `outro`, dwells are ints in [1, 300000].
6. On-device clip coverage: every state's `<clip>.rfav` opens + parses as RFAV-3
   720x1280 (first-boot check; failures logged per clip, the show continues —
   the missing clip renders the bg).

## Files

- `/avatars/config.json` on SD — the S132 control file above (current unit:
  `{"version":"1.0.0","active":"neko_voss"}`, no bg override; both models'
  `avatar.json` files coexist under `/avatars/`).
- `assets/avatar_elena.json`  — Elena (20 states / 38 transitions / 18 rows;
  `state_machine` = `assets/state_machine.json` verbatim; S121 dwells:
  idle 4000, gaze span 5000, speech 4000, think 10000).
- `assets/avatar_neko_voss.json` — Neko Voss (20 states / 36 transitions / 20 rows;
  no gaze chains — `look_left_talk` is a plain one-shot; `think.end` dwell 10000 =
  its 120-frame/10.00 s silent thinking clip; one-shots: angry, angry_talk, cry,
  happy, happy_talk, headshake, laugh, look_left_talk, nod, popcorn, sad,
  sad_talk, shy, surprised, surprised_talk; audio only on cry/headshake/laugh/
  nod/popcorn/shy — the rest silent by design).
- Builder/validator: `tools/s122_avatar_json.py` (regenerates both + re-checks
  SD clip coverage against `D:\AVATARS`).
