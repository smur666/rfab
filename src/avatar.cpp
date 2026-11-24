// ============================================================================
// S122: AVATAR LOADER — /avatars/<model>/avatar.json (v1.0.0) -> AvatarDef
// ----------------------------------------------------------------------------
// Called once from setup() AFTER the ensureSd() block, BEFORE
// initBackground() (the bg candidate list is built from the active avatar).
//
//   1. S132: read /avatars/config.json —
//      {"version":"1.0.0", "active":"<model>", "bg":"<file>.jpg"}
//      — the data-driven control plane: which model to load + an optional
//      background override. Swap avatars (or override the bg) by editing
//      that ONE file on the SD (no renames, no recompile; a brand-new model
//      is addable the same way). Missing/bad config or unknown model -> 2.
//   2. Try the candidate models in priority order (elena; neko_voss): read
//      /avatars/<name>/avatar.json (<= 12288 B), parse (ArduinoJson v7) +
//      validate (spec docs/Tab5/avatar_json_spec.md). Pre-S132 behavior —
//      still the default when no config.json exists.
//   3. First VALID file wins -> it becomes the active avatar.
//   4. ANY failure (SD absent / no file / parse / invalid) -> the built-in
//      Elena (avatar_elena.h, the S121 tables verbatim) — the device never
//      wedges.
//
// Spec rule 6 — on-device clip coverage (LOG-ONLY): every state's
// /avatars/<name>/rfav/<clip>.rfav must open + parse as RFAV-3 and be
// 720x1280. Failures warn per clip; the show continues (a missing clip
// renders the background).
// ============================================================================

#include "avatar.h"
#include "avatar_elena.h"
#include "rfav3.h"

#include <Arduino.h>
#include <SD_MMC.h>
#include <ArduinoJson.h>
#include <string.h>

namespace av {
namespace {

const AvatarDef *s_active = &kBuiltinElena;  // fallback until load()
bool s_from_sd = false;
bool s_from_config = false;  // S132: the active pick came from config.json
char s_cfg_active[32] = {};  // S132: config.json "active" ("" = not set)
char s_cfg_bg[48] = {};      // S132: config.json "bg" override ("" = not set)

// Fallback priority order (pre-S132 behavior; also the no-config default).
const char *const kCandidates[] = {"elena", "neko_voss"};
constexpr size_t kCandidateN = sizeof(kCandidates) / sizeof(kCandidates[0]);
constexpr size_t kMaxJson = 12288;  // spec target < 12 KB; the read cap

int nameIdx(const AvatarDef *d, const char *n) {
  if (!n || !n[0]) return -1;
  for (int i = 0; i < d->nStates; i++)
    if (strcmp(d->states[i].name, n) == 0) return i;
  return -1;
}

// Spec rule 4: every non-terminal state can reach idle, ignoring guards.
// idle itself is trivially ok. A "*" edge expands to every non-initial,
// non-terminal state except the edge's own target.
bool reachesIdle(const AvatarDef *d) {
  for (int s = 0; s < d->nStates; s++) {
    if (d->states[s].terminal || s == d->idle) continue;
    bool vis[24] = {};
    int8_t q[24] = {};
    int qh = 0, qt = 0;
    q[qt++] = (int8_t)s;
    vis[s] = true;
    bool found = false;
    while (qh < qt && !found) {
      const int u = q[qh++];
      for (int t = 0; t < d->nTrans; t++) {
        const Trans &tr = d->trans[t];
        int v = -1;
        if (tr.from == u) v = tr.to;
        else if (tr.from == kAnyFrom && u != d->initial &&
                 !d->states[u].terminal && tr.to != u)
          v = tr.to;
        if (v < 0) continue;
        if (v == d->idle) { found = true; break; }
        if (!vis[v]) { vis[v] = true; q[qt++] = (int8_t)v; }
      }
    }
    if (!found) {
      printf("avatar: invalid — state '%s' cannot reach idle\n", d->states[s].name);
      return false;
    }
  }
  return true;
}

// Parse + validate the JSON in buf (n bytes) into d (a caller-owned static).
// Returns true on success; prints the first failure reason on failure.
bool parseAndValidate(const char *buf, size_t n, AvatarDef &d) {
  static StaticJsonDocument<24576> doc;  // function-static (NOT stack, ~24 KB)
  const DeserializationError err = ArduinoJson::deserializeJson(doc, buf, n);
  if (err) {
    printf("avatar: JSON parse failed — %s\n", err.c_str());
    return false;
  }
  if (!doc.is<JsonObject>()) {
    printf("avatar: invalid — root is not an object\n");
    return false;
  }

  // ---- top level (rules 1-2) ----
  const char *version = doc["version"].as<const char *>();
  if (!version || strcmp(version, "1.0.0") != 0) {
    printf("avatar: invalid — version must be \"1.0.0\" (got \"%s\")\n",
           version ? version : "?");
    return false;
  }
  const char *name = doc["name"].as<const char *>();
  if (!name || !name[0] || strchr(name, '/') || strchr(name, '\\')) {
    printf("avatar: invalid — bad name '%s'\n", name ? name : "?");
    return false;
  }
  const char *bg = doc["bg"].as<const char *>();
  if (!bg || !bg[0]) {
    printf("avatar: invalid — empty bg\n");
    return false;
  }
  const JsonObject sm = doc["state_machine"].as<JsonObject>();
  if (!sm) {
    printf("avatar: invalid — state_machine object missing\n");
    return false;
  }
  const JsonObject statesObj = sm["states"].as<JsonObject>();
  if (!statesObj || statesObj.size() == 0) {
    printf("avatar: invalid — states object missing/empty\n");
    return false;
  }

  memset(&d, 0, sizeof(d));
  snprintf(d.name, sizeof(d.name), "%s", name);
  snprintf(d.bg, sizeof(d.bg), "%s", bg);

  // ---- states ----
  for (JsonPair p : statesObj) {
    if (d.nStates >= 24) {
      printf("avatar: invalid — more than 24 states\n");
      return false;
    }
    const char *sname = p.key().c_str();
    const JsonObject so = p.value().as<JsonObject>();
    if (!so) {
      printf("avatar: invalid — state '%s' is not an object\n", sname);
      return false;
    }
    const char *clip = so["clip"].as<const char *>();
    if (!clip || !clip[0]) {
      printf("avatar: invalid — state '%s' has no clip\n", sname);
      return false;
    }
    State &st = d.states[d.nStates];
    snprintf(st.name, sizeof(st.name), "%s", sname);
    snprintf(st.clip, sizeof(st.clip), "%s", clip);
    st.loop = so["loop"].as<bool>();
    st.auto_return = so["auto_return"].as<bool>();
    st.terminal = so["terminal"].as<bool>();
    // S125: on_finish is validated in a SECOND pass after ALL states are
    // parsed (the single-pass check ran against the partial table — a state
    // whose on_finish named a state defined LATER in the file, e.g.
    // intro.on_finish="idle", was falsely rejected -> built-in fallback).
    const char *of = so["on_finish"].as<const char *>();
    if (of && of[0])
      snprintf(st.on_finish, sizeof(st.on_finish), "%s", of);
    st.prio = so["priority"].is<int>() ? (int8_t)so["priority"].as<int>() : 0;
    if (st.loop && st.auto_return) {
      printf("avatar: invalid — state '%s' is both loop and auto_return\n", sname);
      return false;
    }
    d.nStates++;
  }

  // ---- on_finish references (second pass — the state table is complete) ----
  for (int s = 0; s < d.nStates; s++) {
    if (d.states[s].on_finish[0] && nameIdx(&d, d.states[s].on_finish) < 0) {
      printf("avatar: invalid — state '%s' on_finish '%s' is not a state\n",
             d.states[s].name, d.states[s].on_finish);
      return false;
    }
  }

  // ---- initial / default_state / terminal[] ----
  const char *inName = sm["initial"].as<const char *>();
  const int initial = nameIdx(&d, inName);
  if (initial < 0) {
    printf("avatar: invalid — initial '%s' is not a state\n", inName ? inName : "?");
    return false;
  }
  d.initial = (uint8_t)initial;
  d.idle = (uint8_t)nameIdx(&d, sm["default_state"].as<const char *>());
  if (nameIdx(&d, sm["default_state"].as<const char *>()) < 0) {
    printf("avatar: invalid — default_state missing or not a state\n");
    return false;
  }
  const JsonArray termArr = sm["terminal"].as<JsonArray>();
  if (termArr) {
    for (size_t t = 0; t < termArr.size(); t++) {
      const char *tn = termArr[t].as<const char *>();
      const int ti = nameIdx(&d, tn);
      if (ti < 0) {
        printf("avatar: invalid — terminal '%s' is not a state\n", tn ? tn : "?");
        return false;
      }
      d.states[ti].terminal = true;
    }
  }

  // ---- transitions ----
  const JsonArray transArr = sm["transitions"].as<JsonArray>();
  if (!transArr) {
    printf("avatar: invalid — transitions array missing\n");
    return false;
  }
  for (size_t t = 0; t < transArr.size(); t++) {
    if (d.nTrans >= 48) {
      printf("avatar: invalid — more than 48 transitions\n");
      return false;
    }
    const JsonObject to = transArr[t].as<JsonObject>();
    if (!to) {
      printf("avatar: invalid — transition %u is not an object\n", (unsigned)t);
      return false;
    }
    const char *fromS = to["from"].as<const char *>();
    const char *toS   = to["to"].as<const char *>();
    const char *evS   = to["event"].as<const char *>();
    if (!fromS || !toS || !evS || !evS[0]) {
      printf("avatar: invalid — transition %u missing from/to/event\n", (unsigned)t);
      return false;
    }
    int fi;
    if (strcmp(fromS, "*") == 0) {
      fi = kAnyFrom;  // valid wildcard — never a name-lookup failure
    } else {
      // S127: the S126 two-pass fix cleared the on_finish false fails, but
      // this fi<0 check still ran against the kAnyFrom=-1 sentinel (same
      // value as nameIdx's not-found) -> EVERY from:"*" transition was
      // rejected ("transition N from '*' is not a state") and both SD
      // avatars fell back to built-in. Only the name-lookup path may fail.
      fi = nameIdx(&d, fromS);
      if (fi < 0) {
        printf("avatar: invalid — transition %u from '%s' is not a state\n",
               (unsigned)t, fromS);
        return false;
      }
    }
    const int ti = nameIdx(&d, toS);
    if (ti < 0) {
      printf("avatar: invalid — transition %u to '%s' is not a state\n",
             (unsigned)t, toS);
      return false;
    }
    if (fi != kAnyFrom && d.states[fi].terminal) {
      printf("avatar: invalid — transition %u leaves terminal state '%s'\n",
             (unsigned)t, fromS);
      return false;
    }
    if (strncmp(evS, "play:", 5) == 0 && nameIdx(&d, evS + 5) < 0) {
      printf("avatar: invalid — transition %u play:'%s' is not a state\n",
             (unsigned)t, evS + 5);
      return false;
    }
    Trans &tr = d.trans[d.nTrans++];
    tr.from = (int8_t)fi;
    tr.to   = (uint8_t)ti;
    snprintf(tr.event, sizeof(tr.event), "%s", evS);
    const char *guard = to["guard"].as<const char *>();
    if (guard && guard[0]) snprintf(tr.guard, sizeof(tr.guard), "%s", guard);
  }

  // ---- script rows (rules 3, 5) ----
  const JsonArray rowsArr = doc["script"].as<JsonArray>();
  if (!rowsArr || rowsArr.size() == 0) {
    printf("avatar: invalid — script missing or empty\n");
    return false;
  }
  for (size_t r = 0; r < rowsArr.size(); r++) {
    if (d.nRows >= 32) {
      printf("avatar: invalid — more than 32 script rows\n");
      return false;
    }
    const JsonObject ro = rowsArr[r].as<JsonObject>();
    if (!ro) {
      printf("avatar: invalid — script row %u is not an object\n", (unsigned)r);
      return false;
    }
    const char *evS = ro["event"].as<const char *>();
    if (!evS || !evS[0]) {
      printf("avatar: invalid — script row %u has no event\n", (unsigned)r);
      return false;
    }
    if (!ro["dwell_ms"].is<int>() || ro["dwell_ms"].as<int>() < 1 ||
        ro["dwell_ms"].as<int>() > 300000) {
      printf("avatar: invalid — script row %u dwell_ms out of range\n", (unsigned)r);
      return false;
    }
    if (strncmp(evS, "play:", 5) == 0 && nameIdx(&d, evS + 5) < 0) {
      printf("avatar: invalid — script row %u play:'%s' is not a state\n",
             (unsigned)r, evS + 5);
      return false;
    }
    Row &row = d.rows[d.nRows];
    snprintf(row.event, sizeof(row.event), "%s", evS);
    row.dwell_ms = (uint32_t)ro["dwell_ms"].as<int>();
    const char *asS = ro["after_state"].as<const char *>();
    if (asS && asS[0]) {
      const int ai = nameIdx(&d, asS);
      if (ai < 0) {
        printf("avatar: invalid — script row %u after_state '%s' is not a state\n",
               (unsigned)r, asS);
        return false;
      }
      row.after = (int8_t)ai;
    } else {
      row.after = kNoState;
    }
    d.nRows++;
  }
  if (strcmp(d.rows[d.nRows - 1].event, "outro") != 0) {
    printf("avatar: invalid — script must end with 'outro'\n");
    return false;
  }

  // ---- tuning (defaults when absent/invalid) ----
  const JsonObject tun = doc["tuning"].as<JsonObject>();
  if (tun && tun["idle_min_ms"].is<int>() && tun["idle_min_ms"].as<int>() > 0)
    d.idle_min_ms = (uint32_t)tun["idle_min_ms"].as<int>();
  else
    d.idle_min_ms = 4000;
  if (tun && tun["look_hold_ms"].is<int>() && tun["look_hold_ms"].as<int>() > 0)
    d.look_hold_ms = (uint32_t)tun["look_hold_ms"].as<int>();
  else
    d.look_hold_ms = 5000;

  // ---- reachability (rule 4) ----
  if (!reachesIdle(&d)) return false;
  return true;
}

// Spec rule 6 — on-device clip coverage (LOG-ONLY; the show continues).
void checkClipCoverage(const AvatarDef *d) {
  for (int s = 0; s < d->nStates; s++) {
    char cp[96];
    snprintf(cp, sizeof(cp), "/avatars/%s/rfav/%s.rfav", d->name, d->states[s].clip);
    File cf = SD_MMC.open(cp);
    if (!cf) {
      printf("avatar: clip MISSING — %s (state '%s'; renders bg until fixed)\n",
             cp, d->states[s].name);
      continue;
    }
    // S127: Rfav3 carries two MAX_FRAMES(4096) uint32 tables = ~33 KB — on
    // loopTask's 8 KB stack the first SD-success boot died in a stack
    // canary trip (Guru 'Stack protection fault', SP ~25 KB under the
    // stack floor) the instant this frame was allocated. This path had
    // never run before (the validator always failed first). Function-
    // static, same pattern as s_buf/doc/s_sd above.
    static Rfav3 probe;
    if (!probe.open(cf)) {
      printf("avatar: clip BAD HEADER — %s (state '%s'): %s\n", cp,
             d->states[s].name, probe.err);
    } else if (probe.width != 720 || probe.height != 1280) {
      printf("avatar: clip is %ux%u, expected 720x1280 — %s (state '%s')\n",
             probe.width, probe.height, cp, d->states[s].name);
    }
    cf.close();
  }
}

}  // namespace

// ------------------------------ public -------------------------------------

const AvatarDef *active() { return s_active; }
const char *activeName()   { return s_active->name; }
bool fromSd()              { return s_from_sd; }
const char *configBg()     { return s_cfg_bg; }  // S132: "" = no override

// S132: load one model's /avatars/<name>/avatar.json. On success: set
// s_active (the shared s_sd home), print, check clip coverage, return true.
// All failures print their reason and return false (the caller falls on).
bool tryModel(const char *name, bool from_config) {
  char path[48];
  snprintf(path, sizeof(path), "/avatars/%s/avatar.json", name);
  File f = SD_MMC.open(path);
  if (!f) {
    printf("avatar: %s — missing\n", path);
    return false;
  }
  static uint8_t s_buf[kMaxJson + 1];  // function-static (NOT stack)
  const int n = f.read(s_buf, kMaxJson);
  f.close();
  if (n <= 0) {
    printf("avatar: %s — read failed\n", path);
    return false;
  }
  s_buf[n] = 0;
  static AvatarDef s_sd;  // the SD avatar's home (function-static)
  if (!parseAndValidate((const char *)s_buf, (size_t)n, s_sd)) return false;
  s_active = &s_sd;
  s_from_sd = true;
  s_from_config = from_config;
  printf("avatar: %s (sd%s) — %u states / %u trans / %u rows; initial=%s; "
         "bg=/avatars/%s\n",
         s_sd.name, from_config ? ", config" : "", (unsigned)s_sd.nStates,
         (unsigned)s_sd.nTrans, (unsigned)s_sd.nRows,
         s_sd.states[s_sd.initial].clip, s_sd.bg);
  checkClipCoverage(&s_sd);
  return true;
}

// S132: /avatars/config.json (<=512 B) — the data-driven control plane:
//   { "version": "1.0.0",
//     "active": "neko_voss",                            // which model to load
//     "bg": "reality_fabricator_portrait_bg90.jpg" }    // optional bg override
// Swap avatars (or override the background) by editing this ONE file on the
// SD + reboot — no renames, no recompile. Both values are used straight in
// a /avatars/<value> path, so reject path separators + hidden names. Fills
// s_cfg_active/s_cfg_bg (either may end up empty = "not set"); returns false
// (fields left empty) when the file is missing/unreadable/unparseable.
bool readConfig() {
  File f = SD_MMC.open("/avatars/config.json");
  if (!f) return false;  // no config = pre-S132 behavior, silently
  static uint8_t s_cfg[512];  // function-static (NOT stack, S127 rule)
  const int n = f.read(s_cfg, 511);
  f.close();
  if (n <= 0) {
    printf("avatar: config.json — read failed; ignoring\n");
    return false;
  }
  s_cfg[n] = 0;
  static StaticJsonDocument<256> doc;
  const DeserializationError err =
      ArduinoJson::deserializeJson(doc, (const char *)s_cfg, (size_t)n);
  if (err) {
    printf("avatar: config.json — JSON parse failed (%s); ignoring\n",
           err.c_str());
    return false;
  }
  const char *active = doc["active"].as<const char *>();
  if (active && active[0] && strlen(active) < sizeof(s_cfg_active) &&
      !strchr(active, '/') && !strchr(active, '\\') && active[0] != '.')
    snprintf(s_cfg_active, sizeof(s_cfg_active), "%s", active);
  const char *bg = doc["bg"].as<const char *>();
  if (bg && bg[0] && strlen(bg) < sizeof(s_cfg_bg) && !strchr(bg, '/') &&
      !strchr(bg, '\\') && bg[0] != '.')
    snprintf(s_cfg_bg, sizeof(s_cfg_bg), "%s", bg);
  return true;
}

void load() {
  // 1) S132: the data-driven pick — /avatars/config.json "active".
  readConfig();
  if (s_cfg_active[0]) {
    if (tryModel(s_cfg_active, true)) return;
    printf("avatar: config.json wants '%s' — invalid; trying candidates\n",
           s_cfg_active);
  }
  // 2) candidate list (pre-S132 behavior; the no-config default).
  for (size_t c = 0; c < kCandidateN; c++) {
    if (tryModel(kCandidates[c], false)) return;
  }
  // 3) built-in — the device never wedges.
  s_active = &kBuiltinElena;
  s_from_sd = false;
  s_from_config = false;
  printf("avatar: elena (built-in fallback) — no valid /avatars/<name>/avatar.json "
         "on SD\n");
}

}  // namespace av
