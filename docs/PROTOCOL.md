# V2V Message Protocol

The V2V link carries all telemetry and negotiation commands between Car A (server) and Car B (client). This document specifies every message type, its fields, when it is emitted, how it is consumed, and the end-to-end sequences that combine them.

---

## 1. Transport

| Property | Value |
|---|---|
| Protocol | TCP |
| Address | `127.0.0.1:5555` (configurable via `V2V_HOST` / `V2V_PORT` in both scripts) |
| Encoding | UTF-8 bytes of a JSON object |
| Connection lifetime | One persistent connection per BeamNG session (no reconnect logic) |
| Framing | **None** — messages are concatenated without delimiters; the receiver uses a `'}{'` split heuristic |
| Direction | Full-duplex; either side may send at any time after handshake |

> ⚠ **Framing note.** Because there is no length prefix or newline delimiter, two messages that happen to arrive in the same `recv()` call appear as `{...}{...}`. The receiver calls `data.split('}{')` and re-adds the missing braces. This works reliably because no field value contains a literal `}{` substring and the payloads are small (< 1 KB).

---

## 2. Message Envelope

Every message is a flat JSON object with a `type` discriminator:

```json
{
  "type": "<MESSAGE_TYPE>",
  ...type-specific fields...
}
```

Messages that are measured for latency also carry:

```json
{ "send_ts": 1712345678.912 }
```

`send_ts` is `time.time()` (wall clock, seconds). The receiver computes one-way latency as `(time.time() − send_ts) × 1000` ms.

---

## 3. Message Types

### 3.1 `HELLO` — `B → A` (once, at connection)

```json
{
  "type": "HELLO",
  "my_vid": "car_b_vehicle_id"
}
```

| Field | Type | Meaning |
|---|---|---|
| `my_vid` | string | BeamNG vehicle ID of Car B |

**Semantics.** First message Car B sends after `connect()`. Car A reads it synchronously with a 5-second timeout and stores it in `car_b_vid` for later use (VID tie-breaker).

---

### 3.2 `START_RUN` — `A → B` (once per run)

```json
{
  "type": "START_RUN",
  "run": 3,
  "target_vid": "car_a_vehicle_id",
  "base_speed": 12.5
}
```

| Field | Type | Meaning |
|---|---|---|
| `run` | int | Monotonic run counter (Car A's `run_count`) |
| `target_vid` | string | Car A's vehicle ID — Car B sets its AI `chase` target to this |
| `base_speed` | float | Speed limit in m/s (e.g. 12.5 m/s = 45 km/h) |

**Semantics.** Triggers Car B to:
1. Reset its per-run metrics (`metrics.start_run(run)`).
2. Set AI: `chase` mode, drive-in-lane on, speed-limit = `base_speed`, aggression 0.3.

---

### 3.3 `POSITION` — `A ↔ B` (continuous, ~20 Hz)

```json
{
  "type": "POSITION",
  "pos":    [123.4, 567.8, 10.0],
  "speed":  [12.5, 0.0, 0.0],
  "status": "DRIVING",
  "send_ts": 1712345678.912
}
```

| Field | Type | Meaning |
|---|---|---|
| `pos` | `[x, y, z]` floats | World position in metres |
| `speed` | `[vx, vy, vz]` floats | Velocity vector in m/s |
| `status` | string *(B → A only)* | Current car status for GUI display |
| `send_ts` | float | `time.time()` when the message was built |

**Semantics.** Primary telemetry stream. The receiver uses `pos` to compute `d = ||p_A − p_B||`, `speed` for kinetic-energy comparison, and `send_ts` to sample latency.

Car A does not emit `status` in its own `POSITION` messages (it owns the GUI and knows its own status). Car B does emit `status` so that Car A's GUI can reflect Car B's state.

---

### 3.4 `WARNING` — `A → B` (once per run, when `d < 30 m`)

```json
{
  "type": "WARNING",
  "distance": 29.87,
  "message": "Oncoming vehicle ahead!",
  "send_ts": 1712345680.123
}
```

| Field | Type | Meaning |
|---|---|---|
| `distance` | float | Inter-vehicle distance (m) at warning instant |
| `message` | string | Human-readable note (displayed in the GUI log) |
| `send_ts` | float | See above |

**Semantics.** Notifies Car B that a collision is imminent. Both cars drop to the safe passing speed of 5.6 m/s (≈ 20 km/h). On the Car B side, this also calls `metrics.log_warning_received(...)` and `metrics.log_brake_applied(...)`.

---

### 3.5 `YIELDING` — `A → B` (only when A yields)

```json
{
  "type": "YIELDING",
  "distance": 33.2,
  "send_ts": 1712345680.445
}
```

**Semantics.** "I am yielding; you have right-of-way." Car B switches from `chase` to `random` mode at 5.6 m/s, activates pass-completion detection on its main loop, and proceeds past A.

---

### 3.6 `PROCEED` — `A → B` (only when A passes)

```json
{
  "type": "PROCEED",
  "distance": 33.2,
  "send_ts": 1712345680.445
}
```

**Semantics.** "I am proceeding; you must yield." Car B switches to AI `stop` mode, activates pass-completion detection, and waits for A to clear.

> **Invariant.** Exactly one of `YIELDING` or `PROCEED` is sent per run, per Car A's KE-based decision. Car B only ever receives one of the two.

---

### 3.7 `SLOW` — `A → B` (defined but currently unused)

```json
{
  "type": "SLOW",
  "target_speed": 5.6,
  "distance": 18.4
}
```

| Field | Type | Meaning |
|---|---|---|
| `target_speed` | float | AI speed limit to apply (m/s) |
| `distance` | float | Inter-vehicle distance at emission (m) |

**Semantics.** Receive-only in Car B's listener. Reserved for a finer-grained gradual slowdown scheme that was superseded by the single-step `WARNING → 5.6 m/s` behaviour in the current implementation. Retained in the handler for backward compatibility and as an extension hook.

---

## 4. Sequence Diagrams

### 4.1 Normal pass (Car A yields)

```
 Car A                          Car B                       Sim
 -----                          -----                       ---
 [bind, listen]
                                [connect]
                         <──── HELLO(my_vid) ───
 [store B's vid]
 [prompt base speed]
 [show GUI]

 ─── START_RUN(run, target_vid, base_speed) ──>
                                [start_run, chase A]

 ═══════════════ per-tick loop (~20 Hz) ════════════════
 ── POSITION(pos, speed, ts) ──>
                         <── POSITION(pos, speed, status, ts) ──
 d = ||pA − pB||
 ═══════════════════════════════════════════════════════

 d < 30:
   log_warning_sent(d, sA, sB)
 ─── WARNING(d, ts) ──────────>
                                log_warning_received(d, sB, ts)
                                ai_set_speed(5.6)
                                log_brake_applied()

 d < 35:
   sA < sB  →  A yields
 ─── YIELDING(d, ts) ─────────>
                                ai_set_mode('random'), 5.6 m/s
                                pass_detection_active = True
 A:  ai_set_mode('stop')

 ═══════════════ distance rises (B passes) ═════════════
 d > d_min + 1 AND d > 10:
   Car A: "Pass complete"  →  ai_set_mode('random') @ base_speed
                                Same happens on Car B's main loop
                                end_run() on both → CSV append

```

### 4.2 Normal pass (Car B yields)

Only the negotiation step changes:

```
 d < 35:
   sB < sA  →  A proceeds
 ─── PROCEED(d, ts) ──────────>
                                ai_set_mode('stop')
                                pass_detection_active = True
 A:  ai_set_mode('random') @ 5.6 m/s
```

### 4.3 Emergency fallback

```
 (negotiation fails / very high closing speed)
 d < 3.0 on Car A side:
   car_a.control(throttle=0, brake=1.0, parkingbrake=1.0)
   gui.log("EMERGENCY ABORT - TOO CLOSE!")
 (Car B relies on its earlier WARNING-triggered braking)
```

---

## 5. State Transitions Driven by Messages

| Received message | Car B action | Car B status |
|---|---|---|
| `START_RUN` | Reset metrics; AI chase + base_speed | `DRIVING` |
| `POSITION` | Update `car_a_pos`; sample latency | (unchanged) |
| `WARNING` | AI speed limit → 5.6 m/s; log brake | `WARNING` |
| `SLOW` | AI speed limit → `target_speed` | `SLOWING` |
| `YIELDING` | AI random mode @ 5.6 m/s; activate pass detection | `DRIVING` |
| `PROCEED` | AI stop mode; activate pass detection | `STOPPED` |

Car A's state is driven by its own computations, not by incoming messages; it treats Car B's `POSITION` stream as pure input.

---

## 6. Error Handling

| Condition | Behaviour |
|---|---|
| `json.JSONDecodeError` on a message | Silently skipped (`except json.JSONDecodeError: pass`) |
| `socket.timeout` (50 ms) | Expected — loop continues |
| `BrokenPipeError` / `ConnectionResetError` | Car A logs "Connection to Car B lost!" and breaks the run loop |
| Car A never receives `HELLO` within 5 s | `car_b_vid` stays `None`; VID tie-breaker falls back to comparing `None` < any string (which is False in Py3 and would raise) — in practice ties are vanishingly rare |
| Car B can't connect | Prints `[!] ERROR: {e}` and exits immediately |

---

## 7. Extending the Protocol

To add a new message type:

1. Pick a unique `type` string (ALL_CAPS_SNAKE_CASE).
2. Add a branch to Car B's `v2v_listener` and/or Car A's in-loop parser.
3. If latency is relevant, embed `send_ts`.
4. Document the schema in this file.
5. If the message should be measured, add counters to `MetricsCollector`.

**Pitfall.** Because the framing is `'}{'`-based, adding a nested JSON object inside a message will break parsing. Keep messages flat (no nested `{}`).
