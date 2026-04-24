# Architecture

This document describes the structural design of the V2V Collision Avoidance system — its components, the data and control flow between them, the concurrency model, and the yield/pass state machine.

---

## 1. System Components

The system is composed of **five logical layers**, each with a clear responsibility:

```
┌──────────────────────────────────────────────────────────────────────┐
│ Layer 5: Presentation  │ v2v_gui.py  (Tkinter dashboard)             │
├──────────────────────────────────────────────────────────────────────┤
│ Layer 4: Instrumentation│ v2v_metrics.py  (MetricsCollector)         │
├──────────────────────────────────────────────────────────────────────┤
│ Layer 3: Application   │ car_a.py · car_b.py                         │
│                         (negotiation logic, main loop, lifecycle)    │
├──────────────────────────────────────────────────────────────────────┤
│ Layer 2: Transport     │ Python socket (TCP, 127.0.0.1:5555)         │
│                         JSON message envelopes                       │
├──────────────────────────────────────────────────────────────────────┤
│ Layer 1: Simulation    │ beamngpy  →  BeamNG.drive (Torque3D)        │
└──────────────────────────────────────────────────────────────────────┘
```

### 1.1 Simulation layer (BeamNG.drive + `beamngpy`)

- Provides the **world** (terrain, roads, physics) and the **AI controllers** (`chase`, `stop`, `random` modes, lane enforcement, aggression).
- Queried from Python via `beamngpy`'s RPC bridge. Each `Vehicle` handle exposes `.state` (position + velocity), `.control(...)`, `.ai_set_mode(...)`, `.queue_lua_command(...)`, etc.
- The engine runs at its default fixed timestep; the Python loop polls at ~20 Hz.

### 1.2 Transport layer (sockets + JSON)

- A single persistent **TCP connection** between Car A (server) and Car B (client) on `127.0.0.1:5555`.
- Messages are JSON objects UTF-8 encoded to bytes. Each message has a `type` discriminator and optional `send_ts` for latency measurement.
- The socket carries all telemetry, warnings, and negotiation commands. See [PROTOCOL.md](PROTOCOL.md) for full schemas.

### 1.3 Application layer ([car_a.py](../car_a.py), [car_b.py](../car_b.py))

- **Car A** is the socket **server** and the scenario orchestrator — it prompts the user for base speed, drives its vehicle, tracks both cars' positions, makes the yield/pass decision, and broadcasts commands.
- **Car B** is the socket **client** — it echoes its own position, listens for `WARNING` / `YIELDING` / `PROCEED`, and obeys the resulting commands.
- Each holds one `MetricsCollector` instance scoped to its own CSV file.

### 1.4 Instrumentation layer ([v2v_metrics.py](../v2v_metrics.py))

- `MetricsCollector` is a stateful event recorder: it captures timestamped events (warning sent/received, brake applied, full stop), accumulates counters (messages sent/received, position updates), and collects latency samples.
- On `end_run()` it computes derived metrics (reaction time, stopping distance, TTC, MDR, Hz) and appends a single row to a CSV.

### 1.5 Presentation layer ([v2v_gui.py](../v2v_gui.py))

- `V2VDashboard` is a lightweight Tkinter window owned by Car A. It shows the current inter-vehicle distance, both cars' status and speed, and a scrolling, colour-coded V2V message log.
- Updated synchronously from Car A's main loop via `gui.tick()` — Tkinter is **not** thread-safe, so all GUI updates happen on the main thread.

---

## 2. Data & Control Flow

### 2.1 Startup sequence

```
Car A                               Car B                         BeamNG
-----                               -----                         ------
1. bng.open(launch=False or True)  -------------------------->  attach or spawn
2. scan vehicles, flash pink, pick Car A  ── set blue
3. socket.bind() + listen
                                    4. bng.open(launch=False) -> attach
                                    5. scan + pick Car B -> set red
                                    6. client.connect(A)
7. accept conn
                                    8. send {HELLO, my_vid}
9. recv HELLO, store car_b_vid
10. prompt for base speed
11. init GUI                        (Car B stays headless)
12. start run loop
```

### 2.2 Per-run loop (Car A)

At each tick (~50 ms):

1. Poll `car_a.sensors` → read position + velocity.
2. Send `POSITION` message to Car B (embeds `send_ts`).
3. Receive queued Car B messages; decode JSON.
4. Compute `d = ||p_A − p_B||` (2-D).
5. Evaluate state transitions (see §4).
6. Update GUI (distance, speeds, status, log).
7. Check emergency fallback (`d < 3 m`).
8. Sleep 50 ms.

### 2.3 Per-run loop (Car B)

Split across **two threads** (see §3):

- **Main thread**: polls sensors, sends `POSITION`, runs pass-completion detection, calls GUI-free metric logging.
- **Listener thread**: blocks on `socket.recv`, parses messages, updates shared state (`warning_received`, `car_a_pos`, `pass_detection_active`, etc.), issues AI commands.

---

## 3. Concurrency Model

| Process | Threads | Purpose |
|---|---|---|
| `car_a.py` | 1 (main) | Drives everything synchronously; GUI on main thread |
| `car_b.py` | 2 (main + `v2v_listener` daemon) | Main sends telemetry + runs pass detection; listener receives Car A's messages |

Why the split is asymmetric:

- **Car A owns the GUI.** Tkinter must be updated only from the thread that created it, so Car A keeps everything on the main thread and interleaves `socket.recv` with a short timeout (`conn.settimeout(0.05)`).
- **Car B has no GUI.** It can afford a blocking listener thread because nothing else on its side has Tkinter's single-thread constraint.

Shared state in `car_b.py` uses **module globals** (e.g. `warning_received`, `pass_detection_active`). This works because:
- Writes are dominated by simple scalar assignments or boolean flips (atomic under CPython's GIL).
- The listener thread never touches Tkinter.
- Pass-completion detection — which must update the `ai_set_*` calls — runs on the main thread to keep `beamngpy` calls serialised.

---

## 4. Yield / Pass State Machine

Each car traverses the same state graph. Transitions are driven by inter-vehicle distance `d` and received V2V messages.

```
          ┌──────────┐
          │   IDLE   │  (before START_RUN)
          └────┬─────┘
               │  START_RUN received / issued
               ▼
          ┌──────────┐
          │ DRIVING  │  AI mode: chase, speed: BASE_SPEED
          └────┬─────┘
               │  d < 30 m  →  send/recv WARNING
               ▼
          ┌──────────┐
          │ WARNING  │  AI speed limit → 5.6 m/s (20 km/h)
          └────┬─────┘
               │  d < 35 m  →  KE comparison
         ┌─────┴─────┐
         │           │
   slower│           │faster (or VID-tied winner)
         ▼           ▼
    ┌────────┐  ┌────────┐
    │STOPPED │  │DRIVING │  AI mode: stop   ·  AI mode: random @ 5.6 m/s
    │(YIELD) │  │(PASS)  │  brakes engaged  ·  continues past yielder
    └───┬────┘  └───┬────┘
        │           │
        │  pass-complete: d > d_min + 1 AND d > 10 m
        ▼           ▼
          ┌──────────┐
          │ SUCCESS  │  AI mode: random @ BASE_SPEED
          └──────────┘
```

### Emergency override

From **any** state, if `d < 3.0 m` the car applies `throttle=0, brake=1.0, parkingbrake=1.0` and enters an emergency stop. This is a fallback for when the cooperative logic fails to converge in time (e.g. due to very high closing speeds or a communication stall).

### Decision rule formally

At the negotiation instant `t_neg` (first tick where `d < 35 m` after the warning):

```
if speed_A(t_w) < speed_B(t_w):          A yields
elif speed_B(t_w) < speed_A(t_w):        B yields
else (equal):                            min(str(vid_A), str(vid_B)) yields
```

The VID comparison is a lexicographic string compare — it is deterministic and independent of runtime state, so both sides reach the same decision without needing a round-trip handshake.

---

## 5. Coordinate System & Units

- **Positions** (`p = [x, y, z]`): BeamNG world coordinates in metres.
- **Velocities** (`v = [vx, vy, vz]`): m/s in world frame.
- **Scalar speed**: `s = ||v||` (Euclidean norm in 3-D; z-component usually small on flat roads).
- **Distance**: 2-D planar (`sqrt((x_A−x_B)² + (y_A−y_B)²)`), ignoring z — justified because the experimental roads are approximately flat. Full 3-D is used for **stopping distance** in `MetricsCollector._stopping_distance_m()`.
- **User-facing speed**: km/h in prompts and GUI. Internal maths uses m/s. Conversion: `km/h × 3.6 → m/s` is actually `m/s = km/h / 3.6`; we use both directions (`BASE_SPEED_MPS = BASE_SPEED_KMH / 3.6`, GUI shows `speed_mps × 3.6`).

---

## 6. Timing Discipline

All timing-critical measurements use `time.perf_counter()` (monotonic, nanosecond resolution). Wall-clock `time.time()` is used only for cross-process latency, where the receiver compares `time.time()` to a `send_ts` the sender also stamped with `time.time()`.

Because both processes share the OS clock, `time.time()` latency is accurate to within clock-skew noise (< 1 ms on the same machine).

| Event | Clock | Reason |
|---|---|---|
| Warning timestamp | `perf_counter()` | Local only; high resolution needed |
| Brake timestamp | `perf_counter()` | Local only |
| Full-stop timestamp | `perf_counter()` | Local only |
| `send_ts` in messages | `time.time()` | Must be comparable across processes |

---

## 7. File Responsibilities Cheat-Sheet

| File | Lines | Role |
|---|---|---|
| [car_a.py](../car_a.py) | ~340 | Server, scenario driver, GUI owner, yield/pass decision maker |
| [car_b.py](../car_b.py) | ~280 | Client, listener thread, obedient actor, pass completion monitor |
| [v2v_gui.py](../v2v_gui.py) | ~175 | Tkinter dashboard — purely presentational |
| [v2v_metrics.py](../v2v_metrics.py) | ~325 | Event collection + CSV output — no I/O with network or simulator |
| [results/generate_report.py](../results/generate_report.py) | ~190 | Post-hoc CSV → Markdown + LaTeX aggregator |

---

## 8. Design Invariants

1. **Exactly one yielder per run.** The KE + VID tie-breaker guarantees this, preventing the deadlock where both cars stop.
2. **The GUI is touched only from Car A's main thread.** All Car B-side updates are relayed through `POSITION` messages and rendered on Car A.
3. **Every row in the CSV corresponds to exactly one run.** `end_run()` is called once per run per car.
4. **Messages are self-contained.** Each JSON message includes everything the receiver needs — no implicit state dependencies on prior messages beyond the `HELLO` handshake.
5. **No shared files between Car A and Car B at runtime.** The only coupling is the socket. This lets the two processes run on different machines with only the `V2V_HOST` constant changed.
