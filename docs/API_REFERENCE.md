# API Reference

Module-by-module reference for every public class, function, and top-level entry point in the project. Line numbers refer to the current `main` branch.

---

## 1. [`v2v_metrics.py`](../v2v_metrics.py) — Metrics Collection

### `class MetricsCollector`

Captures timestamped events and computes per-run performance metrics for **one** car. Each instance is bound to a single `car_id` and a single CSV file.

#### Constructor

```python
MetricsCollector(car_id: str, csv_filename: str = "v2v_results.csv")
```

| Parameter | Description |
|---|---|
| `car_id` | Identifier embedded in CSV rows (`"CAR_A"` or `"CAR_B"`) |
| `csv_filename` | Output CSV path; parent directories are auto-created |

The constructor initialises per-run state and writes the CSV header if the file does not yet exist.

#### Lifecycle methods

| Method | Purpose |
|---|---|
| `start_run(run_id: int)` | Resets all per-run state. Call once at the beginning of each test run. |
| `end_run()` | Computes derived metrics, prints the per-run summary, and appends a row to the CSV. |

#### Event loggers

Call these from `car_a.py` / `car_b.py` at the appropriate moments:

| Method | Triggered by |
|---|---|
| `log_message_sent()` | Every successful `socket.send` |
| `log_message_received()` | Every successfully-parsed inbound JSON message |
| `log_position_update()` | Every `POSITION` tick (used for Hz) |
| `log_latency(send_timestamp: float)` | Per received `send_ts` — appends one sample to `_latency_samples` |
| `log_distance(distance: float)` | Every distance sample — tracks `min_distance` and flips `collision` if `d < 1.0` |
| `log_warning_sent(distance, my_speed, other_speed=0.0)` | Car A, when the `WARNING` is emitted |
| `log_warning_received(distance, my_speed, send_timestamp=None)` | Car B, when the `WARNING` is received |
| `log_brake_applied(position: list, speed: float)` | Immediately after issuing a braking AI command |
| `log_full_stop(position: list, speed: float = 0.0)` | When the car reaches its post-manoeuvre stop |

#### Derived metric helpers (internal)

All prefixed with `_` — computed lazily during `end_run()`:

| Method | Formula |
|---|---|
| `_braking_reaction_time_ms()` | `(brake_time − warning_time) × 1000` |
| `_stopping_distance_m()` | `||brake_position − stop_position||` (3-D Euclidean) |
| `_time_to_stop_ms()` | `(stop_time − brake_time) × 1000` |
| `_message_delivery_rate()` | `received / max(sent, received, 1) × 100` |
| `_position_update_hz()` | `position_updates / elapsed_seconds` |
| `_run_duration_s()` | `end − start` (perf_counter) |
| `_avg_latency_ms()` | `mean(_latency_samples)` |
| `_time_to_collision_at_warning_s()` | `distance_warn / (my_speed + other_speed)` |
| `_closing_speed_at_warning()` | `my_speed + other_speed` |

#### Output methods

| Method | Output |
|---|---|
| `print_summary()` | Formatted per-run table to stdout |
| `save_csv()` | Appends one row to `csv_filename`; creates header if missing |

#### CSV schema

```
run_id, car_id, timestamp,
warning_trigger_distance_m, speed_at_warning_mps,
braking_reaction_time_ms, stopping_distance_m,
time_to_stop_ms, speed_at_stop_mps,
min_inter_vehicle_distance_m, collision_occurred,
v2v_message_latency_ms, messages_sent, messages_received,
message_delivery_rate_pct, position_update_hz,
run_duration_s, time_to_collision_at_warning_s,
closing_speed_at_warning_mps
```

---

## 2. [`v2v_gui.py`](../v2v_gui.py) — Dashboard

### `class V2VDashboard`

A single-window Tkinter dashboard showing inter-vehicle distance, per-car status and speed, and a scrolling V2V message log.

#### Constructor

```python
V2VDashboard(title: str = "Unified V2V Dashboard")
```

Creates a `640×700` non-resizable window on the calling thread. Because Tkinter is **not** thread-safe, the instantiating thread must also own all `set_*` / `log` / `tick` calls.

#### Public API

| Method | Purpose |
|---|---|
| `set_status(car: str, status: str)` | Update badge for `'A'` or `'B'`. Valid statuses: `IDLE`, `DRIVING`, `WARNING`, `SLOWING`, `STOPPED`, `SUCCESS`. Colour auto-selected. |
| `set_distance(dist_m: float)` | Update the large inter-vehicle distance readout. |
| `set_speed(car: str, speed_mps: float)` | Update per-car speed (displayed as km/h via `× 3.6`). |
| `log(message: str, tag: str = "info", car: str = None)` | Append a timestamped line to the message log. `tag ∈ {info, warn, stop, success}`. If `car` is `'A'` or `'B'`, the prefix is colour-coded. |
| `tick()` | Calls `root.update()`. Invoke every iteration of the main loop to keep the GUI responsive without a separate thread. |
| `close()` | Destroy the Tk root. Safe to call at program exit. |

#### Colour scheme (Catppuccin-inspired)

| Token | Hex |
|---|---|
| `bg` | `#1e1e2e` |
| `card` | `#2a2a3d` |
| `text` | `#cdd6f4` |
| `car_a` (blue) | `#89b4fa` |
| `car_b` (red) | `#f38ba8` |
| `warn` (yellow) | `#f9e2af` |
| `success` (green) | `#a6e3a1` |

---

## 3. [`car_a.py`](../car_a.py) — V2V Server / Scenario Driver

Not a class — a top-level script. Execution phases:

| Phase | Lines | What happens |
|---|---|---|
| Config | 17–27 | Ports, BeamNG path, distance thresholds, `MetricsCollector` |
| Connect | 31–42 | Attach to running BeamNG or launch fresh |
| Identify | 46–71 | Flash all vehicles pink, user picks Car A, recolour blue, set arcade shift |
| Socket | 74–84 | Bind TCP server, accept Car B |
| HELLO | 86–96 | Parse Car B's vehicle ID |
| Speed prompt | 102–106 | Prompt base speed, convert km/h → m/s |
| GUI | 108–113 | Create `V2VDashboard`, set initial statuses |
| Run loop | 116–340 | Outer: per-run init. Inner: per-tick telemetry + negotiation + pass detection |

Key loop variables:

| Variable | Meaning |
|---|---|
| `run_count` | Monotonically incremented run number |
| `warning_sent` | `True` after Car A emits the `WARNING` message |
| `decision_made` | `True` after the KE tie-breaker fires |
| `i_am_yielding` | Car A's role for the current run (derived from KE) |
| `resumed` | `True` after pass-completion is detected |
| `min_dist_recorded` | Running minimum of `d(t)` |
| `emergency_stop` | Latched once when `d < 3 m` |
| `pre_warn_speed_a` / `pre_warn_speed_b` | Speeds captured at the warning instant (used by the KE rule) |

### Keyboard input

Uses `msvcrt.kbhit()` + `msvcrt.getch()` for non-blocking ENTER detection between runs — this is the Windows-only dependency.

---

## 4. [`car_b.py`](../car_b.py) — V2V Client

Top-level script with **two threads**:

### 4.1 Main thread

| Phase | Lines | What happens |
|---|---|---|
| Config + globals | 17–34 | Ports, BeamNG path, shared state, `MetricsCollector` |
| Connect | 162–173 | Attach to running BeamNG |
| Identify | 178–201 | Flash pink, user picks Car B, recolour red |
| Socket | 206–218 | Connect to Car A, send `HELLO`, start listener thread |
| Main loop | 228–275 | Poll sensors, send `POSITION`, run pass-completion detection |

### 4.2 Listener thread: `v2v_listener(sock)`

A blocking `recv()` loop. For each message:

| `type` | Action |
|---|---|
| `START_RUN` | `metrics.start_run`; set AI chase + base speed; `car_b_status = "DRIVING"` |
| `POSITION` | Update `car_a_pos`; sample latency from `send_ts` |
| `WARNING` | Drop speed to 5.6 m/s; log brake; `car_b_status = "WARNING"` |
| `SLOW` | Apply `target_speed`; `car_b_status = "SLOWING"` |
| `YIELDING` | Switch to `random` @ 5.6 m/s; activate pass detection; `car_b_status = "DRIVING"` |
| `PROCEED` | Switch to `stop` mode; activate pass detection; `car_b_status = "STOPPED"` |

### 4.3 Shared globals

| Global | Written by | Read by |
|---|---|---|
| `car_a_pos` | listener (`POSITION`) | main (distance calc) |
| `warning_received` | listener | (reserved) |
| `pass_detection_active` | listener (`YIELDING` / `PROCEED`) | main |
| `min_dist_recorded` | main | main |
| `resumed` | main | main |
| `car_b_status` | listener + main | outbound `POSITION` messages |
| `run_id` | listener (`START_RUN`) | `metrics.log_*` |

---

## 5. [`results/generate_report.py`](../results/generate_report.py) — Aggregation

### `read_csv(filename: str) -> list[dict]`

Loads a CSV and coerces each cell into `float` / `bool` / `None` / `str` according to its value.

### `compute_stats(values: list) -> tuple[float, float, float, float, int]`

Returns `(mean, std, min, max, n)` after filtering out `None` and non-numeric entries. `std` uses sample variance (divisor `n−1`).

### `generate_report()`

Driver function:
1. Reads `v2v_results.csv` and `v2v_results_car_b.csv`.
2. Counts collisions (`collision_occurred is True`).
3. Computes **Collision Avoidance Rate** = `(R − C) / R × 100`.
4. Iterates `METRICS_CONFIG` — 13 metrics — computing stats across **both** cars' rows.
5. Writes:
   - Console formatted summary
   - `results/v2v_metrics_table.tex` — `booktabs` LaTeX table
   - `results/experiment_results.md` — Markdown table

### `METRICS_CONFIG`

An ordered list of `(csv_column, display_label, unit)` tuples controlling which metrics appear in the report and in what order. Add a new tuple to include an additional metric.

---

## 6. Configuration Constants

Centralised so you can grep for the thresholds:

| Constant | File | Default | Meaning |
|---|---|---|---|
| `V2V_PORT` | both | `5555` | TCP port |
| `V2V_HOST` | both | `127.0.0.1` | Socket host (change for cross-machine testing) |
| `BNG_HOME` | both | `r"X:\Steam\..."` | BeamNG install path |
| `WARN_DISTANCE` | `car_a.py` | `30.0` | m, warning threshold |
| `SLOW_DISTANCE` | `car_a.py` | `20.0` | m (reference; not directly branched on) |
| `STOP_DISTANCE` | `car_a.py` | `10.0` | m (reference; pass-complete uses `10.0`) |
| Negotiation distance | `car_a.py` inline | `35.0` | m, KE tie-breaker trigger |
| Emergency distance | both inline | `3.0` | m, emergency-brake threshold |
| Collision threshold | `v2v_metrics.py` | `1.0` | m, flags a collision |
| Safe passing speed | both inline | `5.6` | m/s (= 20 km/h) |
| Default base speed | `car_a.py` prompt | `45` | km/h |
| Loop interval | both | `0.05` s | Main-loop sleep |
| AI aggression | both | `0.3` | `ai_set_aggression` |

---

## 7. Adding a New Metric (Walk-through)

1. Add a field to `MetricsCollector.__init__` (e.g. `self._peak_accel = None`) and reset it in `start_run()`.
2. Add a logger method (e.g. `log_peak_accel(a: float)`).
3. Call the logger from `car_a.py` / `car_b.py` at the correct point.
4. Add the column name to `MetricsCollector.CSV_HEADER`.
5. Append the computed value to the row in `save_csv()`.
6. Add a line to `print_summary()` if you want it in the console output.
7. Add a tuple to `METRICS_CONFIG` in `generate_report.py` for aggregation.

---

## 8. Error-Handling Conventions

| Situation | Convention |
|---|---|
| Parsing an inbound JSON message | Wrap in `try / except json.JSONDecodeError: pass` — never crash on a bad frame |
| Tkinter after window destroyed | Wrap in `try / except tk.TclError: pass` — GUI calls become no-ops |
| `socket.send` on a broken pipe | `except (BrokenPipeError, ConnectionResetError, OSError)` — Car A logs it and breaks the loop |
| `socket.recv` timeout | `except socket.timeout: pass` — expected; loop continues |
| User hits Ctrl+C | `except KeyboardInterrupt` — close GUI, sockets, BeamNG, then `exit()` |
