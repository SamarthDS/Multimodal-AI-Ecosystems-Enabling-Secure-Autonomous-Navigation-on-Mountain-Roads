# Experimental Methodology

This document describes how to run the V2V Collision Avoidance experiment reproducibly, what the independent and controlled variables are, how each metric is measured, and how the resulting dataset should be analysed.

---

## 1. Objective

Measure the effectiveness of a kinetic-energy-based cooperative right-of-way protocol in preventing head-on collisions between two autonomous vehicles on a single-lane road, and characterise the protocol's timing and communication overhead under a controlled simulation environment.

---

## 2. Experimental Setup

### 2.1 Simulation environment

| Parameter | Value |
|---|---|
| Simulator | BeamNG.drive (consumer edition), ≥ v0.31 |
| Physics | Soft-body, default timestep |
| Map | Any flat straight-road segment (e.g. `gridmap_v2`, `west_coast_usa` straightaways) |
| Weather | Default (clear, dry) |
| Time-of-day | Default |
| Road | Single carriageway, straight for ≥ 200 m |
| Traffic | None (only the two test vehicles) |

### 2.2 Vehicles

- **Both vehicles identical where possible** (same model, same mass) so the kinetic-energy comparison reduces to a speed comparison.
- Car A: recoloured **blue**, shift mode `arcade`.
- Car B: recoloured **red**, shift mode `arcade`.

### 2.3 Initial conditions

- Vehicles spawned at opposite ends of the same road.
- Facing each other (heading vectors ≈ 180° apart).
- Initial speed: 0 m/s (AI accelerates them up to the base speed).
- Both in AI `chase` mode targeting each other (single-lane driving).

### 2.4 Controlled variables

| Variable | Value | How enforced |
|---|---|---|
| Base speed | User-entered per run (default 45 km/h) | `ai_set_speed(base_speed, mode='limit')` |
| Safe passing speed | 20 km/h (5.6 m/s) | Hard-coded in both scripts |
| AI aggression | 0.3 | `ai_set_aggression(0.3)` |
| Lane | Left (Indian norms) | `ai_drive_in_lane(True)` + Lua `ai.driveInLane("on")` |
| Warning distance | 30 m | `WARN_DISTANCE` in `car_a.py` |
| Negotiation distance | 35 m | Inline in `car_a.py` |
| Emergency distance | 3 m | Inline in both |
| Loop interval | 50 ms | `time.sleep(0.05)` |

### 2.5 Independent variable

The only variable the operator is expected to sweep is **base speed**. Suggested protocol:

| Base speed (km/h) | Purpose |
|---|---|
| 30 | Low-speed baseline — easy case |
| 45 | Nominal urban test |
| 60 | Stress test — higher closing speed |
| 75 | Boundary test — near-limit for the 30 m warning zone |

Execute at least **N ≥ 5 runs per speed** for meaningful statistics; 10 runs per speed is recommended.

---

## 3. Run Procedure

1. Start BeamNG via `python car_a.py`. Spawn two vehicles on the chosen road.
2. In a second terminal, `python car_b.py`. Pick the non-blue car.
3. In Car A's terminal, type the desired base speed (e.g. `45`), press ENTER.
4. The vehicles accelerate toward each other. The GUI opens.
5. Observe — negotiation happens at ~35 m and one car yields.
6. When the pass is complete, both terminals print a `─── Run #N complete ───` banner and append a row to the CSVs.
7. Press ENTER in Car A's terminal to start the next run (vehicles remain spawned; they'll be re-targeted by AI).
8. After all runs, Ctrl+C both terminals and run `python results/generate_report.py`.

---

## 4. Metric Definitions & Measurement Points

### 4.1 Safety metrics

| Metric | Definition | Measurement point |
|---|---|---|
| **Warning Trigger Distance** (m) | `d(t_w)` when first `d < 30 m` | At the emission of the `WARNING` message |
| **Min Inter-Vehicle Distance** (m) | `d_min = min_t d(t)` over the run | Continuously updated in `log_distance()` |
| **Collision Occurred** (bool) | `d_min < 1.0 m` | Threshold flip in `log_distance()` |
| **Collision Avoidance Rate** (%) | `(R − C) / R × 100`, C = runs with collision | Post-hoc in `generate_report.py` |

### 4.2 Timing metrics

| Metric | Definition | Clock |
|---|---|---|
| **Braking Reaction Time** (ms) | `t_brake − t_warn` | `time.perf_counter()` |
| **Time to Full Stop** (ms) | `t_stop − t_brake` | `time.perf_counter()` |
| **Stopping Distance** (m) | `‖p_brake − p_stop‖` (3-D) | BeamNG position |
| **Speed at Warning** (m/s) | `‖v(t_w)‖` | BeamNG velocity |
| **Speed at Stop** (m/s) | `‖v(t_stop)‖` (should be ≈ 0) | BeamNG velocity |
| **Closing Speed at Warning** (m/s) | `s_A(t_w) + s_B(t_w)` | From both cars' speeds |
| **Time-to-Collision at Warning** (s) | `d(t_w) / (s_A + s_B)` | Derived |
| **Run Duration** (s) | `t_end − t_start` | `time.perf_counter()` |

### 4.3 Communication metrics

| Metric | Definition | Measurement |
|---|---|---|
| **V2V Message Latency** (ms) | `mean((t_recv − send_ts) × 1000)` | Sampled on every received message with `send_ts` |
| **Messages Sent** (int) | Counter of successful `socket.send` calls | `log_message_sent()` on send |
| **Messages Received** (int) | Counter of successfully parsed JSON messages | `log_message_received()` on parse |
| **Message Delivery Rate** (%) | `received / max(sent, received, 1) × 100` | Post-hoc in `end_run` |
| **Position Update Frequency** (Hz) | `position_updates / elapsed_seconds` | Post-hoc in `end_run` |

---

## 5. Data Output

### 5.1 Per-run CSV rows

Each run produces one row in **each** of:

- `results/v2v_results.csv` (Car A's view)
- `results/v2v_results_car_b.csv` (Car B's view)

The CSVs have the same schema — the two rows for a single run differ in `car_id` and, where applicable, in locally-measured values (e.g. Car B's `speed_at_warning_mps` is its own speed, not Car A's).

### 5.2 Aggregate report (`generate_report.py`)

Produces:

- `results/experiment_results.md` — Markdown table of mean ± std / min / max / N for each metric across both cars.
- `results/v2v_metrics_table.tex` — `booktabs`-styled LaTeX table ready to be embedded via `\input{v2v_metrics_table.tex}`.

---

## 6. Statistical Analysis

For each metric across `N` runs:

| Statistic | Formula |
|---|---|
| Mean | `μ = Σxᵢ / N` |
| Sample std | `σ = √(Σ(xᵢ − μ)² / (N − 1))` |
| Min / Max | Order statistics |

### Reporting recommendation

Always report `μ ± σ` with the sample count (`N`) so the reader can reconstruct standard-error bars (`SE = σ / √N`) if desired.

### Grouping by base speed

For speed-sweep analysis, split rows by the `base_speed` you entered (not stored directly in the CSV today — you must tag rows externally, e.g. by running each speed as a separate batch and saving the CSV between batches).

---

## 7. Validity Considerations

### 7.1 Internal validity

- Deterministic **VID tie-breaker** eliminates nondeterministic deadlock, so repeated runs at identical speeds do not randomly swap yielder/passer.
- BeamNG's AI aggression is fixed; the only true stochasticity is in the physics integrator's micro-jitter.
- `time.perf_counter()` is monotonic and high-resolution, so timing measurements are not skewed by NTP adjustments or wall-clock jumps.

### 7.2 External validity (threats)

- **Loopback latency ≠ real V2V latency.** TCP over `localhost` is typically < 1 ms with 100% delivery. Real DSRC / C-V2X on moving vehicles sees 5–50 ms latency and 0.1–5 % packet loss under load. Treat the measured MDR and latency as **lower bounds**.
- **Ground-truth positioning.** Positions come from simulator state; real GPS/GNSS adds 1–3 m of horizontal noise.
- **Single-scenario.** Head-on oncoming only — not representative of intersections, lane-change conflicts, or multi-lane merging.
- **Identical vehicle masses.** The KE tie-breaker implicitly assumes `m_A ≈ m_B`. Mixed-mass fleets would need an explicit mass term.

### 7.3 Mitigations in this design

- Fixed thresholds and AI parameters are documented explicitly (§2.4) so reviewers can reproduce.
- CSV + Markdown + LaTeX outputs are emitted without manual copy-paste, eliminating transcription error.
- The report generator computes CAR across **both** Car A and Car B rows, so any one-sided logging failure is detectable.

---

## 8. Expected Results (Reference Targets)

Values we observed on our development machine (i7-class CPU, 16 GB RAM, BeamNG default graphics) as a sanity-check range. Yours will differ by hardware.

| Metric | Typical range | Notes |
|---|---|---|
| V2V Message Latency | 0.2 – 2.0 ms | Loopback-bound |
| Message Delivery Rate | 95 – 100 % | Occasional frame-split drops if the `'}{'` heuristic fails |
| Position Update Frequency | 15 – 20 Hz | Capped by 50 ms loop + BeamNG sensor poll overhead |
| Braking Reaction Time | 50 – 150 ms | Dominated by Python loop + AI-mode command propagation |
| Min Inter-Vehicle Distance | 3 – 8 m | Depends on base speed |
| Collision Avoidance Rate | 100 % | At base speeds ≤ 60 km/h on a straight road |

---

## 9. Troubleshooting the Data

| Observation | Likely cause | Fix |
|---|---|---|
| Many `None` values in CSV | Run ended before `log_full_stop` fired | Let the pass complete; avoid Ctrl+C mid-run |
| `v2v_message_latency_ms` is negative | Clock skew between processes (shouldn't happen on one machine) | Rerun; confirm both scripts share the OS clock |
| `position_update_hz` << 20 | BeamNG is bottlenecked (too many vehicles, heavy graphics) | Reduce graphics; ensure only the two test vehicles are spawned |
| `collision_occurred = True` frequently | Base speed too high for 30 m warning zone | Lower base speed or increase `WARN_DISTANCE` |
| Pass never completes | Vehicles drift out of lane | Check `ai_drive_in_lane(True)` is honoured on your map |
