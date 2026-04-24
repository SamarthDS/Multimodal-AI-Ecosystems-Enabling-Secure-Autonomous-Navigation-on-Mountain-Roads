# BeamNG V2V Collision Avoidance

> A Python-based Vehicle-to-Vehicle (V2V) communication system that demonstrates **cooperative collision avoidance** and **kinetic-energy-based right-of-way negotiation** inside the [BeamNG.drive](https://www.beamng.com/) soft-body physics simulator.

![Platform](https://img.shields.io/badge/platform-Windows%2010%2F11-blue)
![Python](https://img.shields.io/badge/python-3.8%2B-green)
![Simulator](https://img.shields.io/badge/BeamNG.drive-1.28%2B-orange)
![Status](https://img.shields.io/badge/status-research--prototype-yellow)

Two AI-controlled vehicles approach each other head-on on a single-lane road. They exchange telemetry over a TCP socket, compute a time-to-collision, and deterministically decide which car yields and which passes — without any human input. A real-time dashboard visualises the handshake while a metrics logger captures reaction time, latency, and clearance for research-grade analysis.

---

## Table of Contents

1. [Key Features](#key-features)
2. [System Architecture](#system-architecture)
3. [Quick Start](#quick-start)
4. [Project Structure](#project-structure)
5. [How It Works](#how-it-works)
6. [Documentation](#documentation)
7. [Results](#results)
8. [Known Limitations](#known-limitations)
9. [Future Work](#future-work)
10. [Citation](#citation)
11. [Acknowledgments](#acknowledgments)

---

## Key Features

- **TCP Socket V2V Link** — Persistent `127.0.0.1:5555` channel with JSON-encoded telemetry, warnings, and yield/pass commands.
- **Kinetic-Energy Right-of-Way Negotiation** — The slower vehicle (lower kinetic energy → shorter stopping distance) yields; a deterministic vehicle-ID tie-breaker eliminates deadlocks.
- **Three-Zone Distance Logic** — Warning (30 m) → Slow (20 m) → Stop (10 m), with a 3 m emergency-brake fallback.
- **Real-Time Tkinter Dashboard** — Live distance, per-vehicle status/speed, and timestamped V2V message log.
- **Automated Metrics Pipeline** — Per-run CSVs capture 18+ metrics (TTC, reaction time, stopping distance, latency, MDR). A report generator emits LaTeX-ready tables and Markdown summaries.
- **Indian Left-Lane Driving Norms** — AI pathing explicitly enforces left-lane adherence for culturally-accurate scenarios.
- **Multi-Run Support** — Execute consecutive runs from a single BeamNG session; metrics auto-aggregate across runs.

---

## System Architecture

```
                        ┌────────────────────────────────────┐
                        │        BeamNG.drive Engine         │
                        │  (Soft-body physics + AI pathing)  │
                        └───────────▲──────────────▲─────────┘
                                    │ beamngpy     │ beamngpy
                                    │ (RPC)        │ (RPC)
              ┌─────────────────────┴─┐          ┌─┴──────────────────────┐
              │        car_a.py       │          │        car_b.py        │
              │    (V2V SERVER)       │          │     (V2V CLIENT)       │
              │  - Spawns/attaches    │          │  - Attaches to BeamNG  │
              │  - Drives Car A (AI)  │          │  - Drives Car B (AI)   │
              │  - Launches GUI       │          │  - Listener thread     │
              │  - Logs metrics       │          │  - Logs metrics        │
              └──────────▲────────────┘          └───────────▲────────────┘
                         │                                   │
                         │     TCP Socket 127.0.0.1:5555     │
                         │     JSON messages @ ~20 Hz        │
                         └──────────────┬────────────────────┘
                                        │
                         HELLO │ START_RUN │ POSITION │ WARNING │
                            YIELDING │ PROCEED │ SLOW
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the full component breakdown, threading model, and state machine.

---

## Quick Start

### Prerequisites

| Requirement | Details |
|---|---|
| OS | Windows 10 / 11 (uses `msvcrt` for non-blocking keyboard input) |
| Python | 3.8+ |
| BeamNG.drive | Installed via Steam — note the path (e.g. `X:\Steam\steamapps\common\BeamNG.drive`) |

### Installation

```powershell
# 1. Clone and enter the project
git clone <your-repo-url>
cd MAJOR-PROJECT

# 2. Create and activate a virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1

# 3. Install dependencies
pip install -r requirements.txt
```

### Configure BeamNG Path

In **both** [car_a.py](car_a.py#L21) and [car_b.py](car_b.py#L20), set:

```python
BNG_HOME = r"X:\Steam\steamapps\common\BeamNG.drive"   # ← Your install path
```

### Run

**Terminal 1 — Start Car A (server + BeamNG launcher):**
```powershell
python car_a.py
```
Spawn two vehicles on opposite ends of a road, then select Car A by index.

**Terminal 2 — Start Car B (client):**
```powershell
python car_b.py
```
Select Car B (the vehicle that is **not** currently Blue).

**Execute a run:** In Car A's terminal, enter a base speed (default 45 km/h) and press ENTER. The GUI launches and the vehicles autonomously negotiate the pass.

**Generate the aggregate report (after one or more runs):**
```powershell
python results/generate_report.py
```

See [SETUP.md](SETUP.md) for the full setup walkthrough and troubleshooting.

---

## Project Structure

```
MAJOR-PROJECT/
├── car_a.py                   # V2V server — manages BeamNG + drives Car A
├── car_b.py                   # V2V client — connects to Car A + drives Car B
├── v2v_gui.py                 # Tkinter dashboard (distance, status, speeds, log)
├── v2v_metrics.py             # MetricsCollector — events, CSV logging, summaries
├── requirements.txt           # Python dependencies (beamngpy ≥ 1.28)
│
├── README.md                  # ← this file
├── SETUP.md                   # Setup + troubleshooting
├── MATHEMATICAL_MODEL.md      # Full math: TTC, KE, zones, latency, MDR, CAR
├── FEATURE_ANALYSIS.md        # Inventory of 49 features (30 ✅ / 19 ❌) + reasoning
├── presentation_script.md     # 4-speaker presentation script
│
├── docs/
│   ├── ARCHITECTURE.md        # System design, threading, state machine
│   ├── PROTOCOL.md            # V2V message schemas + sequence diagrams
│   ├── API_REFERENCE.md       # Python module/class/function reference
│   └── EXPERIMENTAL_METHODOLOGY.md  # Test protocol, metric definitions
│
└── results/
    ├── generate_report.py          # Aggregates CSV → Markdown + LaTeX
    ├── v2v_results.csv             # Car A per-run metrics (auto-generated)
    ├── v2v_results_car_b.csv       # Car B per-run metrics (auto-generated)
    ├── experiment_results.md       # Aggregate Markdown summary (auto-generated)
    └── v2v_metrics_table.tex       # Aggregate LaTeX table (auto-generated)
```

---

## How It Works

### 1. Connection phase
- `car_a.py` binds a TCP server on `127.0.0.1:5555` and waits.
- `car_b.py` connects and transmits a `HELLO` message with its vehicle ID.
- Both scripts attach to the running BeamNG.drive instance via `beamngpy`.

### 2. Driving phase
- Both cars are placed in AI `chase` mode targeting each other, constrained to the left lane (`ai.driveInLane("on")`).
- Position and velocity are exchanged at ~20 Hz over the socket.

### 3. Negotiation phase
- When inter-vehicle distance `d < 30 m`, Car A broadcasts a `WARNING` message; both cars drop to the safe passing speed (20 km/h).
- At `d < 35 m`, the **kinetic-energy tie-breaker** fires:
  - `KE_i = ½ m_i s_i²` → the **slower** car has less energy and yields.
  - Ties broken by lexicographic comparison of vehicle IDs.
- The yielder sends `YIELDING` and switches to AI `stop` mode. The passer sends `PROCEED` and switches to AI `random` mode at 20 km/h.

### 4. Completion phase
- Once `d > d_min + 1.0 m` **AND** `d > 10.0 m`, the pass is considered complete.
- Both cars accelerate back to base speed, metrics are saved to CSV, and the system is ready for the next run.

### 5. Emergency fallback
- If negotiation fails and `d < 3 m`, Car A applies `brake=1.0, parkingbrake=1.0` as a last-resort.
- A collision is flagged when `d_min < 1.0 m` (for the Collision Avoidance Rate statistic).

See [MATHEMATICAL_MODEL.md](MATHEMATICAL_MODEL.md) for all equations and thresholds.

---

## Documentation

| Document | What it covers |
|---|---|
| [SETUP.md](SETUP.md) | Prerequisites, venv, BeamNG path, first run, troubleshooting |
| [MATHEMATICAL_MODEL.md](MATHEMATICAL_MODEL.md) | Vehicle state vectors, distance zones, TTC, KE-based yield rule, latency, MDR, CAR |
| [FEATURE_ANALYSIS.md](FEATURE_ANALYSIS.md) | 49-feature inventory — what's implemented, what isn't, and why |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Components, data flow, threading, state machine |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | JSON message schemas, sequence diagrams, error handling |
| [docs/API_REFERENCE.md](docs/API_REFERENCE.md) | `MetricsCollector`, `V2VDashboard`, entry-point modules |
| [docs/EXPERIMENTAL_METHODOLOGY.md](docs/EXPERIMENTAL_METHODOLOGY.md) | Test procedure, controlled variables, metric definitions |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute, code style, testing |
| [CHANGELOG.md](CHANGELOG.md) | Version history |

---

## Results

Per-run metrics auto-logged to `results/v2v_results.csv` (Car A) and `results/v2v_results_car_b.csv` (Car B). After running `python results/generate_report.py`, aggregate statistics are emitted to both Markdown and LaTeX.

Representative metrics captured per run:

| Metric | Unit | Source |
|---|---|---|
| Warning Trigger Distance | m | Distance at which `WARNING` was sent |
| Braking Reaction Time | ms | `t_brake − t_warn` via `time.perf_counter()` |
| Stopping Distance | m | Euclidean distance from brake-apply position to full-stop position |
| Time-to-Collision at Warning | s | `d / (s_A + s_B)` at warning instant |
| Minimum Inter-Vehicle Distance | m | Closest approach during the manoeuvre |
| Collision Avoidance Rate | % | `(R − C) / R × 100`, where C = runs with `d_min < 1 m` |
| V2V Message Latency | ms | Mean of `(t_recv − send_ts)` across received messages |
| Message Delivery Rate | % | `received / max(sent, received) × 100` |
| Position Update Frequency | Hz | `N_updates / Δt_elapsed` |

See [docs/EXPERIMENTAL_METHODOLOGY.md](docs/EXPERIMENTAL_METHODOLOGY.md) for the full experimental protocol.

---

## Known Limitations

The project is a **research prototype**, not a production V2V stack. Key limitations (documented in full in [FEATURE_ANALYSIS.md](FEATURE_ANALYSIS.md)):

- **Software-only communication.** Uses TCP over `localhost` — not DSRC (IEEE 802.11p) or C-V2X. No wireless channel modelling (fading, interference, packet loss).
- **No standardised message format.** Uses custom JSON, not SAE J2735 BSMs (Basic Safety Messages).
- **No security layer.** Plaintext JSON, no PKI / SCMS / certificate management.
- **Ground-truth positioning.** Coordinates come from BeamNG's simulation state (zero GPS noise).
- **Single-scenario.** Head-on, single-lane, two-vehicle only. No intersections, roundabouts, or multi-lane merging.
- **Windows-only.** `msvcrt` is used for non-blocking keyboard input in `car_a.py`.
- **No 3D/map visualisation.** The GUI is a 2D status panel.

Three major scope reductions — hill-road map import, 8–12 car traffic, and BeamNG.tech sensor suite — are explained in detail in the original [README.md (Issues and Constraints)](#issues-and-constraints) section and in [FEATURE_ANALYSIS.md](FEATURE_ANALYSIS.md).

---

## Future Work

Natural extensions, ordered by effort:

1. **Port the non-blocking input** in `car_a.py` to a cross-platform library (e.g. `prompt_toolkit`) to remove the Windows dependency.
2. **Add a replay mode** that re-drives saved CSV runs for deterministic regression testing.
3. **Integrate a wireless channel model** (log-distance path loss + Rayleigh fading) as a delay/drop stage in front of the socket.
4. **Implement SAE J2735 BSMs** with ASN.1 encoding (e.g. `asn1tools`) for real-standard-compliant messaging.
5. **Extend to intersection scenarios** with a V2I virtual RSU that broadcasts signal-phase-and-timing (SPaT) messages.
6. **Couple with SUMO + NS-3** for large-scale traffic + realistic radio propagation.

---

## Citation

If this project informs academic work, please cite:

```bibtex
@misc{beamng_v2v_collision_avoidance,
  title   = {BeamNG V2V Collision Avoidance: A Kinetic-Energy-Based
             Cooperative Right-of-Way Negotiation System},
  author  = {Anand, Adithya and contributors},
  year    = {2026},
  note    = {Major Project — Vehicle-to-Vehicle Communication},
  howpublished = {\url{https://github.com/<your-org>/<your-repo>}}
}
```

---

## Acknowledgments

- **[BeamNG GmbH](https://www.beamng.com/)** — for the BeamNG.drive soft-body physics simulator.
- **[beamngpy](https://github.com/BeamNG/BeamNGpy)** — the Python API that makes the scripted vehicle control possible.
- **Catppuccin** — dashboard colour palette.

---

## Issues and Constraints

During development, several ambitious extensions were explored but ultimately had to be set aside due to technical, licensing, and engine-level constraints.

### 1. Importing Real-World Hill-Road Maps

We initially planned to recreate real Indian hill-road environments (e.g., Himalayan ghat roads) inside BeamNG to test V2V negotiation on narrow, winding mountain terrain.

**Why it was abandoned:**

- **Heightmap format restrictions.** BeamNG's Torque3D engine requires terrain heightmaps to be 16-bit grayscale PNGs with power-of-2 square dimensions (e.g., 2048×2048 or 4096×4096). Most publicly available DEM (Digital Elevation Model) data from sources like USGS SRTM or Bhuvan comes in GeoTIFF or HGT formats at non-square, non-power-of-2 resolutions, requiring extensive re-projection and resampling that degrades vertical accuracy.
- **Texture and material licensing.** Realistic road surfaces, guardrails, cliff textures, and vegetation assets for Indian mountain roads are not included in BeamNG's default asset library. Sourcing these would require licenses from commercial terrain-asset providers (e.g., Quixel, Poliigon) or satellite imagery providers, which were outside the project's academic budget.
- **No collision on tiled terrain.** BeamNG supports tiling multiple heightmap blocks for larger map areas, but the physics engine does not simulate vehicle collisions on the additional tiles — only the primary terrain block has full collision support. This made it impossible to build a continuous hill-road stretch longer than ~4 km at usable detail.
- **World Editor instability.** Heightmaps above 4096×4096 caused persistent crashes and excessive memory usage in the BeamNG World Editor, making iterative road-layout editing impractical.

### 2. Multi-Vehicle Traffic Simulation

The second goal was to extend the V2V scenario to a multi-car traffic environment — spawning 8–12 AI-controlled vehicles on the same road and testing cooperative collision avoidance in congested conditions.

**Why it was abandoned:**

- **AI behaviour unpredictability.** When we assigned individual AI modes (e.g., `chase`, `random`) to multiple vehicles through `beamngpy`, vehicles frequently ignored lane constraints, drifted off-road, or collided with each other in ways unrelated to our V2V logic — making controlled experiments unrepeatable.
- **No programmatic traffic-law enforcement.** BeamNG does not enforce traffic laws on AI traffic, so our custom socket-based negotiation was constantly overridden by the engine's internal AI pathing.
- **CPU and physics bottleneck.** Spawning more than 5–6 soft-body vehicles dropped the simulation below real-time (< 1× factor), introducing artificial latency that invalidated timing-sensitive metrics.
- **No automatic traffic generation API.** `beamngpy` lacked a dedicated API for reproducibly spawning and managing traffic fleets at the time of development.

### 3. BeamNG.tech (Research-Licensed Version)

We obtained an academic research license for BeamNG.tech for its advanced sensor suite — simulated LiDAR, RADAR, camera with instance annotations, IMU, GPS.

**Why we reverted to BeamNG.drive:**

- **Map rendering and terrain glitches.** Stock maps that worked flawlessly in BeamNG.drive exhibited floating road segments, missing terrain, and inconsistent surface normals in BeamNG.tech.
- **Physics timestep mismatch.** Altering the base simulation frequency (required to synchronise sensor sampling) changed tire grip and suspension response, making BeamNG.tech braking-distance measurements incomparable to our BeamNG.drive baseline.
- **Sensor integration complexity.** LiDAR point-cloud export and camera ground-truth annotation required a separate data pipeline beyond the project timeline.
- **API incompatibilities.** The `beamngpy` version compatible with our BeamNG.tech build had breaking changes in vehicle control, sensor polling, and scenario APIs.

Given these compounding issues, we made the pragmatic decision to continue on the consumer BeamNG.drive edition, where the environment was stable, reproducible, and sufficient for demonstrating the core V2V collision avoidance protocol.
