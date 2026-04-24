# Contributing

Thanks for your interest in improving the BeamNG V2V Collision Avoidance project. This guide covers the workflow, coding conventions, and testing expectations for contributions.

---

## 1. Development Setup

Follow [SETUP.md](SETUP.md) to get a local environment running. In addition, for development work:

```powershell
# Always work inside the venv
.\venv\Scripts\Activate.ps1

# Install any linting / formatting helpers you personally prefer
pip install black ruff
```

We do not ship a `requirements-dev.txt` — the runtime dependency list is intentionally minimal (just `beamngpy`).

---

## 2. Branching & Commits

1. Fork the repository (or branch off `main` if you have push access).
2. Create a topic branch: `feat/short-description` or `fix/short-description`.
3. Make focused commits — one logical change per commit.
4. Follow the existing commit-message style: a short imperative subject, optionally followed by a body explaining *why*.

Example:

```
Add SLOW message handler with configurable target_speed

Extends the listener to honour SLOW messages with a per-message
target_speed so intermediate slowdown steps can be tuned without
editing car_a.py.
```

5. Open a pull request describing the change, the motivation, and how you tested it.

---

## 3. Code Style

- **Python 3.8+** — use f-strings, `typing` annotations where useful.
- **4-space indentation**, no tabs.
- **Match existing style** in the module you're editing. This codebase is deliberately simple — prefer clarity over cleverness.
- **Constants at the top** of each module in an `# ===== CONFIG =====` block.
- **Avoid adding dependencies.** Any new third-party package requires justification in the PR.
- **Comments**: only when the *why* is non-obvious. Don't narrate the *what*.
- **Formatter**: if you run Black or Ruff, keep line length at 120.

---

## 4. Where New Features Go

| Type of change | File(s) |
|---|---|
| A new V2V message type | [car_a.py](car_a.py) emitter, [car_b.py](car_b.py) listener, [docs/PROTOCOL.md](docs/PROTOCOL.md) |
| A new metric | [v2v_metrics.py](v2v_metrics.py) + [results/generate_report.py](results/generate_report.py) |
| A new GUI element | [v2v_gui.py](v2v_gui.py) — keep the thread-safety caveats |
| A new distance threshold | [car_a.py](car_a.py) constants + [MATHEMATICAL_MODEL.md](MATHEMATICAL_MODEL.md) §14 |
| A new AI behaviour | The relevant branch in `car_a.py` / `car_b.py` — document in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) state machine |

If you're adding a new metric, see the walkthrough in [docs/API_REFERENCE.md §7](docs/API_REFERENCE.md#7-adding-a-new-metric-walk-through).

---

## 5. Testing Changes

There is no automated test suite — validation is by live simulation. For any non-trivial change:

1. Run **at least 5 successful test runs** at the default base speed (45 km/h).
2. Verify:
   - Inter-vehicle distance on the GUI matches the last log line of each run.
   - The CSVs in `results/` grow by exactly one row per run.
   - `python results/generate_report.py` runs without errors and produces sensible numbers.
3. If you touched timing code, also check that:
   - `braking_reaction_time_ms` is positive (not `None`).
   - `v2v_message_latency_ms` is positive.
4. If you touched the GUI, resize the window, run several iterations, and make sure it doesn't lock up.
5. Describe your test runs (base speeds, number of runs, any anomalies) in the PR description.

---

## 6. Documentation Expectations

- Every public-facing change updates the relevant file in `docs/`.
- Updates to `FEATURE_ANALYSIS.md` are welcome for new capabilities (move rows from ❌ to ✅ with the technical explanation).
- Updates to `MATHEMATICAL_MODEL.md` are required for any change to thresholds, formulas, or units.

---

## 7. Reporting Bugs

Open an issue with:

- **Steps to reproduce** (minimum viable — base speed, map, any non-default config).
- **Expected behaviour** vs **actual behaviour**.
- **Console output** from both `car_a.py` and `car_b.py` (relevant excerpts — don't paste thousands of lines).
- **Environment**: Windows version, Python version, BeamNG.drive version, `beamngpy` version.
- Relevant CSV rows from `results/` if the issue is metric-related.

---

## 8. Out-of-Scope Contributions

Contributions that shift the project away from its core scope are unlikely to be accepted. In particular:

- **Cross-platform ports of `msvcrt`** are welcome (→ `prompt_toolkit` or similar), but they must not regress Windows behaviour.
- **Replacing TCP sockets with a different IPC mechanism** is a major change — open an issue to discuss before implementing.
- **Changing the negotiation logic** (e.g. swapping KE for a different heuristic) is acceptable *only if* the existing KE protocol remains available behind a flag, because it is the subject of published results.

---

## 9. Code of Conduct

Be respectful. Critique ideas, not people. Assume good faith. If you encounter unacceptable behaviour, contact the repository maintainer directly.

---

Thank you for contributing. 🚗💬🚗
