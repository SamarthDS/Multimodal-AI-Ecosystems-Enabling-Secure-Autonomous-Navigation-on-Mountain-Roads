# V2V Collision Avoidance — Setup Guide

## Prerequisites

| Requirement | Details |
|---|---|
| **OS** | Windows 10/11 (required — uses `msvcrt` for non-blocking keyboard input) |
| **Python** | 3.8 or higher |
| **BeamNG.drive** | Installed via Steam (note the install path, e.g. `X:\Steam\steamapps\common\BeamNG.drive`) |

---

## 1. Create & Activate a Virtual Environment

Open **PowerShell** in the project root:

```powershell
# Create the virtual environment
python -m venv venv

# Activate it
.\venv\Scripts\Activate.ps1
```

> **Note:** If you get an _Execution Policy_ error, run this once as Administrator:
> ```powershell
> Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
> ```

---

## 2. Install Dependencies

```powershell
pip install -r requirements.txt
```

This installs [`beamngpy`](https://github.com/BeamNG/BeamNGpy) — the Python API for BeamNG.drive.  
All other imports (`tkinter`, `msvcrt`, `socket`, `json`, `math`, `csv`, `threading`, `os`, `time`, `datetime`, `collections`) are part of the Python standard library and require no additional installation.

---

## 3. Configure BeamNG Path

In both `car_a.py` (line 21) and `car_b.py` (line 20), update the `BNG_HOME` variable to match your BeamNG.drive install location:

```python
BNG_HOME = r"X:\Steam\steamapps\common\BeamNG.drive"   # ← Change this
```

---

## 4. Run the Project

### Terminal 1 — Start Car A (Server)

```powershell
python car_a.py
```

- This launches BeamNG.drive (or attaches to an already-running instance).
- Spawn **two vehicles** on opposite ends of a road facing each other.
- Select Car A by number when prompted.

### Terminal 2 — Start Car B (Client)

```powershell
python car_b.py
```

- Connects to the existing BeamNG instance.
- Select Car B (the one that is **not** blue) by number.

### Execute a Test Run

1. In Car A's terminal, enter a **Base Speed** (e.g. `45` km/h).
2. Press **ENTER** to trigger both cars.
3. The GUI dashboard launches and the vehicles autonomously negotiate the pass.

---

## 5. Generate the Report (Optional)

After completing test runs:

```powershell
python results/generate_report.py
```

This reads the CSV data in `results/` and generates:
- `results/experiment_results.md` — Markdown summary table
- `results/v2v_metrics_table.tex` — LaTeX table for research papers

---

## Project Structure

```
MAJOR-PROJECT/
├── car_a.py              # V2V Server — manages BeamNG and drives Car A
├── car_b.py              # V2V Client — connects to Car A and drives Car B
├── v2v_gui.py            # Tkinter dashboard for real-time visualization
├── v2v_metrics.py        # Background data logger and metrics collector
├── requirements.txt      # Python dependencies
├── SETUP.md              # This file
├── README.md             # Project overview and constraints
├── FEATURE_ANALYSIS.md   # Feature implementation analysis
└── results/
    ├── generate_report.py      # Aggregates CSV data into final report
    ├── v2v_results.csv         # Car A metrics (auto-generated)
    ├── v2v_results_car_b.csv   # Car B metrics (auto-generated)
    ├── experiment_results.md   # Markdown summary (auto-generated)
    └── v2v_metrics_table.tex   # LaTeX table (auto-generated)
```

---

## Troubleshooting

| Issue | Solution |
|---|---|
| `ModuleNotFoundError: No module named 'beamngpy'` | Make sure the venv is activated and you ran `pip install -r requirements.txt` |
| `Could not connect to BeamNG` | Ensure `car_a.py` is started first and the `BNG_HOME` path is correct |
| `Address already in use` (port 5555) | A previous run is still holding the port. Kill orphaned Python processes or wait a few seconds |
| `Execution Policy` error on `Activate.ps1` | Run `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser` in an admin PowerShell |
| GUI freezes or closes unexpectedly | Tkinter is single-threaded; avoid resizing during a test run |
