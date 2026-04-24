"""
V2V Experiment Report Generator
================================
Reads v2v_results.csv and v2v_results_car_b.csv and produces:
  1. Aggregate statistics (mean ± std, min, max) across runs
  2. A LaTeX-ready table for the research paper
  3. A summary markdown file (experiment_results.md)

Usage:
    python generate_report.py
"""

import csv
import os
import math
from collections import defaultdict


def read_csv(filename):
    """Read a CSV file and return list of dicts with numeric conversion."""
    if not os.path.exists(filename):
        print(f"[!] {filename} not found. Run some test runs first.")
        return []
    rows = []
    with open(filename, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            converted = {}
            for k, v in row.items():
                if v in (None, '', 'None', 'N/A'):
                    converted[k] = None
                elif v in ('True', 'true'):
                    converted[k] = True
                elif v in ('False', 'false'):
                    converted[k] = False
                else:
                    try:
                        converted[k] = float(v)
                    except ValueError:
                        converted[k] = v
            rows.append(converted)
    return rows


def compute_stats(values):
    """Return (mean, std, min, max, n) for a list of numeric values."""
    clean = [v for v in values if v is not None and isinstance(v, (int, float))]
    if not clean:
        return None, None, None, None, 0
    n = len(clean)
    mean = sum(clean) / n
    if n > 1:
        variance = sum((x - mean) ** 2 for x in clean) / (n - 1)
        std = math.sqrt(variance)
    else:
        std = 0.0
    return mean, std, min(clean), max(clean), n


METRICS_CONFIG = [
    ("warning_trigger_distance_m",    "Warning Trigger Distance",     "m"),
    ("speed_at_warning_mps",          "Speed at Warning",             "m/s"),
    ("closing_speed_at_warning_mps",  "Closing Speed at Warning",     "m/s"),
    ("time_to_collision_at_warning_s","Time-to-Collision at Warning", "s"),
    ("braking_reaction_time_ms",      "Braking Reaction Time",        "ms"),
    ("stopping_distance_m",           "Stopping Distance",            "m"),
    ("time_to_stop_ms",               "Time to Full Stop",            "ms"),
    ("speed_at_stop_mps",             "Speed at Stop",                "m/s"),
    ("min_inter_vehicle_distance_m",  "Min Inter-Vehicle Distance",   "m"),
    ("v2v_message_latency_ms",        "Avg V2V Message Latency",      "ms"),
    ("message_delivery_rate_pct",     "Message Delivery Rate",        "%"),
    ("position_update_hz",            "Position Update Frequency",    "Hz"),
    ("run_duration_s",                "Run Duration",                 "s"),
]


def generate_report():
    print("=" * 70)
    print("  V2V EXPERIMENT REPORT GENERATOR")
    print("=" * 70)

    # --- Load data ---
    # Ensure results directory exists
    os.makedirs("results", exist_ok=True)

    car_a_rows = read_csv("results/v2v_results.csv")
    car_b_rows = read_csv("results/v2v_results_car_b.csv")

    if not car_a_rows and not car_b_rows:
        print("[!] No data files found. Complete some test runs first.")
        return

    all_rows = car_a_rows + car_b_rows
    total_runs = max(
        len(car_a_rows),
        len(car_b_rows),
        1
    )

    # Count collisions
    collisions = sum(1 for r in all_rows if r.get("collision_occurred") is True)
    avoidance_rate = ((total_runs - collisions) / total_runs) * 100 if total_runs else 0

    # --- Console summary ---
    print(f"\n  Total test runs: {total_runs}")
    print(f"  Collision Avoidance Rate: {avoidance_rate:.1f}%")
    print(f"  Data sources: Car A ({len(car_a_rows)} rows), Car B ({len(car_b_rows)} rows)")

    # --- Per-metric stats (combine both cars) ---
    print(f"\n  {'Metric':<35} {'Mean ± Std':>20} {'Min':>10} {'Max':>10} {'N':>5}")
    print(f"  {'-'*35} {'-'*20} {'-'*10} {'-'*10} {'-'*5}")

    stats_data = {}
    for key, label, unit in METRICS_CONFIG:
        values = [r.get(key) for r in all_rows]
        mean, std, vmin, vmax, n = compute_stats(values)
        stats_data[key] = (mean, std, vmin, vmax, n, label, unit)
        if mean is not None:
            print(f"  {label:<35} {mean:>8.2f} ± {std:>6.2f} {unit:<3} {vmin:>8.2f} {vmax:>10.2f} {n:>5}")
        else:
            print(f"  {label:<35} {'N/A':>20} {'N/A':>10} {'N/A':>10} {n:>5}")

    # --- LaTeX table ---
    latex_lines = []
    latex_lines.append(r"\begin{table}[htbp]")
    latex_lines.append(r"\centering")
    latex_lines.append(r"\caption{V2V Collision Avoidance System Performance Metrics}")
    latex_lines.append(r"\label{tab:v2v_metrics}")
    latex_lines.append(r"\begin{tabular}{lcccc}")
    latex_lines.append(r"\toprule")
    latex_lines.append(r"\textbf{Metric} & \textbf{Mean $\pm$ Std} & \textbf{Min} & \textbf{Max} & \textbf{N} \\")
    latex_lines.append(r"\midrule")

    for key, label, unit in METRICS_CONFIG:
        mean, std, vmin, vmax, n, _, _ = stats_data[key]
        if mean is not None:
            latex_lines.append(
                f"{label} ({unit}) & {mean:.2f} $\\pm$ {std:.2f} & {vmin:.2f} & {vmax:.2f} & {n} \\\\"
            )
        else:
            latex_lines.append(f"{label} ({unit}) & N/A & N/A & N/A & {n} \\\\")

    latex_lines.append(r"\midrule")
    latex_lines.append(f"Collision Avoidance Rate & \\multicolumn{{4}}{{c}}{{{avoidance_rate:.1f}\\%}} \\\\")
    latex_lines.append(r"\bottomrule")
    latex_lines.append(r"\end{tabular}")
    latex_lines.append(r"\end{table}")

    latex_str = "\n".join(latex_lines)

    print(f"\n{'='*70}")
    print("  LATEX TABLE (copy into your paper)")
    print(f"{'='*70}")
    print(latex_str)

    # --- Save LaTeX to file ---
    with open("results/v2v_metrics_table.tex", "w") as f:
        f.write(latex_str)
    print(f"\n[+] LaTeX table saved to results/v2v_metrics_table.tex")

    # --- Markdown summary ---
    md_lines = []
    md_lines.append("# V2V Collision Avoidance — Experiment Results\n")
    md_lines.append(f"**Total Runs:** {total_runs}  ")
    md_lines.append(f"**Collision Avoidance Rate:** {avoidance_rate:.1f}%\n")
    md_lines.append("| Metric | Mean ± Std | Min | Max | N |")
    md_lines.append("|--------|-----------|-----|-----|---|")

    for key, label, unit in METRICS_CONFIG:
        mean, std, vmin, vmax, n, _, _ = stats_data[key]
        if mean is not None:
            md_lines.append(f"| {label} ({unit}) | {mean:.2f} ± {std:.2f} | {vmin:.2f} | {vmax:.2f} | {n} |")
        else:
            md_lines.append(f"| {label} ({unit}) | N/A | N/A | N/A | {n} |")

    md_str = "\n".join(md_lines)

    with open("results/experiment_results.md", "w") as f:
        f.write(md_str)
    print(f"[+] Markdown summary saved to results/experiment_results.md")
    print(f"\n{'='*70}")
    print("  REPORT GENERATION COMPLETE")
    print(f"{'='*70}")


if __name__ == "__main__":
    generate_report()
