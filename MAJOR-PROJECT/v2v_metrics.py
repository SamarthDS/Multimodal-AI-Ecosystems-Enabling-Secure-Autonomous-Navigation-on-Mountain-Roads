"""
V2V Metrics Collector
=====================
Captures and records performance metrics for each V2V test run.
Used by car_a.py and car_b.py to instrument the collision avoidance system.

Generates:
  - Per-run console summary table
  - Cumulative CSV file (v2v_results.csv) for multi-run analysis
"""

import time
import math
import csv
import os
from datetime import datetime


class MetricsCollector:
    """Collects timestamped events and computes performance metrics for one car."""

    CSV_HEADER = [
        "run_id", "car_id", "timestamp",
        "warning_trigger_distance_m", "speed_at_warning_mps",
        "braking_reaction_time_ms", "stopping_distance_m",
        "time_to_stop_ms", "speed_at_stop_mps",
        "min_inter_vehicle_distance_m", "collision_occurred",
        "v2v_message_latency_ms", "messages_sent", "messages_received",
        "message_delivery_rate_pct", "position_update_hz",
        "run_duration_s", "time_to_collision_at_warning_s",
        "closing_speed_at_warning_mps",
    ]

    def __init__(self, car_id: str, csv_filename: str = "v2v_results.csv"):
        self.car_id = car_id
        self.csv_filename = csv_filename

        # --- Per-run state (reset on start_run) ---
        self.run_id = 0
        self._run_start_time = 0.0
        self._run_end_time = 0.0

        # Warning / braking events
        self._warning_time = None
        self._warning_distance = None
        self._warning_speed = None
        self._warning_other_speed = None  # other car's speed at warning
        self._brake_time = None
        self._brake_position = None
        self._brake_speed = None
        self._stop_time = None
        self._stop_position = None
        self._stop_speed = None

        # Distance tracking
        self._min_distance = float('inf')
        self._collision = False

        # Message counters
        self._messages_sent = 0
        self._messages_received = 0
        self._position_updates = 0
        self._position_update_start = 0.0

        # Latency samples
        self._latency_samples = []

        # Ensure output directory exists
        csv_dir = os.path.dirname(self.csv_filename)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        # Ensure CSV header exists
        self._ensure_csv_header()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start_run(self, run_id: int):
        """Call at the beginning of each test run."""
        self.run_id = run_id
        self._run_start_time = time.perf_counter()
        self._run_end_time = 0.0

        self._warning_time = None
        self._warning_distance = None
        self._warning_speed = None
        self._warning_other_speed = None
        self._brake_time = None
        self._brake_position = None
        self._brake_speed = None
        self._stop_time = None
        self._stop_position = None
        self._stop_speed = None

        self._min_distance = float('inf')
        self._collision = False

        self._messages_sent = 0
        self._messages_received = 0
        self._position_updates = 0
        self._position_update_start = time.perf_counter()

        self._latency_samples = []

        print(f"\n[METRICS-{self.car_id}] Run #{run_id} — recording started.")

    def end_run(self):
        """Call at the end of each test run. Prints summary and saves to CSV."""
        self._run_end_time = time.perf_counter()
        self.print_summary()
        self.save_csv()
        print(f"[METRICS-{self.car_id}] Run #{self.run_id} — data saved to {self.csv_filename}")

    # ------------------------------------------------------------------
    # Event loggers  (call these from car_a / car_b at the right moments)
    # ------------------------------------------------------------------
    def log_message_sent(self):
        """Increment sent counter (call every time a V2V message is sent)."""
        self._messages_sent += 1

    def log_message_received(self):
        """Increment received counter."""
        self._messages_received += 1

    def log_position_update(self):
        """Record a position-exchange tick (for Hz calculation)."""
        self._position_updates += 1

    def log_latency(self, send_timestamp: float):
        """Record one-way latency from a send_timestamp embedded in a message.
        
        NOTE: Uses time.time() (wall clock) because send_timestamp comes from
        a different process where perf_counter() has a different epoch.
        """
        latency = (time.time() - send_timestamp) * 1000  # ms
        if latency >= 0:
            self._latency_samples.append(latency)

    def log_distance(self, distance: float):
        """Update minimum inter-vehicle distance and collision flag."""
        if distance < self._min_distance:
            self._min_distance = distance
        if distance < 1.0:  # less than 1 m → collision
            self._collision = True

    def log_warning_sent(self, distance: float, my_speed: float,
                         other_speed: float = 0.0):
        """Car A calls this when it sends the WARNING message."""
        self._warning_time = time.perf_counter()
        self._warning_distance = distance
        self._warning_speed = my_speed
        self._warning_other_speed = other_speed

    def log_warning_received(self, distance: float, my_speed: float,
                             send_timestamp: float = None):
        """Car B calls this when it receives the WARNING message."""
        self._warning_time = time.perf_counter()
        self._warning_distance = distance
        self._warning_speed = my_speed
        if send_timestamp is not None:
            self.log_latency(send_timestamp)

    def log_brake_applied(self, position: list, speed: float):
        """Call immediately after the brake command is issued."""
        self._brake_time = time.perf_counter()
        self._brake_position = list(position)
        self._brake_speed = speed

    def log_full_stop(self, position: list, speed: float = 0.0):
        """Call once the car's speed drops to ≈0."""
        self._stop_time = time.perf_counter()
        self._stop_position = list(position)
        self._stop_speed = speed

    # ------------------------------------------------------------------
    # Computed metrics
    # ------------------------------------------------------------------
    def _braking_reaction_time_ms(self):
        """Time from warning event to brake command (ms)."""
        if self._warning_time and self._brake_time:
            return (self._brake_time - self._warning_time) * 1000
        return None

    def _stopping_distance_m(self):
        """Euclidean distance from brake position to stop position."""
        if self._brake_position and self._stop_position:
            return math.sqrt(sum(
                (a - b) ** 2
                for a, b in zip(self._brake_position, self._stop_position)
            ))
        return None

    def _time_to_stop_ms(self):
        """Time from brake command to full stop (ms)."""
        if self._brake_time and self._stop_time:
            return (self._stop_time - self._brake_time) * 1000
        return None

    def _message_delivery_rate(self):
        """Percentage of sent messages that were received."""
        total = max(self._messages_sent, self._messages_received, 1)
        return (self._messages_received / total) * 100

    def _position_update_hz(self):
        """Average position update frequency."""
        elapsed = time.perf_counter() - self._position_update_start
        if elapsed > 0:
            return self._position_updates / elapsed
        return 0

    def _run_duration_s(self):
        end = self._run_end_time or time.perf_counter()
        return end - self._run_start_time

    def _avg_latency_ms(self):
        if self._latency_samples:
            return sum(self._latency_samples) / len(self._latency_samples)
        return None

    def _time_to_collision_at_warning_s(self):
        """TTC = distance / closing speed at the moment of warning."""
        if self._warning_distance and self._warning_speed is not None:
            closing = self._warning_speed
            if self._warning_other_speed:
                closing += self._warning_other_speed
            if closing > 0:
                return self._warning_distance / closing
        return None

    def _closing_speed_at_warning(self):
        closing = (self._warning_speed or 0)
        if self._warning_other_speed:
            closing += self._warning_other_speed
        return closing if closing > 0 else None

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def print_summary(self):
        """Print a formatted summary table for the current run."""
        react = self._braking_reaction_time_ms()
        stop_dist = self._stopping_distance_m()
        stop_time = self._time_to_stop_ms()
        delivery = self._message_delivery_rate()
        hz = self._position_update_hz()
        latency = self._avg_latency_ms()
        ttc = self._time_to_collision_at_warning_s()
        duration = self._run_duration_s()
        closing = self._closing_speed_at_warning()

        def fmt(val, unit="", decimals=2):
            if val is None:
                return "N/A"
            return f"{val:.{decimals}f}{unit}"

        print(f"\n{'='*60}")
        print(f"  METRICS SUMMARY — {self.car_id} — Run #{self.run_id}")
        print(f"{'='*60}")
        print(f"  {'Metric':<36} {'Value':>18}")
        print(f"  {'-'*36} {'-'*18}")
        print(f"  {'Warning Trigger Distance':<36} {fmt(self._warning_distance, ' m'):>18}")
        print(f"  {'Speed at Warning':<36} {fmt(self._warning_speed, ' m/s'):>18}")
        print(f"  {'Closing Speed at Warning':<36} {fmt(closing, ' m/s'):>18}")
        print(f"  {'Time-to-Collision at Warning':<36} {fmt(ttc, ' s'):>18}")
        print(f"  {'Braking Reaction Time':<36} {fmt(react, ' ms'):>18}")
        print(f"  {'Stopping Distance':<36} {fmt(stop_dist, ' m'):>18}")
        print(f"  {'Time to Full Stop':<36} {fmt(stop_time, ' ms'):>18}")
        print(f"  {'Speed at Stop':<36} {fmt(self._stop_speed, ' m/s'):>18}")
        print(f"  {'Min Inter-Vehicle Distance':<36} {fmt(self._min_distance if self._min_distance != float('inf') else None, ' m'):>18}")
        print(f"  {'Collision Occurred':<36} {'YES ⚠' if self._collision else 'NO ✓':>18}")
        print(f"  {'Avg V2V Message Latency':<36} {fmt(latency, ' ms'):>18}")
        print(f"  {'Messages Sent':<36} {str(self._messages_sent):>18}")
        print(f"  {'Messages Received':<36} {str(self._messages_received):>18}")
        print(f"  {'Message Delivery Rate':<36} {fmt(delivery, ' %'):>18}")
        print(f"  {'Position Update Frequency':<36} {fmt(hz, ' Hz'):>18}")
        print(f"  {'Run Duration':<36} {fmt(duration, ' s'):>18}")
        print(f"{'='*60}\n")

    def _ensure_csv_header(self):
        """Write CSV header if the file doesn't exist yet."""
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, 'w', newline='') as f:
                csv.writer(f).writerow(self.CSV_HEADER)

    def save_csv(self):
        """Append one row of metrics for this run to the CSV file."""
        react = self._braking_reaction_time_ms()
        stop_dist = self._stopping_distance_m()
        stop_time = self._time_to_stop_ms()
        delivery = self._message_delivery_rate()
        hz = self._position_update_hz()
        latency = self._avg_latency_ms()
        ttc = self._time_to_collision_at_warning_s()
        duration = self._run_duration_s()
        closing = self._closing_speed_at_warning()

        row = [
            self.run_id,
            self.car_id,
            datetime.now().isoformat(),
            self._warning_distance,
            self._warning_speed,
            react,
            stop_dist,
            stop_time,
            self._stop_speed,
            self._min_distance if self._min_distance != float('inf') else None,
            self._collision,
            latency,
            self._messages_sent,
            self._messages_received,
            delivery,
            hz,
            duration,
            ttc,
            closing,
        ]

        self._ensure_csv_header()
        with open(self.csv_filename, 'a', newline='') as f:
            csv.writer(f).writerow(row)
