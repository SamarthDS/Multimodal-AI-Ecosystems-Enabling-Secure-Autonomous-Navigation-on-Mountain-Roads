"""
V2V Unified Dashboard GUI
=========================
Simple tkinter GUI to visualize V2V communication status for both cars.
Non-blocking — call dashboard.update() inside your main loop.
"""

import tkinter as tk
from tkinter import scrolledtext
from datetime import datetime


class V2VDashboard:
    """Lightweight unified tkinter dashboard for V2V collision avoidance."""

    # Color scheme
    COLORS = {
        "bg":       "#1e1e2e",
        "card":     "#2a2a3d",
        "text":     "#cdd6f4",
        "dim":      "#6c7086",
        "green":    "#a6e3a1",
        "yellow":   "#f9e2af",
        "orange":   "#fab387",
        "red":      "#f38ba8",
        "blue":     "#89b4fa",
        "car_a":    "#89b4fa",  # Blue
        "car_b":    "#f38ba8",  # Red
    }

    STATUS_COLORS = {
        "IDLE":     "#6c7086",
        "DRIVING":  "#a6e3a1",
        "WARNING":  "#f9e2af",
        "SLOWING":  "#fab387",
        "STOPPED":  "#f38ba8",
        "SUCCESS":  "#a6e3a1",
    }

    def __init__(self, title="Unified V2V Dashboard"):
        self.root = tk.Tk()
        self.root.title(title)
        self.root.configure(bg=self.COLORS["bg"])
        self.root.geometry("640x700")
        self.root.resizable(False, False)

        self._status_labels = {}
        self._speed_labels = {}
        
        self._build_ui(title)
        self.set_status('A', "IDLE")
        self.set_status('B', "IDLE")
        self.root.update()

    def _build_ui(self, title):
        C = self.COLORS
        pad = {"padx": 12, "pady": 4}

        # --- Title ---
        tk.Label(
            self.root, text=title, font=("Consolas", 16, "bold"),
            fg=C["text"], bg=C["bg"]
        ).pack(pady=(12, 8))

        # --- Distance (Shared) ---
        dist_frame = tk.Frame(self.root, bg=C["card"], highlightbackground=C["dim"], highlightthickness=1)
        dist_frame.pack(fill="x", **pad)
        tk.Label(dist_frame, text="INTER-VEHICLE DISTANCE", font=("Consolas", 10), fg=C["dim"], bg=C["card"]).pack(pady=(6,0))
        self._dist_label = tk.Label(dist_frame, text="-- m", font=("Consolas", 28, "bold"), fg=C["text"], bg=C["card"])
        self._dist_label.pack(pady=(0, 8))

        # --- 2-Column Layout for Cars ---
        cols_frame = tk.Frame(self.root, bg=C["bg"])
        cols_frame.pack(fill="x", **pad)

        # Car A Column (Left)
        car_a_frame = tk.Frame(cols_frame, bg=C["card"], highlightbackground=C["car_a"], highlightthickness=2)
        car_a_frame.pack(side="left", expand=True, fill="both", padx=(0, 6))
        
        tk.Label(car_a_frame, text="CAR A (BLUE)", font=("Consolas", 12, "bold"), fg=C["car_a"], bg=C["card"]).pack(pady=(8,4))
        tk.Label(car_a_frame, text="STATUS", font=("Consolas", 9), fg=C["dim"], bg=C["card"]).pack()
        self._status_labels['A'] = tk.Label(car_a_frame, text="IDLE", font=("Consolas", 18, "bold"), fg=C["dim"], bg=C["card"])
        self._status_labels['A'].pack(pady=(0, 6))
        
        tk.Label(car_a_frame, text="SPEED", font=("Consolas", 9), fg=C["dim"], bg=C["card"]).pack()
        self._speed_labels['A'] = tk.Label(car_a_frame, text="-- km/h", font=("Consolas", 18, "bold"), fg=C["text"], bg=C["card"])
        self._speed_labels['A'].pack(pady=(0, 12))

        # Car B Column (Right)
        car_b_frame = tk.Frame(cols_frame, bg=C["card"], highlightbackground=C["car_b"], highlightthickness=2)
        car_b_frame.pack(side="right", expand=True, fill="both", padx=(6, 0))
        
        tk.Label(car_b_frame, text="CAR B (RED)", font=("Consolas", 12, "bold"), fg=C["car_b"], bg=C["card"]).pack(pady=(8,4))
        tk.Label(car_b_frame, text="STATUS", font=("Consolas", 9), fg=C["dim"], bg=C["card"]).pack()
        self._status_labels['B'] = tk.Label(car_b_frame, text="IDLE", font=("Consolas", 18, "bold"), fg=C["dim"], bg=C["card"])
        self._status_labels['B'].pack(pady=(0, 6))
        
        tk.Label(car_b_frame, text="SPEED", font=("Consolas", 9), fg=C["dim"], bg=C["card"]).pack()
        self._speed_labels['B'] = tk.Label(car_b_frame, text="-- km/h", font=("Consolas", 18, "bold"), fg=C["text"], bg=C["card"])
        self._speed_labels['B'].pack(pady=(0, 12))

        # --- Message Log (Shared) ---
        tk.Label(self.root, text="V2V MESSAGE LOG", font=("Consolas", 10), fg=C["dim"], bg=C["bg"]).pack(anchor="w", padx=12, pady=(12, 0))
        self._log = scrolledtext.ScrolledText(
            self.root, height=14, font=("Consolas", 9),
            bg=C["card"], fg=C["text"], insertbackground=C["text"],
            relief="flat", borderwidth=0, state="disabled", wrap="word"
        )
        self._log.pack(fill="both", expand=True, padx=12, pady=(2, 12))

        # Configure log tag colors
        self._log.tag_configure("time", foreground=C["dim"])
        self._log.tag_configure("warn", foreground=C["yellow"])
        self._log.tag_configure("stop", foreground=C["red"])
        self._log.tag_configure("info", foreground=C["text"])
        self._log.tag_configure("success", foreground=C["green"])
        self._log.tag_configure("A", foreground=C["car_a"])
        self._log.tag_configure("B", foreground=C["car_b"])

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def set_status(self, car: str, status: str):
        """Update the status badge for 'A' or 'B'. One of: IDLE, DRIVING, WARNING, SLOWING, STOPPED, SUCCESS."""
        try:
            if car in self._status_labels:
                color = self.STATUS_COLORS.get(status, self.COLORS["text"])
                self._status_labels[car].config(text=status, fg=color)
        except tk.TclError:
            pass

    def set_distance(self, dist_m: float):
        """Update the distance readout."""
        try:
            self._dist_label.config(text=f"{dist_m:.1f} m")
        except tk.TclError:
            pass

    def set_speed(self, car: str, speed_mps: float):
        """Update speed readout for 'A' or 'B'."""
        try:
            if car in self._speed_labels:
                kmh = speed_mps * 3.6
                self._speed_labels[car].config(text=f"{kmh:.0f} km/h")
        except tk.TclError:
            pass

    def log(self, message: str, tag: str = "info", car: str = None):
        """Append a log message. car can be 'A' or 'B' to prefix it."""
        try:
            ts = datetime.now().strftime("%H:%M:%S")
            self._log.config(state="normal")
            self._log.insert("end", f"[{ts}] ", "time")
            if car:
                self._log.insert("end", f"[CAR {car}] ", car)
            self._log.insert("end", f"{message}\n", tag)
            self._log.see("end")
            self._log.config(state="disabled")
        except tk.TclError:
            pass

    def tick(self):
        """Call this every iteration of your main loop to keep the GUI responsive."""
        try:
            self.root.update()
        except tk.TclError:
            pass

    def close(self):
        try:
            self.root.destroy()
        except:
            pass
