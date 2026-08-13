"""
Real-time 3D visualization of VIO.exe's pose output.

Launches VIO.exe itself, reads its stdout as poses stream in, and plots the
trajectory (translation path) live, updating as you move the camera.

Usage:
    python visualize_trajectory_live.py
    (Ctrl+C in the terminal, or close the plot window, to stop)
"""

import re
import subprocess
import sys
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

VIO_EXE = r"build\Release\VIO.exe"

POSE_RE = re.compile(
    r"R:\s*\[\s*"
    r"([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+);\s*"
    r"([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+);\s*"
    r"([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+)\s*"
    r"\]\s*"
    r"t:\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
    re.DOTALL,
)

positions = []    # list of (x, y, z)
timestamps = []   # arrival time (time.time()) for each entry in `positions`, same index
lock = threading.Lock()


def reader_thread(proc):
    buf = ""
    for line in proc.stdout:
        buf += line
        while True:
            m = POSE_RE.search(buf)
            if not m:
                break
            vals = [float(g) for g in m.groups()]
            t = vals[9:12]
            with lock:
                positions.append(t)
                timestamps.append(time.time())
            buf = buf[m.end():]


def current_fps(window_seconds=2.0):
    """Poses/sec averaged over the last `window_seconds` of arrivals."""
    with lock:
        ts = list(timestamps)
    if len(ts) < 2:
        return 0.0
    now = ts[-1]
    cutoff = now - window_seconds
    recent = [t for t in ts if t >= cutoff]
    if len(recent) < 2:
        return 0.0
    span = recent[-1] - recent[0]
    if span <= 0:
        return 0.0
    return (len(recent) - 1) / span


def main():
    proc = subprocess.Popen(
        [VIO_EXE], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1
    )
    threading.Thread(target=reader_thread, args=(proc,), daemon=True).start()

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Live VIO trajectory (0 poses so far)")
    (line,) = ax.plot([], [], [], color="blue", linewidth=1, marker="o", markersize=4)
    start_scatter = ax.scatter([], [], [], color="green", s=60, label="start")
    latest_scatter = ax.scatter([], [], [], color="red", s=60, label="latest")
    ax.legend(loc="upper left")

    def update(_frame):
        with lock:
            pts = list(positions)
        if not pts:
            return line, start_scatter, latest_scatter

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]

        line.set_data(xs, ys)
        line.set_3d_properties(zs)

        start_scatter._offsets3d = ([xs[0]], [ys[0]], [zs[0]])
        latest_scatter._offsets3d = ([xs[-1]], [ys[-1]], [zs[-1]])

        span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.05)
        cx, cy, cz = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2
        r = span / 2 * 1.3 + 0.02
        ax.set_xlim(cx - r, cx + r)
        ax.set_ylim(cy - r, cy + r)
        ax.set_zlim(cz - r, cz + r)
        ax.set_title(f"Live VIO trajectory ({len(pts)} poses, {current_fps():.1f} keyframes/s)")
        return line, start_scatter, latest_scatter

    ani = animation.FuncAnimation(fig, update, interval=200, cache_frame_data=False)

    try:
        plt.show()
    finally:
        proc.terminate()


if __name__ == "__main__":
    main()
