"""
Real-time 3D visualization of the OAK-D's raw accelerometer direction.

Connects directly to the device (no need to run VIO.exe first - in fact VIO.exe
must NOT be running at the same time, only one process can hold the device).

Blue arrow = live accelerometer reading direction (should point "up", roughly
+Z, when the device is held level and still - that's gravity's reaction force).
Red arrow = reference "up" direction for comparison.

Usage:
    python visualize_imu_live.py
"""

import depthai as dai
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

device = dai.Device()
pipeline = dai.Pipeline(device)
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 100)
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imuQueue = imu.out.createOutputQueue(maxSize=50, blocking=False)

pipeline.start()

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")
LIM = 11
ax.set_xlim(-LIM, LIM)
ax.set_ylim(-LIM, LIM)
ax.set_zlim(-LIM, LIM)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Live accelerometer direction (blue) vs reference up (red)")

# static reference arrow: where "up" (+Z, ~9.81 m/s^2) should be if level
ax.quiver(0, 0, 0, 0, 0, 9.81, color="red", linewidth=2, label="reference up")
ax.legend(loc="upper left")

state = {"quiver": None, "last": (0, 0, 9.81)}


def update(_frame):
    latest = None
    while True:
        imuData = imuQueue.tryGet()
        if imuData is None:
            break
        if imuData.packets:
            latest = imuData.packets[-1].acceleroMeter

    if latest is not None:
        state["last"] = (latest.x, latest.y, latest.z)

    if state["quiver"] is not None:
        state["quiver"].remove()
    x, y, z = state["last"]
    state["quiver"] = ax.quiver(0, 0, 0, x, y, z, color="blue", linewidth=3)
    ax.set_title(f"accel = ({x:.2f}, {y:.2f}, {z:.2f})  |mag|={(x*x+y*y+z*z)**0.5:.2f}")
    return (state["quiver"],)


ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)

try:
    plt.show()
finally:
    pipeline.stop()
