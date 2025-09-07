#!/usr/bin/env python3

TARGET_HZ = 100.0
TARGET_DT = 1.0 / TARGET_HZ

import rclpy
import time
from rclpy.node import Node
from gbr_control.packages.gbr_direct_interface import GBRDirectInterface
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# ─── SETPOINT: Desired [x, y, z] position in world frame ─────────────────────
XYZ_REF = np.array([0.0, 0.0, 0.0])  # Example: 2 meters forward, 1 meter up

# ─── INTEGRAL STATE (optional) ───────────────────────────────────────────────
integral_xyz = np.zeros(3)

# ─── LQR CONTROL FUNCTION WITH XYZ SETPOINT SUPPORT ──────────────────────────
def lqr_control(state, K, dt=0.0, xyz_ref=XYZ_REF, integral=False):
    global integral_xyz

    pos_err = state[[0, 2, 4]] - xyz_ref
    error = state.copy()
    error[0] -= xyz_ref[0]
    error[2] -= xyz_ref[1]
    error[4] -= xyz_ref[2]

    if integral:
        if dt <= 0:
            raise ValueError("dt must be > 0 when using integral control.")
        integral_xyz += pos_err * dt
        aug_error = np.concatenate((error, integral_xyz))
        u = -K @ aug_error
    else:
        u = -K @ error

    return u

# ─── LQR GAIN (standard 12×8) ────────────────────────────────────────────────
K = np.array([
    [-1.57029, -1.28541, -1.57029, -1.28541, -0.,      -0.,      0.,      -0.,     -0.,     -0.,     -1.43233, -1.02294],
    [ 1.57029,  1.28541, -1.57029, -1.28541, -0.,      -0.,      0.,      -0.,      0.,      0.,      1.43233,  1.02294],
    [-1.57029, -1.28541,  1.57029,  1.28541,  0.,       0.,     -0.,       0.,      0.,      0.,      1.43233,  1.02294],
    [ 1.57029,  1.28541,  1.57029,  1.28541,  0.,       0.,     -0.,       0.,     -0.,     -0.,     -1.43233, -1.02294],
    [-0.,      -0.,       0.,       0.,       1.57178,  0.78299, 1.45681,  1.04881, 1.48299, 1.21216,  0.,       0.     ],
    [-0.,      -0.,       0.,       0.,       1.57178,  0.78299,-1.45681, -1.04881, 1.48299, 1.21216,  0.,       0.     ],
    [ 0.,       0.,       0.,       0.,       1.57178,  0.78299, 1.45681,  1.04881,-1.48299,-1.21216, -0.,      -0.     ],
    [ 0.,       0.,       0.,       0.,       1.57178,  0.78299,-1.45681, -1.04881,-1.48299,-1.21216, -0.,      -0.     ]
])
max_force = 20.0

K = K*2

# ─── PLOT SETUP ──────────────────────────────────────────────────────────────
PLOT_WINDOW = 20.0
time_data, roll_data, pitch_data, yaw_data = deque(), deque(), deque(), deque()
x_data, y_data, z_data = deque(), deque(), deque()

fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, figsize=(10, 14))
line1, = ax1.plot([], [], label="Roll")
line2, = ax2.plot([], [], label="Pitch")
line3, = ax3.plot([], [], label="Yaw")
line4, = ax4.plot([], [], label="X")
line5, = ax5.plot([], [], label="Y")
line6, = ax6.plot([], [], label="Z")

for ax in (ax1, ax2, ax3, ax4, ax5, ax6):
    ax.legend()
    ax.grid(True)
ax1.set_ylabel("Roll (deg)")
ax2.set_ylabel("Pitch (deg)")
ax3.set_ylabel("Yaw (deg)")
ax4.set_ylabel("X (m)")
ax5.set_ylabel("Y (m)")
ax6.set_ylabel("Z (m)")
ax6.set_xlabel("Time (s)")

plt.ion()
plt.show()

start_time = time.time()

# ─── PLOT UPDATE ─────────────────────────────────────────────────────────────
def update_plot():
    t_now = time.time()
    while time_data and t_now - time_data[0] > PLOT_WINDOW:
        time_data.popleft()
        roll_data.popleft()
        pitch_data.popleft()
        yaw_data.popleft()
        x_data.popleft()
        y_data.popleft()
        z_data.popleft()

    t = np.array(time_data) - start_time
    line1.set_data(t, roll_data)
    line2.set_data(t, pitch_data)
    line3.set_data(t, yaw_data)
    line4.set_data(t, x_data)
    line5.set_data(t, y_data)
    line6.set_data(t, z_data)

    for ax in (ax1, ax2, ax3, ax4, ax5, ax6):
        ax.relim()
        ax.autoscale_view()
        ax.set_xlim(max(0, t[-1] - PLOT_WINDOW), t[-1] + 0.1)

    plt.pause(0.001)

# ─── MAIN CONTROL LOOP ───────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = Node('direct_interface_python')
    rov = GBRDirectInterface(node)
    prev_time = time.time()

    try:
        while rclpy.ok():
            loop_start = time.time()

            # ─── Get pose and velocity ─────────────────────
            lin_pos, ang_pos = rov.get_pose()
            lin_vel, ang_vel = rov.get_velocity()

            state = np.array([
                lin_pos[0], lin_vel[0],
                lin_pos[1], lin_vel[1],
                lin_pos[2], lin_vel[2],
                ang_pos[0], ang_vel[0],
                ang_pos[1], ang_vel[1],
                ang_pos[2], ang_vel[2]
            ])

            dt = time.time() - prev_time

            # ─── Compute control input ─────────────────────
            U = lqr_control(state, K, dt=dt, xyz_ref=XYZ_REF, integral=False)
            U = np.clip(U, -max_force, max_force)
            rov.set_thrusters(U.tolist())

            # ─── Plotting ──────────────────────────────────
            current_time = time.time()
            time_data.append(current_time)
            roll_data.append(np.degrees(ang_pos[0]))
            pitch_data.append(np.degrees(ang_pos[1]))
            yaw_data.append(np.degrees(ang_pos[2]))
            x_data.append(lin_pos[0])
            y_data.append(lin_pos[1])
            z_data.append(lin_pos[2])
            update_plot()

            # ─── Loop control ─────────────────────────────
            loop_hz = 1.0 / (time.time() - prev_time)
            prev_time = time.time()
            print(f"Loop frequency: {loop_hz:.1f} Hz")

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, TARGET_DT - elapsed))
            rclpy.spin_once(node, timeout_sec=0)

    except KeyboardInterrupt:
        pass
    finally:
        rov.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
