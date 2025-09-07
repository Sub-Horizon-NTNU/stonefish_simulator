#!/usr/bin/env python3

TARGET_HZ = 100.0
TARGET_DT = 1.0 / TARGET_HZ

import rclpy
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from rclpy.node import Node
from packages.gbr_direct_interface import GBRDirectInterface
from pynput import keyboard

# PID Controller Class
class PID:
    def __init__(self, Kp, Ki, Kd, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, setpoint, current_value):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = np.clip(output, -self.max_output, self.max_output)
        self.prev_error = error
        return output

# Configuration
MAX_THRUST = 20.0
PLOT_WINDOW = 20.0  # seconds

# Thrust mapping dictionary
THRUST_MAP = {
    's': [20.0, 20.0, -20.0, -20.0, 0.0, 0.0, 0.0, 0.0],
    'w': [-20.0, -20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0],
    'a': [20.0, -20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0],
    'd': [-20.0, 20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],
    'q': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, 20.0, 20.0],
    'e': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, -20.0, -20.0],
    'Key.up': [0.0, 0.0, 0.0, 0.0, -20.0, -20.0, 20.0, 20.0],
    'Key.down': [0.0, 0.0, 0.0, 0.0, 20.0, 20.0, -20.0, -20.0],
    'Key.left': [0.0, 0.0, 0.0, 0.0, 20.0, -20.0, 20.0, -20.0],
    'Key.right': [0.0, 0.0, 0.0, 0.0, -20.0, 20.0, -20.0, 20.0],
    'Key.page_up': [20.0, -20.0, -20.0, 20.0, 0.0, 0.0, 0.0, 0.0],
    'Key.page_down': [-20.0, 20.0, 20.0, -20.0, 0.0, 0.0, 0.0, 0.0]
}

# Mapping from control dimensions to keys
CONTROL_MAP = {
    'surge': ('w', 's'),
    'sway': ('d', 'a'),
    'heave': ('e', 'q'),
    'roll': ('Key.right', 'Key.left'),
    'pitch': ('Key.down', 'Key.up'),
    'yaw': ('Key.page_down', 'Key.page_up')
}

class ROVController(Node):
    def __init__(self):
        super().__init__('rov_controller')
        self.rov = GBRDirectInterface(self)
        
        # Initialize PID controllers with low P values
        self.pids = {
            'surge': PID(10.0, 0.0, 0.0, MAX_THRUST),
            'sway': PID(10.0, 0.0, 0.0, MAX_THRUST),
            'heave': PID(10.0, 0.2, 0.0, MAX_THRUST),
            'roll': PID(0.1, 0.0, 0.0, MAX_THRUST),
            'pitch': PID(0.1, 0.0, 0.1, MAX_THRUST),
            'yaw': PID(0.1, 0.0, 0.0, MAX_THRUST)
        }
        
        # Setpoints
        self.setpoints = {
            'surge': 0.0, 'sway': 0.0, 'heave': 3.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        
        # Setup plots
        self.setup_plots()
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()

    def setup_plots(self):
        self.time_data = deque()
        self.data = {dim: deque() for dim in self.setpoints}
        self.setpoint_data = {dim: deque() for dim in self.setpoints}

        plt.ion()
        self.fig, axs = plt.subplots(6, 1, figsize=(10, 12))
        self.lines = {}
        
        titles = {
            'surge': 'Surge (Y)', 'sway': 'Sway (X)', 'heave': 'Heave (Z)',
            'roll': 'Roll', 'pitch': 'Pitch', 'yaw': 'Yaw'
        }
        
        for idx, dim in enumerate(['surge', 'sway', 'heave', 'roll', 'pitch', 'yaw']):
            ax = axs[idx]
            ax.set_title(titles[dim])
            ax.grid(True)
            self.lines[dim], = ax.plot([], [], label='Actual')
            self.lines[f"{dim}_sp"], = ax.plot([], [], 'r--', label='Setpoint')
            ax.legend()
            ax.set_xlim(0, PLOT_WINDOW)
        
        plt.tight_layout()

    def update_plot(self):
        t = np.array(self.time_data) - self.start_time
        
        for dim in self.setpoints:
            self.lines[dim].set_data(t, self.data[dim])
            self.lines[f"{dim}_sp"].set_data(t, self.setpoint_data[dim])
            
            ax = self.lines[dim].axes
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(max(0, t[-1] - PLOT_WINDOW) if len(t) > 0 else 0, 
                       t[-1] + 0.1 if len(t) > 0 else PLOT_WINDOW)
        
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def on_key_press(self, key):
        try:
            key_str = key.char
        except AttributeError:
            key_str = str(key)
        
        # Setpoint adjustments
        step = 0.1
        if key_str == 'w': self.setpoints['surge'] += step
        elif key_str == 's': self.setpoints['surge'] -= step
        elif key_str == 'd': self.setpoints['sway'] += step
        elif key_str == 'a': self.setpoints['sway'] -= step
        elif key_str == 'e': self.setpoints['heave'] += step
        elif key_str == 'q': self.setpoints['heave'] -= step
        elif key_str == 'Key.up': self.setpoints['pitch'] += step
        elif key_str == 'Key.down': self.setpoints['pitch'] -= step
        elif key_str == 'Key.left': self.setpoints['roll'] += step
        elif key_str == 'Key.right': self.setpoints['roll'] -= step
        elif key_str == 'Key.page_up': self.setpoints['yaw'] += step
        elif key_str == 'Key.page_down': self.setpoints['yaw'] -= step

    def run(self):
        self.start_time = time.time()
        prev_time = time.time()
        
        try:
            while rclpy.ok():
                loop_start = time.time()
                
                # Get current state
                lin_pos, ang_pos = self.rov.get_pose()
                current_state = {
                    'surge': lin_pos[1],
                    'sway': lin_pos[0],
                    'heave': lin_pos[2],
                    'roll': np.degrees(ang_pos[0]),
                    'pitch': np.degrees(ang_pos[1]),
                    'yaw': np.degrees(ang_pos[2])
                }
                
                # Compute PID outputs
                thrusters = np.zeros(8)
                for dim in self.pids:
                    output = self.pids[dim].compute(
                        self.setpoints[dim], current_state[dim]
                    )
                    pos_key, neg_key = CONTROL_MAP[dim]
                    key = pos_key if output >= 0 else neg_key
                    thrusters += np.array(THRUST_MAP[key]) * abs(output) / MAX_THRUST
                
                # Apply thrusters
                thrusters = np.clip(thrusters, -MAX_THRUST, MAX_THRUST)
                self.rov.set_thrusters(thrusters.tolist())
                
                # Update plots
                current_time = time.time()
                self.time_data.append(current_time)
                for dim in current_state:
                    self.data[dim].append(current_state[dim])
                    self.setpoint_data[dim].append(self.setpoints[dim])
                
                # Maintain plot window
                while len(self.time_data) > PLOT_WINDOW / TARGET_DT:
                    self.time_data.popleft()
                    for dim in self.data:
                        self.data[dim].popleft()
                        self.setpoint_data[dim].popleft()
                
                self.update_plot()
                
                # Frequency control
                elapsed = time.time() - loop_start
                sleep_time = TARGET_DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                rclpy.spin_once(self, timeout_sec=0)

        except KeyboardInterrupt:
            pass
        finally:
            self.rov.stop()
            self.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    controller = ROVController()
    controller.run()