
![maintext](./images/Sub_Horizon_-_Stonefish_Simulator.png)

![GIF demo](./images/floating_rov.gif)

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Build](https://img.shields.io/badge/build-passing-brightgreen)
![GPU Support](https://img.shields.io/badge/rendering-GPU%20optional-lightgrey)

## Table of Contents

- [Installation](#installation)
- [Running the Simulator](#running-the-simulator)


## Installation
![installation](./images/Installation.png)

### Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy

### Clone Repository

The cloned repository **is** the ROS2 workspace. You do not need to create your own.

SSH:
```bash
  git clone --recurse-submodules git@github.com:Joranikus/stonefish_simulator.git
```
HTTPS:
```bash
  git clone --recurse-submodules https://github.com/Joranikus/stonefish_simulator.git
```

### Updating

```bash
  sudo apt update && rosdep update
```
### Installing Dependencies

```bash
  sudo apt install -y tmux libglm-dev python3-pynput python3-opencv ros-jazzy-cv-bridge ros-jazzy-rqt-image-view libsdl2-dev

```

### Building Stonefish Library

```bash
  cd stonefish_simulator/include/stonefish
```
```bash
  mkdir build && cd build
```
```bash
  cmake ..
```
```bash
  make -j$(nproc)
```
```bash
  sudo make install
```
Return to repository root:

```bash
  cd ../../..
```

### Building The Simualtor

Needs to be run in git root:
```bash
  colcon build --symlink-install
```
```bash
  source install/setup.bash
```

### Running DEMO Script to Test

Needs to be run in git root:
```bash
  bash src/stonefish_sim/scripts/gbr_keyboard_demo.sh
```

## Running the Simulator
![runningsimulator](./images/Running_Simulator.png)

The simulator can be run either through prepared scripts or manually with custom parameters.

### Using Prepared Scripts

The simulator comes with several pre-configured scripts located in `src/stonefish_sim/scripts/`:

Demo script:
```bash
bash src/stonefish_sim/scripts/gbr_keyboard_demo.sh
```
### Manual Launch

To run scenarios manually, use the ROS2 launch command with desired parameters:

Basic launch with default parameters:
```bash
ros2 launch stonefish_sim simulation.launch.py scenario:=gbr_keyboard_demo
```
Launch with custom resolution and rendering quality
```bash
ros2 launch stonefish_sim simulation.launch.py scenario:=gbr_keyboard_demo window_res_x:=1920 window_res_y:=1080 rendering_quality:=low
```

#### Available Scenarios

The following scenarios are available:
- `gbr_keyboard_demo`: Controlling ROV with keyboard
- `gbr_pipeline`: Pipeline inspection (TAC Challenge)
- `gbr_docking`: Docking station (TAC Challenge)
- `gbr_structure`: Underwater structure inspection (TAC Challenge)

#### Available Parameters

- `scenario`: Name of the scenario file (required)
- `window_res_x`: Window width in pixels (default: 1920)
- `window_res_y`: Window height in pixels (default: 1080)
- `rendering_quality`: Graphics quality - low/medium/high (default: high)


## Repository purpose and scope

This ROS 2 Jazzy workspace simulates the GBR underwater ROV using Stonefish. It contains underwater scenarios, vehicle models, keyboard-control examples and DYNSYS exercises.

Start with [scenarios](src/stonefish_sim/scenarios/), [vehicle definitions](src/stonefish_sim/drones/) and [direct-control examples](src/gbr/direct_control/). Stonefish and its ROS 2 integration are Git submodules. For Selene surface-vessel/Autodrone simulation, see the separate [usv_simulator](https://github.com/Sub-Horizon-NTNU/usv_simulator).

Known documentation discrepancies are recorded in [docs/INCONSISTENCIES.md](docs/INCONSISTENCIES.md).
