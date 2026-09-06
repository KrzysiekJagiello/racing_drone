# ROS 2 ArduPilot SITL Template

A template repository for running ArduPilot SITL simulations with Gazebo and MAVROS in a ROS 2 environment, based on DevContainers.

## Prerequisites

* Docker
* Visual Studio Code
* Dev Containers extension for VS Code

## Quick Start

1. Clone the repository as a template.
2. Open the project folder in VS Code.
3. Accept the "Reopen in Container" prompt (or press F1 and type "Dev Containers: Reopen in Container").
4. Wait for the container to build. The environment will mount the local folder as a ROS 2 workspace.

## Running the Simulation

Before running for the first time, build the package:

```bash
colcon build --symlink-install
source install/setup.bash
```

Run the simulation using the helper script:
```bash
./start.sh
```

The scritp initializes:
* Gazebo simulator (iris_runway.sdf world)
* ArduPilot SITL flight controller
* MAVROS node integrating the system with ROS 2

## Directory Structure
* .devcontainer/ - Container environment definition for VS Code
* src/sitl_launcher/ - ROS 2 package containing .launch.py files and parameter configuration (including use_sim_time synchronization)
* start.sh - Startup script.
