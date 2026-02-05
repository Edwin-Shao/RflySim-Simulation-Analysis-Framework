# RflySim Simulation Analysis Framework

A basic framework for UAV simulation and real-flight experiments based on the RflySim platform, enabling flight control simulation, real-flight data acquisition, and consistency evaluation between simulation and real flight.

## Project Overview

This project provides a complete toolchain for:
1.  **Flight Control**: Precise UAV trajectory control using MAVROS.
2.  **Simulation Integration**: Seamless interaction with the RflySim simulation platform via MAVLink protocol for digital twin applications.
3.  **Data Analysis**: Visualization of flight trajectory errors and quantitative evaluation of consistency between simulation and real-flight data.

## Environment Dependencies

*   **Hardware Platform**: PX4
*   **Operating System**: Ubuntu 20.04 LTS (or WSL2)
*   **ROS Version**: ROS Noetic
*   **Simulation Platform**: RflySim (Windows)
*   **Dependencies**:
    *   `mavros`
    *   `rospy`
    *   `matplotlib`
    *   `pandas`

## Module Description

### 1. Core Control (`mavros_plog8`)
Responsible for controlling UAV flight trajectories.
*   Developed based on the `mavros` interface.
*   Communicates with the simulation environment via RflySim interfaces.

### 2. Data Forwarding (`mav_transfer`)
Handles data link forwarding and bridging.
*   Ensures data interoperability between the ROS environment and RflySim on Windows, enabling digital twin.

### 3. Visualization & Analysis (`plot_error`)
Trajectory and error visualization tool.
*   Plots comparison charts of desired and actual trajectories.
*   Offline visualization of position and attitude errors.

### 4. Consistency Evaluation (`calculate_group_data`)
Module for evaluating consistency between simulation and real flight.
*   Processes flight log data.
*   Calculates key metrics (e.g., RMSE).
*   Automatically exports analysis results to Excel reports.

### 5. Auxiliary Tools (`bat`)
*   Includes a series of Windows batch scripts for one-click startup of RflySim simulation scenarios and related services.

## Quick Start

> **Prerequisites**: Please ensure **ROS Noetic** is installed on Ubuntu/WSL, and **RflySim** is installed on Windows.

### Start Simulation
Run the startup scripts to initialize the RflySim scenario.
```bash
SITLRunROS.bat # Run SITL in Windows

HTTLRun.bat # Run HITL in Windows
~/mavros_run.sh 1 # Run mavros in WSL
```

### Run Forwarding Node (Windows)
```bash
python3 mav_transfer.py
```

### Run Control Node (Ubuntu/WSL)
```bash
python3 mavros_plot8.py
```
### Analysis (Windows)
```bash
python3 calculate_group_data.py
python3 plot_error.py
```

## References
*   [RflySim Official Documentation](https://www.rflysim.com/)
*   [MAVROS Documentation](http://wiki.ros.org/mavros)
*   [PX4 Open Source Project](https://px4.io/)

## License
MIT License
