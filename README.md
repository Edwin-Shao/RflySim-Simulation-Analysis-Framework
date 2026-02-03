# RflySim Simulation Analysis Framework

基于 RflySim 平台开发的无人机仿真与实飞实验基本框架，实现飞行控制仿真、实飞数据采集以及仿真与实飞的一致性评估。

## 项目简介

本项目提供了一套完整的工具链，用于：
1.  **飞行控制**：利用 MAVROS 实现精确的无人机轨迹控制。
2.  **仿真对接**：通过 MAVLink 协议与 RflySim 仿真平台无缝交互。
3.  **数据分析**：可视化飞行轨迹误差，并定量计算仿真与实飞数据的一致性。

## 环境依赖

*   **硬件平台**: PX4
*   **操作系统**: Ubuntu 20.04 LTS (或 WSL2)
*   **ROS 版本**: ROS Noetic
*   **仿真平台**: RflySim (Windows)
*   **依赖库**:
    *   `mavros`
    *   `rospy`
    *   `matplotlib`
    *   `pandas`

## 模块说明

### 1. 核心控制 (`mavros_plog8`)
负责控制无人机飞行轨迹的核心模块。
*   基于 `mavros` 接口开发。
*   通过 RflySim 提供的接口与仿真环境通信。

### 2. 数据转发 (`mav_transfer`)
负责数据链路的转发与桥接。
*   确保 ROS 环境与 RflySim Windows 端的数据互通。

### 3. 可视化分析 (`plot_error`)
轨迹与误差可视化工具。
*   绘制期望轨迹与实际轨迹的对比图。
*   离线展示位置与姿态误差。

### 4. 一致性评估 (`calculate_group_data`)
仿真与实飞一致性计算模块。
*   处理飞行日志数据。
*   计算关键指标（如均方根误差 RMSE 等）。
*   自动导出分析结果至 Excel 报表。

### 5. 辅助工具 (`bat`)
*   包含一系列 Windows 批处理脚本，用于一键快速启动 RflySim 仿真场景及相关服务。

## 快速开始

> **前置条件**：请确保您已在 Ubuntu/WSL 系统中安装 **ROS Noetic**，并在 Windows 系统中安装 **RflySim** 仿真平台。

### 启动仿真
运行启动脚本，初始化 RflySim 场景。
```bash
SITLRunROS.bat # Run SITL in Windows

HTTLRun.bat # Run HITL in Windows
~/mavros_run.sh 1 # Run mavros in WSL
```

### 运行控制节点 (Ubuntu/WSL)
```bash
python3 mavros_plog8.py
```

## 参考文档
*   [RflySim 官方文档](https://www.rflysim.com/)
*   [MAVROS 文档](http://wiki.ros.org/mavros)
*   [PX4 开源项目](https://px4.io/)

## License
MIT License
