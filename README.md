# DDCL - Dynamic CAV Dedicated Lane Management System

[中文版本](#中文说明) | [English](#overview)

A study for dynamic dedicated lane management using SUMO traffic simulation.

## Overview

This project implements a Dynamic CAV (Connected Autonomous Vehicle) Dedicated Lane (DDCL) management system using SUMO (Simulation of Urban Mobility) traffic simulation. The system intelligently manages dedicated lanes for CAVs on a highway, optimizing traffic flow, safety, and throughput.

## System Architecture

### Scenario Description

**Highway Layout (4000m total):**
- **Upstream Area** (1000m): Edge 1 - Normal 3-lane traffic
- **Control Area** (2000m): Edges 2-11 - Dynamic dedicated lane management zone (10 segments × 200m)
- **Bottleneck Area** (300m): Edge 12 - Lane drop from 3 to 2 lanes  
- **Downstream Area** (700m): Edge 13 - 3-lane recovery zone

**Lane Configuration:**
- Lane 0 (Rightmost): Controllable dedicated lane (can be CDL/HML/RML)
- Lane 1 (Middle): Buffer/transition lane
- Lane 2 (Leftmost): Regular mixed traffic lane

### Vehicle Types

1. **HV (Human Vehicle)** - vClass: custom1
   - Reaction time: 1.21s
   - Max speed: 120 km/h (33.33 m/s)
   - Length: 5.0m, Width: 1.8m
   - Ratio: ~67% of traffic

2. **CAV (Connected Autonomous Vehicle)** - vClass: custom2
   - Reaction time: 0.5s (minimal delay)
   - Max speed: 120 km/h (33.33 m/s)
   - Length: 5.0m, Width: 1.8m
   - Ratio: ~33% of traffic

### Control Modes

The system supports three experimental modes:

1. **MODE_NO_CONTROL (0)**: Mixed traffic, no dedicated lanes, SUMO default LC2013 control
2. **MODE_BASELINE (1)**: Fixed dedicated lane policy, SUMO default LC2013 control
3. **MODE_CUSTOM (2)**: Dynamic dedicated lane with custom 4-phase lane change framework

## File Structure

```
V1.6/
├── scenario/              # SUMO simulation files
│   ├── net.net.xml       # Network definition (highway geometry)
│   ├── rou.rou.xml       # Route and vehicle flow definitions
│   ├── vTypeDistributions.add.xml  # Vehicle type parameters
│   ├── det.add.xml       # Detector configurations
│   ├── test.sumocfg      # Main SUMO configuration
│   ├── viewsettings.xml  # GUI visualization settings
│   └── e2_lane_output.xml # Detector output data
│
├── controller/           # Python control system
│   ├── main.py          # Main simulation runner
│   ├── config.py        # Configuration parameters
│   ├── controllers/     # Lane and vehicle controllers
│   │   ├── lane_manager.py      # Dynamic lane management
│   │   └── vehicle_controller.py # Custom lane change logic
│   └── environment/     # Reinforcement learning environment
│       ├── dcdl_env.py          # GMARL environment
│       ├── reward_calculator.py # Reward computation
│       └── twod_ttc_calculator.py # Safety metric (2D-TTC)
│
└── analysis/            # Results and analysis
    ├── run_comparison.ipynb  # Experiment comparison notebook
    └── result_*.csv         # Performance metrics
```

## Installation & Setup

### Prerequisites

1. **SUMO** (Simulation of Urban Mobility)
   ```bash
   # Ubuntu/Debian
   sudo add-apt-repository ppa:sumo/stable
   sudo apt-get update
   sudo apt-get install sumo sumo-tools sumo-doc
   
   # Set SUMO_HOME environment variable
   export SUMO_HOME=/usr/share/sumo
   ```

2. **Python Dependencies**
   ```bash
   pip install traci numpy pandas matplotlib jupyter
   ```

### Verification

Check SUMO installation:
```bash
sumo --version
which sumo
echo $SUMO_HOME
```

## Running the Simulation

### Method 1: Using Python Controller (Recommended)

```bash
cd V1.6/controller
python main.py
```

The script will:
1. Start SUMO with TraCI interface
2. Initialize lane and vehicle controllers
3. Apply dynamic control strategies
4. Run simulation for 2100 seconds
5. Generate output files

### Method 2: Standalone SUMO

For testing network only (no dynamic control):
```bash
cd V1.6/scenario
sumo-gui -c test.sumocfg
```

### Configuration Options

Edit `V1.6/controller/main.py` to change control mode:

```python
# Line 41 - Change this value:
CURRENT_CONTROL_MODE = MODE_CUSTOM  # Options: MODE_NO_CONTROL, MODE_BASELINE, MODE_CUSTOM

# Line 44-45 - Adjust policy parameters:
FIXED_POLICY_M = 5  # CDL length (number of segments)
FIXED_POLICY_N = 5  # HML/Buffer length (number of segments)
```

## Key Features

### 1. Dynamic Lane Management

The `LaneManager` class dynamically assigns lanes as:
- **CDL (CAV Dedicated Lane)**: Only CAVs allowed
- **HML (HV-Mixed Lane)**: Transition zone for HVs to exit CDL
- **HCL (HV-Clearance Lane)**: Temporary buffer during policy changes
- **RML (Regular Mixed Lane)**: Normal mixed traffic

### 2. Custom Lane Change Framework

Four-phase intelligent lane change control:
1. **Motivation Assessment**: Calculate lane change necessity
2. **Safety Evaluation**: 2D-TTC collision risk assessment
3. **Execution**: Coordinated lane change maneuver
4. **Cooperation**: CAV-to-CAV communication for gap creation

### 3. Safety Metrics

- **2D-TTC (Two-Dimensional Time-to-Collision)**: Advanced safety metric considering both longitudinal and lateral dimensions
- **Stopping sight distance**: Prevents unsafe lane changes near exit
- **Cumulative dissatisfaction**: Manages CAV patience in HML zones

### 4. Performance Metrics

Evaluated in `analysis/run_comparison.ipynb`:
- **Throughput**: Total vehicle-distance traveled
- **Average Speed**: Mean velocity across all vehicles
- **Safety**: Average 2D-TTC values
- **Travel Time**: End-to-end journey duration

## Configuration Parameters

Key parameters from `config.py`:

```python
# Scenario
FREEWAY_LENGTH_M = 4000.0
CONTROL_AREA_M = 2000.0
DESIGN_SPEED_KMH = 120.0
NUM_HVS = 1000
NUM_CAVS = 500

# DCDL Control
SCL_LENGTH_M = 200.0  # Sub-control lane segment length
TTC_2D_THRESHOLD_S = 5.0  # Safety threshold
CUMULATIVE_DISSATISFACTION_S = 4.0  # CAV patience limit
STOPPING_DISTANCE_THRESHOLD_M = 250.0  # Exit safety margin

# Simulation
SIM_STEP_LENGTH_S = 0.1  # 100ms resolution
```

## Troubleshooting

### Common Issues

1. **"SUMO_HOME not set" error**
   ```bash
   export SUMO_HOME=/usr/share/sumo
   # Add to ~/.bashrc for persistence
   ```

2. **"TraCI connection failed"**
   - Ensure SUMO binaries are in PATH
   - Check port 8813 is not in use
   - Verify sumocfg file exists

3. **"vType not found" error**
   - Confirm vTypeDistributions.add.xml is loaded
   - Check vehicle class names (custom1/custom2)

4. **Simulation crashes**
   - Review collision settings in test.sumocfg
   - Check network connectivity (all edges connected)
   - Verify route validity

### Debug Mode

Enable verbose output:
```python
# In main.py, add to sumo_cmd:
"--verbose", "true",
"--log", "sumo.log"
```

## Research Background

This implementation is based on research in:
- Dynamic dedicated lane management for mixed CAV-HV traffic
- Graph-based multi-agent reinforcement learning (GMARL)
- 2D time-to-collision safety metrics
- Intelligent lane change coordination

### References

- SUMO Documentation: https://sumo.dlr.de/docs/
- TraCI Python API: https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
- Vehicle Type Parameters: https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html

## License

This project is for academic research purposes.

---

# 中文说明

## 项目概述

DDCL（动态CAV专用道管理系统）是基于SUMO交通仿真平台的智能交通管理项目，实现高速公路CAV（网联自动驾驶车辆）专用道的动态管理。

## 快速开始

### 环境配置

1. **安装SUMO仿真软件**
   ```bash
   # Ubuntu/Debian
   sudo add-apt-repository ppa:sumo/stable
   sudo apt-get update
   sudo apt-get install sumo sumo-tools sumo-doc
   
   # 设置环境变量
   export SUMO_HOME=/usr/share/sumo
   ```

2. **安装Python依赖**
   ```bash
   pip install traci numpy pandas matplotlib jupyter
   ```

### 运行仿真

```bash
cd V1.6/controller
python main.py
```

## 主要功能

1. **动态车道管理**：智能分配CAV专用道、缓冲区和混行车道
2. **自定义换道控制**：四阶段智能换道框架（动机评估、安全评估、执行、协作）
3. **安全评估**：2D-TTC（二维碰撞时间）安全指标
4. **性能优化**：综合优化吞吐量、速度和安全性

## 场景设计

**高速公路布局（总长4000米）：**
- **上游区**（1000米）：正常3车道交通流
- **控制区**（2000米）：动态专用道管理区域（10个路段×200米）
- **瓶颈区**（300米）：3车道缩减为2车道
- **下游区**（700米）：3车道恢复区

**车道配置：**
- 车道0（最右侧）：可控专用道（可设为CDL/HML/RML）
- 车道1（中间）：缓冲/过渡车道
- 车道2（最左侧）：普通混行车道

**车辆类型：**
- **HV（人工驾驶车辆）**：约占67%，反应时间1.21秒
- **CAV（网联自动驾驶车辆）**：约占33%，反应时间0.5秒

## 控制模式

系统支持三种实验模式：

1. **模式0（无控制）**：混行交通，无专用道，SUMO默认LC2013换道模型
2. **模式1（基线）**：固定专用道策略，SUMO默认LC2013换道模型
3. **模式2（自定义）**：动态专用道+自定义四阶段换道框架

## 配置参数

在`V1.6/controller/main.py`中修改：

```python
# 第41行 - 修改控制模式：
CURRENT_CONTROL_MODE = MODE_CUSTOM  # 选项：MODE_NO_CONTROL, MODE_BASELINE, MODE_CUSTOM

# 第44-45行 - 调整策略参数：
FIXED_POLICY_M = 5  # CDL长度（路段数）
FIXED_POLICY_N = 5  # HML/缓冲区长度（路段数）
```

## 核心配置参数

来自`config.py`的关键参数：

```python
# 场景参数
FREEWAY_LENGTH_M = 4000.0          # 高速公路总长度（米）
CONTROL_AREA_M = 2000.0            # 控制区域长度（米）
DESIGN_SPEED_KMH = 120.0           # 设计速度（公里/小时）
NUM_HVS = 1000                     # HV数量
NUM_CAVS = 500                     # CAV数量

# DCDL控制参数
SCL_LENGTH_M = 200.0               # 子控制路段长度（米）
TTC_2D_THRESHOLD_S = 5.0           # 安全阈值（秒）
CUMULATIVE_DISSATISFACTION_S = 4.0 # CAV耐心极限（秒）
STOPPING_DISTANCE_THRESHOLD_M = 250.0  # 出口安全距离（米）

# 仿真参数
SIM_STEP_LENGTH_S = 0.1            # 仿真步长（秒）
```

## 性能指标

系统在`analysis/run_comparison.ipynb`中评估以下指标：

- **吞吐量（Throughput）**：车辆总行驶距离
- **平均速度（Average Speed）**：所有车辆的平均速度
- **安全性（Safety）**：平均2D-TTC值
- **行程时间（Travel Time）**：端到端行驶时间

## 故障排除

### 常见问题

1. **"SUMO_HOME未设置"错误**
   ```bash
   export SUMO_HOME=/usr/share/sumo
   # 添加到~/.bashrc使其永久生效
   ```

2. **"TraCI连接失败"**
   - 确保SUMO二进制文件在PATH中
   - 检查端口8813未被占用
   - 验证sumocfg文件存在

3. **"vType未找到"错误**
   - 确认vTypeDistributions.add.xml已加载
   - 检查车辆类别名称（custom1/custom2）

4. **仿真崩溃**
   - 查看test.sumocfg中的碰撞设置
   - 检查网络连通性（所有边已连接）
   - 验证路径有效性

### 调试模式

启用详细输出：
```python
# 在main.py中，添加到sumo_cmd：
"--verbose", "true",
"--log", "sumo.log"
```

## 研究背景

本项目基于以下研究方向：
- 混合CAV-HV交通流的动态专用道管理
- 基于图的多智能体强化学习（GMARL）
- 二维碰撞时间（2D-TTC）安全指标
- 智能换道协调

## 技术支持

详细文档请参考：
- SUMO官方文档：https://sumo.dlr.de/docs/
- TraCI Python接口：https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
- 车辆类型参数：https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html
