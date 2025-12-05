import traci
import os
import sys
from typing import List

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR) 
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from controller.config import CONFIG
from controller.controllers.lane_manager import LaneManager
from controller.controllers.vehicle_controller import (
    VehicleController, 
    MODE_NO_CONTROL, 
    MODE_BASELINE, 
    MODE_CUSTOM
)

# =================================================================
# 1. 全局仿真配置
# =================================================================

SUMO_BINARY = os.environ.get("SUMO_BIN", "sumo-gui") 
SCENARIO_PATH = os.path.join(PROJECT_ROOT, "scenario") 
SUMO_CONFIG_FILE = os.path.join(SCENARIO_PATH, "test.sumocfg") 

# 控制区域定义
CONTROL_EDGES = ["2", "3", "4", "5", "6", "7", "8", "9", "10", "11"]
CONTROLLED_LANE_INDEX = 0
MIDDLE_LANE_INDEX = 1

# =================================================================
# 2. 实验核心配置
# =================================================================

# --- 切换实验模式 ---
# MODE_NO_CONTROL (0): 混行，无专用道，SUMO 默认控制
# MODE_BASELINE   (1): 有专用道，无智能算法，SUMO 默认控制 (LC2013)
# MODE_CUSTOM     (2): 有专用道，使用自定义四阶段换道框架
CURRENT_CONTROL_MODE = MODE_BASELINE 

# --- 宏观策略参数 ---
FIXED_POLICY_M = 5  # CDL 长度
FIXED_POLICY_N = 5  # HML/Buffer 长度

# =================================================================
# 3. 仿真主流程
# =================================================================

def run_simulation(control_mode: int):
    # --- 1. 启动 SUMO ---
    if "SUMO_HOME" not in os.environ:
        print("错误: SUMO_HOME 环境变量未设置。")
        sys.exit(1)
        
    sumo_cmd = [
        os.path.join(os.environ["SUMO_HOME"], "bin", SUMO_BINARY),
        "-c", SUMO_CONFIG_FILE,
        "--step-length", str(CONFIG.scenario.SIM_STEP_LENGTH_S),
        "--quit-on-end",
        "--no-warnings", "true",
        "--seed", "9497"
    ]
    
    print(f"\n>>> 正在启动 SUMO... 模式: {control_mode}")
    try:
        traci.start(sumo_cmd, port=8813, label="dcdl_main_runner")
    except traci.exceptions.FatalTraCIError as e:
        print(f"致命错误: TraCI 无法连接到 SUMO。详情: {e}")
        sys.exit(1)
    
    # --- 2. 实例化控制器 ---
    lane_manager = LaneManager(CONTROL_EDGES, CONTROLLED_LANE_INDEX, MIDDLE_LANE_INDEX)
    veh_controller = VehicleController(CONTROL_EDGES)
    
    # --- 3. 初始化/重置路网策略 (Pre-Simulation) ---
    
    if control_mode == MODE_NO_CONTROL:
        print(">>> [Mode 0] 初始化: 重置为混行模式，清除所有专用道限制。")
        lane_manager.reset_to_mixed_traffic()

    elif control_mode == MODE_BASELINE:
        print(f">>> [Mode 1] 初始化: 应用固定策略 (m={FIXED_POLICY_M}, n={FIXED_POLICY_N})")
        lane_manager.apply_lane_strategy(FIXED_POLICY_M, FIXED_POLICY_N)
        
    else: 
        print(f">>> [Mode {control_mode}] 初始化: 应用固定策略 (m={FIXED_POLICY_M}, n={FIXED_POLICY_N})")
        lane_manager.apply_lane_strategy(FIXED_POLICY_M, FIXED_POLICY_N)

    # --- 4. 仿真主循环 ---
    
    step = 0
    
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            
            # --- 基础设施层 (Lane Manager) ---
            if control_mode != MODE_NO_CONTROL:
                # Mode 1 & 2: 必须运行，因为需要清理 HCL 上的滞留车辆
                lane_manager.step()

            # --- 车辆控制层 (Vehicle Controller) ---
            if control_mode == MODE_CUSTOM:
                # 仅 Mode 2 需要运行 VehicleController
                # Mode 0 & 1 均由 SUMO 默认 (LC2013) 接管

                active_hml = lane_manager.get_active_hml_lanes()
                active_cdl = lane_manager.get_active_cdl_lanes()
                active_ml = lane_manager.get_active_middle_lanes()
                start_edge = lane_manager.get_cdl_start_edge()

                veh_controller.update_vehicle_states(
                    hml_lanes=active_hml,
                    cdl_lanes=active_cdl,
                    ml_lanes=active_ml,
                    cdl_start_edge=start_edge,
                    control_mode=MODE_CUSTOM
                )
            elif control_mode == MODE_NO_CONTROL:
                veh_controller.enforce_nocontrol_mode()
            elif control_mode == MODE_BASELINE:
                veh_controller.enforce_baseline_mode()

            # --- 推进仿真 ---
            traci.simulationStep()
            step += 1
            
            if step % 1000 == 0:
                print(f"Simulation step: {step}")

    except traci.TraCIException as e:
        print(f"运行时错误 (TraCI): {e}")
    finally:
        traci.close()
        print("仿真结束。")

if __name__ == "__main__":
    if not os.path.exists(SUMO_CONFIG_FILE):
        print(f"警告: 找不到配置文件 {SUMO_CONFIG_FILE}")
            
    run_simulation(CURRENT_CONTROL_MODE)