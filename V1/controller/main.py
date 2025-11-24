import traci
import sumolib
import os
import sys
from typing import List

# =================================================================
# 【!! 关键 1: 路径注入 !!】
# =================================================================
try:
    CURRENT_DIR = os.path.dirname(__file__)
    PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
    if PROJECT_ROOT not in sys.path:
        sys.path.append(PROJECT_ROOT)
except Exception as e:
    print(f"路径注入失败: {e}")
    sys.exit(1)
# =================================================================


# =================================================================
# 【!! 关键 2: 绝对导入 !!】
# =================================================================
try:
    from controller.config import CONFIG
    from controller.controllers.lane_manager import LaneManager
    from controller.controllers.vehicle_controller import VehicleController
except ImportError as e:
    print(f"致命错误: 无法导入依赖模块。请确保:")
    print(f"  1. 您已将文件夹 '2_controller' 重命名为 'controller'。")
    print(f"  2. 'controller' 及其子目录中都有 '__init__.py' 文件。")
    print(f"  错误详情: {e}")
    sys.exit(1)
except SyntaxError as e:
    print(f"致命语法错误: {e}")
    sys.exit(1)
# =================================================================


# --- 1. 全局配置 ---

SUMO_BINARY = os.environ.get("SUMO_BIN", "sumo-gui") 
SCENARIO_PATH = os.path.join(PROJECT_ROOT, "scenario") 
SUMO_CONFIG_FILE = os.path.join(SCENARIO_PATH, "test.sumocfg") 

CONTROL_EDGES = ["2", "3", "4", "5", "6", "7", "8", "9", "10", "11"]
CONTROLLED_LANE_INDEX = 0

# --- 2. A/B 测试配置 ---
RUN_MODE = 'CUSTOM_FRAMEWORK' 

# 【!! 核心修正 !!】模拟 GMARL [cite: 383-548] 的双智能体 [cite: 398-399] 固定策略
FIXED_POLICY_M = 5 # CDL 长度 (SCLs)
FIXED_POLICY_N = 5 # HML 长度 (SCLs)

def run_simulation(run_mode: str):
    """
    运行一次完整的 SUMO 仿真。
    """
    
    # 1. 启动 SUMO
    if "SUMO_HOME" not in os.environ:
        print("错误: SUMO_HOME 环境变量未设置。")
        sys.exit(1)
        
    sumo_cmd = [
        os.path.join(os.environ["SUMO_HOME"], "bin", SUMO_BINARY),
        "-c", SUMO_CONFIG_FILE,
        "--step-length", str(CONFIG.scenario.SIM_STEP_LENGTH_S),
        "--quit-on-end",
        "--no-warnings", "true",
        "--seed", "9497" # 【!! 修正 !!】添加一个固定的种子
    ]
    
    print(f"正在启动 SUMO: {' '.join(sumo_cmd)}")
    try:
        traci.start(sumo_cmd, port=8813, label="dcdl_main_runner")
    except traci.exceptions.FatalTraCIError as e:
        print(f"致命错误: TraCI 无法连接到 SUMO。")
        print(f"  错误详情: {e}")
        sys.exit(1)
    
    # 2. 实例化控制器
    lane_manager = LaneManager(CONTROL_EDGES, CONTROLLED_LANE_INDEX)
    veh_controller = VehicleController(CONTROL_EDGES)
    
    # 3. 初始化控制器权限
    lane_manager.initialize_permissions()
    
    # 4. 应用 *固定* 的宏观策略
    print(f"--- 运行模式: {run_mode} ---")
    print(f"应用固定策略: m={FIXED_POLICY_M}, n={FIXED_POLICY_N}")
    lane_manager.apply_lane_strategy(FIXED_POLICY_M, FIXED_POLICY_N)

    # 5. 运行仿真主循环
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        # a. (宏观) 管理 HCL
        lane_manager.step()
        
        # b. (微观) 【!! A/B 测试 !!】
        if run_mode == 'CUSTOM_FRAMEWORK':
            active_hml_lanes = lane_manager.get_active_hml_lanes()
            active_cdl_lanes = lane_manager.get_active_cdl_lanes()
            cdl_start = lane_manager.get_cdl_start_edge()
            
            veh_controller.update_vehicle_states(
                active_hml_lanes, 
                active_cdl_lanes, 
                cdl_start
            )
        
        step += 1
        if step % 100 == 0:
            print(f"Simulation step: {step}")

    # 6. 关闭仿真
    traci.close()
    print("仿真结束。")


# --- 3. 脚本主入口 ---
if __name__ == "__main__":
    
    if not os.path.exists(SCENARIO_PATH):
        print(f"错误: 找不到 scenario 文件夹: {SCENARIO_PATH}")
        sys.exit(1)
        
    vtype_path = os.path.join(SCENARIO_PATH, "vTypeDistributions.add.xml")
    if not os.path.exists(vtype_path):
        print(f"错误: 找不到 {vtype_path}")
        sys.exit(1)
        
    run_simulation(RUN_MODE)