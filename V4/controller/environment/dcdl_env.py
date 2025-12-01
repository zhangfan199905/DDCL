# -----------------------------------------------------------------
# controller/environment/dcdl_env.py
#
# (V4 - 最终 GMARL 环境版)
#
# 功能：修复了所有初始化和方法调用错误，使环境与最终的 V15/V-Paper 控制器兼容。
# -----------------------------------------------------------------

import os
import sys
import traci
from controller.config import CONFIG
from controller.controllers.lane_manager import LaneManager
from controller.controllers.vehicle_controller import VehicleController
from controller.environment.reward_calculator import RewardCalculator
# from environment.graph_builder import GraphBuilder # (假设 GraphBuilder  存在于您的项目中)
import numpy as np

# 这是一个占位符，直到您提供 GraphBuilder 
class GraphBuilder:
    def __init__(self, graph_config_path):
        self.n_nodes = 30 
        self.node_feature_dim = 4
        self.adj_matrix = np.zeros((self.n_nodes, self.n_nodes))
        self.control_edges = ["2", "3", "4", "5", "6", "7", "8", "9", "10", "11"]
    def get_num_nodes(self): return self.n_nodes
    def get_sub_control_lanes(self): return self.control_edges
    def get_adjacency_matrix(self): return self.adj_matrix
    def update_detector_data(self): pass
    def get_node_features(self): return np.zeros(self.state_shape)
    def get_all_lane_ids(self): return [f"{e}_{i}" for e in self.control_edges for i in range(3)]
    def reset(self): pass

# 临时函数用于模拟配置加载，以防您原有的 load_config 不存在
def load_config(config_path):
    # 这里的 config_path  实际上是 config.py  [cite: 1-6, 668-765] 模块本身
    return {
        'project_root': os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scenario_path': 'scenario',
        'sumo_bin': 'sumo-gui',
        'sumo_cfg': 'test.sumocfg',
        'port': 8813,
        'use_gui': True,
        'sleep_time': 0.1,
        'simulation_steps': 21000,
        'control_interval_steps': 600, # 60s  [cite: 1-6, 668-765] / 0.1s  [cite: 1-6, 668-765]
        'action_space': {'min_cdl': 0, 'max_cdl': 10, 'min_hml': 0, 'max_hml': 10}
    }


class DCDLEnv:
    """
    SUMO强化学习环境，用于动态CAV专用道（DCDL）管理。
    """
    def __init__(self, config_path, graph_config_path):
        
        # 1. 配置和路径 (Config and Paths)
        config = load_config(config_path)
        self.project_root = config['project_root']
        self.scenario_path = os.path.join(self.project_root, config['scenario_path'])
        
        # 启动命令
        sumo_bin_path = os.path.join(os.environ['SUMO_HOME'], 'bin', config['sumo_bin'])
        sumo_cfg_path = os.path.join(self.scenario_path, config['sumo_cfg'])

        self.port = config['port']
        self.sumo_cmd = [sumo_bin_path, '-c', sumo_cfg_path]
        
        self.use_gui = config['use_gui']
        if self.use_gui:
            self.sumo_cmd.extend(['--quit-on-end', '--no-warnings', 'true', '--seed', '9497'])

        self.current_step = 0
        self.simulation_steps = config['simulation_steps']
        self.control_interval_steps = config['control_interval_steps']

        # 2. 初始化环境组件 (Initialize Components)
        self.graph_builder = GraphBuilder(graph_config_path) # 假设 GraphBuilder  存在
        control_edges = self.graph_builder.get_sub_control_lanes()
        
        # 【!! 修正 1 !!】 LaneManager  需要 (edges, index)
        self.lane_manager = LaneManager(control_edges, controlled_lane_index=CONFIG.get('CONTROLLED_LANE_INDEX', 0))
        
        # 【!! 修正 2 !!】 VehicleController  需要 (edges)
        self.vehicle_controller = VehicleController(control_edges)
        
        # 【!! 修正 3 !!】 RewardCalculator  [cite: 531-566, 668-765] 需要 VehicleController  实例
        self.reward_calculator = RewardCalculator(self.vehicle_controller) 

        # 3. RL 智能体和动作空间配置
        self.agents = ['cdl_agent', 'hml_agent']
        self.action_space = {
            'cdl_agent': list(range(0, CONFIG.gmarl.MAX_M_SCLS + 1)),
            'hml_agent': list(range(0, CONFIG.gmarl.MAX_N_SCLS + 1))
        }
        self.n_actions = [len(self.action_space['cdl_agent']), len(self.action_space['hml_agent'])]

        # 4. 状态空间维度
        self.n_nodes = self.graph_builder.get_num_nodes()
        self.node_feature_dim = self.graph_builder.node_feature_dim 
        self.state_shape = (self.n_nodes, self.node_feature_dim)
        self.adj_matrix = self.graph_builder.get_adjacency_matrix()

        self.sumo_process = None

    def start_sumo(self):
        """ 使用 traci.start() 启动 SUMO"""
        try:
            traci.start(self.sumo_cmd, port=self.port, label=f"sim-{self.port}")
        except traci.TraCIException as e:
            raise Exception(f"致命错误: TraCI 无法启动或连接到 SUMO。 {e}")

    def reset(self):
        """重置SUMO仿真环境。"""
        if traci.is_connected(): 
            self.close()
            
        self.start_sumo()
        self.current_step = 0
        
        # 重置控制器和管理器
        self.vehicle_controller.reset()
        self.lane_manager.initialize_permissions()
        self.reward_calculator.reset()
        self.graph_builder.reset()

        # 运行一步以初始化车辆并缓存 vType
        traci.simulationStep()
        self.current_step += 1
        
        # 获取初始状态
        state, adj = self.get_state()
        return state, adj

    def step(self, actions):
        """
        在环境中执行一个动作 (更新DCDL设置)，并推进仿真直到下一个决策点。
        """
        # 1. 执行强化学习动作 (更新CDL和HML的子车道数)
        cdl_action_idx, hml_action_idx = actions
        num_cdl_lanes = self.action_space['cdl_agent'][cdl_action_idx]
        num_hml_lanes = self.action_space['hml_agent'][hml_action_idx]

        # 【!! 修正 4 !!】 update_lane_permissions  -> apply_lane_strategy  [cite: 510-511, 668-765]
        self.lane_manager.apply_lane_strategy(num_cdl_lanes, num_hml_lanes)

        # 2. 推进仿真
        reward_accumulator = 0.0
        for _ in range(self.control_interval_steps):
            if self.current_step >= self.simulation_steps:
                break 

            # 【!! 修正 5 !!】 manage_vehicles  -> update_vehicle_states  [cite: 447-456, 668-765]
            # Custom_Framework 模式下，is_custom_framework 始终为 True
            self.vehicle_controller.update_vehicle_states(self.lane_manager.get_active_hml_lanes(), 
                                                     self.lane_manager.get_active_cdl_lanes(),
                                                     self.lane_manager.get_cdl_start_edge(),
                                                     is_custom_framework=True)
            
            self.lane_manager.step()
            traci.simulationStep()
            self.current_step += 1
            
            # 累积奖励数据
            self.reward_calculator.update_per_step()

        # 3. 获取新状态
        state, adj = self.get_state()

        # 4. 计算平均奖励 (在周期末尾计算)
        reward = self.reward_calculator.calculate_reward()

        # 5. 检查是否终止
        done = self.current_step >= self.simulation_steps

        return state, adj, reward, done

    def get_state(self):
        """
        从GraphBuilder获取当前环境状态。
        """
        self.graph_builder.update_detector_data()
        node_features = self.graph_builder.get_node_features()
        adj_matrix = self.adj_matrix 
        
        return node_features, adj_matrix

    def close(self):
        """关闭SUMO仿真。"""
        try:
            if traci.is_connected():
                traci.close()
            
            if self.sumo_process:
                self.sumo_process.terminate()
                self.sumo_process.wait()
                self.sumo_process = None
                
        except traci.TraCIException as e:
            print(f"关闭TraCI时出错: {e}")
        finally:
            self.sumo_process = None


if __name__ == '__main__':
    # ... (用于测试环境的示例代码) ...
    print("正在测试 DCDL (GMARL) 环境...")
    
    # 假定项目结构中 config.py  [cite: 1-6, 668-765] 和 graph_config.json 位于 controller 目录下
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.abspath(os.path.join(current_file_dir, '..')) # controller 目录
    
    config_file_path = os.path.join(config_dir, 'config.py')
    graph_config_file_path = os.path.join(config_dir, 'graph_config.json')

    # ... (测试环境的示例代码 - 保持不变) ...