# -----------------------------------------------------------------
# 2_controller/environment/reward_calculator.py
# (V2 - 修正导入)
#
# 功能：严格按照论文 4.1.4 [cite: 440-446] 计算 GMARL 奖励。
#       导入本地的 2d_ttc_calculator.py  计算 R_saf [cite: 442-443]。
#
# -----------------------------------------------------------------

import traci
import numpy as np
import math
from typing import List, Dict, Optional

# 【!! 关键修正 !!】
# 使用相对导入 ".." 来返回上一级 (2_controller/) 目录
# 使用相对导入 "." 来导入同级目录 (environment/) 中的模块
try:
    from ..config import CONFIG
    from .twod_ttc_calculator import calculate_2d_ttc
except ImportError:
    print("错误: 无法导入 ..config 或 .2d_ttc_calculator。请确保 __init__.py 文件存在。")
    # (临时的 CONFIG 结构)
    class DotDict(dict): __getattr__ = dict.get
    CONFIG = DotDict({
        "gmarl": DotDict({
            "REWARD_ALPHA": 1.0, "REWARD_BETA": 1.0, "REWARD_GAMMA": 1.0,
            "DECISION_CYCLE_S": 60.0
        }),
        "scenario": DotDict({"DESIGN_SPEED_MS": 33.33, "SIM_STEP_LENGTH_S": 0.1}),
        "dcdl": DotDict({"VEHICLE_LENGTH_M": 5.0, "VEHICLE_WIDTH_M": 1.8})
    })
    def calculate_2d_ttc(state_i, state_j): return 10.0 # 占位符


class RewardCalculator:
    """
    在每个决策周期 [cite: 1-6] 结束时计算 GMARL 奖励 [cite: 440-446]。
    """
    def __init__(self, detector_ids: List[str]):
        self.detector_ids = detector_ids 
        self.reset()
        self.TTC_SAFETY_CAP = 10.0 # 用于归一化的TTC上限 (例如10秒)

    def reset(self):
        """重置内部累加器"""
        self.cumulative_speed = 0.0
        self.vehicle_speed_samples = 0
        self.total_vehicle_distance = 0.0

    def update_per_step(self):
        """在 *每个仿真步* (0.1s) [cite: 567-568] 调用，以累积数据。"""
        active_vehicles = traci.vehicle.getIDList()
        
        for veh_id in active_vehicles:
            try:
                speed = traci.vehicle.getSpeed(veh_id)
                self.cumulative_speed += speed
                self.vehicle_speed_samples += 1
                distance_travelled = speed * CONFIG.scenario.SIM_STEP_LENGTH_S
                self.total_vehicle_distance += distance_travelled
            except traci.TraCIException:
                continue # 车辆可能刚刚离开

    def calculate_reward(self, all_vehicles: List[str]) -> float:
        """在 *决策周期* [cite: 1-6] 末尾调用，计算最终奖励。"""
        
        # 1. 计算 R_thr (吞吐量) [cite: 442-443]
        total_distance = self.total_vehicle_distance
        
        # 2. 计算 R_spd (平均速度) [cite: 442-443]
        avg_speed = self.cumulative_speed / self.vehicle_speed_samples if self.vehicle_speed_samples > 0 else 0.0
        
        # 3. 计算 R_saf (安全 - 平均 2D-TTC) [cite: 315-319, 442-443]
        # 【!! 性能瓶颈开始 !!】
        avg_ttc = self._calculate_avg_2d_ttc(all_vehicles)
        # 【!! 性能瓶颈结束 !!】

        # 4. 归一化 (论文 Eq. 49-51 [cite: 442-446])
        max_possible_distance = (
            (self.vehicle_speed_samples / (CONFIG.gmarl.DECISION_CYCLE_S / CONFIG.scenario.SIM_STEP_LENGTH_S)) * CONFIG.scenario.DESIGN_SPEED_MS * CONFIG.gmarl.DECISION_CYCLE_S
        )
        norm_thr = total_distance / max_possible_distance if max_possible_distance > 0 else 0
        norm_spd = avg_speed / CONFIG.scenario.DESIGN_SPEED_MS
        clipped_ttc = min(self.TTC_SAFETY_CAP, max(0.0, avg_ttc))
        norm_saf = clipped_ttc / self.TTC_SAFETY_CAP
        
        # 5. 计算最终奖励 (论文 Eq. 52 [cite: 440-446])
        reward = (
            CONFIG.gmarl.REWARD_ALPHA * norm_thr +
            CONFIG.gmarl.REWARD_BETA * norm_spd +
            CONFIG.gmarl.REWARD_GAMMA * norm_saf
        )
        
        self.reset()
        return reward

    def _get_vehicle_kinematic_state(self, veh_id: str) -> Optional[tuple]:
        """【缓慢的 TraCI 调用】获取 2D-TTC [cite: 315-319] 所需的完整状态"""
        try:
            pos = traci.vehicle.getPosition(veh_id)
            angle = traci.vehicle.getAngle(veh_id)
            speed = traci.vehicle.getSpeed(veh_id)
            length = traci.vehicle.getLength(veh_id)
            width = traci.vehicle.getWidth(veh_id)
            
            math_angle_deg = (450 - angle) % 360
            math_angle_rad = math.radians(math_angle_deg)
            
            return (pos[0], pos[1], speed, math_angle_rad, length, width)
        except traci.TraCIException:
            return None

    def _calculate_avg_2d_ttc(self, all_vehicles: List[str]) -> float:
        """【性能密集型】计算精确的 2D-TTC [cite: 315-319]"""
        if len(all_vehicles) < 2:
            return self.TTC_SAFETY_CAP

        ttc_samples = []
        vehicle_states = {}
        for veh_id in all_vehicles:
            state = self._get_vehicle_kinematic_state(veh_id)
            if state:
                vehicle_states[veh_id] = state

        vehicle_ids = list(vehicle_states.keys())
        for i in range(len(vehicle_ids)):
            veh_id_i = vehicle_ids[i]
            if veh_id_i not in vehicle_states: continue
            state_i = vehicle_states[veh_id_i]
            
            try:
                leader = traci.vehicle.getLeader(veh_id_i, 100.0) 
                if leader:
                    veh_id_j = leader[0]
                    if veh_id_j in vehicle_states:
                        state_j = vehicle_states[veh_id_j]
                        ttc = calculate_2d_ttc(state_i, state_j)
                        if ttc < float('inf'):
                            ttc_samples.append(ttc)
            except traci.TraCIException:
                continue 

        if not ttc_samples:
            return self.TTC_SAFETY_CAP 
        
        return np.mean(ttc_samples)