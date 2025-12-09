import traci
import numpy as np
import math
from typing import Dict, Optional, Tuple, List, Any
import sys

# --- 1. 导入 CONFIG (快速失败) ---
from ..config import CONFIG

VEH_LENGTH = CONFIG.dcdl.VEHICLE_LENGTH_M
VEH_WIDTH = CONFIG.dcdl.VEHICLE_WIDTH_M


# --- 2. 核心 TTC  逻辑 (静态) ---

def _get_relative_kinematics(state_i: tuple, state_j: tuple) -> Tuple[float, float, float, float]:
    x_i, y_i, v_i, theta_i_rad, l_i, w_i = state_i
    x_j, y_j, v_j, theta_j_rad, l_j, w_j = state_j 

    dx = x_j - x_i
    dy = y_j - y_i

    cos_i = math.cos(theta_i_rad)
    sin_i = math.sin(theta_i_rad)

    s_t = dx * cos_i + dy * sin_i
    s_l = -dx * sin_i + dy * cos_i

    v_ix = v_i * math.cos(theta_i_rad)
    v_iy = v_i * math.sin(theta_i_rad)
    v_jx = v_j * math.cos(theta_j_rad)
    v_jy = v_j * math.sin(theta_j_rad)

    dvx = v_jx - v_ix
    dvy = v_jy - v_iy

    v_t = dvx * cos_i + dvy * sin_i
    v_l = -dvx * sin_i + dvy * cos_i

    return s_l, s_t, v_l, v_t

def calculate_2d_ttc(state_i: tuple, state_j: tuple) -> float:
    # 注意：这里 s_t 是纵向距离，s_l 是横向距离
    s_l, s_t, v_l, v_t = _get_relative_kinematics(state_i, state_j) 

    L = state_i[4] 
    W = state_i[5] 

    # --- 1. 修正后的纵向 TTC (Longitudinal) ---
    T_long = float('inf')
    
    # 前车远离 (v_t > 0): 只有当已经重叠时才危险，通常 TTC 为 inf
    if v_t > 0.01: 
        if s_t < L: T_long = 0.0 # 已经重叠
        
    # 后车接近 (v_t < 0): 关键危险场景
    elif v_t < -0.01: 
        dist_long = s_t - L 
        T_long_crit = dist_long / abs(v_t) # 使用绝对值确保时间为正
        if T_long_crit >= 0: T_long = T_long_crit
    
    # --- 2. 修正后的横向 TTC (Lateral) ---
    # 使用 s_l (横向距) 和 v_l (横向速)
    T_lat = float('inf')
    
    # 向左离去/分开
    if v_l > 0.01: 
        if abs(s_l) < W: T_lat = 0.0
        
    # 横向接近
    elif v_l < -0.01: 
        if s_l > 0:
            dist_lat = s_l - W
            T_lat_crit = dist_lat / abs(v_l)
            if T_lat_crit >= 0: T_lat = T_lat_crit
        elif s_l < 0: 
             pass 
    
    is_long_overlap = (abs(s_t) < L)
    is_lat_overlap = (abs(s_l) < W)

    ttc_2d = float('inf')

    if is_long_overlap and is_lat_overlap:
        return 0.0 
    
    if is_lat_overlap:
        ttc_2d = T_long 
    elif is_long_overlap:
        ttc_2d = T_lat 
    else:
        ttc_2d = min(T_long, T_lat)
        
    return ttc_2d

class TwoDTTC_Calculator:
    def __init__(self, time_step: float, ttc_threshold: float = 3.0, **kwargs):
        self.dt = time_step
        self.ttc_threshold = ttc_threshold
        self.L = VEH_LENGTH
        self.W = VEH_WIDTH

    def _calculate_2d_ttc_logic(self, state_i: tuple, state_j: tuple) -> float:
        return calculate_2d_ttc(state_i, state_j)

    def get_vehicle_state_dict(self, veh_id: str) -> Optional[Dict]:
        try:
            x, y = traci.vehicle.getPosition(veh_id)
            v = traci.vehicle.getSpeed(veh_id)
            angle_traci_deg = traci.vehicle.getAngle(veh_id)
            angle_math_rad = math.radians((450.0 - angle_traci_deg) % 360.0) 
            
            return {
                'id': veh_id, 'x': x, 'y': y, 'v': v, 
                'angle_rad': angle_math_rad, 'l': self.L, 'w': self.W
            }
        except traci.TraCIException:
            return None

    def get_all_vehicle_states(self, veh_ids: List[str]) -> Dict[str, Dict]:
        states = {}
        for veh_id in veh_ids:
            state = self.get_vehicle_state_dict(veh_id)
            if state:
                states[veh_id] = state
        return states