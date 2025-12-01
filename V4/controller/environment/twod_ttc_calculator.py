# -----------------------------------------------------------------
# controller/environment/twod_ttc_calculator.py
#
# [V2 - 最终清理版]
#
# 功能：
# 1. 提供 Notebook (run_comparison.ipynb ) 所需的 TwoDTTC_Calculator 类。
# 2. 【!! 核心修正 !!】将核心 TTC  逻辑 (非 TraCI 部分) 转换为静态函数，实现代码解耦。
#
# -----------------------------------------------------------------

import traci
import numpy as np
import math
from typing import Dict, Optional, Tuple, List, Any
import sys

# --- 1. 导入 CONFIG (快速失败) ---
try:
    from ..config import CONFIG
except ImportError:
    try:
        from controller.config import CONFIG
    except ImportError:
        sys.exit("致命错误: 无法在 'twod_ttc_calculator.py'  [cite: 399-439, 538-539, 668-765] 中导入 'config.py' .")

VEH_LENGTH = CONFIG.dcdl.VEHICLE_LENGTH_M
VEH_WIDTH = CONFIG.dcdl.VEHICLE_WIDTH_M


# --- 2. 核心 TTC  逻辑 (静态) ---

def _get_relative_kinematics(state_i: tuple, state_j: tuple) -> Tuple[float, float, float, float]:
    """ 
    严格来自论文 [cite: 315-319] 和您的 .py 文件 [cite: 399-439]。
    state_i, state_j 格式: (x, y, v, angle_rad, length, width)
    """
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
    """
    严格来自您的 twod_ttc_calculator.py  [cite: 399-439, 538-539, 668-765] 文件 (基于论文 Eq. 23-25 [cite: 311-313])。
    """
    try:
        s_l, s_t, v_l, v_t = _get_relative_kinematics(state_i, state_j)
    except (ValueError, ZeroDivisionError):
        return float('inf') 

    L = state_i[4] # 车辆 i 的长度 (论文 L)
    W = state_i[5] # 车辆 i 的宽度 (论文 W)

    # 2. 计算纵向 TTC (T_long, 论文 Eq. 24)
    T_long = float('inf')
    if v_l > 0.01: 
        T_long_crit = (s_l - L) / v_l 
        if T_long_crit >= 0: T_long = T_long_crit
    elif v_l < -0.01: 
        T_long_crit = (s_l + L) / v_l 
        if T_long_crit >= 0: T_long = T_long_crit
    
    # 3. 计算横向 TTC (T_lat, 论文 Eq. 25)
    T_lat = float('inf')
    if v_t > 0.01: 
        T_lat_crit = (s_t - W) / v_t
        if T_lat_crit >= 0: T_lat = T_lat_crit
    elif v_t < -0.01: 
        T_lat_crit = (s_t + W) / v_t
        if T_lat_crit >= 0: T_lat = T_lat_crit

    # 4. 确定关键维度 (C_long, C_lat, 论文 Eq. 23)
    C_long = 1.0 if abs(s_t) < W else 0.0
    C_lat = 1.0 if abs(s_l) < L else 0.0

    # 5. 计算 2D-TTC (论文 Eq. 23)
    ttc_2d = float('inf')
    if C_long > 0 and C_lat > 0: 
        ttc_2d = 0.0
    elif C_long > 0: 
        ttc_2d = min(ttc_2d, T_long)
    elif C_lat > 0: 
        ttc_2d = min(ttc_2d, T_lat)
    elif T_long != float('inf') or T_lat != float('inf'):
        ttc_2d = min(T_long, T_lat)
        
    return ttc_2d


# --- 3. Notebook 期望的 Class (适配器) ---

class TwoDTTC_Calculator:
    """
    Notebook (run_comparison.ipynb ) 的适配器类，仅用于初始化和保留接口。
    """
    def __init__(self, time_step: float, ttc_threshold: float = 3.0, **kwargs):
        self.dt = time_step
        self.ttc_threshold = ttc_threshold
        self.L = VEH_LENGTH
        self.W = VEH_WIDTH

    def _calculate_2d_ttc_logic(self, state_i: tuple, state_j: tuple) -> float:
        """ 实例方法调用静态核心逻辑 """
        return calculate_2d_ttc(state_i, state_j)

    def get_vehicle_state_dict(self, veh_id: str) -> Optional[Dict]:
        """(a) 从 TraCI 获取状态，并转换为字典"""
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
        """(b) (由 Notebook 调用) 获取所有车辆的状态字典"""
        states = {}
        for veh_id in veh_ids:
            state = self.get_vehicle_state_dict(veh_id)
            if state:
                states[veh_id] = state
        return states