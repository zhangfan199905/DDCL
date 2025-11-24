# -----------------------------------------------------------------
# 2_controller/controllers/vehicle_controller.py
#
# (V15 - 最终逻辑修正版)
#
# 功能：
# 1. 【!! 拥堵核心修正 !!】为 HML  区域的 HV  引入 2.5s 的柔性安全阈值，
#    解决换道过于保守导致的拥堵 (解决了 54k 步问题)。
# 2. 【!! 理论修正 !!】 HML  上的 HV  换道动机硬编码为 1.0 (义务)。
# 3. 【!! 代码去重 !!】 删除了重复的 2D-TTC  逻辑，改为从 twod_ttc_calculator.py  [cite: 399-439, 538-539, 668-765] 导入。
# 4. (保留V14修正) 修复了所有 TraCI 订阅崩溃问题。
#
# -----------------------------------------------------------------
import traci
import traci.constants as tc
import math
import random 
from typing import Dict, Optional, Tuple, Set, List
import sys

# -----------------------------------------------------------------
# 1. 导入 Config (快速失败) 和 TTC  核心函数
# -----------------------------------------------------------------
try:
    from ..config import CONFIG
    from ..environment.twod_ttc_calculator import calculate_2d_ttc
except ImportError:
    try:
        from controller.config import CONFIG
        from controller.environment.twod_ttc_calculator import calculate_2d_ttc
    except ImportError:
        print("="*50)
        print("致命错误 (FATAL ERROR): 无法在 'vehicle_controller.py'  中导入 'config.py'  或 TTC  模块。")
        print("="*50)
        raise 

# -----------------------------------------------------------------
# 2. 常量定义
# -----------------------------------------------------------------
_LC_DIR_LEFT = -1
_LC_DIR_RIGHT = 1
_SUMO_DEFAULT_LC_MODE = 1621  # SUMO 默认的 LC2013 模式 [cite: 15-111] (允许所有自主换道)
_TRACI_LC_MODE = 0            # 禁止所有自主换道，由TraCI完全控制
_TRACI_FORCED_LC_MODE = 256   # TraCI受防碰限制的控制
_TTC_HML_FLEXIBLE_THRESHOLD = 2.5 # HML  上强制换道的柔性阈值 (2.5s)

# VARS_TO_SUBSCRIBE  只包含“变量”
VARS_TO_SUBSCRIBE = [
    tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ANGLE, tc.VAR_ROAD_ID,
    tc.VAR_LANE_ID, tc.VAR_LANE_INDEX, tc.VAR_LANEPOSITION, tc.VAR_TYPE,
]


class VehicleController:
    """
    实现论文 3.3 节 [cite: 262-382] 定义的微观换道控制逻辑。
    使用高性能订阅模式和 2D-TTC  决策。
    """
    
    def __init__(self, control_edge_ids: List[str]):
        # 1. 区域定义
        self.CONTROL_EDGE_IDS: List[str] = control_edge_ids
        self.edge_to_order: Dict[str, int] = {edge_id: i for i, edge_id in enumerate(self.CONTROL_EDGE_IDS)}
        
        # 2. 内部内存和参数
        self.cav_dissatisfaction_memory: Dict[str, float] = {}
        self.cav_cdl_distance_memory: Dict[str, float] = {}
        self.controlled_veh_ids: Set[str] = set()
        
        self.SIM_STEP_S = CONFIG.scenario.SIM_STEP_LENGTH_S
        self.TTC_THRESHOLD = CONFIG.dcdl.TTC_2D_THRESHOLD_S 
        self.CUMULATIVE_DISSATISFACTION_S = CONFIG.dcdl.CUMULATIVE_DISSATISFACTION_S
        self.SPEED_DIFF_THRESHOLD_MS = CONFIG.dcdl.SPEED_DIFF_THRESHOLD_MS
        self.STOPPING_DISTANCE_THRESHOLD_M = CONFIG.dcdl.STOPPING_DISTANCE_THRESHOLD_M
        self.SCL_LENGTH_M = CONFIG.dcdl.SCL_LENGTH_M
        self.HV_BRAKE_REACTION_TIME_S = CONFIG.dcdl.HV_BRAKE_REACTION_TIME_S
        self.L = CONFIG.dcdl.VEHICLE_LENGTH_M
        self.W = CONFIG.dcdl.VEHICLE_WIDTH_M

        # 3. 缓存 vType 参数
        self.vtype_cache: Dict[str, Dict] = {}
        self._cache_vtype_parameters()

        # 4. 订阅管理
        self.subscribed_vehicles: Set[str] = set()
        self.all_veh_data: Dict[str, Dict] = {} 

        try:
            traci.simulation.subscribe([tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS])
        except traci.TraCIException: pass 

    def _cache_vtype_parameters(self):
        """缓存 vType 参数 (解决 vtype 属性不能订阅的问题)"""
        print("VehicleController: 正在缓存 vType 参数...")
        try:
            all_types = traci.vehicletype.getIDList()
            for type_id in all_types:
                self.vtype_cache[type_id] = {
                    'length': traci.vehicletype.getLength(type_id),
                    'width': traci.vehicletype.getWidth(type_id),
                    'accel': traci.vehicletype.getAccel(type_id),
                    'decel': traci.vehicletype.getDecel(type_id),
                    'tau': traci.vehicletype.getTau(type_id),
                    'minGap': traci.vehicletype.getMinGap(type_id)
                }
        except traci.TraCIException as e:
            raise

    def _handle_subscriptions(self):
        """管理进入和离开的车辆的订阅"""
        sim_results = traci.simulation.getSubscriptionResults()
        if not sim_results: return

        # 1. 移除已离开的车辆 (ARRIVED = 离开)
        for veh_id in sim_results.get(tc.VAR_ARRIVED_VEHICLES_IDS, []):
            self.subscribed_vehicles.discard(veh_id)
            self.controlled_veh_ids.discard(veh_id)
            if veh_id in self.cav_dissatisfaction_memory: del self.cav_dissatisfaction_memory[veh_id]
            if veh_id in self.cav_cdl_distance_memory: del self.cav_cdl_distance_memory[veh_id]

        # 2. 为新生成的车辆添加订阅 (DEPARTED = 进入)
        for veh_id in sim_results.get(tc.VAR_DEPARTED_VEHICLES_IDS, []):
            if veh_id not in self.subscribed_vehicles:
                try:
                    traci.vehicle.subscribe(veh_id, VARS_TO_SUBSCRIBE)
                    traci.vehicle.subscribeLeader(veh_id, 100.0) 
                    self.subscribed_vehicles.add(veh_id)
                except traci.TraCIException: pass 

    def update_vehicle_states(self, hml_lanes: Set[str], cdl_lanes: Set[str], cdl_start_edge: Optional[str], is_custom_framework: bool = True):
        self._handle_subscriptions()
        self.all_veh_data = traci.vehicle.getSubscriptionResults(None)
        
        dcdl_lanes_combined = hml_lanes.union(cdl_lanes)
        current_dcdl_veh_ids = {
            veh_id for veh_id, data in self.all_veh_data.items() 
            if data and data.get(tc.VAR_LANE_ID) in dcdl_lanes_combined
        }

        # 1. 恢复 LC2013 [cite: 15-111] (车辆离开DCDL区域时)
        released_vehicles = self.controlled_veh_ids - current_dcdl_veh_ids
        for veh_id in released_vehicles:
            try:
                traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)
            except traci.TraCIException: pass
        
        self.controlled_veh_ids = current_dcdl_veh_ids

        # 2. 【!! 核心 !!】基线模式下，只需确保所有车使用默认换道模式
        if not is_custom_framework:
            for veh_id in current_dcdl_veh_ids:
                try:
                    traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)
                except traci.TraCIException: pass
            return

        # --- 自定义框架模式下的逻辑 ---
        # 3. 禁用自主换道 (新进入DCDL的车辆)
        newly_controlled = current_dcdl_veh_ids - self.controlled_veh_ids
        for veh_id in newly_controlled:
            try:
                traci.vehicle.setLaneChangeMode(veh_id, _TRACI_LC_MODE) # 模式 0: 禁止所有自主换道
            except traci.TraCIException: pass
        
        # 4. 对所有受控车辆执行决策
        for veh_id in self.controlled_veh_ids:
            try:
                veh_data = self.all_veh_data[veh_id] 
                self.manage_dcdl_lane_changing(veh_id, veh_data, hml_lanes, cdl_lanes, cdl_start_edge)
            except (KeyError, traci.TraCIException):
                pass
    
    def manage_dcdl_lane_changing(self, veh_id: str, veh_data: Dict, hml_lanes: Set[str], cdl_lanes: Set[str], cdl_start_edge: Optional[str]):
        """实现论文 3.3 节 [cite: 262-382] 的四阶段换道框架，并柔性执行必要换道。"""
        current_lane = veh_data.get(tc.VAR_LANE_ID)
        if not current_lane: return

        prob_lc, coop_request, target_lane, direction = self._get_lane_change_motivation(
            veh_id, veh_data, current_lane, hml_lanes, cdl_lanes, cdl_start_edge
        )

        if prob_lc == 0.0 or target_lane == -1:
            return 
        
        if coop_request < 1.0 and random.random() >= prob_lc:
            return 

        # 【!! 修正 B: 核心安全检查 !!】
        # 使用动态阈值 (2.5s for HV  on HML )
        is_safe, gaps = self._assess_lane_change_safety_2D_TTC(veh_id, veh_data, direction, hml_lanes)

        if is_safe:
            traci.vehicle.changeLane(veh_id, target_lane, duration=1.0)
            return

        # 尝试协作 (如果安全检查失败，且请求了协作)
        if coop_request == 1.0: 
            self.attempt_lane_change_cooperation(veh_id, veh_data, gaps)

    # -----------------------------------------------------
    # 阶段 1 & 2: 换道动机与决策 (0-1 概率模型)
    # -----------------------------------------------------

    def _get_lane_change_motivation(self, veh_id: str, veh_data: Dict, current_lane: str, hml_lanes: Set[str], cdl_lanes: Set[str], cdl_start_edge: Optional[str]) -> Tuple[float, float, int, int]:
        """计算换道动机。(使用订阅数据，无TraCI调用)"""
        
        v_type = veh_data.get(tc.VAR_TYPE, "")
        is_hv = v_type.startswith("hv")
        is_cav = not is_hv
        
        prob_lc = 0.0
        coop_request = 0.0
        
        current_lane_index = veh_data.get(tc.VAR_LANE_INDEX, -1)
        
        if current_lane_index > 0:
            target_lane_index = current_lane_index - 1
            direction = _LC_DIR_LEFT 
        else:
            return 0.0, 0.0, -1, 0 

        # 【!! 修正 1: HV on HML  只有义务 (强制) !!】
        if is_hv and current_lane in hml_lanes:
            # 强制换道，不需评估动机。
            prob_lc = 1.0 
            coop_request = 1.0 
        
        elif is_cav and current_lane in hml_lanes:
            prob_lc = self._calculate_cav_hml_motivation(veh_id, veh_data, target_lane_index)
            coop_request = 1.0 if prob_lc == 1.0 else 0.0 
            
        elif is_cav and current_lane in cdl_lanes:
            prob_lc, p_safe = self._calculate_cav_cdl_motivation(veh_id, veh_data, target_lane_index, cdl_lanes, cdl_start_edge)
            coop_request = 1.0 if p_safe == 1.0 else 0.0 
        
        return prob_lc, coop_request, target_lane_index, direction

    def _calculate_cav_hml_motivation(self, veh_id: str, veh_data: Dict, target_lane_index: int) -> float:
        """(实现 Eq. 17-19 [cite: 281-293]) (使用订阅数据，无TraCI调用)"""
        v_current = veh_data.get(tc.VAR_SPEED, 0.0)
        v_desired = veh_data.get(tc.VAR_ALLOWED_SPEED, v_current)
        
        if v_desired > 1.0:
            dissatisfaction = max(0, (v_desired - v_current) / v_desired) * self.SIM_STEP_S
        else:
            dissatisfaction = 0.0
        
        current_total_d = self.cav_dissatisfaction_memory.get(veh_id, 0.0) + dissatisfaction
        self.cav_dissatisfaction_memory[veh_id] = current_total_d

        prob_eff = min(1.0, current_total_d / self.CUMULATIVE_DISSATISFACTION_S)

        if prob_eff < 1.0:
            return prob_eff 

        if self._get_speed_gain(veh_id, veh_data, target_lane_index) > self.SPEED_DIFF_THRESHOLD_MS:
            self.cav_dissatisfaction_memory[veh_id] = 0.0 
            return 1.0 
        
        return 0.0 
        
    def _calculate_cav_cdl_motivation(self, veh_id: str, veh_data: Dict, target_lane_index: int, cdl_lanes: Set[str], cdl_start_edge: Optional[str]) -> Tuple[float, float]:
        """(实现 Eq. 20-22 [cite: 294-307]) (使用订阅数据)"""
        
        prob_eff = self._calculate_cav_hml_motivation(veh_id, veh_data, target_lane_index)
        
        l_cdl_total = len(cdl_lanes) * self.SCL_LENGTH_M 
        d_cav = self._update_and_get_cdl_distance(veh_id, veh_data, cdl_start_edge)
        d_stop_bar = self.STOPPING_DISTANCE_THRESHOLD_M
        
        l_remaining = l_cdl_total - d_cav
        if l_cdl_total <= 0: l_remaining = d_stop_bar 

        prob_safe = 1.0 - min(1.0, max(0.0, l_remaining / d_stop_bar))
        
        prob_total = 1.0 - (1.0 - prob_eff) * (1.0 - prob_safe)
        
        return prob_total, prob_safe

    # -----------------------------------------------------
    # 阶段 3 & 4: 评估与协作 (使用 2D-TTC)
    # -----------------------------------------------------

    def _assess_lane_change_safety_2D_TTC(self, veh_id: str, veh_data: Dict, direction: int, hml_lanes: Set[str]) -> Tuple[bool, dict]:
        """
        【!! V15 修正 !!】
        使用 2D-TTC  进行安全决策，采用动态阈值。
        """
        
        try:
            lc_state_tuple = traci.vehicle.getLaneChangeState(veh_id, direction)
        except traci.TraCIException:
            return False, {} 
            
        tp_id = lc_state_tuple[7] 
        tf_id = lc_state_tuple[9] 
        gaps = {"tp": tp_id, "tf": tf_id}
        
        state_i = self._format_state_from_sub(veh_id, veh_data)
        if state_i is None: return False, gaps 

        # 3. 动态阈值定义 (解决拥堵核心)
        current_lane = veh_data.get(tc.VAR_LANE_ID)
        v_type = veh_data.get(tc.VAR_TYPE, "")
        is_hv_on_hml = v_type.startswith("hv") and current_lane in hml_lanes
        
        # 强制换道的柔性阈值 (2.5s)；CAV 和非强制换道使用严格的 5.0s 
        effective_ttc_threshold = _TTC_HML_FLEXIBLE_THRESHOLD if is_hv_on_hml else self.TTC_THRESHOLD

        # 4. 检查与前车 (tp) 的 2D-TTC 
        ttc_tp = float('inf')
        if tp_id:
            tp_data = self.all_veh_data.get(tp_id) 
            if tp_data:
                state_j = self._format_state_from_sub(tp_id, tp_data)
                if state_j:
                    ttc_tp = calculate_2d_ttc(state_i, state_j)

        # 5. 检查与后车 (tf) 的 2D-TTC 
        ttc_tf = float('inf')
        if tf_id:
            tf_data = self.all_veh_data.get(tf_id)
            if tf_data:
                state_k = self._format_state_from_sub(tf_id, tf_data)
                if state_k:
                    ttc_tf = calculate_2d_ttc(state_k, state_i)
        
        # 6. 决策
        is_safe = (ttc_tp > effective_ttc_threshold and ttc_tf > effective_ttc_threshold)
        
        return is_safe, gaps

    def attempt_lane_change_cooperation(self, veh_id: str, veh_data: Dict, gaps: dict):
        """(实现 论文 3.3.4 节 [cite: 330-382]) (使用订阅数据+缓存)"""
        tp_id = gaps.get("tp")
        tf_id = gaps.get("tf")
        
        tp_data = self.all_veh_data.get(tp_id) if tp_id else None
        tf_data = self.all_veh_data.get(tf_id) if tf_id else None
        
        tp_is_cav = tp_data and tp_data.get(tc.VAR_TYPE, "").startswith("cav")
        tf_is_cav = tf_data and tf_data.get(tc.VAR_TYPE, "").startswith("cav")
        
        coop_target_id = None
        coop_action = None
        
        if not tp_id and tf_is_cav: 
            coop_target_id = tf_id
            coop_action = "decel"
        elif not tf_id and tp_is_cav:
            coop_target_id = tp_id
            coop_action = "accel"
        elif tp_is_cav: 
            coop_target_id = tp_id
            coop_action = "accel" 
        elif tf_is_cav: 
            coop_target_id = tf_id
            coop_action = "decel" 
        
        if coop_target_id is None:
            return 

        coop_data = self.all_veh_data.get(coop_target_id)
        if not coop_data:
            return

        if not self._check_cooperation_safety_proxy(coop_target_id, coop_data):
            return 

        try:
            coop_duration = self.SIM_STEP_S * 5
            
            coop_vtype = coop_data.get(tc.VAR_TYPE)
            type_params = self.vtype_cache.get(coop_vtype)
            
            if not type_params:
                return 

            if coop_action == "accel":
                accel = type_params['accel']
                traci.vehicle.setAcceleration(coop_target_id, accel, coop_duration)
            
            elif coop_action == "decel":
                decel = type_params['decel']
                traci.vehicle.setAcceleration(coop_target_id, -decel, coop_duration)
        
        except traci.TraCIException:
            pass 
            
    # =================================================================
    # 内部辅助函数 (Internal Helper Functions)
    # =================================================================
    
    def _format_state_from_sub(self, veh_id: str, veh_data: Dict) -> Optional[tuple]:
        """
        从订阅数据字典 和 vtype_cache  构建 2D-TTC [cite: 315-319, 399-439] 所需的元组
        """
        try:
            v_type = veh_data.get(tc.VAR_TYPE)
            if not v_type or v_type not in self.vtype_cache:
                return None 
            
            type_params = self.vtype_cache[v_type]
            
            pos = veh_data[tc.VAR_POSITION]
            v = veh_data[tc.VAR_SPEED]
            angle_traci_deg = veh_data[tc.VAR_ANGLE]
            l = type_params['length'] 
            w = type_params['width']
            
            angle_math_rad = math.radians( (450.0 - angle_traci_deg) % 360.0 )
            
            return (pos[0], pos[1], v, angle_math_rad, l, w)
        except KeyError:
            return None
        except Exception:
            return None

    def _get_speed_gain(self, veh_id: str, veh_data: Dict, target_lane_index: int) -> float:
        """(实现 Eq. 19 [cite: 288-293]) (使用订阅数据)"""
        v_p = -1.0
        leader_info = veh_data.get(tc.VAR_LEADER) 
        if leader_info:
            leader_data = self.all_veh_data.get(leader_info[0], {})
            v_p = leader_data.get(tc.VAR_SPEED, -1.0)
        
        v_tp = -1.0
        try:
            lc_state_tuple = traci.vehicle.getLaneChangeState(veh_id, _LC_DIR_LEFT)
            tp_id = lc_state_tuple[7]
            if tp_id:
                tp_data = self.all_veh_data.get(tp_id, {})
                v_tp = tp_data.get(tc.VAR_SPEED, -1.0)
        except traci.TraCIException:
            v_tp = -1.0 

        if v_p < 0 and v_tp < 0: return 0.0      
        if v_tp < 0: v_tp = CONFIG.scenario.DESIGN_SPEED_MS * 1.5 
        if v_p < 0: v_p = veh_data.get(tc.VAR_SPEED, 0.0) 
        
        return v_tp - v_p
            
    def _update_and_get_cdl_distance(self, veh_id: str, veh_data: Dict, cdl_start_edge: Optional[str]) -> float:
        """(实现 Eq. 21 [cite: 299-300]) (使用订阅数据)"""
        
        current_edge = veh_data.get(tc.VAR_ROAD_ID)
        if (not cdl_start_edge) or (current_edge not in self.edge_to_order):
            self.cav_cdl_distance_memory[veh_id] = 0.0
            return 0.0
        
        cdl_start_order = self.edge_to_order.get(cdl_start_edge, 0)
        current_order = self.edge_to_order.get(current_edge, cdl_start_order)
        
        scl_index = current_order - cdl_start_order
        if scl_index < 0:
            self.cav_cdl_distance_memory[veh_id] = 0.0
            return 0.0
        
        distance_on_edge = veh_data.get(tc.VAR_LANEPOSITION, 0.0) 
        d_cav = (scl_index * self.SCL_LENGTH_M) + distance_on_edge
        
        self.cav_cdl_distance_memory[veh_id] = d_cav
        return d_cav

    def _check_cooperation_safety_proxy(self, coop_veh_id: str, coop_data: Dict) -> bool:
        """(代理实现 Eq. 35/36 [cite: 365-373]) (使用订阅数据+缓存)"""
        leader_info = coop_data.get(tc.VAR_LEADER)
        if leader_info is None:
            return True 
        
        v_type = coop_data.get(tc.VAR_TYPE)
        type_params = self.vtype_cache.get(v_type)
        if not type_params:
            return False 
            
        gap = leader_info[1] 
        v = coop_data.get(tc.VAR_SPEED, 0.0)
        
        tau = type_params['tau']
        if tau < 0.1: tau = self.HV_BRAKE_REACTION_TIME_S
            
        min_gap = type_params['minGap']
        safe_distance = v * tau + min_gap
        
        return gap > (safe_distance * 1.2)