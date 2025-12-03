import traci
import traci.constants as tc
import math
import random 
from typing import Dict, Optional, Tuple, Set, List

# -----------------------------------------------------------------
# 1. 导入 Config 和 TTC 核心函数
# -----------------------------------------------------------------
try:
    from ..config import CONFIG
    from ..environment.twod_ttc_calculator import calculate_2d_ttc
except ImportError:
    pass

# -----------------------------------------------------------------
# 2. 常量定义
# -----------------------------------------------------------------
_LC_DIR_LEFT = 1
_LC_DIR_RIGHT = -1
_SUMO_DEFAULT_LC_MODE = 0b011001010101  
_TRACI_LC_MODE = 0b000100000000 #0b000000000000            
_TRACI_FORCED_LC_MODE = 256
_TRACI_KEEP_LANE_MODE = 0b001000000000   
_TTC_HML_FLEXIBLE_THRESHOLD = 5

_PY_RANDOM_SEED = 9497
random.seed(_PY_RANDOM_SEED)

MODE_NO_CONTROL = 0 
MODE_BASELINE = 1   
MODE_CUSTOM = 2     

VARS_TO_SUBSCRIBE = [
    tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ANGLE, tc.VAR_ROAD_ID,
    tc.VAR_LANE_ID, tc.VAR_LANE_INDEX, tc.VAR_LANEPOSITION, tc.VAR_TYPE, tc.VAR_ALLOWED_SPEED
]

class VehicleController:
    """
    实现论文 3.3 节定义的微观换道控制逻辑。
    修复了 getLaneChangeState 的索引错误，手动计算目标车道邻居。
    """
    
    def __init__(self, control_edge_ids: List[str]):
        # 1. 区域定义
        self.CONTROL_EDGE_IDS: List[str] = control_edge_ids
        self.edge_to_order: Dict[str, int] = {edge_id: i for i, edge_id in enumerate(self.CONTROL_EDGE_IDS)}
        
        # 2. 内部内存和参数
        self.cav_dissatisfaction_memory: Dict[str, float] = {}
        self.cav_cdl_distance_memory: Dict[str, float] = {}
        self.controlled_veh_ids: Set[str] = set()
        
        try:
            self.SIM_STEP_S = CONFIG.scenario.SIM_STEP_LENGTH_S
            self.TTC_THRESHOLD = CONFIG.dcdl.TTC_2D_THRESHOLD_S 
            self.CUMULATIVE_DISSATISFACTION_S = CONFIG.dcdl.CUMULATIVE_DISSATISFACTION_S
            self.SPEED_DIFF_THRESHOLD_MS = CONFIG.dcdl.SPEED_DIFF_THRESHOLD_MS
            self.STOPPING_DISTANCE_THRESHOLD_M = CONFIG.dcdl.STOPPING_DISTANCE_THRESHOLD_M
            self.SCL_LENGTH_M = CONFIG.dcdl.SCL_LENGTH_M
            self.HV_BRAKE_REACTION_TIME_S = CONFIG.dcdl.HV_BRAKE_REACTION_TIME_S
            self.L = CONFIG.dcdl.VEHICLE_LENGTH_M
            self.W = CONFIG.dcdl.VEHICLE_WIDTH_M
        except:
            self.SIM_STEP_S = 0.1
            self.TTC_THRESHOLD = 3.0
            self.CUMULATIVE_DISSATISFACTION_S = 10.0
            self.SPEED_DIFF_THRESHOLD_MS = 2.0
            self.STOPPING_DISTANCE_THRESHOLD_M = 50.0
            self.SCL_LENGTH_M = 100.0
            self.HV_BRAKE_REACTION_TIME_S = 1.0
            self.L = 5.0
            self.W = 2.0

        # 3. 缓存 vType 参数
        self.vtype_cache: Dict[str, Dict] = {}
        self._cache_vtype_parameters()

        # 4. 订阅管理
        self.subscribed_vehicles: Set[str] = set()
        self.all_veh_data: Dict[str, Dict] = {}
        traci.simulation.subscribe([tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS])

    def enforce_keep_lane_mode(self):
        """
        Mode 0（无控制）下，禁止 SUMO 的自主换道，确保车辆保持车道以持续累积排队。
        """
        self._handle_subscriptions()
        self.all_veh_data = traci.vehicle.getSubscriptionResults(None)

        for veh_id in self.all_veh_data:
            traci.vehicle.setLaneChangeMode(veh_id, _TRACI_KEEP_LANE_MODE)

    def _cache_vtype_parameters(self):
        print("VehicleController: 正在缓存 vType 参数...")
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

    def _handle_subscriptions(self):
        sim_results = traci.simulation.getSubscriptionResults()
        if not sim_results: return

        for veh_id in sim_results.get(tc.VAR_ARRIVED_VEHICLES_IDS, []):
            self.subscribed_vehicles.discard(veh_id)
            self.controlled_veh_ids.discard(veh_id)
            if veh_id in self.cav_dissatisfaction_memory: 
                del self.cav_dissatisfaction_memory[veh_id]
            if veh_id in self.cav_cdl_distance_memory: 
                del self.cav_cdl_distance_memory[veh_id]

        for veh_id in sim_results.get(tc.VAR_DEPARTED_VEHICLES_IDS, []):
            if veh_id not in self.subscribed_vehicles:
                traci.vehicle.subscribe(veh_id, VARS_TO_SUBSCRIBE)
                traci.vehicle.subscribeLeader(veh_id, 1000.0) 
                self.subscribed_vehicles.add(veh_id)

    def update_vehicle_states(self, hml_lanes: Set[str], cdl_lanes: Set[str], cdl_start_edge: Optional[str], control_mode: int = MODE_CUSTOM):
        self._handle_subscriptions()
        self.all_veh_data = traci.vehicle.getSubscriptionResults(None)

        dcdl_lanes_combined = hml_lanes.union(cdl_lanes)
        current_dcdl_veh_ids = {
            veh_id for veh_id, data in self.all_veh_data.items()
            if data and data.get(tc.VAR_LANE_ID) in dcdl_lanes_combined
        }

        # 1. 恢复 LC2013
        previously_controlled = set(self.controlled_veh_ids)
        released_vehicles = previously_controlled - current_dcdl_veh_ids
        for veh_id in released_vehicles:
            if veh_id in self.all_veh_data:
                traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)

        # 2. 模式检查
        if control_mode != MODE_CUSTOM:
            for veh_id in current_dcdl_veh_ids:
                traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)
            self.controlled_veh_ids = current_dcdl_veh_ids
            return

        # 3. 禁用自主换道
        newly_controlled = current_dcdl_veh_ids - previously_controlled
        for veh_id in newly_controlled:
            traci.vehicle.setLaneChangeMode(veh_id, _TRACI_LC_MODE)

        self.controlled_veh_ids = current_dcdl_veh_ids

        # 4. 执行决策
        for veh_id in self.controlled_veh_ids:
            veh_data = self.all_veh_data[veh_id]
            self.manage_dcdl_lane_changing(veh_id, veh_data, hml_lanes, cdl_lanes, cdl_start_edge)
    
    def manage_dcdl_lane_changing(self, veh_id: str, veh_data: Dict, hml_lanes: Set[str], cdl_lanes: Set[str], cdl_start_edge: Optional[str]):
        current_lane = veh_data.get(tc.VAR_LANE_ID)
        if not current_lane: return

        prob_lc, coop_request, target_lane, direction = self._get_lane_change_motivation(
            veh_id, veh_data, current_lane, hml_lanes, cdl_lanes, cdl_start_edge
        )

        if prob_lc == 0.0 or target_lane == -1:
            return 
        
        if coop_request < 1.0 and random.random() >= prob_lc:
            return 

        # 安全检查 (2D-TTC)
        is_safe, gaps = self._assess_lane_change_safety_2D_TTC(veh_id, veh_data, direction, hml_lanes)

        if is_safe:
            traci.vehicle.changeLane(veh_id, target_lane, duration=0.1)
            return

        if coop_request == 1.0: 
            self.attempt_lane_change_cooperation(veh_id, veh_data, gaps)

    # -----------------------------------------------------
    # 核心修复: 手动查找目标车道邻居
    # -----------------------------------------------------
    def _get_target_neighbors(self, veh_id: str, veh_data: Dict, direction: int) -> Tuple[Optional[str], Optional[str]]:
        """
        手动查找目标车道的前车(TP)和后车(TF)。
        返回: (tp_id, tf_id)
        """
        edge_id = veh_data.get(tc.VAR_ROAD_ID, "")
        lane_index = veh_data.get(tc.VAR_LANE_INDEX, 0)
        current_lane_pos = veh_data.get(tc.VAR_LANEPOSITION, 0.0)
        
        # 1. 计算目标车道 ID
        target_lane_index = lane_index + direction
        if target_lane_index < 0: return None, None # 越界保护
        
        target_lane_id = f"{edge_id}_{target_lane_index}"
        
        # 2. 获取目标车道上的所有车辆
        try:
            # getLastStepVehicleIDs 返回的是从车道末端到起点的顺序(通常按位置排序)
            vehs_on_target = traci.lane.getLastStepVehicleIDs(target_lane_id)
        except traci.TraCIException:
            # 目标车道不存在
            return None, None

        if not vehs_on_target:
            return None, None

        best_tp = None # 前车
        best_tf = None # 后车
        min_dist_tp = float('inf')
        min_dist_tf = float('inf')

        # 3. 遍历寻找最近的前车和后车
        for other_id in vehs_on_target:
            if other_id == veh_id: continue # 理论上不应该在目标车道出现自己，但防守一下
            
            # 获取对方位置 (优先从缓存取，没有则直接问 SUMO)
            if other_id in self.all_veh_data:
                other_pos = self.all_veh_data[other_id][tc.VAR_LANEPOSITION]
            else:
                try:
                    other_pos = traci.vehicle.getLanePosition(other_id)
                except:
                    continue

            dist = other_pos - current_lane_pos
            
            if dist > 0: # 在我前面 (TP)
                if dist < min_dist_tp:
                    min_dist_tp = dist
                    best_tp = other_id
            else: # 在我后面 (TF)
                if abs(dist) < min_dist_tf:
                    min_dist_tf = abs(dist)
                    best_tf = other_id
                    
        return best_tp, best_tf

    # -----------------------------------------------------
    # 阶段 3 & 4: 评估与协作 (修复版)
    # -----------------------------------------------------

    def _assess_lane_change_safety_2D_TTC(self, veh_id: str, veh_data: Dict, direction: int, hml_lanes: Set[str]) -> Tuple[bool, dict]:
        # 【修复】不再使用 getLaneChangeState 获取邻居，改用手动计算
        tp_id, tf_id = self._get_target_neighbors(veh_id, veh_data, direction)
        gaps = {"tp": tp_id, "tf": tf_id}
        
        state_i = self._format_state_from_sub(veh_id, veh_data) 
        if not state_i: return False, gaps

        current_lane = veh_data.get(tc.VAR_LANE_ID)
        v_type = veh_data.get(tc.VAR_TYPE, "")
        is_hv_on_hml = v_type.startswith("hv") and current_lane in hml_lanes
        
        effective_ttc_threshold = _TTC_HML_FLEXIBLE_THRESHOLD if is_hv_on_hml else self.TTC_THRESHOLD

        # 检查前车 (TP)
        ttc_tp = float('inf')
        if tp_id:
            tp_data = self.all_veh_data.get(tp_id)
            # 如果不在订阅列表里（例如刚生成的车），临时获取一次数据
            if not tp_data:
                 # 简易构造数据用于计算
                 try:
                     tp_data = {
                         tc.VAR_POSITION: traci.vehicle.getPosition(tp_id),
                         tc.VAR_SPEED: traci.vehicle.getSpeed(tp_id),
                         tc.VAR_ANGLE: traci.vehicle.getAngle(tp_id),
                         tc.VAR_TYPE: traci.vehicle.getTypeID(tp_id)
                     }
                 except: pass

            if tp_data:
                state_j = self._format_state_from_sub(tp_id, tp_data)
                if state_j:
                    ttc_tp = calculate_2d_ttc(state_i, state_j)

        # 检查后车 (TF)
        ttc_tf = float('inf')
        if tf_id:
            tf_data = self.all_veh_data.get(tf_id)
            if not tf_data:
                 try:
                     tf_data = {
                         tc.VAR_POSITION: traci.vehicle.getPosition(tf_id),
                         tc.VAR_SPEED: traci.vehicle.getSpeed(tf_id),
                         tc.VAR_ANGLE: traci.vehicle.getAngle(tf_id),
                         tc.VAR_TYPE: traci.vehicle.getTypeID(tf_id)
                     }
                 except: pass

            if tf_data:
                state_k = self._format_state_from_sub(tf_id, tf_data)
                if state_k:
                    ttc_tf = calculate_2d_ttc(state_k, state_i) # 注意顺序: 后车撞我
        
        is_safe = (ttc_tp > effective_ttc_threshold and ttc_tf > effective_ttc_threshold)
        
        return is_safe, gaps

    # -----------------------------------------------------
    # 动机计算 (保持不变)
    # -----------------------------------------------------

    def _get_lane_change_motivation(self, veh_id: str, veh_data: Dict, current_lane: str, hml_lanes: Set[str], cdl_lanes: Set[str], cdl_start_edge: Optional[str]) -> Tuple[float, float, int, int]:
        v_type = veh_data.get(tc.VAR_TYPE, "")
        is_hv = v_type.startswith("hv")
        is_cav = not is_hv
        
        current_lane_index = veh_data.get(tc.VAR_LANE_INDEX, -1)
        if current_lane_index == -1: return 0.0, 0.0, -1, 0

        # 初始化
        prob_lc = 0.0
        coop_request = 0.0
        target_lane_index = -1
        direction = 0

        # 策略 1: HV 在限制车道，必须向左逃离
        if is_hv and (current_lane in cdl_lanes or current_lane in hml_lanes):
            prob_lc = 1.0
            coop_request = 1.0 
            target_lane_index = current_lane_index + 1
            direction = _LC_DIR_LEFT

        # 策略 2: CAV 在 HML (过渡区)，想回 CDL (假设 CDL 在右边? 不，根据之前的逻辑 CDL 在左边，Wait)
        # 让我们回顾 LaneManager: Lane 0 是 CDL?
        # LaneManager: 
        #   LANE_INDEX = controlled_lane_index (通常=0, 最右侧)
        #   hml_lanes/cdl_lanes 都是指 edge_i_0
        #   所以目标是变道去左边(Lane 1) 逃离? 
        #   或者 DCDL 是最左侧?
        #   根据 _get_lane_change_motivation 的逻辑:
        #   target = index + 1 (向左). 说明 DCDL 在最右侧 (Index 0).
        #   HV 在 Index 0 -> 往 Index 1 跑. 正确.
        
        elif is_cav and current_lane in hml_lanes:
            target_idx_candidate = current_lane_index + 1
            prob_lc = self._calculate_cav_hml_motivation(veh_id, veh_data, target_idx_candidate)
            if prob_lc > 0:
                coop_request = 1.0 if prob_lc == 1.0 else 0.0
                target_lane_index = target_idx_candidate
                direction = _LC_DIR_LEFT
            
        elif is_cav and current_lane in cdl_lanes:
            target_idx_candidate = current_lane_index + 1
            prob_lc = self._calculate_cav_cdl_motivation(veh_id, veh_data, target_idx_candidate, cdl_lanes, cdl_start_edge)
            if prob_lc > 0:
                coop_request = 1.0 if prob_lc == 1.0 else 0.0
                target_lane_index = target_idx_candidate
                direction = _LC_DIR_LEFT
        
        return prob_lc, coop_request, target_lane_index, direction

    def _calculate_cav_hml_motivation(self, veh_id: str, veh_data: Dict, target_lane_index: int) -> float:
        v_current = veh_data.get(tc.VAR_SPEED, 0.0)
        v_desired = veh_data.get(tc.VAR_ALLOWED_SPEED, v_current)
        
        if v_desired > 1.0:
            dissatisfaction = max(0, (v_desired - v_current) / v_desired) * self.SIM_STEP_S
        else:
            dissatisfaction = 0.0
        
        current_total_d = self.cav_dissatisfaction_memory.get(veh_id, 0.0) + dissatisfaction
        self.cav_dissatisfaction_memory[veh_id] = current_total_d

        prob_eff = min(1.0, current_total_d / self.CUMULATIVE_DISSATISFACTION_S)
        if prob_eff < 1.0: return prob_eff 

        if self._get_speed_gain(veh_id, veh_data, target_lane_index) > self.SPEED_DIFF_THRESHOLD_MS:
            self.cav_dissatisfaction_memory[veh_id] = 0.0 
            return 1.0 
        return 0.0 
        
    def _calculate_cav_cdl_motivation(self, veh_id: str, veh_data: Dict, target_lane_index: int, cdl_lanes: Set[str], cdl_start_edge: Optional[str]) -> Tuple[float, float]:
        prob_eff = self._calculate_cav_hml_motivation(veh_id, veh_data, target_lane_index)
        l_cdl_total = len(cdl_lanes) * self.SCL_LENGTH_M 
        d_cav = self._update_and_get_cdl_distance(veh_id, veh_data, cdl_start_edge)
        d_stop_bar = self.STOPPING_DISTANCE_THRESHOLD_M
        l_remaining = l_cdl_total - d_cav
        prob_safe = 1.0 - min(1.0, max(0.0, l_remaining / d_stop_bar))
        prob_total = 1.0 - (1.0 - prob_eff) * (1.0 - prob_safe)
        return prob_total

    # -----------------------------------------------------
    # 协作与其他辅助
    # -----------------------------------------------------

    def attempt_lane_change_cooperation(self, veh_id: str, veh_data: Dict, gaps: dict):
        tp_id = gaps.get("tp")
        tf_id = gaps.get("tf")
        
        tp_data = self.all_veh_data.get(tp_id) if tp_id else None
        tf_data = self.all_veh_data.get(tf_id) if tf_id else None
        
        # 安全读取 type，防止 None
        tp_type = tp_data.get(tc.VAR_TYPE, "") if tp_data else ""
        tf_type = tf_data.get(tc.VAR_TYPE, "") if tf_data else ""

        tp_is_cav = tp_type.startswith("cav")
        tf_is_cav = tf_type.startswith("cav")
        
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
        
        if coop_target_id is None: return 

        coop_data = self.all_veh_data.get(coop_target_id)
        if not coop_data: return

        if not self._check_cooperation_safety_proxy(coop_target_id, coop_data): return 

        coop_duration = self.SIM_STEP_S * 5
        coop_vtype = coop_data.get(tc.VAR_TYPE)
        if coop_vtype in self.vtype_cache:
            type_params = self.vtype_cache[coop_vtype]
            if coop_action == "accel":
                traci.vehicle.setAcceleration(coop_target_id, type_params['accel'], coop_duration)
            elif coop_action == "decel":
                traci.vehicle.setAcceleration(coop_target_id, -type_params['decel'], coop_duration)
            
    def _format_state_from_sub(self, veh_id: str, veh_data: Dict) -> Optional[tuple]:
        v_type = veh_data.get(tc.VAR_TYPE)
        if not v_type or v_type not in self.vtype_cache: return None
        
        type_params = self.vtype_cache[v_type]
        
        try:
            pos = veh_data[tc.VAR_POSITION]
            v = veh_data[tc.VAR_SPEED]
            angle_traci_deg = veh_data[tc.VAR_ANGLE]
            l = type_params['length'] 
            w = type_params['width']
            angle_math_rad = math.radians( (90.0 - angle_traci_deg) % 360.0 )
            return (pos[0], pos[1], v, angle_math_rad, l, w)
        except KeyError:
            return None

    def _get_speed_gain(self, veh_id: str, veh_data: Dict, target_lane_index: int) -> float:
        v_p = -1.0
        leader_info = veh_data.get(tc.VAR_LEADER) 
        if leader_info:
            leader_data = self.all_veh_data.get(leader_info[0], {})
            v_p = leader_data.get(tc.VAR_SPEED, -1.0)
        
        # 修复: getLaneChangeState 不返回速度，这里需要用 _get_target_neighbors 的逻辑取 TP 的速度
        tp_id, _ = self._get_target_neighbors(veh_id, veh_data, _LC_DIR_LEFT)
        v_tp = -1.0
        if tp_id and tp_id in self.all_veh_data:
             v_tp = self.all_veh_data[tp_id].get(tc.VAR_SPEED, -1.0)

        if v_p < 0 and v_tp < 0: v_tp = CONFIG.scenario.DESIGN_SPEED_MS * 1.5      
        if v_tp < 0: v_tp = CONFIG.scenario.DESIGN_SPEED_MS * 1.5 
        if v_p < 0: v_p = veh_data.get(tc.VAR_SPEED, 0.0) 
        
        return v_tp - v_p
            
    def _update_and_get_cdl_distance(self, veh_id: str, veh_data: Dict, cdl_start_edge: Optional[str]) -> float:
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
        leader_info = coop_data.get(tc.VAR_LEADER)
        if leader_info is None: return True 
        
        v_type = coop_data.get(tc.VAR_TYPE)
        type_params = self.vtype_cache.get(v_type)
        if not type_params: return False 
            
        gap = leader_info[1] 
        v = coop_data.get(tc.VAR_SPEED, 0.0)
        tau = type_params['tau']
        if tau < 0.1: tau = self.HV_BRAKE_REACTION_TIME_S
        min_gap = type_params['minGap']
        safe_distance = v * tau + min_gap
        return gap > (safe_distance * 1)