import traci
import traci.constants as tc
import math
import random 
from typing import Dict, Optional, Tuple, Set, List

# -----------------------------------------------------------------
# 1. 导入 Config 和 TTC 核心函数
# -----------------------------------------------------------------

from ..config import CONFIG
from ..environment.twod_ttc_calculator import calculate_2d_ttc
# -----------------------------------------------------------------
# 2. 常量定义
# -----------------------------------------------------------------
_LC_DIR_LEFT = 1
_LC_DIR_RIGHT = -1
_SUMO_DEFAULT_LC_MODE = 0b011001010101  
_TRACI_LC_MODE = 0b000100000000 #0b000000000000
_TRACI_FORCED_LC_MODE = 256
_TRACI_KEEP_LANE_MODE = 0b000000000000
_TTC_HML_FLEXIBLE_THRESHOLD = 5
_COOP_COOLDOWN_STEPS = 10

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
    def __init__(self, control_edge_ids: List[str]):
        # 1. 区域定义
        self.CONTROL_EDGE_IDS: List[str] = control_edge_ids
        self.edge_to_order: Dict[str, int] = {edge_id: i for i, edge_id in enumerate(self.CONTROL_EDGE_IDS)}

        self.edge_lengths: Dict[str, float] = {}
        for edge_id in self.CONTROL_EDGE_IDS:
            self.edge_lengths[edge_id] = traci.lane.getLength(f"{edge_id}_0")
        
        # 2. 内部内存和参数
        self.cav_dissatisfaction_memory: Dict[str, float] = {}
        self.cav_cdl_distance_memory: Dict[str, float] = {}
        self.cooperation_cooldowns: Dict[str, int] = {}
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
        traci.simulation.subscribe([tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS])

    def _tick_cooperation_cooldowns(self):
        expired: List[str] = []
        for veh_id, remaining in self.cooperation_cooldowns.items():
            if remaining <= 1:
                expired.append(veh_id)
            else:
                self.cooperation_cooldowns[veh_id] = remaining - 1

        for veh_id in expired:
            del self.cooperation_cooldowns[veh_id]

    def enforce_nocontrol_mode(self):
        self._handle_subscriptions()
        self.all_veh_data = traci.vehicle.getSubscriptionResults(None)

        for veh_id in self.all_veh_data:
            traci.vehicle.setLaneChangeMode(veh_id, _TRACI_KEEP_LANE_MODE)

    def enforce_baseline_mode(self):
        self._handle_subscriptions()
        self.all_veh_data = traci.vehicle.getAllSubscriptionResults()

        if not hasattr(self, 'veh_type_cache'):
            self.veh_type_cache = {}

        for veh_id in self.all_veh_data:
            if veh_id not in self.veh_type_cache:
                self.veh_type_cache[veh_id] = traci.vehicle.getTypeID(veh_id)

            v_type = self.veh_type_cache[veh_id]
            is_cav = "CAV" in v_type or "cav" in v_type

            if is_cav:
                traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)
            else:
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
            if veh_id in self.cooperation_cooldowns:
                del self.cooperation_cooldowns[veh_id]

        for veh_id in sim_results.get(tc.VAR_DEPARTED_VEHICLES_IDS, []):
            if veh_id not in self.subscribed_vehicles:
                traci.vehicle.subscribe(veh_id, VARS_TO_SUBSCRIBE)
                traci.vehicle.subscribeLeader(veh_id, 1000.0) 
                self.subscribed_vehicles.add(veh_id)

    def update_vehicle_states(self, hml_lanes: Set[str], cdl_lanes: Set[str], ml_lanes: Set[str], cdl_start_edge: Optional[str], control_mode: int = MODE_CUSTOM):
        self._handle_subscriptions()
        self._tick_cooperation_cooldowns()
        self.all_veh_data = traci.vehicle.getSubscriptionResults(None)

        all_controlled_lanes = hml_lanes.union(cdl_lanes).union(ml_lanes)

        current_controlled_veh_ids = {
            veh_id for veh_id, data in self.all_veh_data.items()
            if data and data.get(tc.VAR_LANE_ID) in all_controlled_lanes
        }

        # 1. 恢复 LC2013
        previously_controlled = set(self.controlled_veh_ids)
        released_vehicles = previously_controlled - current_controlled_veh_ids
        for veh_id in released_vehicles:
            if veh_id in self.all_veh_data:
                traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)

        # 2. 模式检查
        if control_mode != MODE_CUSTOM:
            for veh_id in current_controlled_veh_ids:
                traci.vehicle.setLaneChangeMode(veh_id, _SUMO_DEFAULT_LC_MODE)
            self.controlled_veh_ids = current_controlled_veh_ids
            return

        # 3. 禁用自主换道
        newly_controlled = current_controlled_veh_ids - previously_controlled
        for veh_id in newly_controlled:
            traci.vehicle.setLaneChangeMode(veh_id, _TRACI_LC_MODE)

        self.controlled_veh_ids = current_controlled_veh_ids

        # 4. 执行决策
        for veh_id in self.controlled_veh_ids:
            veh_data = self.all_veh_data[veh_id]
            self.manage_dcdl_lane_changing(veh_id, veh_data, hml_lanes, cdl_lanes, ml_lanes, cdl_start_edge)
    
    def manage_dcdl_lane_changing(self, veh_id: str, veh_data: Dict, hml_lanes: Set[str], cdl_lanes: Set[str], ml_lanes: Set[str], cdl_start_edge: Optional[str]):
        current_lane = veh_data.get(tc.VAR_LANE_ID)
        if not current_lane: return

        prob_lc, coop_request, target_lane, direction = self._get_lane_change_motivation(
            veh_id, veh_data, current_lane, hml_lanes, cdl_lanes, ml_lanes, cdl_start_edge
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

        if prob_lc == 1.0 and coop_request == 1.0: 
            self.attempt_lane_change_cooperation(veh_id, veh_data, gaps)

    def _get_target_neighbors(self, veh_id: str, veh_data: Dict, direction: int) -> Tuple[Optional[str], Optional[str]]:
        current_edge = veh_data.get(tc.VAR_ROAD_ID, "")
        current_lane_idx = veh_data.get(tc.VAR_LANE_INDEX, 0)
        current_pos = veh_data.get(tc.VAR_LANEPOSITION, 0.0)
        
        # 1. 确定目标车道索引
        target_lane_idx = current_lane_idx + direction
        if target_lane_idx < 0: return None, None
        if target_lane_idx >= traci.edge.getLaneNumber(current_edge):return None, None
        
        # 2. 确定需要搜索的 Edge 列表
        search_edges = []
        curr_order = self.edge_to_order.get(current_edge)
        
        if curr_order is None: return None, None

        # 添加上游 (Prev)
        if curr_order > 0:
            search_edges.append({
                "edge": self.CONTROL_EDGE_IDS[curr_order - 1], 
                "type": "prev"
            })

        # 添加当前 (Curr)
        search_edges.append({
            "edge": current_edge, 
            "type": "curr"
        })

        # 添加下游 (Next)
        if curr_order < len(self.CONTROL_EDGE_IDS) - 1:
            search_edges.append({
                "edge": self.CONTROL_EDGE_IDS[curr_order + 1], 
                "type": "next"
            })

        # 3. 遍历搜索并计算统一距离
        best_tp = None
        best_tf = None
        min_dist_tp = float('inf')
        min_dist_tf = float('inf')
        
        current_edge_len = self.edge_lengths.get(current_edge, 1000.0)

        for item in search_edges:
            edge_id = item['edge']
            edge_type = item['type']
            target_lane_id = f"{edge_id}_{target_lane_idx}"
            
            # 获取该路段目标车道的所有车辆
            try:
                vehs = traci.lane.getLastStepVehicleIDs(target_lane_id)
            except:
                continue # 可能该路段车道数变少，跳过
            
            if not vehs: continue

            # 获取该路段长度 (仅 Prev 需要用到)
            edge_len = self.edge_lengths.get(edge_id, 1000.0)

            for other_id in vehs:
                if other_id == veh_id: continue

                # 获取对方位置
                if other_id in self.all_veh_data:
                    other_pos = self.all_veh_data[other_id].get(tc.VAR_LANEPOSITION, -1)
                else:
                    try:
                        other_pos = traci.vehicle.getLanePosition(other_id)
                    except: continue
                
                if other_pos < 0: continue

                # --- 核心：计算相对距离 (dist = other - ego) ---
                dist = float('inf')
                
                if edge_type == "curr":
                    # 同一路段: 直接相减
                    dist = other_pos - current_pos
                    
                elif edge_type == "prev":
                    # 上游: 对方位置 - 对方路段剩余长度 - 我当前走过的长度
                    # dist = other_pos - edge_len - current_pos (这是错的)
                    # 正确逻辑: 距离 = -(我离起点的距离 + 对方离终点的距离)
                    # 对方离终点 = edge_len - other_pos
                    # dist = -(current_pos + (edge_len - other_pos))
                    dist = other_pos - edge_len - current_pos
                    
                elif edge_type == "next":
                    # 下游: (我离终点的距离 + 对方离起点的距离)
                    # 我离终点 = current_edge_len - current_pos
                    dist = (current_edge_len - current_pos) + other_pos

                # --- 更新 TP / TF ---
                if dist > 0: # 前方
                    if dist < min_dist_tp:
                        min_dist_tp = dist
                        best_tp = other_id
                else: # 后方
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
            if tp_data:
                state_j = self._format_state_from_sub(tp_id, tp_data)
                if state_j:
                    ttc_tp = calculate_2d_ttc(state_i, state_j)

        # 检查后车 (TF)
        ttc_tf = float('inf')
        if tf_id:
            tf_data = self.all_veh_data.get(tf_id)
            if tf_data:
                state_k = self._format_state_from_sub(tf_id, tf_data)
                if state_k:
                    ttc_tf = calculate_2d_ttc(state_k, state_i)
        
        is_safe = (ttc_tp > effective_ttc_threshold and ttc_tf > effective_ttc_threshold)
        
        return is_safe, gaps

    # -----------------------------------------------------
    # 动机计算 (保持不变)
    # -----------------------------------------------------

    def _get_lane_change_motivation(self, veh_id: str, veh_data: Dict, current_lane: str, hml_lanes: Set[str], cdl_lanes: Set[str], ml_lanes: Set[str], cdl_start_edge: Optional[str]) -> Tuple[float, float, int, int]:
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

        if is_cav and current_lane in ml_lanes:
            target_idx_candidate = current_lane_index + 1
            
            edge_id = veh_data.get(tc.VAR_ROAD_ID, "")
            num_lanes = traci.edge.getLaneNumber(edge_id)
            
            if target_idx_candidate < num_lanes:
                # 复用 HML 的逻辑 (基于速度收益和不满度)
                prob_lc = self._calculate_cav_hml_motivation(veh_id, veh_data, target_idx_candidate)
                
                if prob_lc > 0:
                    coop_request = 1.0 if prob_lc == 1.0 else 0.0
                    target_lane_index = target_idx_candidate
                    direction = _LC_DIR_LEFT


        # 策略 1: HV 在限制车道，必须向左逃离
        elif is_hv and (current_lane in cdl_lanes or current_lane in hml_lanes):
            prob_lc = 1.0
            coop_request = 1.0 
            target_lane_index = current_lane_index + 1
            direction = _LC_DIR_LEFT

        # 策略 2: CAV 在 HML (过渡区)，想回 CDL (假设 CDL 在右边? 不，根据之前的逻辑 CDL 在左边，Wait)      
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

        if current_total_d <= self.CUMULATIVE_DISSATISFACTION_S:
            return 0.0

        speed_gain = self._get_speed_gain(veh_id, veh_data, target_lane_index)
        if speed_gain <= 0.0:
            return 0.0

        if speed_gain < self.SPEED_DIFF_THRESHOLD_MS:
            return speed_gain / self.SPEED_DIFF_THRESHOLD_MS

        self.cav_dissatisfaction_memory[veh_id] = 0.0
        return 1.0
        
    def _calculate_cav_cdl_motivation(self, veh_id: str, veh_data: Dict, target_lane_index: int, cdl_lanes: Set[str], cdl_start_edge: Optional[str]) -> Tuple[float, float]:
        prob_eff = self._calculate_cav_hml_motivation(veh_id, veh_data, target_lane_index)
        l_cdl_total = len(cdl_lanes) * self.SCL_LENGTH_M 
        d_cav = self._update_and_get_cdl_distance(veh_id, veh_data, cdl_start_edge)
        d_stop_bar = self.STOPPING_DISTANCE_THRESHOLD_M
        denom = l_cdl_total - d_stop_bar
        prob_safe = 1.0 if d_cav >= denom else d_cav / denom
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

        if self.cooperation_cooldowns.get(coop_target_id, 0) > 0:
            return

        if not self._check_cooperation_safety_proxy(coop_target_id, coop_data): return

        coop_duration = self.SIM_STEP_S * 5
        coop_vtype = coop_data.get(tc.VAR_TYPE)
        if coop_vtype in self.vtype_cache:
            type_params = self.vtype_cache[coop_vtype]
            gap_to_leader = float('inf')
            leader_info = coop_data.get(tc.VAR_LEADER)
            if leader_info:
                gap_to_leader = leader_info[1]

            v = coop_data.get(tc.VAR_SPEED, 0.0)
            tau = type_params['tau'] if type_params['tau'] >= 0.1 else self.HV_BRAKE_REACTION_TIME_S
            min_gap = type_params['minGap']
            safe_distance = v * tau + min_gap

            if coop_action == "accel":
                if gap_to_leader <= safe_distance * 1.2:
                    return
                accel = min(type_params['accel'] * 0.5, (gap_to_leader - safe_distance) / max(self.SIM_STEP_S, 0.1))
                traci.vehicle.setAcceleration(coop_target_id, accel, coop_duration)
            elif coop_action == "decel":
                if gap_to_leader == float('inf') or gap_to_leader >= safe_distance * 1.1:
                    return
                deficit = safe_distance - gap_to_leader
                if deficit <= 0:
                    return
                decel = min(type_params['decel'] * 0.6, deficit / max(self.SIM_STEP_S, 0.1))
                traci.vehicle.setAcceleration(coop_target_id, -decel, coop_duration)

            self.cooperation_cooldowns[coop_target_id] = _COOP_COOLDOWN_STEPS
            
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

        if v_p < 0 and v_tp < 0: v_tp = CONFIG.scenario.DESIGN_SPEED_MS     
        if v_tp < 0: v_tp = CONFIG.scenario.DESIGN_SPEED_MS
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