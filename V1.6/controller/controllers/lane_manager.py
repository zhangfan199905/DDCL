import traci
from typing import List, Set, Dict, Optional

# -----------------------------------------------------------------
# 1. 导入 Config (快速失败)
# -----------------------------------------------------------------
from ..config import CONFIG

class LaneManager:
    def __init__(self, control_edge_ids: List[str], controlled_lane_index: int = 0, middle_lane_index: int = 1):
        self.SCL_EDGES: List[str] = control_edge_ids
        self.TOTAL_SCL_COUNT: int = len(self.SCL_EDGES)
        self.LANE_INDEX: int = controlled_lane_index
        self.ML_INDEX: int = middle_lane_index
        
        self.current_m: int = 0 
        self.current_n: int = 0
        self.current_r: int = self.TOTAL_SCL_COUNT 
        
        self.hml_lanes: Set[str] = set()
        self.cdl_lanes: Set[str] = set()
        self.hcl_lanes: Set[str] = set()
        self.middle_lanes: Set[str] = set() 
        self.hcl_clearance_map: Dict[str, str] = {}
        
        # 尝试从 Config 获取，否则使用默认值
        
        self.VCLASS_HV = CONFIG.dcdl.VCLASS_HV
        self.VCLASS_CAV = CONFIG.dcdl.VCLASS_CAV           
        self.ALL_VCLASSES = [self.VCLASS_HV, self.VCLASS_CAV]

        print(f"LaneManager (V-Paper) 初始化完毕，管理 {self.TOTAL_SCL_COUNT} 个 SCL 路段。")

    def initialize_permissions(self):
        self.middle_lanes.clear()
        for edge_id in self.SCL_EDGES:
            lane_count = traci.edge.getLaneNumber(edge_id)
            for i in range(lane_count):
                lane_id = f"{edge_id}_{i}"
                traci.lane.setAllowed(lane_id, self.ALL_VCLASSES)

                if i == self.ML_INDEX:
                    self.middle_lanes.add(lane_id)

    def reset_to_mixed_traffic(self):
        self.initialize_permissions()
        self.current_m = 0
        self.current_n = 0
        self.current_r = self.TOTAL_SCL_COUNT
        self.hml_lanes.clear()
        self.cdl_lanes.clear()
        self.hcl_lanes.clear()
        self.hcl_clearance_map.clear()

    def apply_lane_strategy(self, m: int, n: int):      
        # 1. 【!! 核心 - 论文 (r,n,m) 逻辑 !!】
        if (m + n) > self.TOTAL_SCL_COUNT:
            # print(f"错误: 动作 (m={m}, n={n}) 超出 SCL 总数。")
            m = min(m, self.TOTAL_SCL_COUNT)
            n = min(n, self.TOTAL_SCL_COUNT - m)
        
        new_m = m
        new_n = n
        new_r = self.TOTAL_SCL_COUNT - new_m - new_n
        
        hml_start_index = new_r
        cdl_start_index = new_r + new_n
        
        # 2. 重置所有车道为 RML
        self.initialize_permissions()
        
        # 缓存上一个周期的状态 (用于 HCL 计算)
        last_hml_start = self.current_r
        last_hml_end = self.current_r + self.current_n
        
        self.hml_lanes.clear()
        self.cdl_lanes.clear()

        # 3. 设置 HML
        for i in range(hml_start_index, cdl_start_index):
            edge_id = self.SCL_EDGES[i]
            lane_id = f"{edge_id}_{self.LANE_INDEX}" 
            traci.lane.setAllowed(lane_id, self.ALL_VCLASSES) 
            self.hml_lanes.add(lane_id)

        # 4. 设置 CDL
        for i in range(cdl_start_index, self.TOTAL_SCL_COUNT):
            edge_id = self.SCL_EDGES[i]
            lane_id = f"{edge_id}_{self.LANE_INDEX}" 
            traci.lane.setAllowed(lane_id, [self.VCLASS_CAV]) 
            self.cdl_lanes.add(lane_id)
            
        # 5. 处理 HCL
        hcl_start = max(last_hml_start, cdl_start_index)
        hcl_end = min(last_hml_end, self.TOTAL_SCL_COUNT)
        
        self.hcl_lanes.clear()
        self.hcl_clearance_map.clear()
        
        for i in range(hcl_start, hcl_end):
            edge_id = self.SCL_EDGES[i]
            lane_id = f"{edge_id}_{self.LANE_INDEX}"
            if lane_id in self.cdl_lanes: 
                self.hcl_lanes.add(lane_id)
                self.hcl_clearance_map[lane_id] = edge_id
                traci.lane.setAllowed(lane_id, [self.VCLASS_CAV]) 

        # 6. 更新内部状态
        self.current_m = new_m
        self.current_n = new_n
        self.current_r = new_r

    def step(self):
        if not self.hcl_lanes:
            return 

        cleared_lanes = set()
        for hcl_lane_id in self.hcl_lanes:
            veh_ids = traci.lane.getLastStepVehicleIDs(hcl_lane_id)
            has_hv = False
            for veh_id in veh_ids:
                if traci.vehicle.getVehicleClass(veh_id) == self.VCLASS_HV:
                    has_hv = True
                    break
            
            if not has_hv:
                cleared_lanes.add(hcl_lane_id)
                if hcl_lane_id in self.hcl_clearance_map:
                    del self.hcl_clearance_map[hcl_lane_id]
        
        self.hcl_lanes.difference_update(cleared_lanes)

    # =================================================================
    # 辅助函数 (Getters)
    # =================================================================
    
    def get_active_hml_lanes(self) -> Set[str]:
        return self.hml_lanes

    def get_active_cdl_lanes(self) -> Set[str]:
        return self.cdl_lanes

    def get_active_hcl_lanes(self) -> Set[str]:
        return self.hcl_lanes
    
    def get_active_middle_lanes(self) -> Set[str]:
        return self.middle_lanes

    def get_cdl_start_edge(self) -> Optional[str]:
        if self.current_m > 0:
            cdl_start_index = self.current_r + self.current_n
            if cdl_start_index < self.TOTAL_SCL_COUNT:
                return self.SCL_EDGES[cdl_start_index]
        return None