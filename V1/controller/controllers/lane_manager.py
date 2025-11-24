import traci
from typing import List, Set, Dict, Optional

try:
    from ..config import CONFIG
except ImportError:
    try:
        from controller.config import CONFIG
    except ImportError:
        print("="*50)
        print("致命错误 (FATAL ERROR): 无法在 'lane_manager.py' 中导入 'config.py' ")
        print("="*50)
        raise


class LaneManager:
    """
    管理动态CAV专用道 (DCDL) 的宏观功能。
    【V-Paper】: 严格遵循论文 (r, n, m)  布局和 HCL 逻辑 。
    """
    
    def __init__(self, control_edge_ids: List[str], controlled_lane_index: int = 0): #控制车道索引为0
        self.SCL_EDGES: List[str] = control_edge_ids
        self.TOTAL_SCL_COUNT: int = len(self.SCL_EDGES)
        self.LANE_INDEX: int = controlled_lane_index
        
        if self.TOTAL_SCL_COUNT != CONFIG.gmarl.TOTAL_SCL_COUNT:
            print(f"警告: LaneManager SCL 数量 ({self.TOTAL_SCL_COUNT}) 与 "
                  f"config.py ({CONFIG.gmarl.TOTAL_SCL_COUNT}) 不匹配。") 
        
        self.current_m: int = 0 
        self.current_n: int = 0
        self.current_r: int = self.TOTAL_SCL_COUNT 
        
        self.hml_lanes: Set[str] = set()
        self.cdl_lanes: Set[str] = set()
        self.hcl_lanes: Set[str] = set() 
        self.hcl_clearance_map: Dict[str, str] = {}
        
        self.VCLASS_HV = CONFIG.dcdl.VCLASS_HV
        self.VCLASS_CAV = CONFIG.dcdl.VCLASS_CAV
        self.ALL_VCLASSES = [self.VCLASS_HV, self.VCLASS_CAV]

        print(f"LaneManager (V-Paper) 初始化完毕，管理 {self.TOTAL_SCL_COUNT} 个 SCL 路段。")

    def initialize_permissions(self):
        """确保启动时所有车道对所有车辆开放 (使用 setAllowed )"""
        for edge_id in self.SCL_EDGES:
            try:
                for i in range(traci.edge.getLaneNumber(edge_id)):
                    lane_id = f"{edge_id}_{i}"
                    traci.lane.setAllowed(lane_id, self.ALL_VCLASSES)
            except traci.TraCIException:
                print(f"警告: 初始化路段 {edge_id} 权限失败。")

    def apply_lane_strategy(self, m: int, n: int):
        """
        **GMARL 智能体的动作执行入口** (V-Paper)
        """
        
        # 1. 【!! 核心 - 论文 (r,n,m)  逻辑 !!】
        if (m + n) > self.TOTAL_SCL_COUNT:
            print(f"错误: 动作 (m={m}, n={n}) 超出 SCL 总数 ({self.TOTAL_SCL_COUNT})。")
            m = min(m, self.TOTAL_SCL_COUNT)
            n = min(n, self.TOTAL_SCL_COUNT - m)
            print(f"修正为: (m={m}, n={n})")
        
        new_m = m
        new_n = n
        new_r = self.TOTAL_SCL_COUNT - new_m - new_n
        
        hml_start_index = new_r
        cdl_start_index = new_r + new_n
        
        # 2. 【!! 关键 !!】 重置所有车道为 RML (允许所有车型)
        self.initialize_permissions()
        
        # 缓存上一个周期的状态 (用于 HCL  计算)
        last_hml_start = self.current_r
        last_hml_end = self.current_r + self.current_n
        
        self.hml_lanes.clear()
        self.cdl_lanes.clear()

        # 3. 设置 HML (HV Mandatory Lane-change) 
        # HML 目标: 允许 CAV 和 HV 两种车型
        for i in range(hml_start_index, cdl_start_index):
            edge_id = self.SCL_EDGES[i]
            lane_id = f"{edge_id}_{self.LANE_INDEX}" 
            traci.lane.setAllowed(lane_id, self.ALL_VCLASSES)  # 允许 HV 和 CAV
            self.hml_lanes.add(lane_id)


        # 4. 设置 CDL (CAV Dedicated Lane) 
        # CDL 目标: 只允许 CAV (使用 setAllowed )
        for i in range(cdl_start_index, self.TOTAL_SCL_COUNT):
            edge_id = self.SCL_EDGES[i]
            lane_id = f"{edge_id}_{self.LANE_INDEX}" 
            traci.lane.setAllowed(lane_id, [self.VCLASS_CAV]) 
            self.cdl_lanes.add(lane_id)
            
        # 5. 处理 HCL (HV Clearance Lane) 
        # 严格执行论文 Eq. 15 [cite: 257-259]: L_hcl = L_hml(T) INTERSECT L_cdl(T+1)
        
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
        """
        在每个仿真步 调用
        用于管理 HCL  的清空状态
        """
        if not self.hcl_lanes:
            return 

        cleared_lanes = set()
        for hcl_lane_id in self.hcl_lanes:
            try:
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
            except traci.TraCIException:
                pass
        
        self.hcl_lanes.difference_update(cleared_lanes)

    # =================================================================
    # 辅助函数 (Getters) - 供 vehicle_controller  调用
    # =================================================================
    
    def get_active_hml_lanes(self) -> Set[str]:
        return self.hml_lanes

    def get_active_cdl_lanes(self) -> Set[str]:
        return self.cdl_lanes

    def get_active_hcl_lanes(self) -> Set[str]:
        """返回当前正在清空的 HCL 车道的 ID"""
        return self.hcl_lanes

    def get_cdl_start_edge(self) -> Optional[str]:
        """返回 CDL 区域的第一个 SCL Edge ID [cite: 299-300]"""
        if self.current_m > 0:
            cdl_start_index = self.current_r + self.current_n
            if cdl_start_index < self.TOTAL_SCL_COUNT:
                return self.SCL_EDGES[cdl_start_index]
        return None