# -----------------------------------------------------------------
# controller/config.py
# (V5 - 最终版)
#
# 功能：存放项目所有仿真和模型参数的中央配置文件。
# -----------------------------------------------------------------

import math

# =================================================================
# 1. 交通场景参数 (Traffic Scenario Parameters)
# =================================================================
class ScenarioConfig:
    """配置SUMO仿真场景的宏观参数"""
    FREEWAY_LENGTH_M = 4000.0
    UPSTREAM_AREA_M = 1000.0
    CONTROL_AREA_M = 2000.0
    BOTTLENECK_AREA_M = 300.0
    DOWNSTREAM_AREA_M = 700.0
    
    DESIGN_SPEED_KMH = 120.0
    DESIGN_SPEED_MS = DESIGN_SPEED_KMH / 3.6

    NUM_HVS = 1000
    NUM_CAVS = 500
    TOTAL_VEHICLES = NUM_HVS + NUM_CAVS
    
    SIM_STEP_LENGTH_S = 0.1

# =================================================================
# 2. 动态CAV专用道参数 (DCDL Parameters)
# 来源: 论文 11.5.docx
# =================================================================
class DCDLConfig:
    """
    配置DCDL控制逻辑和微观模型参数
    """
    VCLASS_HV = "custom1" 
    VCLASS_CAV = "custom2" 

    VEHICLE_LENGTH_M = 5.0
    VEHICLE_WIDTH_M = 1.8

    SCL_LENGTH_M = 200.0
    
    HV_BRAKE_REACTION_TIME_S = 1.21
    CAV_DELAY_TIME_S = 0.0
    SSD_DECELERATION_RATE_MS2 = 3.5

    SPEED_DIFF_THRESHOLD_MS = 3.0
    CUMULATIVE_DISSATISFACTION_S = 4.0
    
    STOPPING_DISTANCE_THRESHOLD_M = 250.0
    
    TTC_2D_THRESHOLD_S = 5.0     # 论文 默认的严格阈值
    MIN_SAFETY_DISTANCE_M = 2.0

# =================================================================
# 3. GMARL 智能体参数 (GMARL Agent Parameters)
# 来源: 论文 4.1 & 4.2 + 标准实践
# =================================================================
class GMARLConfig:
    """配置GMARL智能体的训练参数"""
    ALGORITHM = 'QMIX'
    LEARNING_RATE = 0.0001
    RL_GAMMA = 0.99
    BUFFER_SIZE = 50000
    BATCH_SIZE = 64
    EPOCHS = 5000
    
    MIXING_NET_HIDDEN_DIM = 64
    ACTOR_HIDDEN_DIM = 128
    CRITIC_HIDDEN_DIM = 128
    TARGET_UPDATE_RATE = 0.01
    
    GAT_NUM_HEADS = 3
    GAT_HIDDEN_DIM = 128
    GAT_OUTPUT_DIM = 64
    
    # 【!! 关键 !!】决策周期 (来自 det.add.xml 的 freq)
    DECISION_CYCLE_S = 60.0 
    
    REWARD_ALPHA = 1.0
    REWARD_BETA = 1.0
    REWARD_GAMMA = 1.0
    
    TOTAL_SCL_COUNT = int(ScenarioConfig.CONTROL_AREA_M / DCDLConfig.SCL_LENGTH_M)
    
    MAX_M_SCLS = TOTAL_SCL_COUNT
    MAX_N_SCLS = TOTAL_SCL_COUNT

# =================================================================
# 4. 辅助工具类 (Helper Classes for Config Access)
# =================================================================
class Config:
    """用于全局访问所有配置的便捷类"""
    def __init__(self):
        self.scenario = ScenarioConfig()
        self.dcdl = DCDLConfig()
        self.gmarl = GMARLConfig()

# 全局单例
CONFIG = Config()