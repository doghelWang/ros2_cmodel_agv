#!/usr/bin/env python3
"""
Dijkstra Topological Road-Network Planner (Dijkstra 拓扑路网最优路径规划器)
Supports multiple warehouse scenarios (Cross-dock, Narrow Aisle VNA, FMS Workshop)
with dynamic obstacle edge blockage detection, automatic rerouting, and strict orthogonal alignment.
"""

import math
import heapq
from typing import List, Tuple, Dict, Any


SCENARIO_DEFINITIONS: Dict[str, Dict[str, Any]] = {
    "grid_9_square": {
        "id": "grid_9_square",
        "name": "正方形九宫格智能立体仓",
        "description": "3×3 矩阵高密度存储区，四纵四横标准正交路网，全向无冲突调度",
        "origin": {"x": -2.1, "y": -6.4, "yaw": 0.0},
        "shelves": [
            {"name": "货架区 1 (NW 原料库)", "x1": -5.3, "y1": 3.1, "x2": -3.1, "y2": 5.3, "color": "#1e293b", "border": "#38bdf8"},
            {"name": "货架区 2 (N 标准件)", "x1": -1.1, "y1": 3.1, "x2": 1.1, "y2": 5.3, "color": "#1e293b", "border": "#818cf8"},
            {"name": "货架区 3 (NE 结构件)", "x1": 3.1, "y1": 3.1, "x2": 5.3, "y2": 5.3, "color": "#1e293b", "border": "#c084fc"},
            {"name": "货架区 4 (W 半成品)", "x1": -5.3, "y1": -1.1, "x2": -3.1, "y2": 1.1, "color": "#1e293b", "border": "#34d399"},
            {"name": "货架区 5 (C 核心立体库)", "x1": -1.1, "y1": -1.1, "x2": 1.1, "y2": 1.1, "color": "#1e293b", "border": "#06b6d4"},
            {"name": "货架区 6 (E 电控总成)", "x1": 3.1, "y1": -1.1, "x2": 5.3, "y2": 1.1, "color": "#1e293b", "border": "#f472b6"},
            {"name": "货架区 7 (SW 成品暂存)", "x1": -5.3, "y1": -5.3, "x2": -3.1, "y2": -3.1, "color": "#1e293b", "border": "#fbbf24"},
            {"name": "货架区 8 (S 出库打包)", "x1": -1.1, "y1": -5.3, "x2": 1.1, "y2": -3.1, "color": "#1e293b", "border": "#f97316"},
            {"name": "货架区 9 (SE 返修质检)", "x1": 3.1, "y1": -5.3, "x2": 5.3, "y2": -3.1, "color": "#1e293b", "border": "#ef4444"}
        ],
        "walls": [
            (-7.5, -7.5, 7.5, -7.5), (7.5, -7.5, 7.5, 7.5), (7.5, 7.5, -7.5, 7.5), (-7.5, 7.5, -7.5, -7.5),
            # Shelf 1
            (-5.3, 3.1, -3.1, 3.1), (-5.3, 5.3, -3.1, 5.3), (-5.3, 3.1, -5.3, 5.3), (-3.1, 3.1, -3.1, 5.3),
            # Shelf 2
            (-1.1, 3.1, 1.1, 3.1), (-1.1, 5.3, 1.1, 5.3), (-1.1, 3.1, -1.1, 5.3), (1.1, 3.1, 1.1, 5.3),
            # Shelf 3
            (3.1, 3.1, 5.3, 3.1), (3.1, 5.3, 5.3, 5.3), (3.1, 3.1, 3.1, 5.3), (5.3, 3.1, 5.3, 5.3),
            # Shelf 4
            (-5.3, -1.1, -3.1, -1.1), (-5.3, 1.1, -3.1, 1.1), (-5.3, -1.1, -5.3, 1.1), (-3.1, -1.1, -3.1, 1.1),
            # Shelf 5
            (-1.1, -1.1, 1.1, -1.1), (-1.1, 1.1, 1.1, 1.1), (-1.1, -1.1, -1.1, 1.1), (1.1, -1.1, 1.1, 1.1),
            # Shelf 6
            (3.1, -1.1, 5.3, -1.1), (3.1, 1.1, 5.3, 1.1), (3.1, -1.1, 3.1, 1.1), (5.3, -1.1, 5.3, 1.1),
            # Shelf 7
            (-5.3, -5.3, -3.1, -5.3), (-5.3, -3.1, -3.1, -3.1), (-5.3, -5.3, -5.3, -3.1), (-3.1, -5.3, -3.1, -3.1),
            # Shelf 8
            (-1.1, -5.3, 1.1, -5.3), (-1.1, -3.1, 1.1, -3.1), (-1.1, -5.3, -1.1, -3.1), (1.1, -5.3, 1.1, -3.1),
            # Shelf 9
            (3.1, -5.3, 5.3, -5.3), (3.1, -3.1, 5.3, -3.1), (3.1, -5.3, 3.1, -3.1), (5.3, -5.3, 5.3, -3.1)
        ],
        "stations": [
            {"id": "grid_st_in", "name": "1号原料入库位", "x": -2.1, "y": 6.4, "dock_yaw": 0.0, "color": "#38bdf8"},
            {"id": "grid_st_qc", "name": "2号质检抽样位", "x": 2.1, "y": 6.4, "dock_yaw": 0.0, "color": "#818cf8"},
            {"id": "grid_st_pick", "name": "3号智能拣货位", "x": -6.4, "y": 2.1, "dock_yaw": 1.5708, "color": "#34d399"},
            {"id": "grid_st_pack", "name": "4号成品打包位", "x": 6.4, "y": 2.1, "dock_yaw": -1.5708, "color": "#f472b6"},
            {"id": "grid_st_out", "name": "5号出库发运位", "x": 2.1, "y": -6.4, "dock_yaw": 3.14159, "color": "#f97316"},
            {"id": "grid_st_charge", "name": "九宫格极速充电桩", "x": -2.1, "y": -6.4, "dock_yaw": -1.5708, "color": "#10b981"},
            {"id": "grid_st_idle", "name": "路网待命调度中心", "x": 2.1, "y": -2.1, "dock_yaw": 0.0, "color": "#06b6d4"}
        ],
        "nodes": {
            "N_G_R1_C1": (-6.4, 6.4), "N_G_R1_C2": (-2.1, 6.4), "N_G_R1_C3": (2.1, 6.4), "N_G_R1_C4": (6.4, 6.4),
            "N_G_R2_C1": (-6.4, 2.1), "N_G_R2_C2": (-2.1, 2.1), "N_G_R2_C3": (2.1, 2.1), "N_G_R2_C4": (6.4, 2.1),
            "N_G_R3_C1": (-6.4, -2.1), "N_G_R3_C2": (-2.1, -2.1), "N_G_R3_C3": (2.1, -2.1), "N_G_R3_C4": (6.4, -2.1),
            "N_G_R4_C1": (-6.4, -6.4), "N_G_R4_C2": (-2.1, -6.4), "N_G_R4_C3": (2.1, -6.4), "N_G_R4_C4": (6.4, -6.4)
        },
        "connections": [
            ("N_G_R1_C1", "N_G_R1_C2"), ("N_G_R1_C2", "N_G_R1_C3"), ("N_G_R1_C3", "N_G_R1_C4"),
            ("N_G_R2_C1", "N_G_R2_C2"), ("N_G_R2_C2", "N_G_R2_C3"), ("N_G_R2_C3", "N_G_R2_C4"),
            ("N_G_R3_C1", "N_G_R3_C2"), ("N_G_R3_C2", "N_G_R3_C3"), ("N_G_R3_C3", "N_G_R3_C4"),
            ("N_G_R4_C1", "N_G_R4_C2"), ("N_G_R4_C2", "N_G_R4_C3"), ("N_G_R4_C3", "N_G_R4_C4"),
            ("N_G_R1_C1", "N_G_R2_C1"), ("N_G_R2_C1", "N_G_R3_C1"), ("N_G_R3_C1", "N_G_R4_C1"),
            ("N_G_R1_C2", "N_G_R2_C2"), ("N_G_R2_C2", "N_G_R3_C2"), ("N_G_R3_C2", "N_G_R4_C2"),
            ("N_G_R1_C3", "N_G_R2_C3"), ("N_G_R2_C3", "N_G_R3_C3"), ("N_G_R3_C3", "N_G_R4_C3"),
            ("N_G_R1_C4", "N_G_R2_C4"), ("N_G_R2_C4", "N_G_R3_C4"), ("N_G_R3_C4", "N_G_R4_C4")
        ]
    },

    "standard_cross": {
        "id": "standard_cross",
        "name": "标准十字仓储物流中心",
        "description": "四大立体货架区 (原材料/半成品/成品/辅料)，中央十字通道 + 外围大环线",
        "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "shelves": [
            {"name": "货架区 A (原材料)", "x1": 1.8, "y1": 1.8, "x2": 5.0, "y2": 3.8, "color": "#1f2937", "border": "#3b82f6"},
            {"name": "货架区 B (半成品)", "x1": -5.0, "y1": 1.8, "x2": -1.8, "y2": 3.8, "color": "#1f2937", "border": "#8b5cf6"},
            {"name": "货架区 C (成品库)", "x1": 1.8, "y1": -3.8, "x2": 5.0, "y2": -1.8, "color": "#1f2937", "border": "#10b981"},
            {"name": "货架区 D (辅料库)", "x1": -5.0, "y1": -3.8, "x2": -1.8, "y2": -1.8, "color": "#1f2937", "border": "#f59e0b"}
        ],
        "walls": [
            (-7.5, -7.5, 7.5, -7.5), (7.5, -7.5, 7.5, 7.5), (7.5, 7.5, -7.5, 7.5), (-7.5, 7.5, -7.5, -7.5),
            (1.8, 1.8, 5.0, 1.8), (1.8, 3.8, 5.0, 3.8), (1.8, 1.8, 1.8, 3.8), (5.0, 1.8, 5.0, 3.8),
            (-5.0, 1.8, -1.8, 1.8), (-5.0, 3.8, -1.8, 3.8), (-5.0, 1.8, -5.0, 3.8), (-1.8, 1.8, -1.8, 3.8),
            (1.8, -1.8, 5.0, -1.8), (1.8, -3.8, 5.0, -3.8), (1.8, -1.8, 1.8, -3.8), (5.0, -1.8, 5.0, -3.8),
            (-5.0, -1.8, -1.8, -1.8), (-5.0, -3.8, -1.8, -3.8), (-5.0, -1.8, -5.0, -3.8), (-1.8, -1.8, -1.8, -3.8)
        ],
        "stations": [
            {"id": "st_a", "name": "工位 A (上料)", "x": 4.5, "y": 0.0, "dock_yaw": 0.0, "color": "#58a6ff"},
            {"id": "st_b", "name": "工位 B (出库)", "x": -4.5, "y": 0.0, "dock_yaw": 3.14159, "color": "#bc8cff"},
            {"id": "st_charge", "name": "自动充电桩", "x": 0.0, "y": -5.5, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "st_c", "name": "工位 C (质检)", "x": 0.0, "y": 4.0, "dock_yaw": 1.5708, "color": "#e3b341"},
            {"id": "st_idle", "name": "待命中心", "x": 0.0, "y": 0.0, "dock_yaw": 0.0, "color": "#f0883e"}
        ],
        "nodes": {
            "N_ORIGIN": (0.0, 0.0),
            "N_STATION_A": (4.5, 0.0),
            "N_STATION_B": (-4.5, 0.0),
            "N_CHARGING": (0.0, -5.5),
            "N_STATION_C": (0.0, 4.0),
            "N_CROSS_N": (0.0, 5.5),
            "N_CROSS_E": (6.2, 0.0),
            "N_CROSS_W": (-6.2, 0.0),
            "N_CORNER_NE": (6.2, 5.5),
            "N_CORNER_NW": (-6.2, 5.5),
            "N_CORNER_SE": (6.2, -5.5),
            "N_CORNER_SW": (-6.2, -5.5)
        },
        "connections": [
            ("N_ORIGIN", "N_STATION_C"), ("N_STATION_C", "N_CROSS_N"),
            ("N_ORIGIN", "N_CHARGING"),
            ("N_ORIGIN", "N_STATION_A"), ("N_STATION_A", "N_CROSS_E"),
            ("N_ORIGIN", "N_STATION_B"), ("N_STATION_B", "N_CROSS_W"),
            ("N_CROSS_N", "N_CORNER_NE"), ("N_CORNER_NE", "N_CROSS_E"),
            ("N_CROSS_E", "N_CORNER_SE"), ("N_CORNER_SE", "N_CHARGING"),
            ("N_CHARGING", "N_CORNER_SW"), ("N_CORNER_SW", "N_CROSS_W"),
            ("N_CROSS_W", "N_CORNER_NW"), ("N_CORNER_NW", "N_CROSS_N")
        ]
    },

    "narrow_aisle": {
        "id": "narrow_aisle",
        "name": "高密窄巷道立体库",
        "description": "多排高架垂直货架阵列，包含 3 条典型窄巷道与南北双向出入库主干道",
        "origin": {"x": 0.0, "y": -5.0, "yaw": 0.0},
        "shelves": [
            {"name": "1号高架排架", "x1": -5.6, "y1": -3.2, "x2": -4.4, "y2": 3.2, "color": "#1f2937", "border": "#38bdf8"},
            {"name": "2号高架排架", "x1": -2.6, "y1": -3.2, "x2": -1.4, "y2": 3.2, "color": "#1f2937", "border": "#818cf8"},
            {"name": "3号高架排架", "x1": 1.4, "y1": -3.2, "x2": 2.6, "y2": 3.2, "color": "#1f2937", "border": "#34d399"},
            {"name": "4号高架排架", "x1": 4.4, "y1": -3.2, "x2": 5.6, "y2": 3.2, "color": "#1f2937", "border": "#fbbf24"}
        ],
        "walls": [
            (-7.5, -7.5, 7.5, -7.5), (7.5, -7.5, 7.5, 7.5), (7.5, 7.5, -7.5, 7.5), (-7.5, 7.5, -7.5, -7.5),
            (-5.6, -3.2, -4.4, -3.2), (-5.6, 3.2, -4.4, 3.2), (-5.6, -3.2, -5.6, 3.2), (-4.4, -3.2, -4.4, 3.2),
            (-2.6, -3.2, -1.4, -3.2), (-2.6, 3.2, -1.4, 3.2), (-2.6, -3.2, -2.6, 3.2), (-1.4, -3.2, -1.4, 3.2),
            (1.4, -3.2, 2.6, -3.2), (1.4, 3.2, 2.6, 3.2), (1.4, -3.2, 1.4, 3.2), (2.6, -3.2, 2.6, 3.2),
            (4.4, -3.2, 5.6, -3.2), (4.4, 3.2, 5.6, 3.2), (4.4, -3.2, 4.4, 3.2), (5.6, -3.2, 5.6, 3.2)
        ],
        "stations": [
            {"id": "vna_st1", "name": "1号巷道进料口", "x": -3.5, "y": 4.0, "dock_yaw": 1.5708, "color": "#58a6ff"},
            {"id": "vna_st2", "name": "2号巷道存储位", "x": 0.0, "y": 2.0, "dock_yaw": 1.5708, "color": "#bc8cff"},
            {"id": "vna_st3", "name": "3号巷道拣货位", "x": 3.5, "y": 4.0, "dock_yaw": 1.5708, "color": "#e3b341"},
            {"id": "vna_charge", "name": "高架专用充电机", "x": -3.5, "y": -5.0, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "vna_buffer", "name": "出库主缓存站", "x": 3.5, "y": -5.0, "dock_yaw": 0.0, "color": "#ec4899"},
            {"id": "vna_idle", "name": "窄巷道待命位", "x": 0.0, "y": -5.0, "dock_yaw": 0.0, "color": "#f0883e"}
        ],
        "nodes": {
            "N_VNA_1_TOP": (-3.5, 5.0),
            "N_VNA_1_STATION": (-3.5, 4.0),
            "N_VNA_1_MID": (-3.5, 0.0),
            "N_VNA_1_BOT": (-3.5, -5.0),
            "N_VNA_2_TOP": (0.0, 5.0),
            "N_VNA_2_STATION": (0.0, 2.0),
            "N_VNA_2_MID": (0.0, 0.0),
            "N_VNA_2_BOT": (0.0, -5.0),
            "N_VNA_3_TOP": (3.5, 5.0),
            "N_VNA_3_STATION": (3.5, 4.0),
            "N_VNA_3_MID": (3.5, 0.0),
            "N_VNA_3_BOT": (3.5, -5.0),
            "N_VNA_W_TOP": (-6.2, 5.0),
            "N_VNA_W_BOT": (-6.2, -5.0),
            "N_VNA_E_TOP": (6.2, 5.0),
            "N_VNA_E_BOT": (6.2, -5.0)
        },
        "connections": [
            ("N_VNA_W_TOP", "N_VNA_1_TOP"), ("N_VNA_1_TOP", "N_VNA_2_TOP"), ("N_VNA_2_TOP", "N_VNA_3_TOP"), ("N_VNA_3_TOP", "N_VNA_E_TOP"),
            ("N_VNA_W_BOT", "N_VNA_1_BOT"), ("N_VNA_1_BOT", "N_VNA_2_BOT"), ("N_VNA_2_BOT", "N_VNA_3_BOT"), ("N_VNA_3_BOT", "N_VNA_E_BOT"),
            ("N_VNA_W_TOP", "N_VNA_W_BOT"), ("N_VNA_E_TOP", "N_VNA_E_BOT"),
            ("N_VNA_1_TOP", "N_VNA_1_STATION"), ("N_VNA_1_STATION", "N_VNA_1_MID"), ("N_VNA_1_MID", "N_VNA_1_BOT"),
            ("N_VNA_2_TOP", "N_VNA_2_STATION"), ("N_VNA_2_STATION", "N_VNA_2_MID"), ("N_VNA_2_MID", "N_VNA_2_BOT"),
            ("N_VNA_3_TOP", "N_VNA_3_STATION"), ("N_VNA_3_STATION", "N_VNA_3_MID"), ("N_VNA_3_MID", "N_VNA_3_BOT")
        ]
    },

    "rect_loop": {
        "id": "rect_loop",
        "name": "长方形环线制造车间",
        "description": "长方形柔性生产大环线，包含中心双岛制造单元与中央南北直通旁路",
        "origin": {"x": -3.0, "y": -4.0, "yaw": 0.0},
        "shelves": [
            {"name": "精密 CNC 机械加工岛", "x1": -4.6, "y1": -2.0, "x2": -1.6, "y2": 2.0, "color": "#1e293b", "border": "#06b6d4"},
            {"name": "工业机器人柔性装配岛", "x1": 1.6, "y1": -2.0, "x2": 4.6, "y2": 2.0, "color": "#1e293b", "border": "#10b981"}
        ],
        "walls": [
            (-7.5, -5.5, 7.5, -5.5), (7.5, -5.5, 7.5, 5.5), (7.5, 5.5, -7.5, 5.5), (-7.5, 5.5, -7.5, -5.5),
            (-4.6, -2.0, -1.6, -2.0), (-4.6, 2.0, -1.6, 2.0), (-4.6, -2.0, -4.6, 2.0), (-1.6, -2.0, -1.6, 2.0),
            (1.6, -2.0, 4.6, -2.0), (1.6, 2.0, 4.6, 2.0), (1.6, -2.0, 1.6, 2.0), (4.6, -2.0, 4.6, 2.0)
        ],
        "stations": [
            {"id": "rect_st_load", "name": "1号原料上线工位", "x": -3.0, "y": 4.0, "dock_yaw": 0.0, "color": "#38bdf8"},
            {"id": "rect_st_asm", "name": "2号柔性装配工位", "x": 3.0, "y": 4.0, "dock_yaw": 0.0, "color": "#c084fc"},
            {"id": "rect_st_aoi", "name": "3号AOI智能终检位", "x": 6.0, "y": 0.0, "dock_yaw": -1.5708, "color": "#fbbf24"},
            {"id": "rect_st_pack", "name": "4号成品下线包装", "x": 3.0, "y": -4.0, "dock_yaw": 3.14159, "color": "#10b981"},
            {"id": "rect_st_charge", "name": "5号环线专用快充桩", "x": -3.0, "y": -4.0, "dock_yaw": -1.5708, "color": "#22c55e"},
            {"id": "rect_st_buffer", "name": "6号物料缓存等待位", "x": -6.0, "y": 0.0, "dock_yaw": 1.5708, "color": "#f97316"},
            {"id": "rect_st_center", "name": "环线中央调度中枢", "x": 0.0, "y": 0.0, "dock_yaw": 0.0, "color": "#06b6d4"}
        ],
        "nodes": {
            "N_L_NW": (-6.0, 4.0), "N_L_N_W": (-3.0, 4.0), "N_L_N_MID": (0.0, 4.0), "N_L_N_E": (3.0, 4.0), "N_L_NE": (6.0, 4.0),
            "N_L_W_MID": (-6.0, 0.0), "N_L_CENTER": (0.0, 0.0), "N_L_E_MID": (6.0, 0.0),
            "N_L_SW": (-6.0, -4.0), "N_L_S_W": (-3.0, -4.0), "N_L_S_MID": (0.0, -4.0), "N_L_S_E": (3.0, -4.0), "N_L_SE": (6.0, -4.0)
        },
        "connections": [
            ("N_L_NW", "N_L_N_W"), ("N_L_N_W", "N_L_N_MID"), ("N_L_N_MID", "N_L_N_E"), ("N_L_N_E", "N_L_NE"),
            ("N_L_NE", "N_L_E_MID"), ("N_L_E_MID", "N_L_SE"),
            ("N_L_SE", "N_L_S_E"), ("N_L_S_E", "N_L_S_MID"), ("N_L_S_MID", "N_L_S_W"), ("N_L_S_W", "N_L_SW"),
            ("N_L_SW", "N_L_W_MID"), ("N_L_W_MID", "N_L_NW"),
            ("N_L_N_MID", "N_L_CENTER"), ("N_L_CENTER", "N_L_S_MID")
        ]
    },

    "fms_workshop": {
        "id": "fms_workshop",
        "name": "自动化柔性制造车间",
        "description": "岛式生产布局（CNC机加工/SMT贴片/机器人装配/智能测试/中央缓存塔）",
        "origin": {"x": 0.0, "y": 3.5, "yaw": 0.0},
        "shelves": [
            {"name": "CNC 机械加工岛", "x1": 2.0, "y1": 2.0, "x2": 5.2, "y2": 4.5, "color": "#1f2937", "border": "#06b6d4"},
            {"name": "SMT 贴片流水线", "x1": -5.2, "y1": 2.0, "x2": -2.0, "y2": 4.5, "color": "#1f2937", "border": "#ec4899"},
            {"name": "机器人装配岛", "x1": 2.0, "y1": -4.5, "x2": 5.2, "y2": -2.0, "color": "#1f2937", "border": "#10b981"},
            {"name": "智能质检测试岛", "x1": -5.2, "y1": -4.5, "x2": -2.0, "y2": -2.0, "color": "#1f2937", "border": "#f59e0b"},
            {"name": "中央立体缓存塔", "x1": -1.2, "y1": -1.0, "x2": 1.2, "y2": 1.0, "color": "#1f2937", "border": "#8b5cf6"}
        ],
        "walls": [
            (-7.5, -7.5, 7.5, -7.5), (7.5, -7.5, 7.5, 7.5), (7.5, 7.5, -7.5, 7.5), (-7.5, 7.5, -7.5, -7.5),
            (2.0, 2.0, 5.2, 2.0), (2.0, 4.5, 5.2, 4.5), (2.0, 2.0, 2.0, 4.5), (5.2, 2.0, 5.2, 4.5),
            (-5.2, 2.0, -2.0, 2.0), (-5.2, 4.5, -2.0, 4.5), (-5.2, 2.0, -5.2, 4.5), (-2.0, 2.0, -2.0, 4.5),
            (2.0, -4.5, 5.2, -4.5), (2.0, -2.0, 5.2, -2.0), (2.0, -4.5, 2.0, -2.0), (5.2, -4.5, 5.2, -2.0),
            (-5.2, -4.5, -2.0, -4.5), (-5.2, -2.0, -2.0, -2.0), (-5.2, -4.5, -5.2, -2.0), (-2.0, -4.5, -2.0, -2.0),
            (-1.2, -1.0, 1.2, -1.0), (-1.2, 1.0, 1.2, 1.0), (-1.2, -1.0, -1.2, 1.0), (1.2, -1.0, 1.2, 1.0)
        ],
        "stations": [
            {"id": "fms_smt", "name": "SMT 供料工位", "x": -3.5, "y": 0.0, "dock_yaw": 1.5708, "color": "#ec4899"},
            {"id": "fms_cnc", "name": "CNC 进料工位", "x": 3.5, "y": 0.0, "dock_yaw": 1.5708, "color": "#06b6d4"},
            {"id": "fms_robot", "name": "机器人装配位", "x": 3.5, "y": -0.0, "dock_yaw": -1.5708, "color": "#10b981"},
            {"id": "fms_qc", "name": "质检包装工位", "x": -3.5, "y": -0.0, "dock_yaw": -1.5708, "color": "#f59e0b"},
            {"id": "fms_charge", "name": "自动化快充岛", "x": 0.0, "y": -5.5, "dock_yaw": -1.5708, "color": "#3fb950"},
            {"id": "fms_idle", "name": "车间待命总站", "x": 0.0, "y": 3.5, "dock_yaw": 0.0, "color": "#8b5cf6"}
        ],
        "nodes": {
            "N_FMS_N_MID": (0.0, 5.5),
            "N_FMS_S_MID": (0.0, -5.5),
            "N_FMS_NE": (6.2, 5.5),
            "N_FMS_NW": (-6.2, 5.5),
            "N_FMS_SE": (6.2, -5.5),
            "N_FMS_SW": (-6.2, -5.5),
            "N_FMS_CROSS_W": (-6.2, 0.0),
            "N_FMS_STATION_SMT": (-3.5, 0.0),
            "N_FMS_LOOP_W": (-1.6, 0.0),
            "N_FMS_CROSS_E": (6.2, 0.0),
            "N_FMS_STATION_CNC": (3.5, 0.0),
            "N_FMS_LOOP_E": (1.6, 0.0),
            "N_FMS_LOOP_NW": (-1.6, 1.5),
            "N_FMS_LOOP_NE": (1.6, 1.5),
            "N_FMS_LOOP_SW": (-1.6, -1.5),
            "N_FMS_LOOP_SE": (1.6, -1.5),
            "N_FMS_STATION_IDLE": (0.0, 3.5)
        },
        "connections": [
            ("N_FMS_NW", "N_FMS_N_MID"), ("N_FMS_N_MID", "N_FMS_NE"),
            ("N_FMS_NE", "N_FMS_CROSS_E"), ("N_FMS_CROSS_E", "N_FMS_SE"),
            ("N_FMS_SE", "N_FMS_S_MID"), ("N_FMS_S_MID", "N_FMS_SW"),
            ("N_FMS_SW", "N_FMS_CROSS_W"), ("N_FMS_CROSS_W", "N_FMS_NW"),
            ("N_FMS_CROSS_W", "N_FMS_STATION_SMT"), ("N_FMS_STATION_SMT", "N_FMS_LOOP_W"),
            ("N_FMS_CROSS_E", "N_FMS_STATION_CNC"), ("N_FMS_STATION_CNC", "N_FMS_LOOP_E"),
            ("N_FMS_LOOP_W", "N_FMS_LOOP_NW"), ("N_FMS_LOOP_NW", "N_FMS_LOOP_NE"), ("N_FMS_LOOP_NE", "N_FMS_LOOP_E"),
            ("N_FMS_LOOP_W", "N_FMS_LOOP_SW"), ("N_FMS_LOOP_SW", "N_FMS_LOOP_SE"), ("N_FMS_LOOP_SE", "N_FMS_LOOP_E"),
            ("N_FMS_N_MID", "N_FMS_STATION_IDLE"), ("N_FMS_STATION_IDLE", "N_FMS_LOOP_NE"),
            ("N_FMS_S_MID", "N_FMS_LOOP_SE")
        ]
    }
}


class DijkstraPlanner:
    def __init__(self, default_scenario: str = "grid_9_square"):
        self.active_scenario_id = default_scenario
        self.nodes = {}
        self.edges = {}
        self.set_scenario(default_scenario)

    def set_scenario(self, scenario_id: str):
        if scenario_id not in SCENARIO_DEFINITIONS:
            scenario_id = "grid_9_square"
        self.active_scenario_id = scenario_id
        sc = SCENARIO_DEFINITIONS[scenario_id]

        self.nodes = dict(sc["nodes"])
        self.edges = {u: [] for u in self.nodes}

        for u, v in sc["connections"]:
            if u in self.nodes and v in self.nodes:
                p_u = self.nodes[u]
                p_v = self.nodes[v]
                dist = math.hypot(p_v[0] - p_u[0], p_v[1] - p_u[1])
                self.edges[u].append((v, dist))
                self.edges[v].append((u, dist))

    def get_scenario_metadata(self) -> Dict[str, Any]:
        sc = SCENARIO_DEFINITIONS[self.active_scenario_id]
        walls = sc.get("walls", [])
        min_x = min([w[0] for w in walls[:4]] + [w[2] for w in walls[:4]]) if len(walls) >= 4 else -7.5
        max_x = max([w[0] for w in walls[:4]] + [w[2] for w in walls[:4]]) if len(walls) >= 4 else 7.5
        min_y = min([w[1] for w in walls[:4]] + [w[3] for w in walls[:4]]) if len(walls) >= 4 else -7.5
        max_y = max([w[1] for w in walls[:4]] + [w[3] for w in walls[:4]]) if len(walls) >= 4 else 7.5
        return {
            "id": sc["id"],
            "name": sc["name"],
            "description": sc["description"],
            "shelves": sc["shelves"],
            "stations": sc["stations"],
            "origin": sc["origin"],
            "bounds": {
                "min_x": min_x,
                "max_x": max_x,
                "min_y": min_y,
                "max_y": max_y
            },
            "all_scenarios": [
                {"id": k, "name": v["name"], "description": v["description"]}
                for k, v in SCENARIO_DEFINITIONS.items()
            ]
        }

    def get_walls(self) -> List[Tuple[float, float, float, float]]:
        return list(SCENARIO_DEFINITIONS[self.active_scenario_id]["walls"])

    def get_stations(self) -> List[Dict[str, Any]]:
        return list(SCENARIO_DEFINITIONS[self.active_scenario_id]["stations"])

    def get_topology(self) -> Dict[str, Any]:
        """Export topological nodes and routes for Web rendering."""
        edge_list = []
        seen = set()
        for u, neighbors in self.edges.items():
            for v, dist in neighbors:
                edge_key = tuple(sorted([u, v]))
                if edge_key not in seen:
                    seen.add(edge_key)
                    edge_list.append({
                        "from": u,
                        "to": v,
                        "p1": {"x": self.nodes[u][0], "y": self.nodes[u][1]},
                        "p2": {"x": self.nodes[v][0], "y": self.nodes[v][1]},
                        "distance": round(dist, 2)
                    })

        return {
            "scenario_id": self.active_scenario_id,
            "nodes": {k: {"x": v[0], "y": v[1], "name": k} for k, v in self.nodes.items()},
            "edges": edge_list
        }

    def _find_nearest_node(self, pt: Tuple[float, float]) -> str:
        best_node = None
        min_d = float('inf')
        for node_id, pos in self.nodes.items():
            d = math.hypot(pos[0] - pt[0], pos[1] - pt[1])
            if d < min_d:
                min_d = d
                best_node = node_id
        return best_node

    @staticmethod
    def _is_edge_blocked(p1: Tuple[float, float], p2: Tuple[float, float], obstacles: Any) -> bool:
        """Check if topological edge between p1 and p2 is blocked by any dynamic obstacle."""
        for obs in obstacles:
            if isinstance(obs, dict):
                cx = float(obs.get("x", 0.0))
                cy = float(obs.get("y", 0.0))
                w = float(obs.get("w", 0.8))
                h = float(obs.get("h", 0.8))
                radius = math.hypot(w, h) / 2.0 + 0.35
            else:
                ox1, oy1, ox2, oy2 = obs
                cx = (ox1 + ox2) / 2.0
                cy = (oy1 + oy2) / 2.0
                radius = 0.65

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            l2 = dx*dx + dy*dy
            if l2 == 0:
                d = math.hypot(cx - p1[0], cy - p1[1])
            else:
                t = max(0.0, min(1.0, ((cx - p1[0]) * dx + (cy - p1[1]) * dy) / l2))
                proj_x = p1[0] + t * dx
                proj_y = p1[1] + t * dy
                d = math.hypot(cx - proj_x, cy - proj_y)

            if d < radius:
                return True
        return False

    def plan(self, start_pt: Tuple[float, float], goal_pt: Tuple[float, float], obstacles: List[Tuple[float, float, float, float]] = None) -> List[Tuple[float, float]]:
        obstacles = obstacles or []
        start_node = self._find_nearest_node(start_pt)
        goal_node = self._find_nearest_node(goal_pt)

        if not start_node or not goal_node:
            return [start_pt, goal_pt]

        if start_node == goal_node:
            return [start_pt, self.nodes[start_node], goal_pt]

        # Dijkstra Min-Heap Priority Queue
        dist_map = {node: float('inf') for node in self.nodes}
        parent_map = {}
        dist_map[start_node] = 0.0

        pq = [(0.0, start_node)]

        while pq:
            d_curr, u = heapq.heappop(pq)
            if d_curr > dist_map[u]:
                continue

            if u == goal_node:
                break

            for v, weight in self.edges.get(u, []):
                if obstacles and self._is_edge_blocked(self.nodes[u], self.nodes[v], obstacles):
                    continue

                new_dist = d_curr + weight
                if new_dist < dist_map[v]:
                    dist_map[v] = new_dist
                    parent_map[v] = u
                    heapq.heappush(pq, (new_dist, v))

        if goal_node not in parent_map and start_node != goal_node:
            return [start_pt, goal_pt]

        # Reconstruct path
        node_path = []
        curr = goal_node
        while curr in parent_map:
            node_path.append(curr)
            curr = parent_map[curr]
        node_path.append(start_node)
        node_path.reverse()

        # Build clean, non-redundant waypoint list
        raw_coords = [start_pt]
        for nid in node_path:
            raw_coords.append(self.nodes[nid])
        raw_coords.append(goal_pt)

        clean_coords = [raw_coords[0]]
        for pt in raw_coords[1:]:
            if math.hypot(pt[0] - clean_coords[-1][0], pt[1] - clean_coords[-1][1]) > 0.08:
                clean_coords.append(pt)

        return clean_coords
