#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
88_g2_pick_three_cycles_fixed_height_shortest_axis.py

86_g2_pick_three_cycles.py 的稳定升级入口。
仅修复两类问题：
1. 三循环固定使用同一个桌面Z基准和40 mm产品高度，杜绝24.8/25 mm边界跳变。
2. 长方形产品长轴按180度等价处理，在 yaw / yaw+180 / yaw-180 中选择
   相对当前夹爪姿态变化最小的垂直目标方向。

其他内容由同目录的86版原样执行，包括Waypoint、抓取偏移、闭环、安全限制、
三循环、腰部转身/返回和夹爪控制。
"""

from __future__ import annotations

import copy
import importlib.util
import math
import sys
from pathlib import Path
from typing import Any, Dict, Optional

BASE_FILE = Path(__file__).with_name("86_g2_pick_three_cycles.py")
if not BASE_FILE.exists():
    raise RuntimeError(
        f"未找到基础程序: {BASE_FILE}\n"
        "请将88版与已验证的86_g2_pick_three_cycles.py放在同一目录。"
    )

spec = importlib.util.spec_from_file_location("g2_cycle86_base", str(BASE_FILE))
if spec is None or spec.loader is None:
    raise RuntimeError("无法加载86版程序")
g2 = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = g2
spec.loader.exec_module(g2)

# -----------------------------------------------------------------------------
# 88版高度参数
# -----------------------------------------------------------------------------
FIXED_PRODUCT_HEIGHT_M = 0.040
# None：第1轮首次成功估计桌面Z后锁定，后续循环都使用同一值。
# 若以后完成独立标定，也可直接填绝对Base Z，例如 0.72350。
CALIBRATED_TABLE_Z_M: Optional[float] = None
TABLE_Z_SANITY_DELTA_M = 0.020

_CALIBRATION: Dict[str, Any] = {
    "table_z_m": CALIBRATED_TABLE_Z_M,
    "source": "configured" if CALIBRATED_TABLE_Z_M is not None else None,
    "cycle_first_raw_table_z_m": None,
    "samples": [],
}

_original_detect_product = g2.detect_product
_original_vertical_tool_orientation = g2.vertical_tool_orientation


def _current_cycle(report: Dict[str, Any]) -> int:
    try:
        return int(report.get("current_cycle_index", 1))
    except Exception:
        return 1


def _set_report_calibration(report: Dict[str, Any], raw_table_z: float) -> None:
    report["height_model_v88"] = {
        "fixed_product_height_m": FIXED_PRODUCT_HEIGHT_M,
        "calibrated_table_z_m": _CALIBRATION["table_z_m"],
        "calibration_source": _CALIBRATION["source"],
        "current_raw_table_z_m": raw_table_z,
        "raw_minus_calibrated_m": raw_table_z - _CALIBRATION["table_z_m"],
        "mode": "fixed_table_z_plus_fixed_40mm_product_height",
    }


def detect_product_fixed_height_shortest_axis(model, camera, tf_api, report):
    """复用86版检测，只稳定高度基准并选择180度等价的最短姿态方向。"""
    data = _original_detect_product(model, camera, tf_api, report)
    cycle_index = _current_cycle(report)

    raw_table_z = data.get("table_z_m")
    if raw_table_z is None:
        raise RuntimeError("88版无法标定桌面Z：本轮桌面Z估计为空")
    raw_table_z = float(raw_table_z)

    # 第一次获得有效桌面Z时锁定；后续循环不再更新基准。
    if _CALIBRATION["table_z_m"] is None:
        _CALIBRATION["table_z_m"] = raw_table_z
        _CALIBRATION["source"] = "first_cycle_locked_table_z"
        _CALIBRATION["cycle_first_raw_table_z_m"] = raw_table_z
        print(
            f"[88高度标定] 第{cycle_index}轮锁定桌面Z="
            f"{raw_table_z:.5f}m；后续循环保持不变"
        )
    calibrated_table_z = float(_CALIBRATION["table_z_m"])
    _CALIBRATION["samples"].append({
        "cycle_index": cycle_index,
        "raw_table_z_m": raw_table_z,
        "calibrated_table_z_m": calibrated_table_z,
        "delta_m": raw_table_z - calibrated_table_z,
    })

    if abs(raw_table_z - calibrated_table_z) > TABLE_Z_SANITY_DELTA_M:
        raise RuntimeError(
            f"第{cycle_index}轮桌面Z相对固定标定值偏差过大: "
            f"raw={raw_table_z:.5f}m, calibrated={calibrated_table_z:.5f}m"
        )

    # 强制每轮使用同一桌面Z和固定40mm产品高度。
    data["raw_table_z_m"] = raw_table_z
    data["table_z_m"] = calibrated_table_z
    data["base_xyz"] = list(data["base_xyz"])
    raw_product_z = float(data["base_xyz"][2])
    data["raw_product_z_m"] = raw_product_z
    data["base_xyz"][2] = calibrated_table_z + FIXED_PRODUCT_HEIGHT_M
    data["fixed_product_height_m"] = FIXED_PRODUCT_HEIGHT_M
    data["height_source"] = "fixed_40mm_above_locked_table_z"

    # 长轴具有180度等价性。以当前右臂实际姿态为参考选择最短目标。
    if data.get("angle_valid"):
        raw_yaw = float(data["yaw_rad"])
        current_end = g2.wait_pose(tf_api, g2.RIGHT_FRAME)
        candidate_yaws = [raw_yaw, raw_yaw + math.pi, raw_yaw - math.pi]
        candidates = []
        for candidate_yaw in candidate_yaws:
            candidate_q = _original_vertical_tool_orientation(
                candidate_yaw + math.radians(g2.GRIPPER_YAW_OFFSET_DEG)
            )
            error_deg = g2.q_error_deg(current_end.orientation, candidate_q)
            candidates.append({
                "yaw_rad": candidate_yaw,
                "yaw_deg": math.degrees(candidate_yaw),
                "orientation_change_deg": error_deg,
            })
        chosen = min(candidates, key=lambda item: item["orientation_change_deg"])
        chosen_yaw = float(chosen["yaw_rad"])
        # main() 后续还会加一次 GRIPPER_YAW_OFFSET_DEG，因此这里只写回未加offset的轴角。
        data["raw_yaw_rad"] = raw_yaw
        data["raw_yaw_deg"] = math.degrees(raw_yaw)
        data["yaw_rad"] = chosen_yaw
        data["yaw_deg"] = math.degrees(chosen_yaw)
        data["axis_equivalent_candidates"] = candidates
        data["axis_selection_source"] = "minimum_orientation_change_over_yaw_plus_minus_180"
        print(
            f"[88长轴最短路径] 原始yaw={math.degrees(raw_yaw):.2f}deg, "
            f"选定yaw={math.degrees(chosen_yaw):.2f}deg, "
            f"预计姿态变化={chosen['orientation_change_deg']:.2f}deg"
        )

    _set_report_calibration(report, raw_table_z)
    report.setdefault("height_calibration_history_v88", []).append(
        copy.deepcopy(_CALIBRATION["samples"][-1])
    )
    report["detection_v88"] = copy.deepcopy(data)
    g2.save(report)

    print(
        f"[88固定高度] cycle={cycle_index}, raw_product_z={raw_product_z:.5f}m, "
        f"raw_table_z={raw_table_z:.5f}m, fixed_table_z={calibrated_table_z:.5f}m, "
        f"effective_product_z={data['base_xyz'][2]:.5f}m"
    )
    return data


# 替换检测入口；86版main中的高度公式由修正后的data自然得到固定40mm结果。
g2.detect_product = detect_product_fixed_height_shortest_axis

# 明确固定产品高度，防止86版的边界逻辑再次切换。
g2.PRODUCT_NOMINAL_HEIGHT_M = FIXED_PRODUCT_HEIGHT_M
g2.PRODUCT_HEIGHT_VALID_MIN_M = FIXED_PRODUCT_HEIGHT_M - 1e-6
g2.PRODUCT_HEIGHT_VALID_MAX_M = FIXED_PRODUCT_HEIGHT_M + 1e-6


if __name__ == "__main__":
    print("=" * 78)
    print("88_g2_pick_three_cycles_fixed_height_shortest_axis.py")
    print("高度模式：三循环锁定同一个桌面Z，固定产品高度40mm")
    print("角度模式：yaw / yaw+180 / yaw-180 中选择最短姿态路径")
    print("其余控制逻辑、Waypoint、抓取偏移和安全限制继承已验证86版")
    print("=" * 78)
    g2.main()
