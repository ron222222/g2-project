#!/usr/bin/python3.10
# -*- coding: utf-8 -*-
"""
90_wait_until_product_stable.py

产品稳定门控模块（独立文件，不启动机器人运动）。
用于把经过验证的抓取程序中的“单次detect_product后立即抓取”替换为：
持续YOLO检测，直到产品位置连续稳定，再把稳定检测结果交给后续动作。

注意：本文件不包含用户本机86/88/89版的完整机器人运动源码，因为该源码
未随当前请求上传。它提供可直接复用、无第三方本地程序依赖的稳定门控实现。
"""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, asdict
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple


# ========================= 参数 =========================
WAIT_UNTIL_PRODUCT_STABLE = True
STABLE_FRAMES_REQUIRED = 20
STABILITY_WINDOW_SIZE = 20
MAX_STABLE_Z_RANGE_M = 0.015
MAX_STABLE_XY_STD_M = 0.005
MAX_STABLE_YAW_STD_DEG = 6.0
STABLE_MIN_CONFIDENCE = 0.35
DETECTION_TIMEOUT_S = 120.0
DETECTION_RETRY_SLEEP_S = 0.05

# 固定高度模型，三循环不再因24.8/25 mm边界切换
FIXED_PRODUCT_HEIGHT_M = 0.040
LOCK_TABLE_Z_ACROSS_CYCLES = True
TABLE_Z_SANITY_DELTA_M = 0.020


@dataclass
class StableDetectionResult:
    base_xyz: List[float]
    confidence: float
    yaw_rad: float
    yaw_deg: float
    table_z_m: float
    fixed_product_height_m: float
    stable_frames: int
    z_range_m: float
    x_std_m: float
    y_std_m: float
    yaw_std_deg: float
    raw_detection: Dict[str, Any]


class ProductStabilityGate:
    """跨循环稳定检测门控。

    detect_once 必须返回字典，至少包含：
      base_xyz: [x, y, z]
      confidence: float
      angle_valid: bool
      yaw_rad 或 yaw_deg
      table_z_m: float

    refresh_ui(stage, info) 可选，用于持续刷新YOLO大画面。
    """

    def __init__(self) -> None:
        self.locked_table_z_m: Optional[float] = None

    @staticmethod
    def _axis_mean_rad(values: Sequence[float]) -> float:
        """180度等价长轴平均。"""
        s = statistics.fmean(math.sin(2.0 * v) for v in values)
        c = statistics.fmean(math.cos(2.0 * v) for v in values)
        return 0.5 * math.atan2(s, c)

    @staticmethod
    def _axis_std_deg(values: Sequence[float]) -> float:
        """180度等价长轴的近似标准差。"""
        if len(values) < 2:
            return 0.0
        mean_axis = ProductStabilityGate._axis_mean_rad(values)
        diffs_deg = []
        for value in values:
            diff = math.degrees(value - mean_axis)
            while diff > 90.0:
                diff -= 180.0
            while diff < -90.0:
                diff += 180.0
            diffs_deg.append(diff)
        return statistics.pstdev(diffs_deg)

    def _resolve_table_z(self, raw_table_z: float) -> float:
        if not LOCK_TABLE_Z_ACROSS_CYCLES:
            return raw_table_z
        if self.locked_table_z_m is None:
            self.locked_table_z_m = raw_table_z
            print(f"[稳定门控] 锁定三循环桌面Z={raw_table_z:.5f}m")
            return raw_table_z
        delta = abs(raw_table_z - self.locked_table_z_m)
        if delta > TABLE_Z_SANITY_DELTA_M:
            raise RuntimeError(
                "当前桌面Z与三循环锁定值差异过大: "
                f"raw={raw_table_z:.5f}m, "
                f"locked={self.locked_table_z_m:.5f}m, "
                f"delta={delta:.5f}m"
            )
        return self.locked_table_z_m

    def wait(
        self,
        detect_once: Callable[[], Dict[str, Any]],
        cycle_index: int = 1,
        refresh_ui: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> StableDetectionResult:
        if not WAIT_UNTIL_PRODUCT_STABLE:
            det = detect_once()
            xyz = [float(v) for v in det["base_xyz"]]
            yaw_rad = float(det.get("yaw_rad", math.radians(float(det.get("yaw_deg", 0.0)))))
            table_z = self._resolve_table_z(float(det["table_z_m"]))
            return StableDetectionResult(
                base_xyz=xyz,
                confidence=float(det.get("confidence", 0.0)),
                yaw_rad=yaw_rad,
                yaw_deg=math.degrees(yaw_rad),
                table_z_m=table_z,
                fixed_product_height_m=FIXED_PRODUCT_HEIGHT_M,
                stable_frames=1,
                z_range_m=0.0,
                x_std_m=0.0,
                y_std_m=0.0,
                yaw_std_deg=0.0,
                raw_detection=det,
            )

        samples: Deque[Dict[str, Any]] = deque(maxlen=STABILITY_WINDOW_SIZE)
        deadline = time.monotonic() + DETECTION_TIMEOUT_S
        accepted_total = 0
        rejected_total = 0

        print(
            f"[第{cycle_index}轮稳定门控] 等待产品稳定: "
            f"窗口={STABILITY_WINDOW_SIZE}, "
            f"需要连续稳定={STABLE_FRAMES_REQUIRED}, "
            f"Z极差<={MAX_STABLE_Z_RANGE_M*1000:.1f}mm, "
            f"XY标准差<={MAX_STABLE_XY_STD_M*1000:.1f}mm"
        )

        while time.monotonic() < deadline:
            try:
                det = detect_once()
            except Exception as exc:
                rejected_total += 1
                if refresh_ui:
                    refresh_ui("WAIT_PRODUCT", {
                        "cycle": cycle_index,
                        "message": f"detect error: {exc}",
                        "stable": 0,
                        "required": STABLE_FRAMES_REQUIRED,
                    })
                time.sleep(DETECTION_RETRY_SLEEP_S)
                continue

            confidence = float(det.get("confidence", 0.0))
            angle_valid = bool(det.get("angle_valid", False))
            xyz = det.get("base_xyz")
            raw_table_z = det.get("table_z_m")

            if (
                confidence < STABLE_MIN_CONFIDENCE
                or not angle_valid
                or not isinstance(xyz, (list, tuple))
                or len(xyz) != 3
                or raw_table_z is None
            ):
                samples.clear()
                rejected_total += 1
                if refresh_ui:
                    refresh_ui("WAIT_PRODUCT", {
                        "cycle": cycle_index,
                        "confidence": confidence,
                        "angle_valid": angle_valid,
                        "stable": 0,
                        "required": STABLE_FRAMES_REQUIRED,
                        "message": "target not accepted",
                    })
                time.sleep(DETECTION_RETRY_SLEEP_S)
                continue

            yaw_rad = float(det.get("yaw_rad", math.radians(float(det.get("yaw_deg", 0.0)))))
            sample = {
                "x": float(xyz[0]),
                "y": float(xyz[1]),
                "z": float(xyz[2]),
                "yaw": yaw_rad,
                "confidence": confidence,
                "table_z": float(raw_table_z),
                "raw": det,
            }
            samples.append(sample)
            accepted_total += 1

            xs = [s["x"] for s in samples]
            ys = [s["y"] for s in samples]
            zs = [s["z"] for s in samples]
            yaws = [s["yaw"] for s in samples]

            z_range = max(zs) - min(zs)
            x_std = statistics.pstdev(xs) if len(xs) > 1 else float("inf")
            y_std = statistics.pstdev(ys) if len(ys) > 1 else float("inf")
            yaw_std_deg = self._axis_std_deg(yaws)

            window_full = len(samples) >= STABLE_FRAMES_REQUIRED
            stable = (
                window_full
                and z_range <= MAX_STABLE_Z_RANGE_M
                and x_std <= MAX_STABLE_XY_STD_M
                and y_std <= MAX_STABLE_XY_STD_M
                and yaw_std_deg <= MAX_STABLE_YAW_STD_DEG
            )

            info = {
                "cycle": cycle_index,
                "confidence": confidence,
                "stable": len(samples) if stable else 0,
                "window": len(samples),
                "required": STABLE_FRAMES_REQUIRED,
                "x_std_mm": x_std * 1000.0,
                "y_std_mm": y_std * 1000.0,
                "z_range_mm": z_range * 1000.0,
                "yaw_std_deg": yaw_std_deg,
                "product_xyz": [statistics.median(xs), statistics.median(ys), statistics.median(zs)],
                "message": "STABLE" if stable else "waiting for stable product",
            }
            if refresh_ui:
                refresh_ui("WAIT_PRODUCT_STABLE", info)

            print(
                "\r"
                f"[第{cycle_index}轮] window={len(samples):02d}/{STABLE_FRAMES_REQUIRED}, "
                f"conf={confidence:.3f}, "
                f"Zrange={z_range*1000:5.1f}mm, "
                f"Xstd={x_std*1000:4.1f}mm, "
                f"Ystd={y_std*1000:4.1f}mm, "
                f"YawStd={yaw_std_deg:4.1f}deg",
                end="",
                flush=True,
            )

            if stable:
                print("\n[稳定门控] 产品连续稳定，允许进入抓取动作。")
                table_z = self._resolve_table_z(statistics.median(s["table_z"] for s in samples))
                result = StableDetectionResult(
                    base_xyz=[statistics.median(xs), statistics.median(ys), statistics.median(zs)],
                    confidence=statistics.median(s["confidence"] for s in samples),
                    yaw_rad=self._axis_mean_rad(yaws),
                    yaw_deg=math.degrees(self._axis_mean_rad(yaws)),
                    table_z_m=table_z,
                    fixed_product_height_m=FIXED_PRODUCT_HEIGHT_M,
                    stable_frames=len(samples),
                    z_range_m=z_range,
                    x_std_m=x_std,
                    y_std_m=y_std,
                    yaw_std_deg=yaw_std_deg,
                    raw_detection=samples[-1]["raw"],
                )
                if refresh_ui:
                    refresh_ui("PRODUCT_STABLE_LOCKED", asdict(result))
                return result

            time.sleep(DETECTION_RETRY_SLEEP_S)

        raise RuntimeError(
            "等待产品稳定超时: "
            f"timeout={DETECTION_TIMEOUT_S:.1f}s, "
            f"accepted={accepted_total}, rejected={rejected_total}, "
            f"last_window={len(samples)}"
        )


# ========================= 集成示例 =========================
# 在完整抓取程序中创建一次（放在三循环for循环之外）：
#
#     stability_gate = ProductStabilityGate()
#
# 每轮到达Waypoint_2后，把原来的：
#
#     det = detect_product(...)
#
# 替换为：
#
#     stable = stability_gate.wait(
#         detect_once=lambda: detect_product_once(
#             model, camera, tf_api, report
#         ),
#         cycle_index=cycle_index,
#         refresh_ui=refresh_live_window,
#     )
#
#     det = dict(stable.raw_detection)
#     det["base_xyz"] = stable.base_xyz
#     det["confidence"] = stable.confidence
#     det["yaw_rad"] = stable.yaw_rad
#     det["yaw_deg"] = stable.yaw_deg
#     det["table_z_m"] = stable.table_z_m
#     det["product_z_range_m"] = stable.z_range_m
#     det["fixed_product_height_m"] = stable.fixed_product_height_m
#
# 随后保持原抓取程序的PreGrasp、姿态、抓取、腰部和循环逻辑不变。


if __name__ == "__main__":
    print("90_wait_until_product_stable.py")
    print("这是产品稳定门控模块，不会控制机器人。")
    print("请将ProductStabilityGate集成到已验证的三循环抓取主程序。")
