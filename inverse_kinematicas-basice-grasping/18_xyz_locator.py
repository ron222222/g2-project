#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
18_xyz_locator.py
G2 头部 RGBD 像素点 -> 深度相机三维坐标测试

功能：
1. 读取 kHeadColor 与 kHeadDepth
2. 鼠标点击彩色图像
3. 将彩色图像像素按分辨率映射到深度图
4. 使用深度相机内参把 (u, v, depth) 反投影为相机坐标 X/Y/Z
5. 使用点击点附近有效深度的中位数，降低黑色产品深度空洞/噪声影响

注意：
- 输出 XYZ 当前属于“头部深度相机坐标系”，不是 base_link。
- 本程序只做坐标测量，不控制机械臂。
- 下一阶段才使用 TF/外参把相机坐标转换到 base_link。
"""

import time
import math
import cv2
import numpy as np
import agibot_gdk

WINDOW_NAME = 'G2 Head RGBD XYZ Locator'
DEPTH_WINDOW_RADIUS = 3          # 7x7邻域
MIN_VALID_DEPTH = 50.0
MAX_VALID_DEPTH = 10000.0

click_x = -1
click_y = -1


def mouse_callback(event, x, y, flags, param):
    global click_x, click_y
    if event == cv2.EVENT_LBUTTONDOWN:
        click_x, click_y = x, y
        print(f'\nColor pixel clicked: ({x}, {y})')


def decode_color(image):
    data = np.frombuffer(image.data, dtype=np.uint8)
    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame is not None:
        return frame

    # 兼容未压缩格式
    try:
        if image.bit_depth == 8 and len(data) == image.width * image.height * 3:
            return data.reshape((image.height, image.width, 3))
    except Exception:
        pass
    return None


def decode_depth(image):
    try:
        if image.bit_depth == 16:
            arr = np.frombuffer(image.data, dtype=np.uint16)
        elif image.bit_depth == 32:
            arr = np.frombuffer(image.data, dtype=np.float32)
        else:
            # G2常见深度格式为Z16；无法判断时优先尝试uint16
            arr = np.frombuffer(image.data, dtype=np.uint16)
        return arr.reshape((image.height, image.width))
    except Exception as exc:
        print(f'Depth decode error: {exc}')
        return None


def get_numeric_attr(obj, candidate_names):
    """兼容不同GDK版本可能采用的内参字段命名。"""
    for name in candidate_names:
        if hasattr(obj, name):
            value = getattr(obj, name)
            try:
                return float(value), name
            except (TypeError, ValueError):
                continue
    return None, None


def read_intrinsics(camera):
    intrinsic = camera.get_camera_intrinsic(
        agibot_gdk.CameraType.kHeadDepth
    )

    fx, fx_name = get_numeric_attr(intrinsic, [
        'fx', 'focal_length_x', 'focal_x', 'f_x'
    ])
    fy, fy_name = get_numeric_attr(intrinsic, [
        'fy', 'focal_length_y', 'focal_y', 'f_y'
    ])
    cx, cx_name = get_numeric_attr(intrinsic, [
        'cx', 'principal_point_x', 'principal_x', 'c_x', 'ppx'
    ])
    cy, cy_name = get_numeric_attr(intrinsic, [
        'cy', 'principal_point_y', 'principal_y', 'c_y', 'ppy'
    ])

    # 某些SDK可能把内参放在K矩阵/k数组中
    if None in (fx, fy, cx, cy):
        for matrix_name in ('k', 'K', 'camera_matrix', 'intrinsic_matrix'):
            if hasattr(intrinsic, matrix_name):
                raw = np.asarray(getattr(intrinsic, matrix_name), dtype=float).reshape(-1)
                if raw.size >= 9:
                    fx, fy, cx, cy = raw[0], raw[4], raw[2], raw[5]
                    fx_name = fy_name = cx_name = cy_name = matrix_name
                    break

    if None in (fx, fy, cx, cy):
        public_attrs = [name for name in dir(intrinsic) if not name.startswith('_')]
        print('CameraIntrinsic可见字段:')
        for name in public_attrs:
            try:
                print(f'  {name} = {getattr(intrinsic, name)}')
            except Exception:
                pass
        raise RuntimeError(
            '无法自动识别相机内参字段。请把上面的CameraIntrinsic字段输出发给Copilot。'
        )

    print('HeadDepth内参读取成功:')
    print(f'  fx={fx:.6f} ({fx_name})')
    print(f'  fy={fy:.6f} ({fy_name})')
    print(f'  cx={cx:.6f} ({cx_name})')
    print(f'  cy={cy:.6f} ({cy_name})')
    return fx, fy, cx, cy, intrinsic


def median_valid_depth(depth_map, u, v, radius=DEPTH_WINDOW_RADIUS):
    h, w = depth_map.shape[:2]
    x0, x1 = max(0, u-radius), min(w, u+radius+1)
    y0, y1 = max(0, v-radius), min(h, v+radius+1)
    patch = depth_map[y0:y1, x0:x1].astype(np.float64)
    valid = patch[np.isfinite(patch)]
    valid = valid[(valid > MIN_VALID_DEPTH) & (valid < MAX_VALID_DEPTH)]
    if valid.size == 0:
        return None, 0
    return float(np.median(valid)), int(valid.size)


def infer_depth_scale(depth_value):
    """
    G2当前实测深度显示1577，结合常见Z16格式按毫米解释。
    若设备返回小数米制，则保留米制。
    """
    if depth_value > 20.0:
        return 0.001, 'mm'
    return 1.0, 'm'


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print('GDK初始化失败')
        return

    camera = agibot_gdk.Camera()
    time.sleep(3.0)

    try:
        fx, fy, cx, cy, _ = read_intrinsics(camera)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

        last_text = 'Click product center'
        last_detail = ''
        last_report = None

        while True:
            rgb_image = camera.get_latest_image(
                agibot_gdk.CameraType.kHeadColor, 1000.0
            )
            depth_image = camera.get_latest_image(
                agibot_gdk.CameraType.kHeadDepth, 1000.0
            )

            if rgb_image is None or depth_image is None:
                continue

            frame = decode_color(rgb_image)
            depth_map = decode_depth(depth_image)
            if frame is None or depth_map is None:
                continue

            color_h, color_w = frame.shape[:2]
            depth_h, depth_w = depth_map.shape[:2]

            if click_x >= 0 and click_y >= 0:
                # RGB与Depth分辨率可能不同，先按比例映射
                u_d = int(round(click_x * depth_w / color_w))
                v_d = int(round(click_y * depth_h / color_h))
                u_d = max(0, min(depth_w - 1, u_d))
                v_d = max(0, min(depth_h - 1, v_d))

                raw_depth, valid_count = median_valid_depth(depth_map, u_d, v_d)

                cv2.circle(frame, (click_x, click_y), 7, (0, 0, 255), -1)

                if raw_depth is not None:
                    scale, raw_unit = infer_depth_scale(raw_depth)
                    z_m = raw_depth * scale
                    x_m = (u_d - cx) * z_m / fx
                    y_m = (v_d - cy) * z_m / fy

                    last_text = f'Camera XYZ(m): {x_m:.3f}, {y_m:.3f}, {z_m:.3f}'
                    last_detail = f'RGB({click_x},{click_y}) Depth({u_d},{v_d}) raw={raw_depth:.1f}{raw_unit}'

                    report = (
                        click_x, click_y, u_d, v_d,
                        round(raw_depth, 3),
                        round(x_m, 6), round(y_m, 6), round(z_m, 6)
                    )
                    if report != last_report:
                        print(f'Depth pixel: ({u_d}, {v_d}), valid samples: {valid_count}')
                        print(f'Raw depth: {raw_depth:.3f} {raw_unit}')
                        print('Head depth camera coordinates (m):')
                        print(f'  X={x_m:.6f}')
                        print(f'  Y={y_m:.6f}')
                        print(f'  Z={z_m:.6f}')
                        print('注意：以上坐标还不是base_link，当前程序不会控制机器人。')
                        last_report = report
                else:
                    last_text = 'Invalid depth near clicked point'
                    last_detail = f'RGB({click_x},{click_y}) Depth({u_d},{v_d})'

            cv2.putText(frame, last_text, (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, last_detail, (15, 68),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
            cv2.putText(frame, f'Color={color_w}x{color_h} Depth={depth_w}x{depth_h}',
                        (15, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

    except KeyboardInterrupt:
        print('\n用户中断')
    except Exception as exc:
        print(f'程序异常: {exc}')
        raise
    finally:
        cv2.destroyAllWindows()
        try:
            camera.close_camera()
        except Exception:
            pass
        agibot_gdk.gdk_release()


if __name__ == '__main__':
    main()
