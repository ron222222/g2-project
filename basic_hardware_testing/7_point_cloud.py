import agibot_gdk
import time
import numpy as np
import cv2


def depth_to_point_cloud(depth_img, intrinsic, max_depth=10000):
    """
    将深度图转换为 3D 点云
    depth_img: (H, W) 的 uint16 深度图，单位 mm
    intrinsic: [fx, fy, cx, cy]
    max_depth: 最大有效深度（mm），超过的过滤掉
    """
    fx, fy, cx, cy = intrinsic
    height, width = depth_img.shape

    # 创建像素坐标网格
    u = np.arange(width)
    v = np.arange(height)
    u, v = np.meshgrid(u, v)

    # 获取深度值
    z = depth_img.astype(np.float32)

    # 过滤无效深度（0 或超过 max_depth）
    valid_mask = (z > 0) & (z < max_depth)

    # 只保留有效点
    u = u[valid_mask]
    v = v[valid_mask]
    z = z[valid_mask]

    # 相机坐标系下的 3D 坐标
    # x = (u - cx) * z / fx
    # y = (v - cy) * z / fy
    # z = z
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # 组合成 (N, 3) 点云
    points = np.stack([x, y, z], axis=-1)

    return points, valid_mask


def save_ply(filename, points, colors=None):
    """
    保存点云为 .ply 格式（ASCII）
    points: (N, 3) numpy array
    colors: (N, 3) numpy array, 0-255
    """
    N = len(points)
    has_color = colors is not None

    with open(filename, 'w') as f:
        # 写入 PLY 头
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {N}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        if has_color:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
        f.write("end_header\n")

        # 写入顶点数据
        if has_color:
            for p, c in zip(points, colors):
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} ")
                f.write(f"{int(c[0])} {int(c[1])} {int(c[2])}\n")
        else:
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    print(f"💾 点云已保存: {filename} ({N} 个点)")


def main():
    # ===== 1. 初始化 GDK =====
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 初始化失败")
        return
    print("✅ GDK 初始化成功")

    # ===== 2. 创建相机对象 =====
    camera = agibot_gdk.Camera()
    print("✅ 相机对象创建成功")
    time.sleep(2)

    # ===== 3. 读取头部深度相机 =====
    cam_type = agibot_gdk.CameraType.kHeadDepth
    print(f"\n📷 正在读取 {cam_type} ...")

    try:
        img = camera.get_latest_image(cam_type, 1000.0)
    except RuntimeError as e:
        print(f"⚠️ 获取深度图失败: {e}")
        camera.close_camera()
        agibot_gdk.gdk_release()
        return

    # ===== 4. 解码深度数据 =====
    arr = np.frombuffer(img.data, dtype=np.uint8)
    arr = arr.view(np.uint16)
    depth_raw = arr.reshape((img.height, img.width))

    print(f"✅ 深度图: {depth_raw.shape}, 范围 {depth_raw.min()}~{depth_raw.max()} mm")

    # ===== 5. 获取相机内参 =====
    intrinsic_obj = camera.get_camera_intrinsic(cam_type)
    fx, fy, cx, cy = intrinsic_obj.intrinsic
    print(f"\n📐 相机内参: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

    # ===== 6. 生成点云 =====
    print("\n☁️  正在生成点云...")
    points, mask = depth_to_point_cloud(depth_raw, [fx, fy, cx, cy], max_depth=5000)
    print(f"   有效点数: {len(points)} / {img.width * img.height}")

    # ===== 7. 可选：给点云上色（伪彩色）=====
    # 根据深度值生成颜色
    z_values = points[:, 2]
    z_norm = (z_values - z_values.min()) / (z_values.max() - z_values.min() + 1e-6)

    # 使用 JET 颜色映射
    colors = cv2.applyColorMap((z_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    colors = colors.reshape(-1, 3)  # (N, 3) BGR

    # 保存点云
    save_ply("g2_point_cloud.ply", points, colors)

    # 同时保存一个无颜色的版本（更小）
    save_ply("g2_point_cloud_mono.ply", points)

    # ===== 8. 统计信息 =====
    print(f"\n📊 点云统计:")
    print(f"   X 范围: {points[:, 0].min():.1f} ~ {points[:, 0].max():.1f} mm")
    print(f"   Y 范围: {points[:, 1].min():.1f} ~ {points[:, 1].max():.1f} mm")
    print(f"   Z 范围: {points[:, 2].min():.1f} ~ {points[:, 2].max():.1f} mm")

    # ===== 9. 释放资源 =====
    camera.close_camera()
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("\n✅ GDK 释放成功")
    print("🎉 程序结束")


if __name__ == "__main__":
    main()