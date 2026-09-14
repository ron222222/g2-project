import agibot_gdk
import time
import numpy as np
import cv2


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
        print("   可能原因：深度相机未开启 publish")
        camera.close_camera()
        agibot_gdk.gdk_release()
        return

    # ===== 4. 打印图像信息 =====
    print(f"\n✅ 获取深度图成功！")
    print(f"   尺寸:      {img.width} x {img.height}")
    print(f"   编码:      {img.encoding}")
    print(f"   颜色格式:  {img.color_format}")
    print(f"   位深:      {img.bit_depth}")
    print(f"   时间戳:    {img.timestamp_ns} ns")
    print(f"   数据大小:  {len(img.data)} bytes")

    # ===== 5. 解码深度数据 =====
    arr = np.frombuffer(img.data, dtype=np.uint8)

    if img.encoding == agibot_gdk.Encoding.JPEG:
        # 深度图也可能是 JPEG 压缩
        depth_raw = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            print("❌ JPEG 解码失败")
            return
    else:
        # 原始 16 位深度数据
        if img.bit_depth == 16:
            arr = arr.view(np.uint16)
        depth_raw = arr.reshape((img.height, img.width))

    print(f"   解码后 shape: {depth_raw.shape}")
    print(f"   深度范围: {depth_raw.min()} ~ {depth_raw.max()} mm")

    # ===== 6. 可视化深度图 =====
    # 16 位深度值范围通常是 0~65535 mm，但实际有效范围可能更小
    # 用对数映射或线性映射转为 8 位灰度/伪彩色显示

    # 方法 A：线性映射到 0~255（适合近距离）
    depth_vis = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # 方法 B：伪彩色映射（更直观，近红远蓝）
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

    # 保存
    cv2.imwrite("g2_depth_raw.png", depth_raw)  # 原始 16 位深度图
    cv2.imwrite("g2_depth_color.png", depth_color)  # 伪彩色可视化
    print("\n💾 已保存:")
    print("   g2_depth_raw.png    (原始 16 位深度数据)")
    print("   g2_depth_color.png  (伪彩色可视化)")

    # ===== 7. 显示 =====
    try:
        cv2.imshow("Depth Raw (mm)", depth_vis)
        cv2.imshow("Depth Color", depth_color)
        print("🖥️  图像窗口已打开，按任意键关闭...")
        cv2.waitKey(5000)
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"🖥️  无法显示窗口: {e}")
        print("   请直接打开保存的图片查看")

    # ===== 8. 生成点云（可选）=====
    # 如果有相机内参，可以把深度图转为 3D 点云
    # ===== 8. 查看内参对象结构 =====
    try:
        intrinsic = camera.get_camera_intrinsic(cam_type)
        print(f"\n📐 相机内参对象属性:")
        print(dir(intrinsic))

        # 尝试打印所有属性值
        for attr in dir(intrinsic):
            if not attr.startswith('_'):
                try:
                    val = getattr(intrinsic, attr)
                    if not callable(val):
                        print(f"   {attr} = {val}")
                except:
                    pass
    except Exception as e:
        print(f"\n⚠️ 无法获取内参: {e}")

    # ===== 9. 释放资源 =====
    camera.close_camera()
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("\n✅ GDK 释放成功")
    print("🎉 程序结束")


if __name__ == "__main__":
    main()
