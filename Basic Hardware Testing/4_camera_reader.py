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

    # ===== 3. 读取头部彩色相机 =====
    cam_type = agibot_gdk.CameraType.kHeadColor
    print(f"\n📷 正在读取 {cam_type} ...")

    try:
        img = camera.get_latest_image(cam_type, 1000.0)
    except RuntimeError as e:
        print(f"⚠️ 获取图像失败: {e}")
        camera.close_camera()
        agibot_gdk.gdk_release()
        return

    # ===== 4. 打印图像信息 =====
    print(f"\n✅ 获取图像成功！")
    print(f"   尺寸:      {img.width} x {img.height}")
    print(f"   编码:      {img.encoding}")
    print(f"   颜色格式:  {img.color_format}")
    print(f"   位深:      {img.bit_depth}")
    print(f"   时间戳:    {img.timestamp_ns} ns")
    print(f"   数据大小:  {len(img.data)} bytes")

    # ===== 5. 转为 numpy 数组并解码 =====
    arr = np.frombuffer(img.data, dtype=np.uint8)

    if img.encoding == agibot_gdk.Encoding.JPEG:
        # JPEG 压缩数据，用 cv2 解码
        arr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if arr is None:
            print("❌ JPEG 解码失败")
            return
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        print("   已解码 JPEG 图像 (BGR→RGB)")
    else:
        # 原始像素数据，直接 reshape
        if img.color_format in ['RGB', 'rgb', 'BGR', 'bgr']:
            arr = arr.reshape((img.height, img.width, 3))
        elif img.color_format in ['RGBA', 'rgba', 'BGRA', 'bgra']:
            arr = arr.reshape((img.height, img.width, 4))
        else:
            arr = arr.reshape((img.height, img.width))

        if img.bit_depth == 16:
            arr = arr.view(np.uint16)

    print(f"   解码后 shape: {arr.shape}")

    # ===== 6. 保存图片 =====
    # OpenCV 保存需要 BGR
    save_arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    cv2.imwrite("g2_camera_capture.png", save_arr)
    print("\n💾 已保存到: g2_camera_capture.png")

    # ===== 7. 尝试显示 =====
    try:
        # 显示也用 BGR
        cv2.imshow("G2 Head Color Camera", save_arr)
        print("🖥️  图像窗口已打开，按任意键关闭...")
        cv2.waitKey(5000)
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"🖥️  无法显示窗口（无 GUI）: {e}")
        print("   请直接打开 g2_camera_capture.png 查看")

    # ===== 8. 释放资源 =====
    camera.close_camera()
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("\n✅ GDK 释放成功")
    print("🎉 程序结束")

if __name__ == "__main__":
    main()


'''G2 可用相机类型对照表
表格
相机枚举	                名称	       说明
kHeadColor	        头部彩色相机	推荐先用这个，直观可见
kHeadDepth	        头部深度相机	输出深度图（16位）
kHeadStereoLeft	    头部立体左相机	双目视觉
kHeadStereoRight	头部立体右相机	双目视觉
kHeadLeftFisheye	头部左侧鱼眼	广角环视
kHeadRightFisheye	头部右侧鱼眼	广角环视
kHeadBackFisheye	头部背部鱼眼	后视
kHandLeft	        左手相机	手部操作视角
kHandRight	        右手相机	手部操作视角'''