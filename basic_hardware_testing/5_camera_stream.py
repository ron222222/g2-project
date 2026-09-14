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

    # ===== 3. 选择相机 =====
    cam_type = agibot_gdk.CameraType.kHeadColor
    print(f"\n📷 启动实时视频流: {cam_type}")
    print("   按 Q 键退出\n")

    # 用于计算 FPS
    frame_count = 0
    fps = 0.0
    fps_start = time.time()

    # ===== 4. 实时采集循环 =====
    while True:
        loop_start = time.time()

        # 读取最新一帧（timeout 100ms，避免阻塞太久）
        try:
            img = camera.get_latest_image(cam_type, 100.0)
        except RuntimeError as e:
            # 偶尔丢帧正常，跳过即可
            continue
        # ===== 解码图像 =====
        arr = np.frombuffer(img.data, dtype=np.uint8)

        if img.encoding == agibot_gdk.Encoding.JPEG:
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue
        else:
            # 原始像素
            if img.color_format in ['RGB', 'rgb', 'BGR', 'bgr']:
                frame = arr.reshape((img.height, img.width, 3))
            elif img.color_format in ['RGBA', 'rgba', 'BGRA', 'bgra']:
                frame = arr.reshape((img.height, img.width, 4))
            else:
                frame = arr.reshape((img.height, img.width))

        # ===== 计算并显示 FPS =====
        frame_count += 1
        elapsed = time.time() - fps_start
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            fps_start = time.time()

        # 在画面上叠加信息
        info_text = f"FPS: {fps:.1f} | {img.width}x{img.height} | {img.encoding}"
        cv2.putText(frame, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ===== 显示 =====
        cv2.imshow("G2 Real-time Camera Stream", frame)

        # 按 Q 退出（必须调用 waitKey，否则窗口不刷新）
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n⏹️ 用户按 Q 退出")
            break

        # 控制最大帧率约 20fps（每帧至少等 50ms）
        sleep_time = 0.05 - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # ===== 5. 释放资源 =====
    cv2.destroyAllWindows()
    camera.close_camera()
    if agibot_gdk.gdk_release() != agibot_gdk.GDKRes.kSuccess:
        print("❌ GDK 释放失败")
    else:
        print("✅ GDK 释放成功")
    print("🎉 视频流结束")

if __name__ == "__main__":
    main()