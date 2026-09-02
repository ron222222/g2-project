#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智元G2 手腕相机图像查看程序
支持JPEG编码格式
"""

import agibot_gdk
import time
import sys
import numpy as np


class WristCameraViewer:
    """G2手腕相机查看器"""

    def __init__(self, use_opencv=True):
        print("=" * 60)
        print("G2 手腕相机查看器")
        print("=" * 60)

        # 相机类型
        self.CAM_LEFT = agibot_gdk.CameraType.kHandLeftColor
        self.CAM_RIGHT = agibot_gdk.CameraType.kHandRightColor
        self.CAM_NAMES = {
            self.CAM_LEFT: "左手腕彩色相机",
            self.CAM_RIGHT: "右手腕彩色相机"
        }

        # 初始化GDK
        res = agibot_gdk.gdk_init()
        if res != agibot_gdk.GDKRes.kSuccess:
            print(f"❌ GDK初始化失败: {res}")
            sys.exit(1)
        print("✅ GDK初始化成功")

        self.camera = agibot_gdk.Camera()
        time.sleep(2)
        print("✅ 相机对象就绪")

        self.use_opencv = use_opencv
        self.running = True

        if use_opencv:
            try:
                import cv2
                self.cv2 = cv2
                print("✅ OpenCV已加载")
            except ImportError:
                print("❌ OpenCV未安装，请运行: pip install opencv-python")
                print("   或设置 use_opencv=False 使用Web模式")
                sys.exit(1)
        else:
            self._init_web_server()

    def _init_web_server(self):
        try:
            from flask import Flask, Response, render_template_string
            self.flask = Flask
            self.Response = Response
            self.render_template_string = render_template_string
            print("✅ Flask已加载")
        except ImportError:
            print("❌ Flask未安装，请运行: pip install flask")
            sys.exit(1)

    def decode_image(self, image_data):
        """
        解码GDK图像数据为numpy数组
        支持JPEG编码格式
        """
        if image_data is None:
            return None

        # 获取编码格式字符串
        encoding_str = str(image_data.encoding)
        print(f"   📷 编码格式: {encoding_str}")  # 调试用，可以注释掉

        # ========== 处理JPEG格式 ==========
        if "JPEG" in encoding_str or "jpeg" in encoding_str.lower():
            try:
                # 直接从JPEG数据解码
                img_array = np.frombuffer(image_data.data, dtype=np.uint8)
                img = self.cv2.imdecode(img_array, self.cv2.IMREAD_COLOR)
                if img is not None:
                    return img
                else:
                    print(f"   ⚠️ JPEG解码失败")
                    return None
            except Exception as e:
                print(f"   ⚠️ JPEG解码异常: {e}")
                return None

        # ========== 处理BGR8格式 ==========
        elif "bgr8" in encoding_str.lower():
            try:
                img = np.frombuffer(image_data.data, dtype=np.uint8)
                img = img.reshape((image_data.height, image_data.width, 3))
                return img
            except Exception as e:
                print(f"   ⚠️ BGR8解码失败: {e}")
                return None

        # ========== 处理RGB8格式 ==========
        elif "rgb8" in encoding_str.lower():
            try:
                img = np.frombuffer(image_data.data, dtype=np.uint8)
                img = img.reshape((image_data.height, image_data.width, 3))
                return img[:, :, ::-1]  # RGB -> BGR
            except Exception as e:
                print(f"   ⚠️ RGB8解码失败: {e}")
                return None

        # ========== 处理MONO8格式 ==========
        elif "mono8" in encoding_str.lower():
            try:
                img = np.frombuffer(image_data.data, dtype=np.uint8)
                img = img.reshape((image_data.height, image_data.width))
                return self.cv2.cvtColor(img, self.cv2.COLOR_GRAY2BGR)
            except Exception as e:
                print(f"   ⚠️ MONO8解码失败: {e}")
                return None

        # ========== 未知格式 ==========
        else:
            # 尝试直接作为JPEG解码（兜底方案）
            try:
                img_array = np.frombuffer(image_data.data, dtype=np.uint8)
                img = self.cv2.imdecode(img_array, self.cv2.IMREAD_COLOR)
                if img is not None:
                    print(f"   ✅ 作为JPEG解码成功")
                    return img
            except:
                pass

            print(f"   ⚠️ 未知编码格式: {encoding_str}")
            print(f"      数据大小: {len(image_data.data)} 字节")
            print(f"      图像尺寸: {image_data.width}x{image_data.height}")
            return None

    def get_image(self, camera_type, timeout=500):
        """获取相机图像"""
        try:
            img_data = self.camera.get_latest_image(camera_type, timeout)
            if img_data is None:
                return None
            return self.decode_image(img_data)
        except Exception as e:
            print(f"   ⚠️ 获取图像失败: {e}")
            return None

    def run_opencv(self):
        """使用OpenCV显示图像"""
        print("\n" + "=" * 60)
        print("OpenCV模式 - 实时显示手腕相机图像")
        print("=" * 60)
        print("\n控制说明:")
        print("  - 按 'q' 或 'ESC' 退出")
        print("  - 按 's' 保存当前图像")
        print("  - 按 'f' 切换全屏")
        print("=" * 60)

        # 创建窗口
        self.cv2.namedWindow("G2 左手腕相机", self.cv2.WINDOW_NORMAL)
        self.cv2.namedWindow("G2 右手腕相机", self.cv2.WINDOW_NORMAL)
        self.cv2.resizeWindow("G2 左手腕相机", 640, 480)
        self.cv2.resizeWindow("G2 右手腕相机", 640, 480)

        # 获取图像尺寸
        try:
            shape = self.camera.get_image_shape(self.CAM_LEFT)
            print(f"\n左手腕相机尺寸: {shape[0]}x{shape[1]}")
        except:
            pass
        try:
            shape = self.camera.get_image_shape(self.CAM_RIGHT)
            print(f"右手腕相机尺寸: {shape[0]}x{shape[1]}")
        except:
            pass

        frame_count = 0
        start_time = time.time()
        save_counter = 0
        fps_info = {}

        while self.running:
            # 获取图像
            img_left = self.get_image(self.CAM_LEFT, timeout=300)
            img_right = self.get_image(self.CAM_RIGHT, timeout=300)

            # 显示左腕图像
            if img_left is not None:
                h, w = img_left.shape[:2]
                info = f"Left Hand | {w}x{h}"
                self.cv2.putText(img_left, info, (10, 30),
                                 self.cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                 (0, 255, 0), 2)

                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    if elapsed > 0:
                        fps_info['left'] = frame_count / elapsed

                if 'left' in fps_info:
                    self.cv2.putText(img_left, f"FPS: {fps_info['left']:.1f}",
                                     (10, 60), self.cv2.FONT_HERSHEY_SIMPLEX,
                                     0.6, (0, 255, 255), 2)

                self.cv2.imshow("G2 左手腕相机", img_left)

            # 显示右腕图像
            if img_right is not None:
                h, w = img_right.shape[:2]
                info = f"Right Hand | {w}x{h}"
                self.cv2.putText(img_right, info, (10, 30),
                                 self.cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                 (0, 255, 0), 2)

                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    if elapsed > 0:
                        fps_info['right'] = frame_count / elapsed

                if 'right' in fps_info:
                    self.cv2.putText(img_right, f"FPS: {fps_info['right']:.1f}",
                                     (10, 60), self.cv2.FONT_HERSHEY_SIMPLEX,
                                     0.6, (0, 255, 255), 2)

                self.cv2.imshow("G2 右手腕相机", img_right)

            frame_count += 1

            # 键盘控制
            key = self.cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.running = False
                break
            elif key == ord('s'):
                save_counter += 1
                if img_left is not None:
                    self.cv2.imwrite(f"wrist_left_{save_counter}.jpg", img_left)
                    print(f"   ✅ 保存: wrist_left_{save_counter}.jpg")
                if img_right is not None:
                    self.cv2.imwrite(f"wrist_right_{save_counter}.jpg", img_right)
                    print(f"   ✅ 保存: wrist_right_{save_counter}.jpg")
            elif key == ord('f'):
                self.cv2.setWindowProperty("G2 左手腕相机",
                                           self.cv2.WND_PROP_FULLSCREEN,
                                           self.cv2.WINDOW_FULLSCREEN)
                self.cv2.setWindowProperty("G2 右手腕相机",
                                           self.cv2.WND_PROP_FULLSCREEN,
                                           self.cv2.WINDOW_FULLSCREEN)

        self.cv2.destroyAllWindows()

    def run_web(self):
        """Web模式"""
        from flask import Flask, Response, render_template_string

        HTML = '''
        <!DOCTYPE html>
        <html>
        <head>
            <title>G2 手腕相机</title>
            <style>
                body { background:#1a1a2e; color:white; font-family:Arial; text-align:center; }
                .container { display:flex; justify-content:center; gap:30px; flex-wrap:wrap; padding:20px; }
                .cam-box { background:#16213e; border-radius:10px; padding:15px; }
                .cam-box img { max-width:640px; max-height:480px; border-radius:5px; }
                .info { color:#aaa; font-size:12px; margin-top:5px; }
            </style>
        </head>
        <body>
            <h1>🖐️ G2 手腕相机</h1>
            <div class="container">
                <div class="cam-box">
                    <h3>左手腕</h3>
                    <img id="left_img" src="/stream/left"/>
                    <div class="info" id="left_info">等待图像...</div>
                </div>
                <div class="cam-box">
                    <h3>右手腕</h3>
                    <img id="right_img" src="/stream/right"/>
                    <div class="info" id="right_info">等待图像...</div>
                </div>
            </div>
            <script>
                function updateInfo(side) {
                    const now = new Date();
                    document.getElementById(side + '_info').textContent = '更新: ' + now.toLocaleTimeString();
                }
                setInterval(() => { updateInfo('left'); updateInfo('right'); }, 2000);
            </script>
        </body>
        </html>
        '''

        app = Flask(__name__)

        def generate_frames(camera_type):
            while self.running:
                img = self.get_image(camera_type, timeout=300)
                if img is not None:
                    ret, jpeg = self.cv2.imencode('.jpg', img,
                                                  [self.cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' +
                               jpeg.tobytes() + b'\r\n')
                time.sleep(0.1)

        @app.route('/')
        def index():
            return render_template_string(HTML)

        @app.route('/stream/left')
        def stream_left():
            return Response(generate_frames(self.CAM_LEFT),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route('/stream/right')
        def stream_right():
            return Response(generate_frames(self.CAM_RIGHT),
                            mimetype='multipart/x-mixed-replace; boundary=frame')

        print("\n🌐 服务器启动: http://localhost:5000")
        print("   按 Ctrl+C 停止")
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)

    def run(self):
        if self.use_opencv:
            self.run_opencv()
        else:
            self.run_web()

    def shutdown(self):
        print("\n>>> 释放GDK...")
        agibot_gdk.gdk_release()
        print("✅ GDK释放成功")


def main():
    import sys

    print("=" * 60)
    print("G2 手腕相机查看器")
    print("=" * 60)

    # 检查OpenCV
    try:
        import cv2
        use_opencv = True
        print("✅ OpenCV可用，使用窗口模式")
    except ImportError:
        use_opencv = False
        print("⚠️ OpenCV未安装，使用Web模式")
        print("   安装: pip install opencv-python")

    viewer = None
    try:
        viewer = WristCameraViewer(use_opencv=use_opencv)
        viewer.run()
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    except Exception as e:
        print(f"\n❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if viewer:
            viewer.shutdown()


if __name__ == "__main__":
    main()