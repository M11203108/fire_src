import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

# 初始化 RealSense 管線
pipeline = rs.pipeline()

# 設定配置，啟用彩色流
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

try:
    # 開始拍攝
    pipeline.start(config)
    time.sleep(10)
    print("正在捕獲影像，按 Ctrl+C 停止...")

    # 捕獲單張影像
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()


    if not color_frame:
        raise RuntimeError("未能捕獲到彩色影像")

    # 將彩色影像轉換為 NumPy 陣列
    color_image = np.asanyarray(color_frame.get_data())

    # 顯示影像 (可選)
    cv2.imshow('RealSense Color Image', color_image)
    cv2.waitKey(1000)  # 顯示 1 秒

    # 保存影像到檔案
    cv2.imwrite('captured_image.jpg', color_image)
    print("影像已保存為 'captured_image.jpg'")

finally:
    # 停止管線
    pipeline.stop()
    cv2.destroyAllWindows()
