import cv2
import numpy as np
import matplotlib.pyplot as plt

# 讀取 RGB 圖片
rgb_image = cv2.imread('/home/robot/wheeltec_ros2/captured_image.jpg')

# 將 RGB 圖片轉換為灰度圖像
gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

# 設定亮度範圍（閾值處理）來分割銀色物體
_, bright_thresh = cv2.threshold(gray_image, 200, 255, cv2.THRESH_BINARY)

# 使用 Canny 邊緣檢測
edges = cv2.Canny(bright_thresh, 100, 200)

# 找到邊緣的輪廓
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 遍歷所有輪廓，選擇可能是熱水壺的輪廓
for contour in contours:
    # 計算輪廓的面積，忽略過小或過大的輪廓
    area = cv2.contourArea(contour)
    if 500 < area < 5000:  # 根據實際熱水壺大小調整面積範圍
        # 繪製輪廓到原始 RGB 圖片上
        cv2.drawContours(rgb_image, [contour], -1, (0, 255, 0), 2)

# 顯示原始 RGB 圖片和檢測到輪廓的圖像
fig, axes = plt.subplots(1, 2, figsize=(12, 6))
axes[0].imshow(cv2.cvtColor(gray_image, cv2.COLOR_BGR2RGB))
axes[0].set_title("Original RGB Image")
axes[0].axis('off')

axes[1].imshow(cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
axes[1].set_title("Detected Kettle with Contours")
axes[1].axis('off')

plt.tight_layout()
plt.show()