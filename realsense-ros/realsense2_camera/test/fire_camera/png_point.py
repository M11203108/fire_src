import cv2
import numpy as np

# 初始化一個空的點數組
points_d435 = []

# 滑鼠回調函數，用於在點擊時存儲點的座標
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points_d435.append((x, y))
        print(f"點座標: ({x}, {y})")
        if len(points_d435) == 4:
            cv2.destroyAllWindows()  # 選好四個點後關閉視窗

# 顯示 D435i 的影像並進行滑鼠點擊來標記四個對應點
d435_image = cv2.imread("/home/robot/wheeltec_ros2/src/captured_image1.jpg")  # 這裡使用 D435i 的影像
cv2.imshow("D435i Image", d435_image)
cv2.setMouseCallback("D435i Image", mouse_callback)
cv2.waitKey(0)

# 當你完成四個點的標記後，創建 pts2
if len(points_d435) == 4:
    pts2 = np.array(points_d435, dtype="float32")
    print("pts2: ", pts2)
else:
    print("請標記四個點來生成 pts2")
