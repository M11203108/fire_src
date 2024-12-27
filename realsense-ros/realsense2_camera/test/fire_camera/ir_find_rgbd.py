import numpy as np

# 熱像儀內參
fx_thermal = 268.36 # 透過公式計算得到的焦距
fy_thermal = 271.1
cx_thermal = 128
cy_thermal = 96

# D435i內參
fx_d435 = 654.5083
fy_d435 = 654.5083
cx_d435 = 647.8071
cy_d435 = 356.2889

# 定義內參矩陣
K_thermal = np.array([[fx_thermal, 0, cx_thermal],
                      [0, fy_thermal, cy_thermal],
                      [0, 0, 1]])

K_d435 = np.array([[fx_d435, 0, cx_d435],
                   [0, fy_d435, cy_d435],
                   [0, 0, 1]])

# 熱像儀上的熱點位置 (像素坐標)
x_thermal, y_thermal = 82, 107

# 將像素坐標轉換為相機坐標 (假設Z = 1, 以便進行歸一化)
thermal_pixel_coords = np.array([x_thermal, y_thermal, 1])
thermal_camera_coords = np.linalg.inv(K_thermal).dot(thermal_pixel_coords)

# 相機間的相對位移 (單位：米)
delta_x = 0.079  # 7.73 cm
delta_y = 0.0145   # 1.9 cm
delta_z = 0.00205       # 假設無深度方向的偏移

# 將熱像儀相機坐標轉換到D435i相機坐標系
d435_camera_coords = thermal_camera_coords + np.array([delta_x, delta_y, delta_z])

# 將D435i相機坐標轉換回像素坐標
d435_pixel_coords = K_d435.dot(d435_camera_coords)
d435_pixel_coords /= d435_pixel_coords[2]  # 進行歸一化

# 打印轉換後的D435i深度影像像素位置
x_d435, y_d435 = int(d435_pixel_coords[0]), int(d435_pixel_coords[1])
print(f"轉換後的D435i深度影像位置: (x_d435, y_d435) = ({x_d435}, {y_d435})")
