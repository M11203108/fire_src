import math

# 視場角轉換為弧度
fov_x = math.radians(51)
fov_y = math.radians(39)

# 影像尺寸
W = 256
H = 192

# 計算焦距（像素單位）
fx = W / (2 * math.tan(fov_x / 2))
fy = H / (2 * math.tan(fov_y / 2))

print(f"焦距 fx: {fx:.2f} 像素")
print(f"焦距 fy: {fy:.2f} 像素")
